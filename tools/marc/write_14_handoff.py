"""Finalize the D0 evidence registry and Korean handoff without promoting missing inputs."""
from pathlib import Path
import argparse,datetime,hashlib,json,shutil,subprocess

def run(ctx):
    f=Path(ctx['run_dir']);root=Path(ctx['repo_root'])
    def read(name):return json.loads((f/name).read_text())
    def write(name,data):(f/name).write_text(json.dumps(data,ensure_ascii=False,indent=2)+'\n')
    source=read('source_manifest.json');parts=read('parts_manifest.json');g01=read('evidence/B01_profile.json');g02=read('evidence/G02_kinematics.json');g03=read('evidence/G03_P00_intersections.json')
    assert hashlib.sha256((root/'docs/30-scone-v3-design-plan.md').read_bytes()).hexdigest()==ctx['source_sha256']
    preserved={n:((root/n).exists() and hashlib.sha256((root/n).read_bytes()).hexdigest()==h) for n,h in source['protected_files'].items()}
    assert all(preserved.values()),'Protected file changed during run'
    write('evidence/protected_files_check.json',{'status':'PASS','files':preserved})
    spin=[x for x in parts['parts'] if x['instance_id'] in ('FL_SPIN/sector_PLA','FL_SPIN/tire_TPU')]
    inertia=sum(x['inertia_com_local_kgm2'][2][2]+x['mass_kg']*sum(v*v for v in x['com_local_m'][:2]) for x in spin)
    write('evidence/G05_spin_inertia.json',{'status':'FAIL' if inertia>.0022 else 'PASS','stage':'D0','axis':'Spin component local Z through axle','value_kgm2':inertia,'limit_kgm2':.0022,'excess_percent':100*(inertia/.0022-1),'included':['sector_PLA','tire_TPU'],'excluded':['unresolved horn, shaft, bearing and final fastening'],'method':'Native component inertia at CoM plus parallel-axis translation; installed API tensor convention independently verified'})
    labels={1:'모터 ID·모델·펌웨어·영점·부호·모드',2:'혼·포크·베어링·스터브 인터페이스',3:'실제 타이어 셀·밀도·고정 방식',4:'출력 공정·공차·강도·크리프',5:'센서·Jetson·U2D2 도면과 광학 변환',6:'배터리·전원 회로·보호 조건',7:'실제 모터 운전 한계',8:'케이블·커넥터·교체 경로',9:'시스템 성공 조건과 실제 환경'}
    write('inputs.json',{f'I{i:02}':{'label':label,'value':None,'unit':None,'source':None,'revision':None,'method':None,'measured_at':None,'uncertainty':None,'status':'BLOCKED','candidate_evidence':['evidence/design_plan_snapshot.md'],'reason':'문서 후보값은 보유품 도면/측정 완료를 대신하지 않는다.'} for i,label in labels.items()})
    statuses={'G00':('PASS','Source snapshot, 48 parameter key/unit/value comparisons and protected files verified',['evidence/G00_parameter_match.json','evidence/protected_files_check.json'],[]),'G01':('PASS','Polyline convergence and two valid sector/tire solids',['evidence/B01_profile.json','evidence/B03_sector.json'],[]),'G02':('BLOCKED','CAD passes 12/12 positive increments including affected chains; hardware mapping missing',['evidence/G02_kinematics.json'],['I01']),'G03':('FAIL','24 P00 intersections; 4 solid and 20 envelope pairs. Motor and fork detailed coverage incomplete. Paths not run',['evidence/G03_P00_intersections.json'],['I02','I03','I05','I08']),'G04':('BLOCKED','Actual torque/current/thermal limits and valid contact paths missing',['calculations.json'],['I07','I09']),'G05':('FAIL','Spin inertia exceeds candidate limit by 1.4106%; complete mass/CoM remain blocked',['parts_manifest.json','evidence/G05_spin_inertia.json'],['I02','I03','I05','I06','I08']),'G06':('BLOCKED','D0 solid placeholders and preliminary material values; no assembly or structural acceptance',[],['I02','I03','I04']),'G07':('BLOCKED','Only payload and reserved-space layout; plug/cable/door paths not verified',[],['I05','I08']),'G08':('BLOCKED','Power design and tests absent',[],['I06','I07']),'G09':('BLOCKED','Sensor envelopes only; optical and front-leg occlusion performance not measured',[],['I05','I09']),'G10':('NOT_RUN','Native F3D and STEP are CAD exports; no MuJoCo asset or dynamics verification',[],[]),'G11':('NOT_RUN','Pose/trajectory generator not implemented; no operational motion permission',[],[])}
    checks={}
    for key,(status,reason,evidence,blocking) in statuses.items():checks[key]={'check_id':key,'status':status,'model_stage':'D0','input_sha256':ctx['params_sha256'],'source_sha256':ctx['source_sha256'],'method_version':'marc-D0-initial-1','reason':reason,'evidence':evidence,'blocking_inputs':blocking}
    checks['G02']['subchecks']={'CAD':g02['status'],'hardware':'BLOCKED'}
    checks['G05']['subchecks']={'inertia_units':'PASS','spin_axis_inertia':'FAIL','complete_BOM':'BLOCKED'}
    write('checks.json',checks)
    # Freeze the actual implementation separately from the current editable tools directory.
    snapshot=f/'evidence/tools_snapshot';snapshot.mkdir(exist_ok=True)
    toolhash={}
    for path in sorted((root/'tools/marc').glob('*.py')):
        if path.name=='configure_joints.py':continue # superseded diagnosis, never run this repair again
        target=snapshot/path.name;target.write_bytes(path.read_bytes());toolhash[str(target.relative_to(f))]=hashlib.sha256(target.read_bytes()).hexdigest()
    source.update(tool_sha256=toolhash,completed_at=datetime.datetime.now().astimezone().isoformat(),protected_files_verified=True,document_snapshot='evidence/design_plan_snapshot.md',parameters_sha256=ctx['params_sha256'],known_analytical_conflicts=['Section 6.3 disallows double-counting internal efficiency but section 16 uses design_factor / drive_efficiency. Load reference output is not operational acceptance.'])
    write('source_manifest.json',source)
    for dispatcher in (f/'cad').glob('dispatch_*.py'):
        if dispatcher.name=='dispatch_joint_config.py':continue
        text=dispatcher.read_text().replace(str(root/'tools/marc'),str(snapshot))
        dispatcher.write_text(text)
    # Named phase offsets are command data, independent of the CAD reference pose.
    (f/'gaits').mkdir(exist_ok=True)
    write('gaits/diagonal_phase_candidate.json',{'status':'NOT_RUN','order':['FL','FR','RL','RR'],'absolute_phase_deg':[0,180,180,0],'use':'Future P06 diagnostic only; reference CAD uses alpha=0 on all legs','blocked_by':'P00 intersections and missing G11 motion guards'})
    doc=read('cad/document.json')
    text=f'''# MARC v4 D0 — Fusion 초기 설계 인수인계

실행: `{f.name}`. 원천은 30번 문서 **rev.4**이며, 6족 rev.3를 모델링하지 않았다.
현재 결과는 **4족 D0 기준 형상과 관절이 구현된 검토 모델**이다. 체결·제작·보행 설계가 완료된 상태는 아니다.

## 파일과 식별자

- Fusion SCONE 프로젝트: **{doc['name']}**
- 문서 ID: `{ctx['document_id']}`
- [Fusion 네이티브 저장본](cad/MARC-PARAM-D0.f3d)
- [STEP — 표시 중인 형상 28개](cad/MARC-PARAM-D0.step)
- [전체 보기](evidence/MARC-D0-assembly.png)
- [원문 스냅샷](evidence/design_plan_snapshot.md), [입력값](params.json), [관문별 결과](checks.json)
- 원문 SHA256: `{ctx['source_sha256']}`
- 기존 `SCONEv3 v8`은 변경 없이 열려 있다. 기존 코드·아카이브·30번 문서도 수정하지 않았다.
- `artifacts/scone_v3/20260909_092656_rev3-D0`은 문서 변경 전의 입력 계산만 수행한 이전 준비 기록이다. Fusion 모델 생성에 쓰지 않았다.

## 생성한 형상

| 항목 | 구현 |
|---|---|
| 부채꼴 | 각 다리 PLA 1솔리드. 0.5° 다각형, 림·두 립·웹·360° 무공 허브·7개 리브를 union |
| 타이어 | 각 다리 별도 1솔리드. 외형 포락, 실효 밀도 0.494 g/cm³ |
| 크랭크 | 90+110 mm, 꺾임 25°, 폭 34/높이 37/벽 2.8 mm. 미터 접합, 양 끝만 개방 |
| 요크 | 다리마다 독립 중공 박스 2개. 실제 체결부가 아닌 D0 포락 |
| 몸체 | 130×350 mm. 중공 외곽 빔, 횡 리브 3개, 종 리브 2개, 중앙 판 union; 별도 데크 |
| 탑재물 | 배터리, U2D2, SPDB, Jetson, LiDAR, 스테레오 포락. 카메라 광축 +x에서 30° 하향 |
| 스터디 | `_STUDY_D0_UNRESOLVED`에 포크 4세트, 배터리 슬롯과 마스트 공간. 기본 숨김, BOM 집계 제외 |
| 모터 | 12개 축 마커/관절만 생성. 도면과 변환을 검증하지 못했으므로 모터 형상은 MISSING |

PLA 밀도 1.18 g/cm³를 적용했다. 전장 및 스터디 포락의 Fusion 기본 재료 질량은 집계하지 않는다.
**Fusion 최상위 Properties의 전체 질량을 로봇 질량으로 사용하지 말고 `parts_manifest.json`의 명시 집계를 사용한다.**
PLA 재료의 일부 필드는 복사한 재료의 기본값이 남아 있다. 검증한 밀도 외 물성은 제작/FEA 합격 근거가 아니다.

## 확인 결과

- 변수 48개: 요구 키·단위·값 일치. 압출 폭과 립 오프셋 **17개 식**을 사용자 변수에 연결했다.
- 프로파일 수렴: 동일 각도에서 최대 차이 **{g01['max_support_difference_mm']:.6f} mm** (한계 0.05 mm).
- heave **{g01['heave_mm']:.6f} mm**, 팁 현 **{g01['tip_chord_mm']:.6f} mm**.
- 기준 자세: yaw=0°, q1=−64.176099°, q2=+64.176099°. 축거 464.954969 mm, 트랙 320 mm, 지면 z=−273 mm.
- 네 타이어 지면 높이 일치. 모든 native 관절을 +1°씩 구동해 해당 다리의 하위 링크까지 위치·회전 방향을 검사했고 **12/12 PASS**, 이후 기준 자세로 복귀했다.
- Fusion Drive Joint의 0°가 이 기준 자세다. 논리 q1/q2는 `joints.json`의 `q_reference_rad`를 더한다. 실제 모터 ID와 부호는 null이다.
- 관성 단위는 별도 임시 문서의 회전·이동된 직육면체로 검증했다. 현재 Fusion API의 XYZ 교차항은 부호 있는 텐서 항이며, 원점 기준 텐서에서 평행축 항을 빼고 kg·cm²→kg·m²로 변환한다.

**파라미터 범위:** 프로파일 좌표·크랭크 평면 형상·프레임 base feature·배치 변환은 원문에서 생성한 고정 형상이다. 48개 값을 바꾸면 전 형상이 자동 재설계되는 모델이 아니다. `cad/feature_bindings.json`에 있는 17개 식만 연동된다. 나머지 치수 변경은 새 원천/실행을 만들고 형상을 다시 생성·검증한다.

저장본 검증: `F3D_roundtrip.json`에서 48개 변수·12개 관절·42개 솔리드 인스턴스를 확인했고, `STEP_roundtrip.json`에서 표시 중인 형상 28개가 모두 유효한 솔리드로 다시 열렸다. 숨긴 스터디 14개는 네이티브 파일에만 포함된다.

## 우선 해결할 간섭

P00 기준 자세의 28개 솔리드/포락에 대해 실제 BRep 교차 체적을 검사했다. **24쌍 FAIL**, API 실패는 0건이다. 다리 간 서로의 부채꼴 교차가 아니라, 각 다리의 접합 주변에서 반복되는 문제다.

| 반복되는 쌍 | 다리당 교차 cm³ | 반복 수 |
|---|---:|---:|
| 프레임–요크 #1 | 1.114260 | 4 |
| 요크 #1–요크 #2 | 7.051000 | 4 |
| 요크 #1–링크 | 0.293116 | 4 |
| 요크 #2–링크 | 0.658913 | 4 |
| 링크–PLA 부채꼴 | 2.323546 | 4 |
| 링크–TPU 외형 | 1.175653 | 4 |

이 중 4쌍은 PLA 솔리드 간 교차이고, 나머지 20쌍은 요크 또는 TPU 외형 포락을 포함한다.
실제 모터가 없는 부분과 숨긴 포크 스터디까지 전체 무간섭으로 판정한 것이 아니다.
[쌍별 결과와 자세](evidence/G03_P00_intersections.json)에 이름·체적·분류가 있다. 임의로 접촉 허용 목록에 추가하지 않았다.
문서 §14.2에 따라 초기 관통이 있으므로 P01–P06 경로는 실행하지 않았다. 관절 방향 확인은 제한된 CAD 진단이며 허용 보행 경로가 아니다.

## 질량과 관성

- CAD 구조·타이어 부분합: **{parts['cad_structure_subtotal_kg']:.6f} kg**.
- 모터 데이터시트와 알려진 탑재물 질량을 더한 부분합: **{parts['known_mass_subtotal_kg']:.6f} kg**.
- 질량 미정 항목 {parts['missing_mass_count']}개, 관성 미정 항목 {parts['missing_inertia_count']}개. 포크·혼·베어링·체결·배선·전원·마운트 등이 미완성이므로 총질량 3.90 kg 통과 판정은 보류한다.
- PLA+TPU의 **스핀축 기준** 관성: **{inertia:.10f} kg·m²**. 후보 한계 0.0022보다 **{100*(inertia/.0022-1):.4f}% 초과**. 부품별 CoM 기준 Izz를 단순 합하면 이 초과를 놓친다.
- 무공 허브·무공 링크·타이어 외형은 그대로 보존했다. 임의의 경량화 구멍이나 밀도 조정으로 목표를 맞추지 않았다.

## 다음 실행 순서

1. **I02**: 기존/제조사 모터 CAD의 실제 회전축·장착면·혼·베어링·플러그 변환부터 확정한다. 기존 SCONE CAD에서 모양만 복사하지 않는다.
2. 새 후보에서 **링크 끝–포크–허브 단면**을 정의한다. 현재 링크가 부채꼴 중면을 관통하므로 폭/오프셋/연결 형상과 타이어 회전 여유를 함께 해결한다.
3. **요크 #1/#2와 프레임 장착 높이·연결 단면**을 확정한다. 표의 D0 포락을 합쳐 하나의 제작 부품으로 취급하지 않는다.
4. D1 허브 포켓·실제 타이어 셀·고정 방식(I03/I04)을 반영한 후 질량과 스핀축 관성을 다시 계산한다. D0 림을 근거 없이 얇게 바꾸지 않는다.
5. P00 교차를 해소한 후보에서 B09와 P01–P06을 수행한다. 경로 생성기에 G11의 beta/지지 모드/yaw 제약을 먼저 구현한다. 네 다리의 CAD 기본 스핀은 alpha=0이며, 대각 위상 후보는 `gaits/diagonal_phase_candidate.json`에만 분리했다.
6. 실제 부품 질량·CoM과 운전 한계가 모이면 B10–B13을 진행한다. MuJoCo/RL 자산은 아직 생성하지 않았다.

30번 문서의 §6.3은 내부 효율 중복 적용을 금지하지만 §16 참조 코드는 `design_factor/drive_efficiency`를 사용한다. `calculations.json`은 원문 재현 기록이며 구동 합격 근거가 아니다. 이 모순을 해결한 별도 하중 계산으로 G04를 판정해야 한다.

## 재현과 재개

`context.json`의 문서 ID와 원문/입력 해시를 확인한 뒤 실행한다. 기본 형상은 현재 문서에 이미 있으므로 B03/B04 생성 단계를 다시 호출하지 않는다. 이름만 같은 다른 문서를 수정하지 않는다.

실행 구현의 고정 복사본은 `evidence/tools_snapshot/`이며 dispatcher는 그 경로를 참조한다. 현재 편집용 구현은 `tools/marc/`에 있다. 단계별 실행과 저장본 재열기는 검증했으며, 최종 코드를 새 문서에서 처음부터 한 번에 재생성하는 회귀 실행은 별도로 하지 않았다. 각 파일 해시는 `source_manifest.json`에 기록했다.

```sh
python3 {root}/tools/marc/fusion_rpc.py script {f}/cad/dispatch_G02.py
python3 {root}/tools/marc/fusion_rpc.py script {f}/cad/dispatch_B05.py
python3 {root}/tools/marc/fusion_rpc.py script {f}/cad/dispatch_B09.py
```

- B03/B04는 부분 생성 실패에서 자동 삭제·중복 생성을 하지 않도록 중단한다. 이미 생성한 부품은 `cad/owned_components.json`으로 식별한다.
- B06 완료 상태는 중복 생성하지 않는다. Fusion의 custom line 축 입력이 무시되는 문제가 있어 **방향을 가진 평면·원으로 joint frame을 정의**했다. `rebuild_joint_frames.py`가 적용된 구현이다.
- `configure_joints.py` 및 `dispatch_joint_config.py`는 실패 원인 확인용 이전 시도다. **재실행하지 않는다.** 이 파일은 고정 실행 스냅샷에서 제외했다.
- 최초 클라우드 저장 시 임시 파일 ID와 creationId가 바뀌었다. 이후 guard는 확정된 클라우드 문서 ID와 소유 속성을 검사한다.
- `evidence/*initial*`, `*retry*`, `*response*`는 과정 기록이다. 최신 판정은 `checks.json`과 그 evidence 경로를 따른다.

전체 RELEASE: **미완료**. 현재 인도물은 초기 CAD 형상, 검증된 12축 기구학, P00 실패 증거와 후속 작업의 기준선이다.
'''
    (f/'handoff.md').write_text(text)
    old=root/'artifacts/scone_v3/20260909_092656_rev3-D0'
    if old.exists():(old/'SUPERSEDED.md').write_text('Input preflight only. No Fusion geometry created. Superseded by '+str(f)+' using live docs/30 rev4-D0.\n')
    return {'status':'PASS','handoff':str(f/'handoff.md'),'checks':{k:v['status'] for k,v in checks.items()},'spin_inertia_kgm2':inertia}

if __name__=='__main__':
    parser=argparse.ArgumentParser();parser.add_argument('--run-dir',required=True);args=parser.parse_args();ctx=json.loads((Path(args.run_dir)/'context.json').read_text());print(json.dumps(run(ctx),ensure_ascii=False,indent=2))
