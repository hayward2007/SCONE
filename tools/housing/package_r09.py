"""Package only the final, checked R09 artifacts; omit candidates and failed-attempt logs."""
from pathlib import Path
import json,shutil,zipfile,hashlib
BASE=Path(__file__).resolve().parents[2]/'artifacts/housing/20260915_MARC_v10/R09_DELIVERY';P=BASE/'PRINT_PACKAGE'
def read(n):return json.loads((BASE/n).read_text(encoding='utf-8'))
def run():
 checks=read('checks.json');access=read('screw_access.json');wires=read('wiring_clearance.json');yaw=read('yaw_path_screen.json');moving=read('moving_parts_path.json');assembly=read('assembly_screen.json');legs=read('all_legs_motion.json');manifest=read('PRINT_PACKAGE/print_manifest.json');step=read('PRINT_PACKAGE/step_validation.json');archive=read('PRINT_PACKAGE/archive_validation.json');identity=read('PRINT_PACKAGE/fusion_identity.json');protect=read('PRINT_PACKAGE/leg_preservation.json');source=read('PRINT_PACKAGE/preservation_check.json')
 assert all(not v for k,v in checks.items() if k.endswith('_hits'))
 assert all(not v for k,v in checks['camera_assembly'].items() if k.endswith('_hits'))
 assert checks['camera_assembly']['ffc_reserve_cover_overlap_mm3']<=.01
 assert access['failed']==0 and access['total']==118 and all(not r['hits'] for r in wires['routes'])
 assert not yaw['hits'] and not moving['hits'] and not assembly['mx_hits']
 assert not legs['four_legs_MX_inward90_hip180_continuous_distal_spin']['hits']
 assert protect['all_unchanged'] and source['independent_copy'] and source['source_current_state_preserved_during_export']
 assert read('assembled_placement_verification.json')['all_housing_at_verified_assembly_coordinates']
 assert len(manifest)==15 and sum(p['quantity'] for p in manifest)==23
 assert all(p['nonmanifold_edge_count']==0 and p['fits_256mm'] for p in manifest)
 assert all(p['valid'] and p['solids']==p['expected_solids'] for p in step) and archive['crc_ok']
 # Evidence must follow the last actual geometry change.
 final_change=(BASE/'flat_mounts_applied.json').stat().st_mtime
 for n in ['checks.json','screw_access.json','wiring_clearance.json','yaw_path_screen.json','moving_parts_path.json','assembly_screen.json','all_legs_motion.json','PRINT_PACKAGE/step_validation.json','PRINT_PACKAGE/print_manifest.json']:
  assert (BASE/n).stat().st_mtime>final_change,n
 old={o['name']:o for o in read('starting_state.json')['occurrences']};new={o['name']:o for o in read('final_inventory.json')['occurrences']};table=[]
 for name in ['R01 Rear smooth PLA chassis:1','R02 Front smooth PLA chassis:1','S01 Rear fairing lid:1','S02 Front camera fairing lid:1']:
  a=[x for x in old[name]['bodies'] if x['visible']][-1];b=[x for x in new[name]['bodies'] if x['visible']][-1]
  table.append(dict(part=name[:3],before_faces=a['faces'],after_faces=b['faces'],before_cm3=a['volume_cm3'],after_cm3=b['volume_cm3']))
 summary=dict(fusion=identity,changes=table,cad_validation='PASS',physical_validation='not performed',tool_access_checks=118,print_types=15,printed_pieces=23,minimum_user_pose_body_clearance_mm=min(v for n,v in yaw['minimum_clearances_at_exact_user_pose_mm']),known_inter_leg_conflicts=legs['exact_user_rear_pose_full_spin_vs_other_legs_in_saved_positions'],FR07_and_all_legs_unchanged=True)
 (P/'validation_summary.json').write_text(json.dumps(summary,ensure_ascii=False,indent=2),encoding='utf-8')
 before=sum(t['before_faces'] for t in table);after=sum(t['after_faces'] for t in table);vol0=sum(t['before_cm3'] for t in table);vol1=sum(t['after_cm3'] for t in table)
 rows='\n'.join(f"| {t['part']} | {t['before_faces']} → {t['after_faces']} | {t['before_cm3']:.2f} → {t['after_cm3']:.2f} |" for t in table)
 text=f'''# R09 검증 기록

작업 모델: **{identity['name']}**. CAD 기하·조립 검사는 통과했다. 실물 출력, 강도, 발열 및 케이블 내구성 검증은 수행하지 않았다.

## 단순화 결과

| 부품 | CAD 면 수 | 재료 체적 cm³ |
|---|---:|---:|
{rows}

네 하우징의 면 수는 {before} → {after}, 약 {(1-after/before)*100:.1f}% 줄었다. 체적은 {vol0:.2f} → {vol1:.2f} cm³로 약 {(1-vol1/vol0)*100:.1f}% 줄었다. 면 수는 형상 복잡도를 확인하는 보조 수치이며, 강도나 성능 점수가 아니다. 기능이 있는 구멍과 리브는 유지했다.

## 통과한 검사

- 사용자 지정 후방 다리 자세에서 하우징과 부품 간 체적 겹침 0건.
- 요청한 MX28 안쪽 90°/힙 XM430 180° 자세에서 20 mm 타이어의 연속 360° 회전 영역 검사: 겹침 0건. 네 다리를 해당 자세로 놓은 검사도 통과.
- 후방 MX28 이동 경로 0~92°, 1° 간격, 사용자 힙 각도 약 −0.14356°에서 휠 전체 회전 영역 및 나머지 이동 부품 검사: 겹침 0건. 타이어 영역에는 반경·축 방향 2.1 mm 여유를 포함.
- 사용자 지정 자세에서 실제 타이어 전체 회전 영역과 몸체·전자부품의 최소 거리 약 {summary['minimum_user_pose_body_clearance_mm']:.2f} mm. 이는 강체 CAD의 거리이며 변형·조립 공차를 입증하지 않는다.
- MX28 상부 삽입 0~80 mm, 2 mm 간격: 간섭 0건. 최종 안착 지점의 평면 받침 접촉 4건은 정상 접촉으로 구분했으며, 동일 형상의 Fusion 체적 검사에서 겹침이 없음을 확인했다.
- 카메라를 분리한 앞 덮개 안으로 넣는 상승 경로 2 mm 간격, 전방 슬라이드 1 mm 간격, Ø4 mm 드라이버 및 FFC 예약 공간: 겹침 0건.
- 드라이버 축·볼트 머리 공간 118건: 실패 0건. 작업 순서에 따라 덮개, 트레이, Jetson 또는 다리가 빠진 상태를 전제로 하며 드라이버 손잡이 전체를 모델링한 검사는 아니다.
- Ø3.2 mm 고정 TTL 배선 예약 통로: 겹침 0건.
- STL 15종 전부 닫힌 메쉬이며 256 mm 출력 범위 이내. STEP 각 부품은 유효한 단일 솔리드, 합본은 23개 솔리드. F3D 내부 압축 파일 CRC 검사 통과.

## 보존한 부품과 제한

R08을 기준으로 모든 LEG 구성품의 형상·위치를 비교했으며 FR07과 FR07 미러를 포함해 동일했다. 모터 축, 5 mm LINK 평판과 반원 휠 형상을 바꾸지 않았다. 원본 MARC v11은 읽기만 했고 저장·형상 변경·자세 복원을 수행하지 않았다. 원본에는 9월 15일의 과거 기록과 다른 조립 자세/타임라인 및 미저장 상태가 관측됐으므로 그 상태를 그대로 보존했다. 과거 기록과 부품 체적·면·에지 수는 같았다.

임의의 모든 다리 각도 조합을 보증하지 않는다. 사용자 후방 다리 자세에서 앞다리를 기본 자세로 두면, 후방 휠의 전체 회전 영역과 앞다리 휠 두 부품이 겹친다. 하우징과 별도로 다리 간 동작 조정이 필요하다.

Jetson 고정 탭의 실제 접촉 형상, 최종 나사/인서트, 구매 배터리·PCB·케이블은 아직 확정/실물 검토가 필요하다. PLA 강도·피로·열, TPU 변형 및 이동 배선의 피로는 이 결과에 포함되지 않는다.

자세 검사는 저장한 `user_pose_before.json`의 변환을 명시적으로 적용했다. Fusion이 저장 시 조인트 자세를 기본값으로 되돌리는 경우에 대비해 작업 파일에는 숨겨진 사용자 후방 자세 참고 구성요소도 보존되어 있다.
'''
 (BASE/'VALIDATION_KO.md').write_text(text,encoding='utf-8')
 for n in ['ASSEMBLY_KO.md','VALIDATION_KO.md','housing_detail.png','internals.png','assembly.png']:shutil.copy2(BASE/n,P/n)
 include=[p for p in P.iterdir() if p.suffix.lower() in ['.stl','.step','.f3d','.md','.png','.json']]
 hashes={p.name:hashlib.sha256(p.read_bytes()).hexdigest() for p in sorted(include)}
 (P/'SHA256.json').write_text(json.dumps(hashes,indent=2),encoding='utf-8');include.append(P/'SHA256.json')
 zip_path=BASE/'MARC_Housing_PLA_R09.zip'
 with zipfile.ZipFile(zip_path,'w',zipfile.ZIP_DEFLATED,compresslevel=6) as z:
  for p in sorted(include):z.write(p,'MARC_Housing_PLA_R09/'+p.name)
 with zipfile.ZipFile(zip_path) as z:assert z.testzip() is None
 print(json.dumps(summary,ensure_ascii=False,indent=2));print(zip_path)
if __name__=='__main__':run()
