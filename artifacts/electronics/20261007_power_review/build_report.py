from pathlib import Path
from xml.sax.saxutils import escape
import json
from reportlab.pdfgen import canvas
from reportlab.platypus import SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, PageBreak, Flowable
from reportlab.lib import colors
from reportlab.lib.styles import ParagraphStyle
from reportlab.lib.enums import TA_LEFT
from reportlab.pdfbase import pdfmetrics
from reportlab.pdfbase.ttfonts import TTFont
from reportlab.lib.pagesizes import A4

ROOT = Path('/Users/hayward_kim/Developer/SCONE')
OUT = ROOT / 'output/pdf/MARC_v4_battery_P1_review_20261007.pdf'
OUT.parent.mkdir(parents=True, exist_ok=True)
pdfmetrics.registerFont(TTFont('Korean', '/System/Library/Fonts/Supplemental/Arial Unicode.ttf'))
INK = colors.HexColor('#172B36')
TEAL = colors.HexColor('#087B7B')
MUTED = colors.HexColor('#526772')
PALE = colors.HexColor('#EFF6F6')
LINE = colors.HexColor('#D7E2E6')
WARN = colors.HexColor('#8F4D13')
W, H = A4
WIDTH = W - 84
styles = {
    'title': ParagraphStyle('title', fontName='Korean', fontSize=23, leading=31, textColor=INK, spaceAfter=12, wordWrap='CJK'),
    'deck': ParagraphStyle('deck', fontName='Korean', fontSize=11, leading=17, textColor=MUTED, spaceAfter=13, wordWrap='CJK'),
    'h': ParagraphStyle('h', fontName='Korean', fontSize=14, leading=21, textColor=TEAL, spaceBefore=11, spaceAfter=7, wordWrap='CJK'),
    'body': ParagraphStyle('body', fontName='Korean', fontSize=10, leading=15.2, textColor=INK, spaceAfter=7, wordWrap='CJK'),
    'small': ParagraphStyle('small', fontName='Korean', fontSize=8.3, leading=12.1, textColor=MUTED, spaceAfter=6, wordWrap='CJK'),
    'cell': ParagraphStyle('cell', fontName='Korean', fontSize=8.8, leading=12.6, textColor=INK, wordWrap='CJK'),
    'tiny': ParagraphStyle('tiny', fontName='Korean', fontSize=7.7, leading=10.6, textColor=INK, wordWrap='CJK'),
    'th': ParagraphStyle('th', fontName='Korean', fontSize=8.8, leading=12.6, textColor=colors.white, wordWrap='CJK'),
}
story = []

def p(text, kind='body'):
    return Paragraph(text, styles[kind])

def add(text, kind='body'):
    story.append(p(text, kind))

def heading(text):
    add(text, 'h')

def link(label, url):
    return f'<a href="{escape(url, {chr(34): "&quot;"})}" color="#087B7B"><u>{escape(label)}</u></a>'

def table(headers, rows, widths, tiny=False):
    data = [[p(escape(str(h)), 'th') for h in headers]]
    data += [[p(str(c), 'tiny' if tiny else 'cell') for c in row] for row in rows]
    t = Table(data, colWidths=widths, repeatRows=1, hAlign='LEFT')
    t.setStyle(TableStyle([
        ('BACKGROUND', (0,0), (-1,0), TEAL),
        ('ROWBACKGROUNDS', (0,1), (-1,-1), [colors.white, PALE]),
        ('VALIGN', (0,0), (-1,-1), 'TOP'),
        ('LEFTPADDING', (0,0), (-1,-1), 7),
        ('RIGHTPADDING', (0,0), (-1,-1), 7),
        ('TOPPADDING', (0,0), (-1,-1), 6 if not tiny else 3),
        ('BOTTOMPADDING', (0,0), (-1,-1), 6 if not tiny else 3),
        ('LINEBELOW', (0,0), (-1,0), .5, TEAL),
        ('LINEBELOW', (0,1), (-1,-1), .3, LINE),
    ]))
    story.append(t)
    story.append(Spacer(1, 8))

def box(text):
    t = Table([[p(text)]], colWidths=[WIDTH])
    t.setStyle(TableStyle([('BACKGROUND',(0,0),(-1,-1),PALE),('BOX',(0,0),(-1,-1),.8,TEAL),
                          ('LEFTPADDING',(0,0),(-1,-1),11),('RIGHTPADDING',(0,0),(-1,-1),11),
                          ('TOPPADDING',(0,0),(-1,-1),9),('BOTTOMPADDING',(0,0),(-1,-1),3)]))
    story.append(t)
    story.append(Spacer(1, 7))

def page(title, deck):
    if story: story.append(PageBreak())
    add(title, 'title')
    add(deck, 'deck')

class BaySection(Flowable):
    def __init__(self):
        super().__init__()
        self.width = WIDTH
        self.height = 185
    def draw(self):
        c = self.canv
        c.setFont('Korean', 8.5)
        c.setFillColor(MUTED)
        c.drawString(0, 173, '폭-높이 단면 개념도  |  길이 방향과 전선은 생략, 동일 축척')
        scale = 1.65
        for x0, name, bw, bh, fill in [(90,'현재 CNHL 16000',45,77,'#D8E1E5'),(330,'권장 Neonergy 10000',42,49,'#9BDBD0')]:
            y0 = 20
            left = x0 - 23*scale
            c.setStrokeColor(MUTED)
            c.setDash(3,2)
            c.rect(left,y0,46*scale,77.5*scale,stroke=1,fill=0)
            c.setDash()
            c.setFillColor(colors.HexColor(fill))
            c.setStrokeColor(TEAL)
            c.rect(x0-bw*scale/2,y0,bw*scale,bh*scale,stroke=1,fill=1)
            c.setFillColor(INK)
            c.drawCentredString(x0, y0+bh*scale/2, f'{bw} x {bh} mm')
            c.drawCentredString(x0, 3, name)
            if bh == 49:
                c.setFillColor(WARN)
                c.drawCentredString(x0, y0+bh*scale+18, '상부 빈틈 28.5 mm')
                c.drawCentredString(x0, y0+bh*scale+5, '누름대 접촉부 변경')
        c.setFillColor(MUTED)
        c.drawString(0, 155, '점선: 기존 브라켓 내부 46 x 77.5 mm')

# 1
page('배터리와 P1 전장 검토', 'MARC v4  |  2026-10-07  |  Fusion 실측 + 국내 구매처 + 첨부 rev A 회로 검토')
box('현재 제작을 이어갈 기본 배터리는 <b>Neonergy 4S 14.8 V 10000 mAh, XT90 옵션</b>을 권장한다. 국내 재고와 네이버 가격비교 연결을 확인했고, 가벼우며 현재 몸체에 들어간다. 배터리를 바꾸면 상부 누름대는 수정해야 한다. 장시간 운용이 우선이면 16000 mAh가 유리하지만 국내 구매 가능 재고는 확인하지 못했다.')
heading('현재 Fusion에서 확인한 공간')
table(['항목','실측 / 의미'],[
    ['검사한 문서','MARC v4 Body v5 BOX <b>v63</b>. 사용자 설명의 v61보다 최신. 검사 전후 modified=False.'],
    ['배터리 칸의 기하학적 최대','<b>길이 186 x 폭 46 x 높이 77.5 mm</b>. 브라켓 벽과 누름대 안쪽 기준.'],
    ['현재 배터리 모델','185 x 45 x 77 mm, 1270 g. 길이 방향을 유지하고 세워 넣은 상태.'],
    ['치수 여유','현재 모델 대비 전체 길이 1 mm, 폭 1 mm, 높이 0.5 mm. 팩 공차·보호 패드까지 고려하면 매우 촘촘하다.'],
    ['구매 치수의 권장 여유','기존 브라켓 그대로라면 대략 L ≤ 182, W ≤ 43~44, H ≤ 74~75 mm를 선호. 실물·선 출구 확인 후 확정.'],
    ['P1 현재 자리','160 x 90 x 1.6 mm, 보드 밑면 z=92.0 mm. 부품 자리 45 mm는 실제 확정 높이가 아닌 placeholder.'],
], [122, WIDTH-122])
story.append(BaySection())
add('치수 순서는 로봇 장착 방향의 길이 x 폭 x 높이이다. 제품 판매 페이지의 L/W/H를 그대로 비교하면 폭과 높이를 혼동할 수 있다. 도면 측정 및 후보 박스의 정적 간섭 계산만 수행했고 Fusion 형상·색상·재료는 수정하지 않았다.', 'small')

# 2
page('구매 후보와 운용 시간', '4S 일반 LiPo: 공칭 14.8 V, 완충 16.8 V. 6S·LiHV·LiFePO4는 현재 설정으로 대체하지 않는다.')
table(['후보 / 구매 근거','장착 L x W x H','무게 / 가격','판단'],[
    ['<b>1. Neonergy 10000 4S</b><br/>'+link('팰콘샵 BU-134360 [S1]','https://www.falconshop.co.kr/shop/goods/goods_view.php?goodsno=100092305'), '175 x 42 x 49 mm<br/>판매 치수 175 x 49 x 42를 회전', '748 g<br/>119,000원<br/>재고 표시 10개', '<b>우선 추천</b><br/>XT90 옵션 선택 가능. 현 브라켓 내 정적 간섭 없음. 상부 누름대 수정.'],
    ['2. EP POWER 10000 4S<br/>'+link('팰콘샵 [S2]','https://www.falconshop.co.kr/shop/goods/goods_view.php?category=019003039&goodsno=100087294'), '160 x 44 x 46 mm<br/>판매 치수 160 x 46 x 44를 회전', '780 g<br/>129,000원<br/>재고 표시 1개', '길이 여유 큼.<br/>EC5 연결이므로 XT90 하네스 또는 판매자 잭 변경 필요.'],
    ['3. CNHL Racing HC 10000 4S<br/>'+link('팰콘샵 [S3]','https://www.falconshop.co.kr/shop/goods/goods_view.php?goodsno=100092386'), '177 x 45 x 49 mm<br/>판매 치수 177 x 49 x 45를 회전', '825 g<br/>139,000원<br/>재고 ∞ 표시', 'XT90 선택 가능.<br/>폭 여유가 1 mm뿐. ∞는 실제 수량이 아니므로 출고 확인 필요.'],
    ['4. CNHL 16000 4S 15C<br/>'+link('CNHL 공식 [S4]','https://chinahobbyline.com/products/cnhl-16000mah-14-8v-4s-15c-lipo-battery-with-xt90-plug'), '185 x 45 x 77 mm<br/>현재 CAD 모델과 일치', '1270 g<br/>236.8 Wh<br/>국내 재고 미확인', '최장 운용 후보.<br/>공식 EU 재고는 한국 배송 확보를 뜻하지 않는다. 공차 때문에 실물 확인 필수.'],
    ['5. Lumenier 16000 4S<br/>'+link('GetFPV [S5]','https://www.getfpv.com/batteries/commercial-consumer-batteries/lumenier-16000mah-4s-20c-lipo-battery.html'), '182 x 40 x 74 mm', '1190 g<br/>236.8 Wh<br/>국내 재고 미확인', '기구 치수 여유 있음.<br/>판매 제목 XT90 / 상세 EC5 표기가 상충. 잭·한국 배송 미확정.'],
], [144, 122, 92, WIDTH-358], tiny=True)
heading('네이버에서 실제 확인한 내용')
add('네이버 쇼핑 전용 검색은 접속 제한이 있었지만, <b>네이버 통합검색의 가격비교 카드</b>에서 Neonergy 4S 10000을 찾고 연결 판매처까지 열었다. 팰콘샵 카드 116,620원, 판매처 상세 기본가 119,000원, 재고 10개, XT90 선택 및 Naver Pay 버튼을 확인했다. 할인 조건과 최종 결제가는 미확정이며 주문은 하지 않았다. [S1, S6]')
heading('예상 사용 시간: 실측 전 계산')
table(['배터리','에너지 / 80% 사용 가정','100 W 평균','130 W 평균','160 W 평균'],[
    ['10000 mAh 4S','148 / 118.4 Wh','71분','55분','44분'],
    ['16000 mAh 4S','236.8 / 189.4 Wh','114분','87분','71분'],
], [103, 165, 81, 81, WIDTH-430])
add('평균 전력은 변환 손실을 포함한 배터리 측 전체 소비전력 가정이다. 보행·팔·Jetson 실측값이 없어 보장 시간이 아니다. 130 W에서 90분에 가까운 운용이 필요하면 16000 mAh를 우선 확보할 이유가 있다. 10000으로 바꾸면 단순 질량 치환 예상은 5161.2 g로, 현재 5683.2 g보다 522 g 가볍다. 브라켓·기판·하네스 변경 무게는 제외했다.', 'small')

# 3
page('브라켓과 PCB 공간 변경안', '몸체와 서보 형상을 유지하는 범위의 제안. 변경 형상 제작이나 CAD 저장은 수행하지 않았다.')
heading('Neonergy 10000을 쓰면 필요한 변경')
table(['부분','권장 조치'],[
    ['배터리 방향','길이 175 mm를 x축에 놓고 42 mm를 폭, 49 mm를 높이로 둔다. 선 출구는 앞쪽 연결 공간에서 실물 확인.'],
    ['B3 바닥 받침 / 가이드','바닥 z=5.8 mm를 유지한다. 46 mm 내부 폭에 42 mm 팩이므로 얇은 측면 보호 패드 약 1~1.5 mm씩을 검토한다.'],
    ['B4 상부 누름대','팩 윗면 z≈54.8, 기존 누름대 밑면 z=83.3. 약 28.5 mm 빈틈이 생긴다. <b>중앙 접촉부를 약 27~28 mm 내리거나 조절식 스트랩·클램프로 교체</b>하고 얇은 완충 패드를 둔다. 실물 측정 후 확정.'],
    ['앞뒤 고정','전체 길이 여유 11 mm. 조절식 끝단 스토퍼로 흔들림을 막되 삽입 여유와 선 굽힘 공간을 남긴다. 배터리에 나사를 직접 누르지 않는다.'],
    ['P2 기판 지지','최소 변경안은 외곽 지지 높이와 P1 z=92를 유지하고 B4 중앙만 내려가는 구조. 큰 빈틈을 통짜 블록으로 채우거나 배터리를 높이 올릴 필요 없다.'],
], [118, WIDTH-118])
heading('P1을 얼마나 줄일 수 있는가')
add('우선 목표는 <b>140 x 90 mm</b>이다. 현재 구멍 간격 120 x 80 mm와 Ø8 keepout을 유지하면서 앞쪽 20 mm를 줄이는 방향으로 12.5% 면적을 줄일 수 있다. 기존 보드 좌표 구멍은 (9,5), (9,85), (129,5), (129,85), Ø3.2 mm이다. 120 x 80 mm급은 약 33% 감소하지만 지지점 변경과 실제 배치·열 검증이 필요해 현재 확정 가능한 크기는 아니다.')
add('부품 높이 45 mm의 일괄 예약은 과하다. 약 11 mm 인덕터, 퓨즈·XT90·다른 커넥터 실물 높이를 반영해 <b>윗면 부품 envelope 18~22 mm</b>를 첫 배치 목표로 둔다. 배터리가 작아졌다면 기판 밑면을 z≈65~70 mm로 내리는 대안도 있지만, 이 경우 지지대·하네스·팬 자리 재설계가 필요하다. 높이 목표는 아직 실제 BOM 3D 모델로 검증하지 않았다.')
heading('더 큰 팩을 위해 PCB를 줄이는 경우')
add('계산상 B3/B4·P1/P2를 제외해도 길이 186 mm의 중앙 박스는 <b>폭 약 47 mm에서 BODY REAR에 막힌다</b>. 90 mm 폭 팩은 몸체와 뒤쪽 hip yaw 서보에도 간섭한다. 따라서 PCB 평면 축소만으로 넓은 배터리를 넣을 수는 없다.')
add('높이는 확장 여지가 있다. 다른 몸체 형상을 유지하는 탐색 박스 186 x 46 x 100~110 mm는 정적 간섭이 없었지만 기판·클램프를 제외한 계산이다. 뚜껑 밑면 z=141.5와 상부 간격을 고려하면 <b>L ≤ 182, W ≤ 44, H 약 100 mm</b>를 검토 목표로 삼는 것이 타당하다. 부품 높이 22 mm라면 P1 밑면 z≈115 mm, 18 mm라면 z≈118 mm를 검토해 뚜껑까지 약 3 mm 이상을 남긴다. 팩-기판 간격은 약 9 mm 이상이다. 누름대 재설계가 필요하며 구매 규격으로 확정한 것은 아니다.')
add('정적 간섭 검사는 임시 BRep 박스로만 수행했다. 팩 실물 형상, 제조 공차, 팽창 여유, 전선·플러그, 탈착 경로, 동적 운동은 포함하지 않았다. 해당 부품을 모델에서 실제 삭제하거나 이동하지 않았다.', 'small')

# 4
page('P1에 들어갈 기능과 핵심 부품', '첨부 rev A 0~12절 전체를 기능 단위로 통합. 부품 번호는 설계 출발점이며 제조 승인 BOM은 아니다.')
box('배터리 → 40 A 인라인 퓨즈 → 고측 BMS → 배터리 이상적 다이오드 → VSYS<br/>외부 입력 → 역극성·과전압 보호 → 외부 이상적 다이오드 → VSYS<br/>외부 입력의 보호 후 전원 → 4S 충전기 → BMS 보호를 거쳐 배터리 충전<br/>VSYS → 12 V 서보 / 5 V S1·주변기기 / 독립 Jetson 보호 전원<br/>배터리·외부 입력 → 상시 3.3 V → MCU·보호 제어·계측')
table(['블록','핵심 구성 / 설계 목표'],[
    ['셀 보호·균형','BQ76952PFBR x1; 고측 충전 FET x2 + 방전 FET x2; 1 mΩ shunt; 4S 셀 탭 필터; PDSG P-FET + 47 Ω 프리차지; 배터리 NTC와 FET NTC. BQ76942 대체는 핀·셀 배선·설정 재확인.'],
    ['전원 합류·외부 보호','LM74700-Q1 x2 + 배터리 병렬 FET x2 / 외부 FET x1. LM74800-Q1 x1 + 역극성·OVP FET x2. TVS, 입력 커패시터, 외부 존재·전압 측정. [S7]'],
    ['4S 충전','BQ25798 x1, 2.2 µH 인덕터, 외부 NTC x1. 16.8 V, 최대 3.5 A 충전 목표. 입력·충전 전류를 시스템 소비와 함께 제한. [S10]'],
    ['서보 12 V','LM5143-Q1 계열 x1, 2상 FET x4, 6.8 µH 인덕터 x2, 상별 7 mΩ shunt x2 병렬, 입력/출력 커패시터. <b>12.08 V, 15 A 연속 / 25 A 100 ms</b>는 검증할 목표. <b>회생 제동·과전압 처리 추가.</b> [S9]'],
    ['5 V / 상시 3.3 V','TPS56A37 x1 + 3.3 µH: 5.10 V, 설계 6 A. LM5165Y x1 + 150 µH: 3.3 V 상시 전원. 각각 디커플링·보상·PG/EN 회로.'],
    ['출력 보호','TPS259474L x4: Jetson, VSYS-AUX, 12V-AUX, S1 5 V. 각 OVLO·EN·R_ILM·DVDT·ITIMER·PGTH 설정 및 PG 처리. [S8]'],
    ['전류·전압·온도 측정','INA238 x3 + 1/2/5 mΩ: SERVO-A/B·Jetson. BMS 팩 전류 추가. ADC 분압 5개 + 인덕터 NTC. 배터리 NTC 2개와 보드 NTC 2개가 필요.'],
    ['제어·비상 정지','STM32G0B1CEU6 x1, AND 논리, 하드웨어 비상 정지 회로, SWD·USB-C·ESD·I2C 풀업·UART·상태 LED. <b>단선 시 정지하는 비상 정지 배선으로 변경.</b> [S12]'],
    ['기구·배선·정비','J1~J26 해당 커넥터, 분기 퓨즈·PTC, 시험점, 탈착 가능한 배터리/NTC 선, 팬, 뒤판 LED5·모드 스위치·버튼. 상세 핀은 7~8쪽.'],
], [115, WIDTH-115], tiny=True)
add('LCSC 식별자 핵심: U1 C2862742; U2/3 C2941042; U4 C3215600; U5 C2876593; U6 C1849541; U7 C22392669; U8 C2864845 x4; U9 C2868250 x3; U11 C2071924; U12 C2829190. 재고 수치는 첨부 시점 값이며 이번 조사에서 LCSC 전 품목을 재조회하지 않았다.', 'small')

# 5
page('회로를 그리기 전에 수정할 3가지', '첨부 문서의 핵심 구조는 유용하지만, 그대로 제작 승인하기에는 아래 문제가 남아 있다.')
heading('1. 서보 회생 에너지는 배터리로 자동 흡수되지 않는다')
add('LM5143 다이오드 에뮬레이션은 변환기가 능동적으로 음의 인덕터 전류를 만드는 것을 억제한다. 서보가 출력 전압을 밀어 올리면 상측 MOSFET 몸 다이오드 경로로 VSYS까지 올라갈 수 있다. 그러나 <b>배터리 쪽과 외부 쪽 LM74700은 둘 다 역전류를 막는다</b>. 첨부의 “배터리면 흡수” 설명은 이 연결에서는 성립하지 않는다. [S7, S9]')
box('필요한 추가 회로: 12V_S 하드웨어 과전압 감지 → 제동 MOSFET + 에너지 소모 저항. VSYS로 전달되는 과도도 함께 검토한다. 저항과 FET는 실제 서보 감속 때 측정한 회생 전력·에너지로 선정하고 배터리에서 열적으로 분리한다. MCU 경고와 TVS는 보조 보호로 사용한다.')
add('470 µF x2만 놓고 12→14 V에서 저장되는 추가 에너지는 0.5 x 940 µF x (14²−12²) = <b>0.024 J</b>에 불과하다. 원문에는 다른 출력 커패시터도 있으나 용량 추가만으로 회생 처리 완료라고 판단할 수 없다. TVS의 반복 펄스 정격·클램프 최대값도 각 부품 한계와 대조해야 한다.', 'small')
heading('2. Jetson OVP 20.06 V는 20 V 운용 상한을 지키지 못한다')
add('NVIDIA 개발키트 캐리어의 DC 입력 운용 범위는 <b>9~20 V</b>이다. 첨부의 eFuse 분압 165 kΩ/10.5 kΩ는 전형값부터 20.06 V다. TPS25947 OVLO 문턱 최대 1.223 V와 저항 각 1% 오차를 적용하면 1.223 x (1 + 166.65/10.395) = <b>20.83 V</b>까지 차단이 늦어질 수 있다. [S8, S11]')
add('어댑터 최대 출력·저항·문턱·과도 오차를 모두 포함해서 Jetson 입력이 20 V를 넘지 않게 설계한다. 정확한 과전압 감지기를 쓰거나, 서보 E-stop과 별개로 유지되는 <b>Jetson 전용 12~15 V 변환 전원</b>을 검토한다. 기존 서보 12 V에 단순 연결하면 비상 정지 때 Jetson도 꺼지므로 독립성이 필요하다.')
heading('3. 19 V 90~120 W는 서보 최대 부하 운전용으로 부족하다')
add('12 V x 15 A = <b>180 W</b>이므로 서보 출력만으로 120 W를 넘는다. 5 V x 6 A = 30 W, Jetson·주변기기 약 25 W라는 예시와 변환 손실을 더하면 연속 입력은 약 250 W가 된다. 여기에 16.8 V x 3.5 A 충전 약 59 W를 더하면 약 310 W다. 실제 부하가 달라지면 다시 산정해야 한다.')
add('<b>90~120 W 어댑터는 책상 개발·충전·부하 제한 모드</b>로 둔다. 최대 연속 운전을 요구하면 250~300 W급, 운전과 충전을 함께 요구하면 300~350 W급을 검토하고 펌웨어는 충전 전류를 먼저 줄여 입력 한계를 지킨다. 19 V 300 W는 약 15.8 A이므로 흔한 5.5/2.5 배럴 잭을 정격 확인 없이 쓸 수 없다. 뒤판 커넥터·선·퓨즈·입력 FET 열 설계를 함께 바꿔야 한다.')
add('BQ25798의 3.3 A 입력 제한은 충전기로 들어가는 가지에만 작용한다. 보드 전체 어댑터 전류 제한이 아니다. 이상적 다이오드 합류만으로 부족한 어댑터와 배터리가 안정적으로 전류를 분담한다고 가정하지 않는다.', 'small')

# 6
page('보호와 제작 조건의 보완', '회생·Jetson·어댑터 문제와 함께 해결해야 하는 구현 조건')
heading('비상 정지: 단선과 재시작까지 포함')
add('원문의 ESTOP_N 풀업은 선이 끊어지면 높은 값으로 남아 서보 허용 상태가 될 수 있다. <b>상시 닫힘 접점/루프를 사용해 단선도 정지로 읽히게</b> 하고 MCU와 별개로 서보 전력단을 차단한다. 버튼 해제만으로 재시작하지 않고 유효한 루프·고장 해제·명시적 servo_on 명령을 모두 요구한다.')
add('U6 EN을 내리는 시간과 실제 12V_S 전압이 떨어지는 시간은 다르다. 출력 커패시터와 서보 운동 에너지가 남으므로 원문의 “1 ms 안 꺼짐”은 아직 보장할 수 없다. 필요한 차단 형태에 따라 출력 차단 FET·방전 경로를 추가하고 전압과 토크 제거 시간을 측정한다. 팬이 12V_S에만 연결되면 E-stop 중에도 충전부가 뜨거울 수 있으므로 독립 전원에서 팬을 구동하는 대안을 검토한다.')
heading('PMP31202는 검증된 회로의 출발점')
add('TI 원본은 <b>LM5143A-Q1, 4상, 20~28 V 입력, 13.8 V 출력의 700 W급</b>이다. 첨부 BOM은 LM5143-Q1, 2상, 12~19 V 입력, 12.08 V 출력으로 조건이 다르다. 따라서 “절반 그대로”로 98% 효율이나 25 A 펄스 성능을 인계받을 수 없다. 컨트롤러 정확한 suffix·패키지, MOSFET VGS/Qg, 게이트 구동 손실, 인덕터 포화, 보상, 출력 커패시터, 최저 입력 듀티, PCB 열을 재계산한다. [S9]')
heading('eFuse 동작과 필수 핀')
add('TPS259474L은 <b>과전류 차단형(circuit breaker)과 latch-off</b> 옵션이다. 설정 5 A를 계속 유지해 주는 전류원으로 보면 안 된다. ITIMER·PGTH·PG·DVDT까지 확정하고 부하의 기동/과도 전류가 차단 지연과 맞는지 확인한다. 28 mΩ 기준 5 A 통과 손실은 약 <b>0.70 W</b>이며 온도 상승 시 더 커진다. 원문의 0.4 W 봉우리 값은 5 A 조건과 일치하지 않는다. [S8]')
heading('배터리 보호가 완성되는 조건')
add('일반 RC LiPo는 팩 내부 BMS를 전제로 하지 않는다. BQ76952의 기본 보호만 믿지 말고 COV/CUV/OCC/OCD/SCD·온도 보호·셀 매핑을 부팅마다 구성하고 읽어 검증한다. 공통 GND 구조와 BAT_N 켈빈 섬을 유지한다. 상시 3.3 V가 BAT_P에서 직접 나와 BMS 방전 차단을 우회하므로 장기 보관 때 팩 분리 또는 독립 저전압 차단이 필요하다.')
heading('제조 전에 남은 값')
add('첨부 12절의 LD/PACK 저항·PDSG 연결·셀 필터·REG1/2 미사용 처리, 충전기 D+/D−, U6 FET와 보상, eFuse 핀, 커패시터·커넥터 정격을 EVM/데이터시트와 대조한다. BQ76942와 TPS56637 대체는 설정·핀 기능·전류 여유까지 검토 후 승인한다. 부품 재고만으로 대체하지 않는다. [S8, S10, S13]')
box('판정: 이 문서는 회로 설계를 진행할 요구사항과 부품 후보로 사용할 수 있다. 아직 회로도/ERC, 실제 배치/DRC, 전체 BOM, 열·회생·전환 시험을 통과한 제조용 설계는 아니다.')

# 7
page('커넥터와 외부 하네스', '첨부 5절의 연결을 정리. 전력 잭과 비상 정지 배선은 앞쪽 보완 사항을 적용한다.')
table(['포트','보드 / 상대','핀 또는 용도'],[
    ['J1 BAT','XT90PW-M 후보 / 팩 XT90','+ / −. 실물 암수 확인. 팩 가까이에 40 A 인라인 퓨즈, 12 AWG 기준 시작.'],
    ['J2 BAL','JST XH 5P','1 C0(−), 2 C1, 3 C2, 4 C3, 5 C4(+). 멀티미터로 셀 순서 확인.'],
    ['J3 / J4 NTC','JST PH 2P 각각','BMS 배터리 NTC / 충전기 배터리 NTC. 두 개 독립 센서.'],
    ['J5 EXT IN','XT30PW-M / 뒤판 입력','중앙+ 입력 계획. 90~120 W 모드와 250 W 이상 모드의 잭·선 정격을 구분.'],
    ['J6 SERVO-A','5.08 mm 나사 2P → PHB','1 +12V, 2 GND. F1 15 A와 Rs2 후단. 16 AWG 약 25 cm. PHB 다른 전원 입력 동시 연결 금지.'],
    ['J7 SERVO-B','JST EH / Molex 3P 후보','팔 TTL DATA·GND 통과, VDD만 12V_B 주입. F2 7.5 A. 실제 서보 커넥터 핀 순서 대조.'],
    ['J8 JETSON','XT30PW-F → DC 5.5/2.5','가운데 +, 18 AWG 약 20 cm. 독립 Jetson 전원 및 수정 OVP를 적용.'],
    ['J9 / J10 AUX','XT30PW-F 각각','12V-AUX 3 A / VSYS-AUX 2 A. 대상 장치 허용 전압 확인.'],
    ['J11 / J12 AUX','JST PH 2P 각각','5V-AUX 1 A / LiDAR AUX 1.5 A. LiDAR 어댑터 보조 입력이 있을 때만 사용.'],
    ['J20 S1 전원','JST XH 4P','1 5V, 2 5V, 3 GND, 4 GND. eFuse 4 A 및 접점 전류 확인.'],
    ['J21 S1 신호','JST GH 6P','1 GND; 2 P1_TX→S1; 3 P1_RX←S1; 4 ESTOP_N; 5 PWR_EVT_N; 6 S1_3V3(alive). 단선 검출 구조 반영.'],
    ['J22 PANEL','JST GH 8P','1~5 LED1~5; 6 SW1; 7 BTN; 8 GND. LED 직렬 저항 포함.'],
    ['J25 FAN','4핀 팬 헤더','1 GND, 2 팬 전원, 3 TACH, 4 PWM. 충전 중 E-stop에도 냉각 가능하도록 검토.'],
    ['J26 점검 USB','USB-C USB 2.0 + ESD','D−/D+, CC1/CC2 각 5.1 kΩ. VBUS가 MCU/다른 USB 전원으로 역급전하지 않는 구조.'],
    ['SWD / 시험점','TC2030 / 다수 패드','3V3, SWDIO, SWCLK, NRST, GND. BAT_P/N, PACK_P, EXT_P, VSYS, 12 V, 5 V, EN, I2C, UART 등.'],
], [77, 150, WIDTH-227], tiny=True)
heading('보드 밖에서도 필요한 것')
add('배터리-퓨즈-XT90 선, 밸런스 연장선, 배터리 NTC 두 개, 외부 입력선, 서보 A/B 선, Jetson 선, S1 전원/신호 두 묶음, 팬, 뒤판 버튼·모드·LED, 잠금식 비상 정지 버튼. S1 USB 네 개의 상대 장치·전원 공급/역급전 구조는 원본 v2가 없어 아직 확정하지 못했다.')
add('AWG는 원문의 시작 규격이다. 실제 연속 전류·길이·묶음·온도·압착 접점·퓨즈 차단능력을 기준으로 재선정한다. 커넥터 암수 표기는 판매자 명칭보다 노출된 금속 접점과 실물 mating을 기준으로 확인한다.', 'small')

# 8
page('MCU 배정과 통신 요구', '첨부 4.11절의 48핀을 빠짐없이 재정리. 회로도 작성 때 UFQFPN-48 일반판과 AF를 최종 대조한다. [S12]')
pins = [
    ('1','PC13','버튼 / EXTI13'),('2','PC14','모드 스위치'),('3','PC15','외부 전원 감지'),
    ('4','VBAT','VDD에 연결'),('5','VREF+','VDDA / 디커플링'),('6','VDD/VDDA','3V3_AO'),('7','VSS/VSSA','GND / 노출 패드'),
    ('8','PF0','HSE 예비'),('9','PF1','HSE 예비'),('10','PF2-NRST','리셋 / SWD'),
    ('11','PA0','VSYS ADC'),('12','PA1','EXT ADC'),('13','PA2','12 V ADC'),('14','PA3','5 V ADC'),
    ('15','PA4','인덕터 NTC ADC'),('16','PA5','PACK_P ADC'),('17','PA6','LED1 PWM'),('18','PA7','LED2 PWM'),
    ('19','PB0','S1_ALIVE'),('20','PB1','충전기 CE'),('21','PB2','BMS DFETOFF'),('22','PB10','BMS ALERT'),
    ('23','PB11','INA ALERT 묶음'),('24','PB12','충전기 INT'),('25','PB13','12 V PG'),('26','PB14','팬 PWM 25 kHz'),
    ('27','PB15','5 V PG'),('28','PA8','SERVO_EN'),('29','PA9','LED3 PWM'),('30','PC6','JETSON_EN'),
    ('31','PC7','AUX12_EN'),('32','PA10','LED4 PWM'),('33','PA11','USB D−'),('34','PA12','USB D+'),
    ('35','PA13','SWDIO'),('36','PA14-BOOT0','SWCLK / 부트 옵션'),('37','PA15','팬 TACH'),('38','PD0','상태 LED'),
    ('39','PD1','LED5 PWM'),('40','PD2','VSYSAUX_EN'),('41','PD3','5V_EN'),('42','PB3','BMS CFETOFF'),
    ('43','PB4','PWR_EVT_N 출력'),('44','PB5','E-stop 감지'),('45','PB6','USART1 TX'),('46','PB7','USART1 RX'),
    ('47','PB8','I2C1 SCL'),('48','PB9','I2C1 SDA'),
]
rows = [[*pins[i], *pins[i+24]] for i in range(24)]
table(['핀','이름','기능','핀','이름','기능'],rows,[28,73,WIDTH/2-101,28,73,WIDTH/2-101],tiny=True)
heading('회로·펌웨어 함께 지킬 조건')
add('EN 기본 꺼짐, 충전 CE 기본 비활성, MCU watchdog와 brownout을 구성한다. I2C 400 kHz, 풀업 2.2 kΩ, BMS 0x08 / 충전기 0x6B / INA 0x40·0x41·0x44. P1-S1 UART는 1 Mbaud, 상태 10 Hz + 고장 사건/종료 ACK/CRC·타임아웃을 정의한다. 원본 S1 프로토콜이 없어 프레임 형식은 미확정이다.')
add('PA4/PA5는 5 V 신호를 직접 넣지 않는다. PC13~15의 전류·입력 조건, PA8/PB15/PD0/PD2의 UCPD 관련 부팅 상태, PA14의 BOOT0 옵션, 내장 클록 USB 동작, 팬 open-drain/PWM 풀업 전압을 ST 데이터시트 및 AN5096과 대조한다. 핀 “확정” 표를 그대로 복사하는 것만으로 전압·부팅 안전성이 검증되지는 않는다.')
add('48핀 표는 첨부 내용의 배정표이며 이번 검사에서 모든 AF·전기 조건을 독립 검증했다는 뜻은 아니다. ST 최신 DS13560 Rev 6(2026-02)와 정확한 주문 suffix를 설계 기준으로 삼는다.', 'small')

# 9
page('보호값과 펌웨어 순서', '첨부 기준값을 유지할 항목과 수정할 항목을 구분. 배터리 제조사 조건과 실제 시험으로 최종 승인한다.')
table(['기능','rev A 기준 / 조치'],[
    ['셀 전압','충전 4.20 V/셀, 팩 16.80 V. 경고 3.50 V, 종료 요청 3.30 V, 마지막 CUV 약 2.99 V/4 s, COV 약 4.25 V/1 s. 셀별 값을 사용.'],
    ['전류 보호','1 mΩ 기준 OCC 6 A/330 ms, OCD1 40 A/330 ms, OCD2 70 A/20 ms, SCD 100 A/30 µs 원문 목표. 정격·측정 공차·퓨즈 시간 곡선과 대조.'],
    ['온도','충전 0~45 °C, 방전 −20~60 °C, FET 85 °C, 보드 90 °C 경고 목표. 센서 단선/단락을 고장 처리. 실제 팩·출력 PLA 열 조건 우선.'],
    ['균형 / 잔량','균형: 3.90 V 이상, 20 mV 시작/10 mV 종료, 0~45 °C. 잔량은 전류 적산 + 휴지 전압 보정. 10000 선택 시 용량·경고·충전 시간 설정을 16000에서 변경.'],
    ['출력 / 퓨즈','서보 A 15 A, B 7.5 A; 배터리 인라인 40 A. Jetson 5 A, AUX 2/3 A, S1 4 A는 분기 목표이며 총전류 합산·차단 타이밍을 검증.'],
    ['수정할 보호값','Jetson/외부 OVP는 공차 포함 다시 설정. 12 V 14.0 V MCU 경고에 하드웨어 회생 제동 추가. E-stop 단선 검출 및 실제 잔류 전압 기준 추가.'],
], [103, WIDTH-103])
heading('전원 순서')
add('<b>시작:</b> 상시 3.3 V → MCU의 모든 EN off → BMS 설정/보호 readback → 프리차지/DSG → PACK_P 확인 → 버튼 → 5 V/PG → S1 alive → Jetson → 명시적 servo_on + E-stop 정상일 때만 서보.')
add('<b>종료:</b> 버튼 또는 셀 저전압 → 사건/UART → S1/ROS 2가 안전 자세와 OS 종료 → ACK 또는 60 s 제한 → 서보·AUX·Jetson·5 V off. BMS 차단 고장은 출력 off 후 회복·프리차지·정상 시작 순서를 거친다. 절차 실행 가능 여부는 전압 저하 상황에서 시험한다.')
add('<b>충전:</b> 16.8 V, 최대 3.5 A, 입력 최대 3.3 A는 출발값. 전체 어댑터 전력 여유에 따라 ICHG/IINDPM을 낮추며 BMS·NTC 고장 시 비활성. 충전기 watchdog 40 s / 10 s 갱신, MCU watchdog 별도. 용량 변경 시 CC 부분만 약 2.9 h(10 Ah) / 4.6 h(16 Ah), CV·균형 시간을 추가한다.')
heading('외부 전원 모드의 검증 항목')
add('19 V > 16.8 V이므로 정상일 때 외부 경로가 우선된다. 제거 후 배터리로 끊김 없이 넘어가는지는 VSYS와 Jetson 입력을 측정해 확인한다. 12 V SMPS는 배터리보다 낮아 CHG에서 배터리가 부하를 계속 맡고, EXT에서 배터리를 차단했다가 제거하면 재시작 공백이 생길 수 있다. 이 두 동작을 같은 “무중단 외부 모드”로 표현하지 않는다.')
add('상시 전원의 BMS 우회 대기 소모와 자동 재시작 정책도 명시한다. 전류/전압/온도/모드/부팅 원인/보호 사건 로그를 ROS 2로 전달하고, 실제 배터리 측 평균 전력으로 운용 시간을 다시 산정한다.', 'small')

# 10
page('제작·시험과 남은 전장 작업', '배터리 구매만으로 전장이 완료되지는 않는다. 부하와 보드 배치를 먼저 고정하고 기구를 마감한다.')
heading('기판 제작 요구사항')
add('4층, 겉층 2 oz / 안층 1 oz를 출발점으로 한다. L2 GND, L3 전력, 겉층 전력 면을 활용한다. 30 A 12 mm 면/비아 20개, 15 A 6 mm/비아 10개는 원문의 배치 지침일 뿐 온도 상승 합격 증거가 아니다. copper/비아 도금 두께, 실제 길이, 부하 시간, 열 측정으로 승인한다.')
add('BAT_N은 셀 모니터 기준과 Rs1 배터리 측 켈빈 섬에만 연결한다. 다른 GND 경로로 shunt를 우회하지 않는다. 전력 루프·스위칭 노드는 짧고 작게, 측정 shunt는 켈빈, I2C/UART/NTC는 스위칭 노드에서 멀리 배치한다. Ø8 장착 keepout과 밑면 지지대 영역을 피하고 배터리 쪽으로 뜨거운 부품을 몰지 않는다.')
add('전체 BOM에는 모든 저항·커패시터·NTC·커넥터의 제조사/MPN/허용오차/온도·전압·전류 정격과 footprint를 포함한다. 전력 부품 3D 모델, 커넥터 mating, 퓨즈 탈착, 드라이버 접근, 케이블 굽힘을 반영해 P1 크기·높이를 최종 결정한다. 필요하면 160 x 90을 유지한다.')
heading('필수 검증 순서')
table(['단계','합격 근거'],[
    ['무전원 / 제한 전원','단락·핀·극성 확인 → 낮은 전류 제한 실험 전원과 셀 모사 입력 → 상시 3.3 V·I2C·설정 readback. 최초부터 실제 팩을 연결하지 않는다.'],
    ['보호 / 충전','셀 OV/UV, 과전류, NTC 단선·단락·고온, 프리차지, eFuse 차단과 재시도, 충전 종료·watchdog. OVP 시험은 Jetson 대신 더미 부하로 한다.'],
    ['변환 / 열','12 V 15 A 연속·25 A 100 ms 목표, 5 V 6 A, 최저/최고 입력. 5분 시험 후 장시간 열평형과 닫힌 몸체에서 온도 재검증.'],
    ['회생 / E-stop','실제 서보 감속 회생 전력·제동 저항 에너지·12 V/VSYS 최대값; 버튼·단선·MCU 정지 모두 차단; 출력 잔류 전압 및 자동 재시작 없음.'],
    ['전환 / 실기','12/19 V 꽂기·빼기·역극성·어댑터 과부하, Jetson 무중단 여부. 퓨즈 포함 실제 배터리 후 보행·팔 부하 로그, 질량·중심·토크 여유 확인.'],
], [112, WIDTH-112], tiny=True)
heading('현재 남아 있는 구매·제작 항목')
add('P1 + S1 회로/PCB/조립, 입력 모드에 맞는 19 V 어댑터, 뒤판 잭·모드·버튼·LED5·비상 정지, 전체 하네스·퓨즈·NTC·팬, P1/S1 펌웨어와 ROS 2, 배터리 누름대·선 출구·PCB 지지대. 배터리를 먼저 구매하면 4S 밸런스 충전기(보관 충전 기능)와 적절한 보관·충전 환경도 필요하다.')
add('검토 순서 권장: 배터리 실물 확인 → 최대 부하와 외부 전원 운용 방식 확정 → 회생/OVP/E-stop 수정 → 실제 BOM 배치·열 검토 → PCB/브라켓 마감 → 제한 전원 시험 → 배터리 실기 시험. 실제 PCB 무게를 측정해 CAD 및 hip servo 부하 계산을 갱신한다.', 'small')

# 11
page('검토 범위와 확인 가능한 출처', '판매 가격·재고는 2026-10-07 확인 시점. 제품 표기 치수는 실물 치수 보증이 아니다.')
heading('문서와 CAD의 범위')
add('첨부 「MARC v4 전원 PCB P1 rev A - 상세 설계 (2026-10-07)」의 0~12절 및 출처 전체를 읽었다. 요약/변경·전원 나무·동작 모드·블록 회로·MCU·커넥터·보호·펌웨어·배치/층·손실·BOM·시험·남은 확인 항목을 이 보고서에 반영했다.')
add('첨부가 인용하는 claude/marc-v4-pcb-v1.md, marc-v4-pcb-v2.md, marc-v4-assembly.md는 현재 작업 폴더와 검색 가능한 로컬 경로에서 찾지 못했다. 따라서 <b>S1 전체 회로·USB 상대 장치·프로토콜 원본, v1 실제 부하표, assembly 상세는 독립 검증하지 못했다</b>. 존재하지 않는 원문을 검토한 것으로 간주하지 않는다.')
add('Fusion v63은 읽기 전용으로 브라켓 평면·배터리/기판 bounds와 임시 솔리드 간섭을 확인했다. 전체 질량 5683.2 g는 CAD 계산값이다. 출력품 측정, 동작 시험, 팩 실물, 보드 열, 강도 해석 또는 모든 동작 범위 검증을 이번 조사에서 수행하지 않았다.')
heading('구매 근거')
refs = [
    ('S1', '팰콘샵 Neonergy 4S 10000 / BU-134360', 'https://www.falconshop.co.kr/shop/goods/goods_view.php?goodsno=100092305'),
    ('S2', '팰콘샵 EP POWER 4S 10000', 'https://www.falconshop.co.kr/shop/goods/goods_view.php?category=019003039&goodsno=100087294'),
    ('S3', '팰콘샵 CNHL Racing HC 4S 10000', 'https://www.falconshop.co.kr/shop/goods/goods_view.php?goodsno=100092386'),
    ('S4', 'CNHL 공식 16000 mAh 4S XT90', 'https://chinahobbyline.com/products/cnhl-16000mah-14-8v-4s-15c-lipo-battery-with-xt90-plug'),
    ('S5', 'GetFPV Lumenier 16000 mAh 4S', 'https://www.getfpv.com/batteries/commercial-consumer-batteries/lumenier-16000mah-4s-20c-lipo-battery.html'),
    ('S6', '네이버 통합검색 - Neonergy 10000 4S 배터리', 'https://search.naver.com/search.naver?query=Neonergy%2010000%204S%20%EB%B0%B0%ED%84%B0%EB%A6%AC'),
]
for ref, name, url in refs: add(f'[{ref}] {link(name, url)}', 'small')
naver_bridge = 'https://cr3.shopping.naver.com/v2/bridge/searchGate?nv_mid=61775229364&cat_id=50004287&query=Neonergy%2010000%204S%20%EB%B0%B0%ED%84%B0%EB%A6%AC&t=muxoycpd&h=26a593db58ac50ce521462074ffe5d8a17040fdd&frm=NVSCPRO'
add('[S6a] '+link('확인한 네이버 가격비교 카드 → 팰콘샵 구매 경로',naver_bridge)+' (리다이렉트 링크는 만료될 수 있음)', 'small')
heading('핵심 회로 검증 근거: 제조사 원문')
refs2 = [
    ('S7', 'TI LM74700-Q1 - 역전류 차단', 'https://www.ti.com/lit/ds/symlink/lm74700-q1.pdf'),
    ('S8', 'TI TPS25947 - OVLO 공차 / 474 차단형 / ITIMER / PGTH', 'https://www.ti.com/lit/ds/symlink/tps25947.pdf'),
    ('S9', 'TI PMP31202 - 원본 입력·출력·컨트롤러·설계 자료', 'https://www.ti.com/tool/PMP31202'),
    ('S9a', 'TI LM5143-Q1 - 2상 제어와 다이오드 에뮬레이션', 'https://www.ti.com/lit/ds/symlink/lm5143.pdf'),
    ('S10', 'TI BQ25798 - 4S 충전 / 입력 제한', 'https://www.ti.com/lit/ds/symlink/bq25798.pdf'),
    ('S11', 'NVIDIA Jetson Orin Nano 캐리어 사양 / Download Center', 'https://developer.nvidia.com/embedded/downloads#?search=Jetson%20Orin%20Nano%20Developer%20Kit%20Carrier'),
    ('S12', 'ST STM32G0B1xB/xC/xE - DS13560 Rev 6', 'https://www.st.com/resource/en/datasheet/stm32g0b1ce.pdf'),
    ('S13', 'TI BQ76952 - EVM / 셀 매핑 / 보호 설정 원문', 'https://www.ti.com/product/BQ76952'),
]
for ref, name, url in refs2: add(f'[{ref}] {link(name, url)}', 'small')
add('치수/간섭 근거는 이번 조사에서 저장한 fusion_inventory.json, fit_results.json, expansion_results.json이다. 회로 제안은 검토자의 계산과 설계 판단이며 검증 완료된 회로도나 구매 주문을 의미하지 않는다.', 'small')

class NumberedCanvas(canvas.Canvas):
    def __init__(self, *args, **kwargs):
        canvas.Canvas.__init__(self, *args, **kwargs)
        self._states = []
    def showPage(self):
        self._states.append(dict(self.__dict__))
        self._startPage()
    def save(self):
        total = len(self._states)
        for state in self._states:
            self.__dict__.update(state)
            self.setStrokeColor(LINE)
            self.setLineWidth(.5)
            self.line(42, H-35, W-42, H-35)
            self.line(42, 35, W-42, 35)
            self.setFont('Korean', 8)
            self.setFillColor(MUTED)
            self.drawString(42,H-26,'SCONE  /  MARC v4  /  ELECTRICAL REVIEW')
            self.drawString(42,22,'2026-10-07  |  Fusion v63  |  설계 검토')
            self.drawRightString(W-42,22,f'{self._pageNumber} / {total}')
            canvas.Canvas.showPage(self)
        canvas.Canvas.save(self)

doc = SimpleDocTemplate(str(OUT), pagesize=A4, rightMargin=42, leftMargin=42, topMargin=52, bottomMargin=48,
                        title='MARC v4 배터리와 P1 전장 검토', author='SCONE design review',
                        pageCompression=1)
doc.build(story, canvasmaker=NumberedCanvas)
print(OUT)

observations = {
    'checked_date':'2026-10-07',
    'fusion_document':'MARC v4 Body v5 BOX v63',
    'fusion_modified_after':False,
    'mass_g':5683.199692,
    'bay_LWH_mm':[186,46,77.5],
    'recommended_battery':'Neonergy NE14.8V10000mAh4S1P100C, XT90 option',
    'naver_query_url':refs[5][2],
    'observed_naver_bridge_url':naver_bridge,
    'naver_price_card_krw':116620,
    'seller_url':refs[0][2],
    'seller_price_krw':119000,
    'seller_inventory_shown':10,
    'installed_LWH_mm':[175,42,49],
    'catalogue_LWH_mm':[175,49,42],
    'battery_mass_g':748,
    'unverified':['checkout discount conditions','actual pack measurements','full operating load','referenced v1/v2/assembly documents','complete S1 schematic','manufacturing validation'],
    'CAD_edits':False,
    'purchase':False,
}
(ROOT / 'artifacts/electronics/20261007_power_review/review_observations.json').write_text(json.dumps(observations,ensure_ascii=False,indent=2))
