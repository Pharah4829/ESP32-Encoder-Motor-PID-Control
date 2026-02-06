# ESP32 H-Bridge Encoder PID Control

ESP32에서 **추가적인 모터 제어 IC 없이**,  
**휘트스톤 브릿지 / H-브릿지 기반의 단순 모터 드라이버**만을 사용해  
엔코더 A/B상을 직접 읽고 **모터 속도 및 위치 PID 제어**를 수행할 수 있는 라이브러리입니다.

- 엔코더 A상 인터럽트 + B상 방향 판별
- Speed / Position / Current 제어 지원
- 전류 센서 선택 사용 가능
- Arduino IDE 환경에서 바로 사용 가능

---

## Library Installation

### 1) GitHub에서 다운로드
1. 이 저장소 상단의 **Code → Download ZIP** 클릭
2. ZIP 압축 해제

### 2) Arduino 라이브러리 폴더에 복사
아래 경로에 라이브러리 폴더를 그대로 복사합니다.

