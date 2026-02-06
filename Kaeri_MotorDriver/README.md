# KaeriMotor (ESP32)

ESP32에서 **추가적인 모터 제어 IC 없이**,  
**휘트스톤 브릿지(또는 H-브릿지) 기반의 단순 모터 드라이버**만으로도  
엔코더 A/B상을 직접 읽어 **모터 속도/위치 PID 제어**를 할 수 있게 만든 라이브러리입니다.

- 엔코더: A상 인터럽트 + B상으로 방향 판별
- 전류센서: 선택사항 (없으면 currentPin 생략 가능)

---

## API (사용자용 요약)

### 1) 생성자

- `KaeriMotor(mA, mB, eA, eB)`
  - 전류센서 없이 사용

- `KaeriMotor(mA, mB, eA, eB, currentPin)`
  - 전류센서(ADC 핀)까지 같이 사용

**파라미터**
- `mA`, `mB`: 모터 드라이버 출력 핀 2개
- `eA`, `eB`: 엔코더 A/B 핀
- `currentPin`: 전류센서 ADC 핀 (선택)

---

### 2) 시작 필수 순서

1. `encoder(ppr, gearRatio)`  
   - 엔코더 해상도(PPR)와 기어비 설정

2. `PIDset(kp, ki, kd)`  
   - PID 게인 설정

3. `Begin(period_ms=2)`  
   - 엔코더 인터럽트 + PID 루프 시작  
   - `period_ms`: PID 실행 주기(ms)

---

### 3) 제어 명령 (원하는 모드만 사용)

- `SetSpeed(rpm)`  
  - 목표 속도(RPM) 제어 (양수/음수 가능)

- `SetPosition(rev)`  
  - 목표 위치(rev)로 이동 (절대 위치)

- `SetPositionInc(rev)`  
  - 현재 위치에서 `rev` 만큼 추가 이동 (증분 이동)

- `SetCurrent(mA)`  
  - 목표 전류(mA) 제어 (전류센서 있을 때 권장)

---

### 4) 상태 읽기 (모니터링)

- `GetSpeed()` : 현재 속도(RPM)
- `GetPosition()` : 현재 위치(rev)
- `GetCurrent()` : 현재 전류(mA) (전류센서 없으면 의미 제한)

---

## 최소 사용 예시 (Speed)

```cpp
#include "KaeriMotor.h"

// 전류센서 미사용이면 마지막 인자 생략 가능
KaeriMotor motor(25, 26, 34, 35);

void setup() {
  motor.encoder(16000, 1);      // ppr, gearRatio
  motor.PIDset(5.0, 0.0, 0.0);  // kp, ki, kd
  motor.Begin(2);               // period_ms
}

void loop() {
  motor.SetSpeed(100);          // RPM
}
