# ESP32 H-Bridge Encoder PID Control

ESP32에서 **추가적인 모터 제어 IC 없이**,  
**휘트스톤 브릿지 / H-브릿지 기반의 단순 모터 드라이버**만을 사용하여  
엔코더 A/B상을 직접 읽고 **모터 속도(Speed) / 위치(Position) PID 제어**를 수행할 수 있는 라이브러리입니다.

- 엔코더 A상 인터럽트 + B상 방향 판별
- Speed / Position / Current 제어 지원
- 전류 센서 선택 사용 가능
- Arduino IDE 환경에서 바로 사용 가능

---

## Library Installation

### 1. Download
GitHub 저장소 상단의 **Code → Download ZIP**을 클릭하여 라이브러리를 다운로드합니다.

### 2. Install to Arduino
압축을 해제한 뒤, 라이브러리 폴더를 아래 경로에 복사합니다.

```
Documents/Arduino/libraries/ESP32-Encoder-Motor-PID-Control
```

### 3. Restart Arduino IDE
Arduino IDE를 재시작하면 라이브러리가 자동으로 인식됩니다.

---

## API (User Guide)

### Constructor

- `KaeriMotor(mA, mB, eA, eB)`  
  전류 센서 없이 사용합니다.

- `KaeriMotor(mA, mB, eA, eB, currentPin)`  
  전류 센서(ADC 핀)를 함께 사용합니다.

**Parameters**
- `mA`, `mB` : 모터 드라이버 출력 핀
- `eA`, `eB` : 엔코더 A / B 핀
- `currentPin` : 전류 센서 ADC 핀 (선택)

---

### Initialization Order (Required)

1. `encoder(ppr, gearRatio)`  
   엔코더 해상도(PPR)와 기어비 설정

2. `PIDset(kp, ki, kd)`  
   PID 게인 설정

3. `Begin(period_ms = 2)`  
   엔코더 인터럽트 및 PID 루프 시작  
   `period_ms` : PID 실행 주기(ms)

---

### Control Commands

- `SetSpeed(rpm)`  
  목표 속도(RPM) 제어

- `SetPosition(rev)`  
  목표 위치(rev)로 이동 (절대 위치)

- `SetPositionInc(rev)`  
  현재 위치에서 rev 만큼 추가 이동

- `SetCurrent(mA)`  
  목표 전류(mA) 제어 (전류 센서 사용 시)

---

### State Monitoring

- `GetSpeed()` : 현재 속도(RPM)
- `GetPosition()` : 현재 위치(rev)
- `GetCurrent()` : 현재 전류(mA)

---

## Encoder Calibration

엔코더 캘리브레이션은 **Encoder_Calibration 예제**를 사용합니다.  
실제 장착된 상태에서 **1회전당 엔코더 tick 수(ticksPerRev)** 를 측정하기 위한 기능입니다.

### How to Use

1. Arduino IDE에서 예제 열기  
   `File → Examples → ESP32-Encoder-Motor-PID-Control → Encoder_Calibration`

2. ESP32 보드에 업로드

3. 시리얼 모니터 설정  
   - Baudrate: **115200**
   - Line Ending: **Newline**

4. 시리얼 입력
   - PWM 값을 숫자로 입력 후 Enter  
     예: `80`
   - 모터가 정확히 1회전한 직후 `s` 입력 후 Enter

5. 시리얼 모니터에 출력된 값이 `ticksPerRev` 입니다.

---

## Minimal Usage Example (Speed Control)

```cpp
#include "KaeriMotor.h"

// current sensor not used
KaeriMotor motor(25, 26, 34, 35);

void setup() {
  motor.encoder(16000, 1);
  motor.PIDset(5.0, 0.0, 0.0);
  motor.Begin(2);
}

void loop() {
  motor.SetSpeed(100);
}
```

---

## Notes

- 엔코더 카운트는 A상 RISING 기준입니다.
- 전류 센서를 사용하지 않으면 `currentPin`을 생략하거나 `-1`로 설정하세요.
