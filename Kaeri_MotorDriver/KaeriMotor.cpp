#include "KaeriMotor.h"
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"

/* PID 태스크 코어 선택 */
static inline BaseType_t pickPidCore() {
#if (defined(portNUM_PROCESSORS) && (portNUM_PROCESSORS > 1))
  return 1;
#else
  return 0;
#endif
}

/* ISR 슬롯 테이블 */
KaeriMotor* KaeriMotor::_slots[MAX_MOTORS] = { nullptr, nullptr };

/* ===== 생성자 ===== */
KaeriMotor::KaeriMotor(int mA, int mB, int eA, int eB, int currentPin)
  : _mA(mA), _mB(mB), _eA(eA), _eB(eB), _currentPin(currentPin)
{
  pinMode(_mA, OUTPUT);
  pinMode(_mB, OUTPUT);
  pinMode(_eA, INPUT);
  pinMode(_eB, INPUT);
  if (_currentPin >= 0) pinMode(_currentPin, INPUT);

  for (int i = 0; i < MAX_MOTORS; i++) {
    if (_slots[i] == nullptr) {
      _slots[i] = this;
      _slot = i;
      break;
    }
  }
}

/* ===== 엔코더 설정 ===== */
void KaeriMotor::encoder(int ppr, float gearRatio)
{
  _encoderPPR = ppr;
  _gearRatio = gearRatio;
  _ticksPerRev = (float)ppr * gearRatio;
}

/* ===== Begin ===== */
bool KaeriMotor::Begin(uint32_t period_ms)
{
  if (_slot < 0 || _ticksPerRev <= 0) return false;

  if (period_ms == 0) period_ms = 1;
  _periodTicks = pdMS_TO_TICKS(period_ms);
  _dt_s = period_ms / 1000.0f;

  if (_slot == 0)
    attachInterrupt(digitalPinToInterrupt(_eA), _encISR0, RISING);
  else
    attachInterrupt(digitalPinToInterrupt(_eA), _encISR1, RISING);

  xTaskCreatePinnedToCore(
    _taskEntry, "KaeriPID", 4096,
    this, 4, (TaskHandle_t*)&_task,
    pickPidCore()
  );

  return true;
}

/* ===== PID 파라미터 ===== */
void KaeriMotor::PIDset(float kp, float ki, float kd)
{
  _kp = kp; _ki = ki; _kd = kd;
}

/* ===== 모드 전환 ===== */
void KaeriMotor::_setMode(ControlMode m)
{
  if (_mode != m) {
    _integral = 0;
    _lastErr = 0;
  }
  _mode = m;
}

/* ===== 제어 명령 ===== */
void KaeriMotor::SetSpeed(int rpm)
{
  _setMode(CONTROL_SPEED);
  _targetRPM = rpm;
}

void KaeriMotor::SetPosition(float rev)
{
  _setMode(CONTROL_POSITION);
  _targetPosTicks = (long)(rev * _ticksPerRev);
}

void KaeriMotor::SetPositionInc(float rev)
{
  _setMode(CONTROL_POSITION);

  noInterrupts();
  long cur = _pos_i;
  interrupts();

  _targetPosTicks = cur + (long)(rev * _ticksPerRev);
}

void KaeriMotor::SetCurrent(float mA)
{
  _setMode(CONTROL_CURRENT);
  _targetCurrent = mA;
}

/* ===== 모니터링 ===== */
float KaeriMotor::GetSpeed()
{
  return _rpmFiltered;
}

float KaeriMotor::GetPosition()
{
  noInterrupts();
  long pos = _pos_i;
  interrupts();

  return (float)pos / _ticksPerRev;
}

float KaeriMotor::GetCurrent()
{
  return _readCurrentmA();
}

/* ===== 전류 측정 (2회 평균) ===== */
float KaeriMotor::_readCurrentmA()
{
  if (_currentPin < 0) return 0.0f;

  int a1 = analogRead(_currentPin);
  int a2 = analogRead(_currentPin);
  float adc = 0.5f * (a1 + a2);

  // ESP32 C3 : 12bit, 3.3V, 200 mV/A, 0V offset
  float voltage = adc * 3.3f / 4095.0f;
  float currentA = voltage / 0.2f;

  return currentA * 1000.0f; // mA
}

/* ===== Calibration ===== */
void KaeriMotor::CalibrationStart(int pwm)
{
  _calibMode = true;

  noInterrupts();
  _pos_i = 0;
  interrupts();

  pwm = constrain(pwm, 0, 255);
  analogWrite(_mA, pwm);
  analogWrite(_mB, 0);
}

float KaeriMotor::CalibrationFinish()
{
  analogWrite(_mA, 0);
  analogWrite(_mB, 0);
  _calibMode = false;

  noInterrupts();
  float measured = fabs((float)_pos_i);
  interrupts();

  _ticksPerRev = measured;
  return measured;
}

/* ===== ISR ===== */
void IRAM_ATTR KaeriMotor::_encISR0() {
  if (_slots[0]) _slots[0]->_handleEncoder();
}
void IRAM_ATTR KaeriMotor::_encISR1() {
  if (_slots[1]) _slots[1]->_handleEncoder();
}

void KaeriMotor::_handleEncoder()
{
  int b = digitalRead(_eB);
  if (b > 0) _pos_i--;
  else       _pos_i++;
}

/* ===== 태스크 ===== */
void KaeriMotor::_taskEntry(void* arg)
{
  KaeriMotor* self = static_cast<KaeriMotor*>(arg);
  TickType_t last = xTaskGetTickCount();

  for (;;) {
    self->_pidStep();
    vTaskDelayUntil(&last, self->_periodTicks);
  }
}

/* ===== PID ===== */
void KaeriMotor::_pidStep()
{
  if (_calibMode) return;

  noInterrupts();
  long snap = _pos_i;
  interrupts();

  long delta = snap - _posPrev;
  _posPrev = snap;

  float vel = delta / _dt_s;
  float rpm = (vel / _ticksPerRev) * 60.0f;

  _rpmFiltered =
    RPM_LPF_A * _rpmFiltered +
    RPM_LPF_B * rpm +
    RPM_LPF_B * _rpmPrev;
  _rpmPrev = rpm;

  float err = 0.0f;
  float derr = 0.0f;

  switch (_mode) {
    case CONTROL_SPEED:
      err = _targetRPM - _rpmFiltered;
      derr = (err - _lastErr) / _dt_s;
      break;

    case CONTROL_POSITION:
      err = (float)(_targetPosTicks - snap);
      derr = -_rpmFiltered;
      break;

    case CONTROL_CURRENT: {
      float currentmA = _readCurrentmA();
      err = _targetCurrent - currentmA;
      derr = (err - _lastErr) / _dt_s;
      break;
    }
  }

  _integral += err * _dt_s;
  float u = _kp * err + _ki * _integral + _kd * derr;
  _lastErr = err;

  u = constrain(u, -255.0f, 255.0f);
  int duty = abs((int)u);

  if (u <= 0) {
    analogWrite(_mA, duty);
    analogWrite(_mB, 0);
  } else {
    analogWrite(_mA, 0);
    analogWrite(_mB, duty);
  }
}
