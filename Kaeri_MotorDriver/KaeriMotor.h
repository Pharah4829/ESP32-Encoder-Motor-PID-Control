#pragma once
#include <Arduino.h>

/* ISR 슬롯용 최대 모터 수 */
#define MAX_MOTORS 2

class KaeriMotor {
public:
  /* 제어 모드 */
  enum ControlMode {
    CONTROL_SPEED,
    CONTROL_POSITION,
    CONTROL_CURRENT
  };

  /* 생성자
   * mA, mB      : 모터 PWM 핀
   * eA, eB      : 엔코더 A/B 핀
   * currentPin : 전류 센서 ADC 핀 (-1이면 미사용)
   */
  KaeriMotor(int mA, int mB, int eA, int eB, int currentPin = -1);

  /* 엔코더 설정 */
  void encoder(int ppr, float gearRatio);

  /* PID 태스크 시작 */
  bool Begin(uint32_t period_ms = 2);

  /* PID 계수 */
  void PIDset(float kp, float ki, float kd);

  /* ===== 제어 명령 ===== */
  void SetSpeed(int rpm);          // RPM
  void SetPosition(float rev);     // 절대 위치 (rev)
  void SetPositionInc(float rev);  // 증분 위치 (rev)
  void SetCurrent(float mA);       // 전류 제어 (mA)

  /* ===== 모니터링 ===== */
  float GetSpeed();     // RPM
  float GetPosition();  // rev
  float GetCurrent();   // mA (2회 평균)

  /* ===== 캘리브레이션 ===== */
  void CalibrationStart(int pwm);
  float CalibrationFinish();

private:
  /* 핀 */
  int _mA, _mB;
  int _eA, _eB;
  int _currentPin;

  /* ISR 슬롯 */
  int _slot = -1;
  static KaeriMotor* _slots[MAX_MOTORS];
  static void IRAM_ATTR _encISR0();
  static void IRAM_ATTR _encISR1();
  void _handleEncoder();

  /* FreeRTOS 태스크 핸들 */
  void* _task = nullptr;

  /* 엔코더 상태 */
  volatile long _pos_i = 0;
  long _posPrev = 0;

  int _encoderPPR = 0;
  float _gearRatio = 1.0f;
  float _ticksPerRev = 0.0f;

  /* 상태값 */
  float _rpmFiltered = 0.0f;
  float _rpmPrev = 0.0f;

  /* PID */
  float _kp = 0, _ki = 0, _kd = 0;
  float _integral = 0;
  float _lastErr = 0;

  /* 목표값 */
  ControlMode _mode = CONTROL_SPEED;
  int   _targetRPM = 0;
  long  _targetPosTicks = 0;
  float _targetCurrent = 0.0f; // mA

  /* 기타 */
  bool _calibMode = false;

  /* 타이밍 */
  unsigned long _periodTicks = 0;
  float _dt_s = 0.002f;

  /* 필터 계수 */
  static constexpr float RPM_LPF_A = 0.854f;
  static constexpr float RPM_LPF_B = 0.0728f;

  /* 내부 */
  static void _taskEntry(void* arg);
  void _pidStep();
  void _setMode(ControlMode m);

  /* 전류 측정 (2회 평균, mA) */
  float _readCurrentmA();
};
