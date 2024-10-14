#include <Servo.h>

// 핀 정의
#define PIN_LED   9   
#define PIN_TRIG  12  
#define PIN_ECHO  13  
#define PIN_SERVO 10  

// 서보 모터 펄스 값 정의
#define _DUTY_MIN 500   
#define _DUTY_NEU 1480  
#define _DUTY_MAX 2450  

// 거리 값 정의 (mm 단위)
#define _DIST_MIN 180.0   
#define _DIST_MAX 360.0  

// 초음파 센서 관련 상수
#define SND_VEL 346.0       
#define INTERVAL 25        
#define PULSE_DURATION 10  
#define TIMEOUT ((INTERVAL / 2) * 1000.0)  
#define SCALE (0.001 * 0.5 * SND_VEL)  

// EMA 필터 알파 값
#define _EMA_ALPHA 0.3  

// 타겟 거리 범위
#define _TARGET_LOW  180.0
#define _TARGET_HIGH 360.0

// 전역 변수
float dist_ema, dist_prev = _DIST_MAX;  
unsigned long last_sampling_time = 0;   
Servo myservo;

void setup() {
  // GPIO 핀 초기화
  pinMode(PIN_LED, OUTPUT);
  pinMode(PIN_TRIG, OUTPUT);  
  pinMode(PIN_ECHO, INPUT);   
  digitalWrite(PIN_TRIG, LOW);  

  myservo.attach(PIN_SERVO); 
  myservo.writeMicroseconds(_DUTY_NEU);  

  // 시리얼 통신 초기화
  Serial.begin(57600);
}

void loop() {
  float dist_raw;

  if (millis() < (last_sampling_time + INTERVAL))
    return;

  dist_raw = USS_measure(PIN_TRIG, PIN_ECHO);

  // 범위 필터 적용 및 LED 제어
  if ((dist_raw == 0.0) || (dist_raw > _DIST_MAX)) {
    dist_raw = dist_prev;  
    digitalWrite(PIN_LED, HIGH);  // LED OFF (active-low)
  } else if (dist_raw < _DIST_MIN) {
    dist_raw = dist_prev;  
    digitalWrite(PIN_LED, HIGH);  // LED OFF (active-low)
  } else {
    dist_prev = dist_raw;  
    digitalWrite(PIN_LED, LOW);   // LED ON (active-low)
  }

  // EMA 필터 적용
  dist_ema = (1 - _EMA_ALPHA) * dist_ema + _EMA_ALPHA * dist_raw;

  // 서보 모터 각도 조정
  if (dist_ema <= _TARGET_LOW) {
    myservo.writeMicroseconds(_DUTY_MIN);  
  } else if (dist_ema >= _TARGET_HIGH) {
    myservo.writeMicroseconds(_DUTY_MAX);  
  } else {
    int pulseWidth = map(dist_ema, _TARGET_LOW, _TARGET_HIGH, _DUTY_MIN, _DUTY_MAX);
    myservo.writeMicroseconds(pulseWidth);
  }

  // 시리얼 출력
  Serial.print("Min:");    Serial.print(_DIST_MIN);
  Serial.print(", dist_raw:");  Serial.print(dist_raw);
  Serial.print(", ema:");  Serial.print(dist_ema);
  Serial.print(", Servo Pulse:"); Serial.print(myservo.readMicroseconds());
  Serial.print(", Max:");   Serial.println(_DIST_MAX);

  last_sampling_time += INTERVAL;
}

// 초음파 센서에서 거리 측정 (단위: mm)
float USS_measure(int TRIG, int ECHO) {
  digitalWrite(TRIG, HIGH);
  delayMicroseconds(PULSE_DURATION);
  digitalWrite(TRIG, LOW);

  return pulseIn(ECHO, HIGH, TIMEOUT) * SCALE;  
}
