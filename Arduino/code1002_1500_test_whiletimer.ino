#include <Wire.h> // I2C통신 설정 – 가속도 측정하기 위함
#include “MPU6050.h” // 가속도 라이브러리 선언(가속도 측정 관련 라이브러리)
#include <SoftwareSerial.h> // 소프트 시리얼 라이브러리 선언(블루투스 통신하기 위함)

#define BUZZER 7 // 부저 핀 번호
#define RESET 4 // 가속도센서(mpu6050) RESET 핀 번호
int TxPin = 5; // 블루투스 통신 TX단자
int RxPin = 6; // 블루투스 통신 RX단자
SoftwareSerial BTSerial(TxPin, RxPin);

// (중요)(중요)
int NOMAL_ANGLE = 50; // 정상기울기 범위, 기울기가 ‘NOMAL_ANGLE’ 안에 들어 올 경우 정상처리
long CRASH_VALUE = 700; // 충돌 값(충돌 판단하는 값, 가속도 변화량 값)

// 구동 변수
long ac_x, ac_y, ac_z, gy_x, gy_y, gy_z ; //acc, gyro data (acc, gyro 계산 수식)
double angle1 = 0, deg ; // angle, deg data (각도계산)
double angle2 = 0;
double dgy_x ;
long mapping_value = 1000;
long normal_x,normal_y,normal_z, deltha_x[3],deltha_y[3],deltha_z[3], deltha, angle_1[3], angle_2[3] ;
long event_value = 1000;

// 처리 변수
const int delay_main = 1; // 전체 시스템 구동하는 데, 걸리는 시간 (즉 구동 시간)이다. (ex) 1000 = 1000ms = 1s = 1초
const int sum_count = 4;
long delay_config = 6000; // 이벤트 발생 후 딜레이
boolean ACCESS_ACCEL = false;
long TIEMR_BLESEND = 0;
bool WARN_STATE = false;
long TIMER_WARN_STATE = 0;
bool buzzer_state2 = true;

// 함수 선언
int ACTION_config();
void value_init();

통신 설정
void setup() {

  // 시리얼통신 & 소프트시리얼 통신 설정
  Serial.begin(115200); BTSerial.begin(9600);

  // 핀모드 및 초기 설정
  pinMode(BUZZER, OUTPUT);pinMode(RESET, OUTPUT); // 출력으로 사용
  tone(BUZZER, 1000, 100); delay(200); tone(BUZZER, 1000, 100); // 부저 울리기
  digitalWrite(RESET, LOW); // RESET 초기화

  // 가속도 센서 설정
  delay(100);
  mpu6050_init(); // 가속도 센서 초기 설정
  ACCESS_ACCEL = ACCEL_ACCESS(); // 가속도 센서 값 맞게 나오는지 확인하기
}
void loop() {

  // 1. 정해진 시간마다 블루투스 통신으로 ‘a’값 보내기 (부팅 되었는지 확인하기 위해)
  if(TIEMR_BLESEND < millis()){
  BTSerial.print(“a”); // 블루투스 데이터 보내기
  TIEMR_BLESEND = millis() + 10000; // 10초 간격으로 데이터 보내기
  }

  // 2. 블루투스 데이터 감지하기
  if (BTSerial.available()){ // 블루투스 데이터를 감지했을 경우 안에 while문이 동작함
  int timeout = 50;
  String response = “”; long int time = millis();
  while( (time+timeout) > millis()) { // 50ms동안 문자열 세는 방식
  while(BTSerial.available()) { // 블루투스 통신이 끝날 때 까지
  char c = BTSerial.read(); // c에 읽어 response에 저장
  response+=c; }}

  String DATA = response; // 컨트롤 데이터에 읽은 데이터 넣기
  Serial.println( “DATA : “ + DATA); // 컨트롤 데이터 출력

  if(DATA.substring(0,2) == “HI”){ // 블루투스 데이터가 “HI”인가? 그럼 ‘a’데이터 보내기
  BTSerial.print(“a”);
  }
}

if(NOMAL_ANGLE) {
    if(deltha > Emergency_value) || if(shock_value < Emergency_shock){State_Parameter=true;}
}
