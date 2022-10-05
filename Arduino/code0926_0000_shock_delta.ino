#include <Arduino.h> // 아두이노 하드웨어 번지수(주소) 라이브러리
#include <Wire.h> // I2C 통신을 하기 위한 라이브러리, 아두이노 우노 기준 : SCL, SDA별도로 있음
#include <SoftwareSerial.h>
SoftwareSerial BTSerial(6,7);
 

void mpu6050_init(); // 가속도 센서 초기 제어
void accel_calculate(); // 가속도 센서 측정하기
void value_init(); // 측정 변수 초기화 하기
void Emergency_state_(); // 긴급 모드 일 시 제어 함수


#define mpu_add 0x68 //mpu6050 칩의 주소
 

long ac_x, ac_y, ac_z, gy_x, gy_y, gy_z ; //acc, gyro data (acc, gyro 계산 수식)
double shock = 0, deg ; // angle, deg data (각도계산)
double dgy_x ; // double type acc data
long int normal_x,normal_y,normal_z, deltha_x[3],deltha_y[3],deltha_z[3], deltha; // 노말라이즈(정규화 데이터), 가속도 변화량 표시
long int shock_value;
int pin = A0;
const int mapping_value = 1000;
 
const int Emergency_value = 1300; // 비상상태로 판단하는 값
const int Emergency_value2 = 1200;// 비상상태로 판단하는 값

const int Emergency_shock = 300;

 
boolean State_Parameter = false; // 상태 변수 0 : 정상, 1 : 비상
 
const int delay_main = 2; // 전체 시스템 구동하는 데, 걸리는 시간 (즉 구동 시간)이다. (ex) 1000 = 1000ms = 1s = 1초
const long int sum_count = 2; // 평균 내는 횟수
 
#define Pin_Relay 12

 
void setup()
{
  Serial.begin(9600); // 컴푸터와 시리얼통신을 하기 위한 것
  mpu6050_init(); // 가속도 센서 초기 설정
  BTSerial.begin(9600);
 

  pinMode(Pin_Relay, OUTPUT);
  digitalWrite(Pin_Relay , HIGH); // HIGH - LED off, LOW - LED on
}

// #6. Loop 문 (반복 문) 시스템 구동
void loop()
{
  State_Parameter = false;
  value_init(); //가속도-각도 관련 초기값 선언

  // if(BTSerial.available())  {                   // BTSerial에 입력이 되면 블루투스
  //   Serial.write(BTSerial.read());           // BTSerial에 입력된 값을 시리얼 모니터에 출력
  // }

  // if(Serial.available())    {                      // 시리얼 모니터에 입력이 되면
  //   BTSerial.write(Serial.read());           // 그 값을 BTSerial에 출력
  // }  //여기까지 블루투스b 

  //첫번째 센싱
  for (int i=0; i < sum_count; i++){ accel_calculate();
  deltha_x[1] = deltha_x[1]+(normal_x); deltha_y[1] = deltha_y[1]+(normal_y); deltha_z[1] = deltha_z[1]+(normal_z); shock_value = shock;}
  deltha_x[1] = int(deltha_x[1]/sum_count); deltha_y[1] = int(deltha_y[1]/sum_count); deltha_z[1] = int(deltha_z[1]/sum_count);

  //두번째 센싱
  for (int i=0; i < sum_count; i++){ accel_calculate();
  deltha_x[2] = deltha_x[2]+(normal_x); deltha_y[2] = deltha_y[2]+(normal_y); deltha_z[2] = deltha_z[2]+(normal_z); shock_value = shock;}
  deltha_x[2] = int(deltha_x[2]/sum_count); deltha_y[2] = int(deltha_y[2]/sum_count); deltha_z[2] = int(deltha_z[2]/sum_count);
 
  //3축 변화량 비교 - 가속도 변화량, 각도 평균 값
  deltha_x[0] = abs(deltha_x[1]-deltha_x[2]); deltha_y[0] = abs(deltha_y[1]-deltha_y[2]); deltha_z[0] = abs(deltha_z[1]-deltha_z[2]);
  deltha = deltha_x[0] + deltha_y[0] + deltha_z[0];
  shock_value = abs(int(shock_value));

  // deltha : 가속도 변화량
  // shock_vlaue: 진동값
  if (deltha > Emergency_value){State_Parameter=true;}
  if (shock_value < Emergency_shock){State_Parameter=true;}
  if ((deltha > Emergency_value2)&&(shock_value < Emergency_shock)){State_Parameter=true;}
 
 
  // State_Parameter - 비상이냐? 정상이냐 상태 변수
  if( State_Parameter == true ){Emergency_state_();}else{
  Serial.print( "deltha : " );
  Serial.print(deltha);
  Serial.println();
  Serial.print("shock_value : ");
  Serial.print(shock_value);
  Serial.println();

  BTSerial.print( "deltha : " );
  BTSerial.print(deltha);
  BTSerial.println();
  BTSerial.print("shock_value : ");
  BTSerial.print(shock_value);
  BTSerial.println();
  }
  delay(100);
  //delay(delay_main);
}
// 비상함수 - LED ON, 부저 ON
void Emergency_state_(){
  Serial.println( "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" );
  Serial.println( " 낙상이 감지되었습니다 " );
  Serial.print( " 가속도 변화량 : " );
  Serial.print(deltha);
  Serial.println();
  Serial.print(" 현재 진동 값 : ");
  Serial.print(shock_value);
  Serial.println();

  BTSerial.println( " 낙상이 감지되었습니다 " );
  BTSerial.print( " 가속도 변화량 : " );
  BTSerial.print(deltha);
  BTSerial.println();
  BTSerial.print(" 현재 진동 값 : ");
  BTSerial.print(shock_value);
  BTSerial.println();

  digitalWrite(Pin_Relay , LOW); // 릴레이핀을 Low시켜서 LED ON시킨다.
}
// 연산 변수 초기화 함수
void value_init(){ // 연산 변수들을 초기화 시켜줌
  normal_x = 0; normal_x = 0; normal_x = 0;
  for (int i=0; i < 3; i++){ deltha_x[i]=0; deltha_y[i]=0; deltha_z[i] = 0; shock = 0; shock_value=0;}
}

// 가속도 센서(mpu6050) 초기설정 함수
void mpu6050_init(){
  Wire.begin(); //I2C통신 시작
  Wire.beginTransmission(mpu_add) ; // 0x68(주소) 찾아가기
  Wire.write(0x6B) ; // 초기 설정 제어 값
  Wire.write(0) ;
  Wire.endTransmission(true) ;
}

void accel_calculate() {
 
 
  ac_x = 0; ac_y = 0; ac_z = 0;
  normal_x = 0; normal_x = 0; normal_x = 0;

  Wire.beginTransmission(mpu_add) ; // 번지수 찾기
  Wire.write(0x3B) ; // 가속도 데이터 보내달라고 컨트롤 신호 보내기
  Wire.endTransmission(false) ; // 기달리고,
  Wire.requestFrom(mpu_add, 6, true) ; // 데이터를 받아 처리
 

  // Data SHIFT
  ac_x = Wire.read() << 8 | Wire.read() ;
  ac_y = Wire.read() << 8 | Wire.read() ;
  ac_z = Wire.read() << 8 | Wire.read() ;
 
 
//맵핑화 시킨 것 - 즉 10000으로 맵핑시킴
  normal_x = map(int(ac_x), -16384, 16384, 0, mapping_value);
  normal_y = map(int(ac_y), -16384, 16384, 0, mapping_value);
  normal_z = map(int(ac_z), -16384, 16384, 0, mapping_value);
 
 
//각도계산 deg -> 각도
  deg = atan2(ac_x, ac_z) * 180 / PI ; //rad to deg
  dgy_x = gy_y / 131. ; //16-bit data to 250 deg/sec
  shock = analogRead(pin);
 
}
