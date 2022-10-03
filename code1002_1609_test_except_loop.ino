#include <Arduino.h> 
#include <Wire.h> 
#include <SoftwareSerial.h>
// #include "MPU6050.h"
// MPU6050 mpu;
SoftwareSerial BTSerial(6,7);

int ACC_ANGLE = 50;
long CRASH_VALUE = 500;
int SHOCK_VALUE = 300;

long ac_x, ac_y, ac_z, gy_x, gy_y, gy_z ; //acc, gyro data (acc, gyro 계산 수식)
double angle1 = 0; // angle, deg data (각도계산)
double angle2 = 0;
double dgy_x ; // double type acc data
long mapping_value = 1000;
long  normal_x,normal_y,normal_z, deltha_x[3],deltha_y[3],deltha_z[3], deltha, angle_1[3], angle_2[3] ; // 노말라이즈(정규화 데이터), 가속도 변화량 표시
long event_value = 1000;

const int delay_main = 1; // 전체 시스템 구동하는 데, 걸리는 시간 (즉 구동 시간)이다. (ex) 1000 = 1000ms = 1s = 1초
const int sum_count = 4;
long delay_config = 6000; // 이벤트 발생 후 딜레이
boolean ACCESS_ACCEL = false;
long TIEMR_BLESEND = 0;
bool WARN_STATE = false;
long TIMER_WARN_STATE = 0;

void value_init(); 
void accel_calculate(); 
void Emergency_state_();
int ACTION_config();
void mpu6050_init();

void setup()
{
  Serial.begin(9600); // 컴푸터와 시리얼통신을 하기 위한 것
  BTSerial.begin(9600);
  pinMode(12, OUTPUT);
  digitalWrite(12, LOW); // RESET 초기화

  delay(100);
  mpu6050_init(); // 가속도 센서 초기 설정
  // ACCESS_ACCEL = ACCEL_ACCESS(); // 가속도 센서 값 맞게 나오는지 확인하기
}

// #6. Loop 문 (반복 문) 시스템 구동
void loop()
{
  int shock = analogRead(A0);
  if(TIEMR_BLESEND < millis()){
    BTSerial.print('a'); // 블루투스 데이터 보내기
    TIEMR_BLESEND = millis() + 10000; // 10초 간격으로 데이터 보내기
  }

  // 2. 블루투스 데이터 감지하기
  if (BTSerial.available()){ // 블루투스 데이터를 감지했을 경우 안에 while문이 동작함
    int timeout = 30;
    String response = ""; 
    long int time = millis();
    while( (time+timeout) > millis()) { // 50ms동안 문자열 세는 방식
      while(BTSerial.available()) { // 블루투스 통신이 끝날 때 까지
      char c = BTSerial.read(); // c에 읽어 response에 저장
      response+=c; }
      }

  String DATA = response; // 컨트롤 데이터에 읽은 데이터 넣기
  Serial.println( 'DATA : ' + DATA); // 컨트롤 데이터 출력

  if(DATA.substring(0,2) == 'HI'){ // 블루투스 데이터가 “HI”인가? 그럼 ‘a’데이터 보내기
  BTSerial.print('CONNECT');}
  }
}

if(WARN_STATE){
// 0.5초마다 확인 하기
  if(TIMER_WARN_STATE < millis()) { 
    TIMER_WARN_STATE = millis() + 100;
    // 각도 측정하기
    value_init(); // 변수 초기화
    accel_calculate(); // 가속도 측정 -> 각도계산
    angle1 = abs(angle1); // 각도 변수에 저장
    angle2 = abs(angle2); // 각도 변수에 저장

    if ( (angle1 < ACC_ANGLE ) && (angle2 < ACC_ANGLE ) ){ // 정상 상태에 돌아왔을 경우 (각도가 안정상태일 경우)
      WARN_STATE = false; // 비상모드 끄기
      BTSerial.print('x'); // 블루투스 데이터 ‘x’보내기
    }
  }
}

value_init();
//첫번째 센싱
for (int i=0; i < sum_count; i++){ accel_calculate();
deltha_x[1] = deltha_x[1]+(normal_x); deltha_y[1] = deltha_y[1]+(normal_y); deltha_z[1] = deltha_z[1]+(normal_z); delay(5);}
deltha_x[1] = int(deltha_x[1]/sum_count); deltha_y[1] = int(deltha_y[1]/sum_count); deltha_z[1] = int(deltha_z[1]/sum_count);

//두번째 센싱
for (int i=0; i < sum_count; i++){ accel_calculate();
deltha_x[2] = deltha_x[2]+(normal_x); deltha_y[2] = deltha_y[2]+(normal_y); deltha_z[2] = deltha_z[2]+(normal_z); delay(5);}
deltha_x[2] = int(deltha_x[2]/sum_count); deltha_y[2] = int(deltha_y[2]/sum_count); deltha_z[2] = int(deltha_z[2]/sum_count); //3축 변화량 비교가속도 변화량 측정하기
deltha_x[0] = abs(deltha_x[1]-deltha_x[2]); deltha_y[0] = abs(deltha_y[1]-deltha_y[2]); deltha_z[0] = abs(deltha_z[1]-deltha_z[2]);
deltha = deltha_x[0] + deltha_y[0] + deltha_z[0];

if(deltha > CRASH_VALUE){ // 충격이 감지 되었을 경우 내부 함수 동작하기
  ACTION_config(); // 기울기를 통해 비상모드인지 판단함
}


// 비상모드인지 판단하는 함수
int ACTION_config(){
  long TIMER1 = (long) millis() + 3000; // 
  int WARN = 0;

// 비상모드 인가? 정상모드 인가? 판단하기
  while( (TIMER1 > millis())&&(WARN == 0)){
    value_init(); 
    accel_calculate();
    angle1 = abs(angle1);
    angle2 = abs(angle2);

  // 정해진 각도 내에 들어가는지 확인하기 (12초 내의 정해진 각도에 들어 갈 경우 정상임)
    if ( (angle1 < NOMAL_ANGLE ) && (angle2 < NOMAL_ANGLE ) ){
      WARN = 1; // WARN = 1이면 while에 나갈 수 있음
    }

    Serial.print( ' ANGLE1 : ' + String(angle1)); 
    Serial.println( ' ANGLE2 : ' + String(angle2));
    delay(100);
    }
  //—————————————//

  // 정상 일 경우 return 시키기
  if(WARN == 1) { return 0; }

  // 비상모드 일 경우 부저 6초간 울리고 블루투스 데이터 보내기
  BTSerial.print('낙상입니다'); // ‘w’ 블루투스데이터 보내기  
  }

void accel_calculate() {

  ac_x = 0; ac_y = 0; ac_z = 0;

  Wire.beginTransmission(mpu_add) ; // 번지수 찾기
  Wire.write(0x3B) ; // 가속도 데이터 보내달라고 컨트롤 신호 보내기
  Wire.endTransmission(false) ; // 기달리고,
  Wire.requestFrom(mpu_add, 6, true) ; // 데이터를 받아 처리

  // Data SHIFT
  ac_x = Wire.read() << 8 | Wire.read() ;
  ac_y = Wire.read() << 8 | Wire.read() ;
  ac_z = Wire.read() << 8 | Wire.read() ;

  // Serial.println( “ X : “ + String(ac_x) + “ Y : “ + String(ac_y) + “ Z : “ + String(ac_z) );
  if(ACCESS_ACCEL) { } else { return; }

  //맵핑화 시킨 것 – 즉 10000으로 맵핑시킴
  normal_x = map(int(ac_x), -16384, 16384, 0, mapping_value);
  normal_y = map(int(ac_y), -16384, 16384, 0, mapping_value);
  normal_z = map(int(ac_z), -16384, 16384, 0, mapping_value);

  float accel_xz, accel_yz;
  const float RADIANS_TO_DEGREES = 180/3.14159;

  // 각도1 계산하기
  accel_yz = sqrt(pow(ac_x, 2) + pow(ac_y, 2));
  angle1 = atan(-ac_z / accel_yz)*RADIANS_TO_DEGREES;

  // 각도2 계산하기
  accel_xz = sqrt(pow(ac_z, 2) + pow(ac_y, 2));

void value_init(){
  normal_x = 0; normal_x = 0; normal_x = 0;
  for (int i=0; i < 3; i++)
    { deltha_x[i]=0; deltha_y[i]=0; deltha_z[i] = 0; angle1 = 0;angle2 = 0; }
}

//   //BTSerial.print("안녕");
//   exit(0);
//   if (angle_init==9999){
//     angle_init=0;
//   }
//   angle_init=angle1;
//   State_Parameter = false;
//   value_init(); //가속도-각도 관련 초기값 선언

//   // if(BTSerial.available())  {                   // BTSerial에 입력이 되면 블루투스
//   //   Serial.write(BTSerial.read());           // BTSerial에 입력된 값을 시리얼 모니터에 출력
//   // }

//   // if(Serial.available())    {                      // 시리얼 모니터에 입력이 되면
//   //   BTSerial.write(Serial.read());           // 그 값을 BTSerial에 출력
//   // }  //여기까지 블루투스b 
//   accel_calculate();
//   angle1 = abs(angle1);
//   minus_angle=abs(angle1-angle_init);
//   //angle2 = abs(angle2);
//   //첫번째 센싱
//   for (int i=0; i < sum_count; i++){ accel_calculate();
//   deltha_x[1] = deltha_x[1]+(normal_x); deltha_y[1] = deltha_y[1]+(normal_y); deltha_z[1] = deltha_z[1]+(normal_z); shock_value = shock;}
//   deltha_x[1] = int(deltha_x[1]/sum_count); deltha_y[1] = int(deltha_y[1]/sum_count); deltha_z[1] = int(deltha_z[1]/sum_count);

//   //두번째 센싱
//   for (int i=0; i < sum_count; i++){ accel_calculate();
//   deltha_x[2] = deltha_x[2]+(normal_x); deltha_y[2] = deltha_y[2]+(normal_y); deltha_z[2] = deltha_z[2]+(normal_z); shock_value = shock;}
//   deltha_x[2] = int(deltha_x[2]/sum_count); deltha_y[2] = int(deltha_y[2]/sum_count); deltha_z[2] = int(deltha_z[2]/sum_count);
 
//   //3축 변화량 비교 - 가속도 변화량, 각도 평균 값
//   deltha_x[0] = abs(deltha_x[1]-deltha_x[2]); deltha_y[0] = abs(deltha_y[1]-deltha_y[2]); deltha_z[0] = abs(deltha_z[1]-deltha_z[2]);
//   deltha = deltha_x[0] + deltha_y[0] + deltha_z[0];
//   shock_value = abs(int(shock_value));

//   // deltha : 가속도 변화량
//   // shock_vlaue: 진동값State_Parameter
//   // if (deltha > Emergency_value){State_Parameter=false;}
//   if (shock_value < Emergency_shock){State_Parameter=true;}
//   // if ((deltha > Emergency_value2)&&(shock_value < Emergency_shock)&&(minus_angle<50)){State_Parameter=true;}
//   if (minus_angle<50){State_Parameter=true;}
//   //if ((angle1 > NOMAL_ANGLE ) && (angle2 > NOMAL_ANGLE )) {State_Parameter=true;}
 
//   // State_Parameter - 비상이냐? 정상이냐 상태 변수
//   if( State_Parameter == true ){Emergency_state_();}else{
//   Serial.print( "deltha : " );
//   Serial.print(deltha);
//   Serial.println();
//   Serial.print("shock_value : ");
//   Serial.print(shock_value);
//   Serial.println();
//   Serial.print("minus_angle : ");
//   Serial.print(minus_angle);
//   Serial.println();
//   //Serial.print("angle2 : ");
//   //Serial.print(angle2);
//   Serial.println();

//   BTSerial.print( "deltha : " );
//   BTSerial.print(deltha);
//   BTSerial.println();
//   BTSerial.print("shock_value : ");
//   BTSerial.print(shock_value);
//   BTSerial.println();
//   BTSerial.print("minus_angle : ");
//   BTSerial.print(minus_angle);
//   BTSerial.println();
//  // BTSerial.print("angle2 : ");
//   //BTSerial.print(angle2);
//   BTSerial.println();
//   }
//   delay(100);
//   //delay(delay_main);
// }
// // 비상함수 - LED ON, 부저 ON
// void Emergency_state_(){
//   Serial.println( "!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!" );
//   Serial.println( " 낙상이 감지되었습니다 " );
//   Serial.print( " 가속도 변화량 : " );
//   Serial.print(deltha);
//   Serial.println();
//   Serial.print(" 현재 진동 값 : ");
//   Serial.print(shock_value);
//   Serial.println();
//   Serial.print("angle1 : ");
//   Serial.print(minus_angle);
//   Serial.println();
//  // Serial.print("angle2 : ");
//   //Serial.print(angle2);
//   Serial.println();

//   BTSerial.println( " 낙상이 감지되었습니다 " );
//   BTSerial.print( " 가속도 변화량 : " );
//   BTSerial.print(deltha);
//   BTSerial.println();
//   BTSerial.print(" 현재 진동 값 : ");
//   BTSerial.print(shock_value);
//   BTSerial.println();
//   BTSerial.print("각도1 : ");
//   BTSerial.print(minus_angle);
//   BTSerial.println();
//   //BTSerial.print("각도2 : ");
//  // BTSerial.print(angle2);
//   BTSerial.println();

//   digitalWrite(Pin_Relay , LOW); // 릴레이핀을 Low시켜서 LED ON시킨다.
// }
// // 연산 변수 초기화 함수
// void value_init(){ // 연산 변수들을 초기화 시켜줌
//   normal_x = 0; normal_x = 0; normal_x = 0;
//   for (int i=0; i < 3; i++){ deltha_x[i]=0; deltha_y[i]=0; deltha_z[i] = 0; shock = 0; shock_value=0;}
// }

// // 가속도 센서(mpu6050) 초기설정 함수
// void mpu6050_init(){
//   Wire.begin(); //I2C통신 시작
//   Wire.beginTransmission(mpu_add) ; // 0x68(주소) 찾아가기
//   Wire.write(0x6B) ; // 초기 설정 제어 값
//   Wire.write(0) ;
//   Wire.endTransmission(true) ;
// }

// void accel_calculate() {
 
 
//   ac_x = 0; ac_y = 0; ac_z = 0;
//   normal_x = 0; normal_x = 0; normal_x = 0;

//   Wire.beginTransmission(mpu_add) ; // 번지수 찾기
//   Wire.write(0x3B) ; // 가속도 데이터 보내달라고 컨트롤 신호 보내기
//   Wire.endTransmission(false) ; // 기달리고,
//   Wire.requestFrom(mpu_add, 6, true) ; // 데이터를 받아 처리
 

//   // Data SHIFT
//   ac_x = Wire.read() << 8 | Wire.read() ;
//   ac_y = Wire.read() << 8 | Wire.read() ;
//   ac_z = Wire.read() << 8 | Wire.read() ;
 
 
// //맵핑화 시킨 것 - 즉 10000으로 맵핑시킴
//   normal_x = map(int(ac_x), -16384, 16384, 0, mapping_value);
//   normal_y = map(int(ac_y), -16384, 16384, 0, mapping_value);
//   normal_z = map(int(ac_z), -16384, 16384, 0, mapping_value);
 
 
// //각도계산 deg -> 각도
//   //deg = atan2(ac_x, ac_z) * 180 / PI ; //rad to deg
//   //dgy_x = gy_y / 131. ; //16-bit data to 250 deg/sec
//   int shock = analogRead(A0);
  
//   float accel_xz, accel_yz;
//   const float RADIANS_TO_DEGREES = 180/3.14159;

//   // 각도1 계산하기
//   accel_yz = sqrt(pow(ac_x, 2) + pow(ac_y, 2));
//   angle1 = atan(-ac_z / accel_yz)*RADIANS_TO_DEGREES;

//   // 각도2 계산하기
//   accel_xz = sqrt(pow(ac_z, 2) + pow(ac_y, 2));
//   //angle2 = atan(-ac_x / accel_xz)*RADIANS_TO_DEGREES;
// }