#include <MPU6050_tockn.h>

#include<Wire.h>
#include <SoftwareSerial.h>                // Serial 통신을 하기 위해 선언
const int MPU_addr=0x68;  // I2C address of the MPU-6050
//int16_t AcX,AcY,AcZ,Tmp,GyX,GyY,GyZ;
int pin = A0;
SoftwareSerial BTSerial(6, 7);             // HC-06모듈 6=TXD , 7=RXD 핀 선언 블루투스

MPU6050 mpu(Wire);

float bx=0.0;
float by=0.0;
float bz=0.0;

void setup(){
  mpu.begin();
  Serial.begin(9600);
  BTSerial.begin(9600);                     // HC-06 모듈 통신 선언 (보드레이트 9600) 블루투스 
  //Wire.begin();
  
  mpu.setGyroOffsets(26.11,-70.08,18.75);
 /* Wire.beginTransmission(MPU_addr);
  Wire.write(0x6B);  // PWR_MGMT_1 register
  Wire.write(0);     // set to zero (wakes up the MPU-6050)
  Wire.endTransmission(true);*/

  bx=mpu.getAngleX();
  by=mpu.getAngleY();
  bz=mpu.getAngleZ();
 
}
void loop(){

  if(BTSerial.available())  {                   // BTSerial에 입력이 되면 블루투스
  Serial.write(BTSerial.read());           // BTSerial에 입력된 값을 시리얼 모니터에 출력
  }
  if(Serial.available())    {                      // 시리얼 모니터에 입력이 되면
  BTSerial.write(Serial.read());           // 그 값을 BTSerial에 출력
  }  //여기까지 블루투스
  int val = analogRead(pin);
  Serial.println(val);
  if(val<512){
  Serial.write("SHOCK!!\n");
  BTSerial.println("충돌 낙상");
  BTSerial.println();
 
   }
  delay(50);

  mpu.update();

  float ax=mpu.getAngleX();
  float ay=mpu.getAngleY();
  float az=mpu.getAngleZ();

  float x = ax-bx;
  float y = ay-by;
  float z = az-bz;

  bx = mpu.getAngleX();
  by = mpu.getAngleY();
  bz = mpu.getAngleZ();
  

  Serial.print(x);
  Serial.print(" ");
  Serial.print(y);
  Serial.print(" ");
  Serial.print(z);
  Serial.println();
  BTSerial.print("변화값 : ");
  BTSerial.print(x);
  BTSerial.print(" ");
  BTSerial.print(y);
  BTSerial.print(" ");
  BTSerial.print(z);
  BTSerial.println();
  delay(50);
  /*Wire.beginTransmission(MPU_addr);
  Wire.write(0x3B);  // starting with register 0x3B (ACCEL_XOUT_H)
  Wire.endTransmission(false);
  Wire.requestFrom(MPU_addr,14,true);  // request a total of 14 registers
  AcX=Wire.read()<<8|Wire.read();  // 0x3B (ACCEL_XOUT_H) & 0x3C (ACCEL_XOUT_L)    
  AcY=Wire.read()<<8|Wire.read();  // 0x3D (ACCEL_YOUT_H) & 0x3E (ACCEL_YOUT_L)
  AcZ=Wire.read()<<8|Wire.read();  // 0x3F (ACCEL_ZOUT_H) & 0x40 (ACCEL_ZOUT_L)
  Tmp=Wire.read()<<8|Wire.read();  // 0x41 (TEMP_OUT_H) & 0x42 (TEMP_OUT_L)
  GyX=Wire.read()<<8|Wire.read();  // 0x43 (GYRO_XOUT_H) & 0x44 (GYRO_XOUT_L)
  GyY=Wire.read()<<8|Wire.read();  // 0x45 (GYRO_YOUT_H) & 0x46 (GYRO_YOUT_L)
  GyZ=Wire.read()<<8|Wire.read();  // 0x47 (GYRO_ZOUT_H) & 0x48 (GYRO_ZOUT_L)
  Serial.print("AcX = "); Serial.print(AcX);
  Serial.print(" | AcY = "); Serial.print(AcY);
  Serial.print(" | AcZ = "); Serial.print(AcZ);
  Serial.print(" | Tmp = "); Serial.print(Tmp/340.00+36.53);  //equation for temperature in degrees C from datasheet
  Serial.print(" | GyX = "); Serial.print(GyX);
  Serial.print(" | GyY = "); Serial.print(GyY);
  Serial.print(" | GyZ = "); Serial.println(GyZ);
  delay(333);*/
  
}
