int LED_R = 4;  // 빨간색 LED핀
int LED_G = 3;  // 초록색 LED핀
int LED_B = 2;  // 파란색 LED핀
void setup()  // 초기화
{
  pinMode(LED_R, OUTPUT);  // 빨간색 LED 출력 설정
  pinMode(LED_G, OUTPUT);  // 초록색 LED 출력 설정
  pinMode(LED_B, OUTPUT);  // 파란색 LED 출력 설정
}
//rpb_on(빨간색 농도, 초록색 농도, 파란색 농도, 유지시간)
void rgb_on(int R, int G, int B, int ontime)
{
  analogWrite(LED_R, R);  // 빨간색 LED핀에 PWM신호 출력
  analogWrite(LED_G, G);  // 초록색 LED핀에 PWM신호 출력
  analogWrite(LED_B, B);  // 파란색 LED핀에 PWM신호 출력
  delay(ontime);  // 입력한 유지시간 만큼 지속
  rgb_off();  // LED 꺼짐
}
void rgb_off()  // LED OFF 함수
{
  analogWrite(LED_R, 0);  // 빨간색 LED핀에 출력 0
  analogWrite(LED_G, 0);  // 초록색 LED핀에 출력 0
  analogWrite(LED_B, 0);  // 파란색 LED핀에 출력 0
}
void loop()  // 무한루프
{
  rgb_on(255, 0, 0, 2000);  // 빨, 2초간 유지
  rgb_on(255, 5, 0, 2000);  // 주, 2초간 유지
  rgb_on(255, 50, 0, 2000);  // 노, 2초간 유지
  rgb_on(0, 100, 0, 2000);  // 초, 2초간 유지
  rgb_on(0, 0, 100, 2000);  // 파, 2초간 유지
  rgb_on(0, 0, 50, 2000);  // 남, 2초간 유지
  rgb_on(150, 10, 120, 2000);  // 보, 2초간 유지
}

