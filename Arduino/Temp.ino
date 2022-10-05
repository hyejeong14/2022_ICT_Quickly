#include <LiquidCrystal_I2C.h>  // LiquidCrystal_I2C 헤더파일 호출
LiquidCrystal_I2C lcd(0x3F, 16, 2); // LCD의 address 주소 및 크기 입력
void setup()  // 초기화
{
  lcd.init();  // LCD 초기화
  lcd.backlight();  // LCD 백라이트 함수
}
void loop()  // 무한루프
{
  lcd.setCursor(0, 0); // 커서 첫번째칸 첫번재줄 위치
  lcd.print("Welcome to");  // Welcome to 문자 출력
  lcd.setCursor(3, 1); // 커서 네번째칸 두번째줄 위치
  lcd.print("the LK world!");  // the LK world! 문자 출력
}
