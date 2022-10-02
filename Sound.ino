int Sound_Sensor = A0;
int Sensor_val;
int count_clap;

void setup() {
  Serial.begin(9600);

}

void loop() {
  Sensor_val = analogRead(Sound_Sensor);
  if(Sensor_val<=28) {
    count_clap++;
    Serial.println(count_clap);
    delay(20);
  }
}
