void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  delay(200);
  
  Serial.println(F("Hello, I am a carbon sensor"));
}

void loop() {
  // put your main code here, to run repeatedly:
  Serial.println(F("Hello, I am a carbon sensor. Again."));
  delay(3000);
}
