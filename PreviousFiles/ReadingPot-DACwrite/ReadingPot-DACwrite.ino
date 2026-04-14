



void setup() {
  // initialize serial communication at 115200 bits per second:
  Serial.begin(9600);

  //set the resolution to 12 bits (0-4095)
  analogReadResolution(12);
}

void loop() {
  // read the analog / millivolts value for pin 2:
  int sensorval = analogRead(36);
  float analogValue = sensorval * 3.3/4095.0;
  int LEDoutput = map(sensorval, 0, 4095, 0, 255);
  //int analogVolts = analogReadMilliVolts(2);

  // print out the values you read:
  Serial.printf("ADC analog value = %f\n", analogValue);
  //Serial.printf("ADC millivolts value = %d\n", analogVolts);
  Serial.println(analogValue);

  dacWrite(25, LEDoutput);
  
  delay(100);  // delay in between reads for clear read from serial
}
