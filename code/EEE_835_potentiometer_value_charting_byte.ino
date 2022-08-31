int in = A0;

void setup() {
  Serial.begin(115200);
  pinMode(in,INPUT);

}

void loop() {
                                                      //Map the value from 10 bits to 8 bits:
  byte val = map(analogRead(in),0,1024.0,0,255);    //Byte = 8 bits --> 2^8 = 255  
  Serial.write(val);                                //Serial.write will send only one byte at a time  
  delay(400);                                        //Small delay between each data send  
}
