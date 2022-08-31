#define R0 10000   //initial value of R0 = 10kohms
#define B 3977     //K
#define VCC 5      // 5V
#define R 10000    // Resistance in series with thermistor

float RT, VR, ln, T, T0, VRT;  //variable


void setup()
{
  T0 = 25 + 273.15; 
  pinMode (A0, INPUT);
  pinMode (A1, INPUT);
  Serial.begin(115200);
}

void loop()
{
  
  int pot1 = analogRead (A0);
  int val = map(pot1,0.0,1024.0,0,255);
  
  int rawTherm = analogRead (A1);
  VRT = (5.00 / 1023.00)*rawTherm;         //conversion to voltage
  VR = VCC - VRT;
  RT = VRT / (VR/R);                //Resistance of RT

  ln = log (RT/R0);
  T = (1/((ln/B) + (1/T0)));        //Temperature from THermistor
  T = T-273.15;                     //Conversion to oC again from K
  

  Serial.print ("RESULT|");
  Serial.print(val);
  Serial.print("|");
  Serial.println(T);
  delay (200);
}
