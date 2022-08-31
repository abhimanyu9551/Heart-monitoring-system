
int UpperThreshold = 2000;
int LowerThreshold = 1900;
int reading = 0;
float BPM = 0.0;
bool IgnoreReading = false;
bool FirstPulseDetected = false;
unsigned long FirstPulseTime = 0;
unsigned long SecondPulseTime = 0;
unsigned long PulseInterval = 0;
//int in=A0;

#include <SPI.h>         //library for SPI communications
#include <Mcp3208.h>     //library for MCP3208 
#define SPI_CS   10      // define the pin for ChipSelect(CS)/SlaveSelect(SS) (Active LOW)
#define ADC_VREF 5000    // define the ADC Vreference voltage
#define ADC_CLK 16000000  // SPI clock 16.0 MHz
#include <avr/io.h>      
//#include <avr/interrupt.h>
//#define interruptPin 2
//#include <avr/sleep.h>
#include <SoftwareSerial.h>

MCP3208 adc (ADC_VREF, SPI_CS);   //pass on the values of ADC_VREF and SPI_CS to adc function 
SoftwareSerial SerialBT (5, 6); //Rx, Tx 
                                // RX is digital pin 5 (connect to TX of other device)
                                //TX is digital pin 6 (connect to RX of other device)



//
//int read_adc = Samples + 1;


void setup()   // put your setup code here, to run once:
{

  //mcp3208 library
  pinMode (SPI_CS, OUTPUT); //set the CS/SS as OUTPUT
  digitalWrite (SPI_CS, HIGH); //set the CS/SS as HIGH initially so no spurious data transfer
  //pinMode(in, INPUT);
  Serial.begin(115200);  //set baud rate for serial tranfser

  
  SPISettings mysettings (2000000, MSBFIRST, SPI_MODE0); // The SPISettings object is used to configure the SPI port for your SPI device with mySettting(Maximum_device_speed, dataOrder, dataMode)
  SPI.begin();                        //initialise SPI
  SPI.beginTransaction(mysettings);  //All 3 parameters from mysettings are combined to a single SPISettings object, which is given to SPI.beginTransaction().




//30s interrupt
//  cli();                          //disable all the interrupts 
//  pinMode (interruptPin, INPUT_PULLUP);
//  attachInterrupt (digitalPinToInterrupt(interruptPin), isr_start, LOW); 
//  
//                                  //Timer Interrupt
//  TCCR1A = 0;                     // set entire TCCR1A register to 0
//  TCCR1B = 0;                     // set entire TCCR1B register to 0
//  TIMSK1 |= (1 << TOIE1);         // enable timer overflow interrupt ISR
//  TCCR1B |=(1<<CS11);             //Set bit CS12 in TCCR1
//  TCNT1 = 0xD8F0;
//  sei();                          //enables all interrupts again
//  
  SerialBT.begin (115200);
//  
//  sleep_enable(); 
//  set_sleep_mode (SLEEP_MODE_PWR_DOWN);
//  sleep_cpu();  
}

void loop()  // put your main code here, to run repeatedly:
{
  uint16_t raw = analogRead(A0);     //read the raw ADC on channel 2 in SINGLE ENDED MODE 
  //uint16_t reading = adc.toAnalog(raw);               //get analog value

  //Serial.print ("value: ");
  //Serial.print (raw);
  //Serial.print (" (");
  //Serial.print (val);
  //Serial.println (" mV)");
 // delay(1000);
 //reading = analogRead(0); 
 //reading=val;

// Heart beat leading edge detected.
      if(raw > UpperThreshold && IgnoreReading == false)
      {
        if(FirstPulseDetected == false)
        {
          FirstPulseTime = millis();
          FirstPulseDetected = true;
        }
        else
        {
          SecondPulseTime = millis();
          PulseInterval = SecondPulseTime - FirstPulseTime;
          FirstPulseTime = SecondPulseTime;
        }
        IgnoreReading = true;
      }
      
// Heart beat trailing edge detected.
      if(raw < LowerThreshold)
      {
        IgnoreReading = false;
      }  
      BPM = (1.0/PulseInterval) * 60.0 * 1000;
      //Serial.print(reading);
      //Serial.print("\t");
      //Serial.print(FirstPulseTime);
      //Serial.print("\t");
     // Serial.print(SecondPulseTime);
      //Serial.print("\t");
      //Serial.print(PulseInterval);
      //Serial.print("\t");
      //Serial.println(BPM);
      //BPM = map(BPM, 0, 4095.0, 0, 255);
      byte reading_type = map(raw,0, 1023.0, 0, 255);
      //byte BPM_byte=map(BPM,0,1024.0,0,255);
      Serial.write(reading_type);
      //Serial.println(" BPM");
      //Serial.flush();
      // Please don't use delay() - this is just for testing purposes.
      delay(10);  
  
}


//ISR(TIMER1_OVF_vect)
//{
// TCNT1 = 0xD8F0;  
// if (read_adc < Samples)
// {
//  uint16_t raw = adc.read(MCP3208::SINGLE_2);     //read the raw ADC on channel 2 in SINGLE ENDED MODE 
//  uint16_t val = adc.toAnalog(raw);               //get analog value
//
////  Serial.print ("value: ");
////  Serial.print (raw);
////  Serial.print (" (");
////  Serial.print (val);
////  Serial.println (" mV)");
////  read_adc+=1;
// //}
// //TCNT1 = 0xD8F0;  
// //if (read_adc < Samples)
// //{
//  //uint16_t raw = adc.read(MCP3208::SINGLE_2);     //read the raw ADC on channel 2 in SINGLE ENDED MODE 
//  //uint16_t val = adc.toAnalog(raw);               //get analog value
//val = map((val), 0, 4095.0, 0, 255);
////  while (Serial.available()>0) 
////  {
////  Serial.print(val);
////  Serial.write(SerialBT.read());
////  Serial.write("\n");
////  SerialBT.write(SerialBT.read());
////  SerialBT.write("\n");
////  }
  
//  Serial.print ("value: ");
//  Serial.print (raw);
//  Serial.print (" (");
//  Serial.print (val);
//  Serial.println (" mV)");
//  SerialBT.write((byte)val);
//  //SerialBT.write('/n');
//  read_adc+=1;
//
// 
//    
// } else 
// {
//  sleep_cpu();  
// }
//}
//
//
//void isr_start() 
//{
//   sleep_disable();
//   read_adc=0;
//}
