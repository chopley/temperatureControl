#include <OneWire.h>
#include <PID_v1.h>
#include <Metro.h>
#define RelayPin 11



#define LEDPin 4
double Setpoint, Input, Output;

double errorVal,errorWindow; //to keep the temperature error
int errorState,ledState; // state of the error (either within athe window or not)

//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint,50,0.1,0, DIRECT);

//int WindowSize = 10000;
//unsigned long windowStartTime;




volatile int HighByte, LowByte, TReading, SignBit, Tc_100, Whole, Fract;
volatile int requestFlag;
volatile int readTimerCounter=0;
unsigned long Time[5];
double Temps[5],dRate;


byte present = 0;
byte data[12];
byte addr[8];
double error;
volatile double Temperature;
/* DS18S20 Temperature chip i/o
 
 */

OneWire  ds(10);  // on pin 10
//int Heater=11;
Metro requestTimer(1000);
Metro readTimer(100);






void requestTemps(){
  byte i;
  if ( !ds.search(addr)) {
    //  Serial.print("No more addresses.\n");
      ds.reset_search();
      return;
  }
  
  if ( OneWire::crc8( addr, 7) != addr[7]) {
      Serial.print("CRC is not valid!\n");
      return;
  }
  ds.reset();
  ds.select(addr);
  ds.write(0x44,1); 
  requestFlag=1;
  
}

double readTemps(){
  char i;
  double Temp;
  
  present = ds.reset();
  ds.select(addr);    
  ds.write(0xBE);         // Read Scratchpad

  
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();

  }

  LowByte = data[0];
  HighByte = data[1];
  TReading = (HighByte << 8) + LowByte;
  SignBit = TReading & 0x8000;  // test most sig bit
  if (SignBit) // negative
  {
    TReading = (TReading ^ 0xffff) + 1; // 2's comp
  }
  Tc_100 = (6 * TReading) + TReading / 4;    // multiply by (100 * 0.0625) or 6.25

  Whole = Tc_100 / 100;  // separate off the whole and fractional portions
  Fract = Tc_100 % 100;


  if (SignBit) // If its negative
  {
     Serial.print("-");
  }
  
  Temp=(float)Whole+(float)Fract/100;
 // Serial.print(Temp);
  return Temp;
  
}

void setup(void) {
  // initialize inputs/outputs
  // start serial port
  //windowStartTime = millis();
  Setpoint = 40;
  errorWindow=0.5;
  ledState=0; //start the indicator LED in off mode
  int jj;
  

  Serial.begin(9600);
    //tell the PID to range between 0 and the full window size
  myPID.SetOutputLimits(0, 255);
  myPID.SetSampleTime(100) ;
  //turn the PID on
  myPID.SetMode(AUTOMATIC);
  //myPID.SetTunings(100,0.1,0);
  pinMode(RelayPin,OUTPUT);

  
  pinMode(LEDPin,OUTPUT);
 
}

void loop(void) {
    
    double dTemp,dTime,dTimeLoop;
    unsigned long currTime;
    unsigned long TimeDiff,TimeDiffLoop;
    //char Output;
    int j;
   
    
  

  if(requestTimer.check()==1){
    //request temperatures every 1 second
       requestTemps();
       readTimerCounter=0; 
       if(errorState==0){
         if(ledState==0){
          ledState=1; 
         }
         else if(ledState==1){
          ledState=0; 
         }
         //we need to blink the LED
      }
        else if(errorState==1){
          ledState=1;
         //we don't need to blink the LED
        }
        digitalWrite(LEDPin,ledState); //write the value and 
  }
  if(readTimer.check()==1){
    readTimerCounter++;
    if(readTimerCounter>=10){
       readTimerCounter==0;
       
      
       Time[4]=Time[3];
       Time[3]=Time[2];
       Time[2]=Time[1];
       Time[1]=Time[0];
       Time[0]=millis();
       
       Temps[4]=Temps[3];
       Temps[3]=Temps[2];
       Temps[2]=Temps[1];
       Temps[1]=Temps[0];
       Temps[0]=readTemps(); //read the latest temperature
       TimeDiff = Time[0]-Time[1];
       dTemp=Temps[0]-Temps[1];
       dRate=100000.*dTemp/(double)TimeDiff;
       //dRate=dRate.;
     
       

  

 // Serial.print("\n");
       
       
        }
    
    
   // dTemp=Temps[0]-Temps[1];
   // dTime=(double)currTime-(double)Time[1];
    Serial.print(Input);
  Serial.print(",");
  Serial.print(Setpoint);
  Serial.print(",");
  Serial.print(Output);
  Serial.print(",");
  Serial.print(dRate);
  Serial.print(",");
  
  for(j=0;j<=7;j++){
    Serial.print(addr[j],HEX);
    Serial.print(" ");
  } 
  errorVal=abs(Setpoint-Input);
  if(errorVal<errorWindow){
   errorState=1 ;
  }
  else if(errorVal>=errorWindow){
   errorState=0; 
  }
  
//  Serial.print("\t");

//  Serial.print("\t");
//  Serial.print(currTime);
//  Serial.print("\t");
//  Serial.print(dTimeLoop);
//   Serial.print("\t");
//  Serial.print(dRate);
  Serial.print("\n");

    
    
  }

 
  

  currTime=millis();
  TimeDiffLoop=currTime-Time[0];
  dTimeLoop=(double)TimeDiffLoop;
  Temperature=Temps[1] + dTimeLoop*dRate/100000.;
  
  Input = Temperature;
  error=Temperature-Setpoint;
  myPID.Compute();
  
  analogWrite(RelayPin,Output);
  
}
