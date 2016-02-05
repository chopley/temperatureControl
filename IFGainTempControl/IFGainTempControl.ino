#include <OneWire.h>
#include <PID_v1.h>
#include <Metro.h>
#define RelayPin1 5
#define RelayPin2 6
#define RelayPin3 7
#define RelayPin4 8
#define TempPin1 9
#define TempPin2 10
#define TempPin3 11
#define TempPin4 12

#define RelayPin 7
#define TempPin 11

#define LEDPin 4
double Setpoint1, Input1, Output1;
double Setpoint2, Input2, Output2;
double Setpoint3, Input3, Output3;
double Setpoint4, Input4, Output4;


double errorVal,errorWindow; //to keep the temperature error
int errorState,ledState; // state of the error (either within athe window or not)

//Specify the links and initial tuning parameters
PID temp1(&Input1, &Output1, &Setpoint1,20,0.1,0, DIRECT);
PID temp2(&Input2, &Output2, &Setpoint2,20,0.1,0, DIRECT);
PID temp3(&Input3, &Output3, &Setpoint3,20,0.1,0, DIRECT);
PID temp4(&Input4, &Output4, &Setpoint4,20,0.1,0, DIRECT);


//int WindowSize = 10000;
//unsigned long windowStartTime;




volatile int HighByte, LowByte, TReading, SignBit, Tc_100, Whole, Fract;
volatile int requestFlag;
volatile int readTimerCounter=0;
unsigned long Time[5];
double Temps1[5],dRate1;
double Temps2[5],dRate2;
double Temps3[5],dRate3;
double Temps4[5],dRate4;


byte present = 0;
byte data1[12];
byte addr1[8];
byte data2[12];
byte addr2[8];
byte data3[12];
byte addr3[8];
byte data4[12];
byte addr4[8];
double error;
volatile double Temperature;
/* DS18S20 Temperature chip i/o
 
 */

OneWire  ds1(TempPin1);  // on pin 10
OneWire  ds2(TempPin2);  // on pin 10
OneWire  ds3(TempPin3);  // on pin 10
OneWire  ds4(TempPin4);  // on pin 10

//int Heater=11;
Metro requestTimer(1000);
Metro readTimer(100);








void setup(void) {
  // initialize inputs/outputs
  // start serial port
  //windowStartTime = millis();

     Setpoint1=35;
Setpoint2=35;
Setpoint3=35;
Setpoint4=35;
  errorWindow=0.5;
  ledState=0; //start the indicator LED in off mode
  int jj;
  

  Serial.begin(9600);
    //tell the PID to range between 0 and the full window size
  temp1.SetOutputLimits(0, 255);
  temp1.SetSampleTime(100) ;
  //turn the PID on
  temp1.SetMode(AUTOMATIC);
  temp2.SetOutputLimits(0, 255);
  temp2.SetSampleTime(100) ;
  //turn the PID on
  temp2.SetMode(AUTOMATIC);
  temp3.SetOutputLimits(0, 255);
  temp3.SetSampleTime(100) ;
  //turn the PID on
  temp3.SetMode(AUTOMATIC);
  temp4.SetOutputLimits(0, 255);
  temp4.SetSampleTime(100) ;
  //turn the PID on
  temp4.SetMode(AUTOMATIC);
  
  
  //myPID.SetTunings(100,0.1,0);
  pinMode(RelayPin1,OUTPUT);
  pinMode(RelayPin2,OUTPUT);
  pinMode(RelayPin3,OUTPUT);
  pinMode(RelayPin4,OUTPUT);
  
  analogWrite(RelayPin1, 0);
  analogWrite(RelayPin2, 0);
  analogWrite(RelayPin3, 0);
  analogWrite(RelayPin4, 0);
  
  
 
}

void loop(void) {
    
    double dTemp,dTime,dTimeLoop;
    unsigned long currTime;
    unsigned long TimeDiff,TimeDiffLoop;
    //char Output;
    int j;

    
  

  if(requestTimer.check()==1){
    //request temperatures every 1 second
       requestTemps1();
        requestTemps2();
      requestTemps3();
      requestTemps4();
      
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
       
       Temps1[4]=Temps1[3];
       Temps1[3]=Temps1[2];
       Temps1[2]=Temps1[1];
       Temps1[1]=Temps1[0];
       Temps1[0]=readTemps1(); //read the latest temperature
       
        Temps2[4]=Temps2[3];
       Temps2[3]=Temps2[2];
       Temps2[2]=Temps2[1];
       Temps2[1]=Temps2[0];
       Temps2[0]=readTemps2(); //read the latest temperature
       
        Temps3[4]=Temps3[3];
       Temps3[3]=Temps3[2];
       Temps3[2]=Temps3[1];
       Temps3[1]=Temps3[0];
       Temps3[0]=readTemps3(); //read the latest temperature
       
        Temps4[4]=Temps4[3];
       Temps4[3]=Temps4[2];
       Temps4[2]=Temps4[1];
       Temps4[1]=Temps4[0];
       Temps4[0]=readTemps4(); //read the latest temperature
       
       
       TimeDiff = Time[0]-Time[1];
       dTemp=Temps1[0]-Temps1[1];
       dRate1=100000.*dTemp/(double)TimeDiff;
       //dRate=dRate.;
     
       

  

 // Serial.print("\n");
       
       
        }
    
    
   // dTemp=Temps[0]-Temps[1];
   // dTime=(double)currTime-(double)Time[1];
    Serial.print(Input1);
  Serial.print(",");
  Serial.print(Setpoint1);
  Serial.print(",");
  Serial.print(Output1);
  Serial.print(",");
   Serial.print(Input2);
  Serial.print(",");
  Serial.print(Setpoint2);
  Serial.print(",");
  Serial.print(Output2);
  Serial.print(",");
   Serial.print(Input3);
  Serial.print(",");
  Serial.print(Setpoint3);
  Serial.print(",");
  Serial.print(Output3);
  Serial.print(",");
   Serial.print(Input4);
  Serial.print(",");
  Serial.print(Setpoint4);
  Serial.print(",");
  Serial.print(Output4);
  Serial.print(",");
  
//  Serial.print(dRate);
//  Serial.print(",");
//  
//  for(j=0;j<=7;j++){
//    Serial.print(addr1[j],HEX);
//    Serial.print(" ");
//  } 
  
  
  errorVal=abs(Setpoint1-Input1);
  if(errorVal<errorWindow){
   errorState=1 ;
  }
  else if(errorVal>=errorWindow){
   errorState=0; 
  }

  Serial.print("\n");

    
    
  }

 
  

  currTime=millis();
  TimeDiffLoop=currTime-Time[0];
  dTimeLoop=(double)TimeDiffLoop;
  Temperature=Temps1[1] + dTimeLoop*dRate1/100000.;
  
  Input1 = Temps1[0];
  temp1.Compute();
   Input2 = Temps2[0];
  temp2.Compute();
   Input3 = Temps3[0];
  temp3.Compute();
   Input4 = Temps4[0];
  temp4.Compute();
  
  analogWrite(RelayPin1,Output1);
  analogWrite(RelayPin2,Output2);
  analogWrite(RelayPin3,Output3);
  analogWrite(RelayPin4,Output4);
  
 // digitalWrite(RelayPin1, LOW);
 // digitalWrite(RelayPin2, LOW);
 // digitalWrite(RelayPin3, LOW);
//digitalWrite(RelayPin4, LOW);

  
}


void requestTemps1(){
  byte i;
  if ( !ds1.search(addr1)) {
    //  Serial.print("No more addresses.\n");
      ds1.reset_search();
      return;
  }
  
  if ( OneWire::crc8( addr1, 7) != addr1[7]) {
      Serial.print("CRC is not valid!\n");
      return;
  }
  ds1.reset();
  ds1.select(addr1);
  ds1.write(0x44,1); 
  requestFlag=1;
  
}

void requestTemps2(){
  byte i;
  if ( !ds2.search(addr2)) {
    //  Serial.print("No more addresses.\n");
      ds2.reset_search();
      return;
  }
  
  if ( OneWire::crc8( addr2, 7) != addr2[7]) {
      Serial.print("CRC is not valid!\n");
      return;
  }
  ds2.reset();
  ds2.select(addr2);
  ds2.write(0x44,1); 
  requestFlag=1;
  
}

void requestTemps3(){
  byte i;
  if ( !ds3.search(addr3)) {
    //  Serial.print("No more addresses.\n");
      ds3.reset_search();
      return;
  }
  
  if ( OneWire::crc8( addr3, 7) != addr3[7]) {
      Serial.print("CRC is not valid!\n");
      return;
  }
  ds3.reset();
  ds3.select(addr3);
  ds3.write(0x44,1); 
  requestFlag=1;
  
}

void requestTemps4(){
  byte i;
  if ( !ds4.search(addr4)) {
    //  Serial.print("No more addresses.\n");
      ds4.reset_search();
      return;
  }
  
  if ( OneWire::crc8( addr4, 7) != addr4[7]) {
      Serial.print("CRC is not valid!\n");
      return;
  }
  ds4.reset();
  ds4.select(addr4);
  ds4.write(0x44,1); 
  requestFlag=1;
  
}

double readTemps1(){
  char i;
  double Temp;
  
  present = ds1.reset();
  ds1.select(addr1);    
  ds1.write(0xBE);         // Read Scratchpad

  
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data1[i] = ds1.read();

  }

  LowByte = data1[0];
  HighByte = data1[1];
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

double readTemps2(){
  char i;
  double Temp;
  
  present = ds2.reset();
  ds2.select(addr2);    
  ds2.write(0xBE);         // Read Scratchpad

  
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data2[i] = ds2.read();

  }

  LowByte = data2[0];
  HighByte = data2[1];
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

double readTemps3(){
  char i;
  double Temp;
  
  present = ds3.reset();
  ds3.select(addr3);    
  ds3.write(0xBE);         // Read Scratchpad

  
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data3[i] = ds3.read();

  }

  LowByte = data3[0];
  HighByte = data3[1];
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

double readTemps4(){
  char i;
  double Temp;
  
  present = ds4.reset();
  ds4.select(addr4);    
  ds4.write(0xBE);         // Read Scratchpad

  
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data4[i] = ds4.read();

  }

  LowByte = data4[0];
  HighByte = data4[1];
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
