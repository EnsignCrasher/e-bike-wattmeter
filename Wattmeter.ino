#include <EEPROM.h>
#include <Wire.h> 
#include <LiquidCrystal_I2C.h>



#define BUTTON1PIN 7
#define BUTTON2PIN 8
#define simulateInputs
#define simulateSpeed 
#define speedPoti
//#define output

#ifdef simulateSpeed
  #include "TimerOne.h"  
#endif

#ifdef speedPoti
  #define SPEEDPOTIPIN A2
#endif

struct taskDefinition{
  long lastUpdate = 0;
  long period = 0;
  };

enum taskID{
  readAndProcessCurrAndVolt,
  alternateCur,
  pageSwitch,
  LCD,
  t_speedPoti,
  t_shortTermAVG,
  t_longTermAVG
};


class circularBuffer{
  
  public:
    float calcAverage(bool ignoreNullValues){
      if(ignoreNullValues){
        float sum = 0;
        for(int i=0;i<(sizeof(data)/sizeof(float));i++){
          sum += data[i];
        }
        return sum/(sizeof(data)/sizeof(float));
      }else{
        float sum = 0;
        int arraySize = sizeof(data)/sizeof(float);
        int actValueCount = 0;
        for(int i=0;i<arraySize;i++){
          if(data[i] != 0){
            sum += data[i];
            actValueCount++;
          }
        }
        return sum/actValueCount;
      }
    }

    void push(float value){
      data[firstItem] = value;
      firstItem++;
      if(firstItem == sizeof(data)/sizeof(float))
        firstItem = 0;
      
    }
  private:
    int firstItem = 0;
    float data[50];
    
};

taskDefinition task[20];


/////////////////////////////////////////////////////////////////////////
////menu variables

int buttonHoldThreshold = 2000;
int debounceDelay = 50;

enum eButtonState{
  idle,
  pressed,
  hold
};

class button{
  private:
    bool _currentState = false;
    bool _lastState = false;
    bool _awaitButtonHoldOrRelease = false;
    long _lastSignalChangedTime = 0;
    long _buttonPressedTime = 0;
    int _pin = 0;
  public:
    void setPin(int pin){
      _pin = pin;
    }
    
    eButtonState getState()
    {
      bool readState = !digitalRead(_pin);

      if(readState != _lastState){
        _lastSignalChangedTime = millis();
      }

      if((millis()-_lastSignalChangedTime)>debounceDelay){
        if(_currentState == false && readState == true) //button was pressed
        {
          _currentState = true;
          if(!_awaitButtonHoldOrRelease)
          { 
            _awaitButtonHoldOrRelease = true;
            _buttonPressedTime = millis();
            return idle;
          }
        }
        else if(_currentState == true && readState == false)//button was released
        {
          _currentState = false;
          if(_awaitButtonHoldOrRelease) 
          {
            _awaitButtonHoldOrRelease = false;
            return pressed;
          }
        }
      }

      if(_awaitButtonHoldOrRelease && (_buttonPressedTime+buttonHoldThreshold) < millis())
      { //button hold detected
        _awaitButtonHoldOrRelease = false;
        return hold;
      }
      _lastState = readState;
      return idle;
    }
};

button button1;
button button2;

/////////////////////////////////////////////////////////////////////////
////LCD
const int lcdLineLength = 20;
const int lcdHeight = 4;
int lcdPage = 0;
LiquidCrystal_I2C lcd(0x27, lcdLineLength, lcdHeight);   

/////////////////////////////////////////////////////////////////////////
////simulation values

#ifdef simulateSpeed
  long hsCycle = 300;
#endif

#ifdef simulateInputs
  int simulatedCurrentValue = 1024;
#endif


/////////////////////////////////////////////////////////////////////////
////voltage

#define VOLTPIN A0
const int cellNumber = 12;
float totalVoltage = 0;
float cellVoltage = 0;
float minCellVoltage = 0;


/////////////////////////////////////////////////////////////////////////
////current

#define CURRPIN A1
float actCurrent = 0;


/////////////////////////////////////////////////////////////////////////
////power

float actPower = 0;


/////////////////////////////////////////////////////////////////////////
////capacity

float batteryPercentage = 0;
long totalBatteryCapacity = 8500; //in mah 
int totalAHEEPROMAddress = 0;
float totalmWH = 0;
float totalmAH = 0;

/////////////////////////////////////////////////////////////////////////
////efficiency 

float actmWHperKM = 0;
float avgmWHperKM = 0;
long WHperKMValueCount = 0;
int avgmWHperKMEEPROMAddress = sizeof(float);

float actmAHperKM = 0;
float avgmAHperKM = 0;
long AHperKMValueCount = 0;
int avgmAHperKMEEPROMAddress = 2*sizeof(float);

long lastUpdateTime = 0;

float expectedRange = 0;


/////////////////////////////////////////////////////////////////////////
////hallsensor values
#define HALLSENSORPIN 3
long hsLastTriggered = 0;
long hallsensorTime = 0;



/////////////////////////////////////////////////////////////////////////
////tachometer
float totalKM = 0;
float currentSpeed = 0;
float tireSize = 2074; //tire circumference in mm


/////////////////////////////////////////////////////////////////////////
////Serial output
int valueUpdatePeriod = 500;
long lastValueUpdate = 0;


/////////////////////////////////////////////////////////////////////////
////averages

circularBuffer lastShortTermWHperKM;
circularBuffer lastShortTermAHperKM;
float actShortTermWHperKM = 0;
float actShortTermAHperKM = 0;

circularBuffer lastLongTermWHperKM;
circularBuffer lastLongTermAHperKM;
float actLongTermWHperKM = 0;
float actLongTermAHperKM = 0;


void setup() {

  task[readAndProcessCurrAndVolt].period = 10;
  task[LCD].period = 500;
  task[pageSwitch].period = 10000;
  task[t_shortTermAVG].period = 30;
  task[t_longTermAVG].period = 12000;
  #ifdef simulateInputs
    task[alternateCur].period = 100;
  #endif

  #ifdef speedPoti
    pinMode(SPEEDPOTIPIN, INPUT);
    task[t_speedPoti].period = 100;
  #endif
  
  #ifdef output
    Serial.begin(57600);
  #endif

  lcd.begin(); 
  lcd.print("test");
  showSplashScreen(2000);
  
  //read values from EEPROM
  //EEPROM.get(totalAHEEPROMAddress,totalAH);
  //EEPROM.get(avgmWHperKM,avgmWHperKMEEPROMAddress);
  
  totalVoltage = readVoltage();
  cellVoltage = totalVoltage/cellNumber;
  minCellVoltage = cellVoltage;
  lastUpdateTime = micros();

  pinMode(HALLSENSORPIN, INPUT);
  pinMode(VOLTPIN, INPUT);
  pinMode(CURRPIN, INPUT);
  pinMode(BUTTON1PIN, INPUT_PULLUP);
  pinMode(BUTTON2PIN, INPUT_PULLUP);

  button1.setPin(BUTTON1PIN);
  button2.setPin(BUTTON2PIN);
  
  #ifdef simulateSpeed
    Timer1.initialize(hsCycle *1000);
    Timer1.attachInterrupt(onHallSensorTriggered);
  #else
    attachInterrupt(digitalPinToInterrupt(HALLSENSORPIN), onHallSensorTriggered, RISING);
  #endif
}

int holdCount = 0;
int pressCount = 0;
int idleCount = 0;
eButtonState lastState = idle;

void loop() {
  eButtonState button1state = button1.getState();
  eButtonState button2state = button2.getState();

  if(button1state == pressed){
    switchPage();
  }

  #ifdef speedPoti
    if(isCalled(t_speedPoti)){
      int aSpeed = analogRead(SPEEDPOTIPIN);
      hsCycle = map(aSpeed, 0, 1023, 249, 7466);
      Timer1.initialize(hsCycle *1000);
    }
  #endif

  if(isCalled(readAndProcessCurrAndVolt)){
    readAndProcessCurrentAndVoltage();
  }

  if(isCalled(t_longTermAVG)){
    if(actShortTermWHperKM != 0)
      lastLongTermWHperKM.push(actShortTermWHperKM);
    if(actShortTermAHperKM != 0)
      lastLongTermAHperKM.push(actShortTermAHperKM);
  }

  if(isCalled(t_shortTermAVG)){
    lastShortTermWHperKM.push(actmWHperKM);
    lastShortTermAHperKM.push(actmAHperKM);
  }

  #ifdef output
     outputValues();
  #else
    if(isCalled(LCD)){
      actShortTermWHperKM = lastShortTermWHperKM.calcAverage(true);
      actShortTermAHperKM = lastShortTermAHperKM.calcAverage(true);
      actLongTermWHperKM = lastLongTermWHperKM.calcAverage(false);
      actLongTermAHperKM = lastLongTermAHperKM.calcAverage(false);
      calculateExpectedRange();
      calculateBatteryPercentage();
      writeValuesToLCD();
    } 
  #endif

  #ifdef simulateInputs
      
    if(isCalled(alternateCur)){
      simulateRealisticCurrent(currentSpeed);
      //alternateCurrent();
      //alternateSpeed();
    }
  #endif

  if(isCalled(pageSwitch)){
    //switchPage();
  }

  if((hsLastTriggered+calcTimeForSpeed(2)) < millis()){
    timeOut();
  }
}

void calculateBatteryPercentage(){
  batteryPercentage = (totalBatteryCapacity-totalmAH)/totalBatteryCapacity*100;
}

void calculateExpectedRange(){
  float capLeft = totalBatteryCapacity - totalmAH;
  expectedRange = capLeft/actLongTermAHperKM;
}

void readAndProcessCurrentAndVoltage(){
  //////////////////////////////////////
  ///read inputs
  
  totalVoltage = readVoltage();
  actCurrent = readCurrent();

  //////////////////////////////////////
  ///calculate and evaluate...
    
  cellVoltage = totalVoltage/cellNumber; 
  if(cellVoltage < minCellVoltage)
    minCellVoltage = cellVoltage;
    
  actPower = actCurrent*totalVoltage;
  
  if((micros() - lastUpdateTime)>0)
  {
    long temp = micros();
    totalmAH += actCurrent * (micros()-lastUpdateTime)/1000/60/60;
    totalmWH += actPower * (micros()-lastUpdateTime)/1000/60/60;
    lastUpdateTime = micros();
    lastUpdateTime = lastUpdateTime - (lastUpdateTime-temp);
  }
}


/////////////////////////////////////////////////////////////////////////
////LCD Display

void switchPage(){
  lcdPage++;
  lcd.clear();
  if(lcdPage > 1)
    lcdPage = 0;
}



void showLcdPage1(){
    //if(fieldVisibility[0][0])
      LCDWriteLeft(cutString(String(cellVoltage), 4) + " Volt", 0);
    //if(fieldVisibility[0][1])
      LCDWriteLeft(cutString(String(minCellVoltage), 4) + " Vmin", 1);
    //if(fieldVisibility[0][2])
      LCDWriteLeft(cutString(String((int)batteryPercentage), 3) + "% Batt", 2);
    //if(fieldVisibility[0][3])
      LCDWriteLeft(cutString(String(actShortTermWHperKM/1000), 4) + " W/km", 3);
    
    //if(fieldVisibility[1][0])
      LCDWriteRight(cutString(String(currentSpeed), 4) + " km/h", 0);
    //if(fieldVisibility[1][1])
      LCDWriteRight(cutString(String(totalKM), 4) + " km  ", 1);
    //if(fieldVisibility[1][2])
      LCDWriteRight(cutString(String((int)expectedRange), 4) + " kmRg", 2);
    //if(fieldVisibility[1][3])
      LCDWriteRight(cutString(String(actLongTermWHperKM/1000), 4) + " AW/k", 3);
}

void showLcdPage2()
{
    //if(fieldVisibility[0][0])
      LCDWriteLeft(cutString(String(actCurrent),4) + " A   ", 0);
    //if(fieldVisibility[0][1])
      LCDWriteLeft(cutString(String((int)actmAHperKM), 4) + " mA/k", 1);
    //if(fieldVisibility[0][2])
      LCDWriteLeft(cutString(String((int)actShortTermAHperKM), 4) + " mA/k", 2);
    //if(fieldVisibility[0][3])
      LCDWriteLeft(cutString(String((int)actLongTermAHperKM), 4) + " mA/k", 3);

    //if(fieldVisibility[1][0])
      LCDWriteRight(String(millis()),0);
    //if(fieldVisibility[1][1])
      LCDWriteRight(cutString(String(actPower), 4) + " Watt", 1);
    //if(fieldVisibility[1][2])
      LCDWriteRight(cutString(String((int)totalmAH), 4) + " mAh ", 2);
    //if(fieldVisibility[1][3])
      LCDWriteRight(cutString(String(totalBatteryCapacity-totalmAH), 4) + " mAhL", 3);
    
    
}

void writeValuesToLCD(){ 
    switch(lcdPage){
      case 0:
        showLcdPage1();
        break;
      case 1:
        showLcdPage2();
        break;
    }
}

String cutString(String value, int pLength){
  if(value.length()>pLength)
      value.remove(pLength);
  else{
    for(int i=value.length();i<pLength;i++){
      value = " "+value;
    }
  }
  return value;
  return "";
}


void LCDWriteCenter(String text, int line){
  int startPos = (lcdLineLength - text.length())/2;
  lcd.setCursor(startPos, line);
  lcd.print(text);
}

void LCDWriteLeft(String text, int line){
  if(text.length()>10)
    text.remove(10);
  int startPos = 0;
  lcd.setCursor(startPos, line);
  lcd.print(text);
  if(text.length()<10)
    LCDCustomClear(line, text.length(), 10);
}

void LCDWriteRight(String text, int line){
  if(text.length()>10)
    text.remove(10);
  int startPos = lcdLineLength- text.length();
  lcd.setCursor(startPos, line);
  lcd.print(text);
  if(text.length()<10)
    LCDCustomClear(line, 10, 19-text.length());
}

void LCDCustomClear(int line, int first, int last){
  for(int i = first;i<=last;i++){
    lcd.setCursor(i, line);
    lcd.print(" ");
  }
}

void showSplashScreen(int _time){
  lcd.clear();
  LCDWriteCenter("Wattmeter V1.0", 1);
  LCDWriteCenter("Commander am Start", 2);    
  delay(_time);
  lcd.clear();
}


/////////////////////////////////////////////////////////////////////////
////Serial output

void outputValues(){
  if(millis()>(lastValueUpdate+valueUpdatePeriod)){
    Serial.print("Time :");Serial.print(millis());
    Serial.print(" Total Voltage: ");Serial.print(totalVoltage, DEC);
    Serial.print(" Cell Voltage: ");Serial.print(cellVoltage, DEC);
    Serial.print(" min Cell Voltage: ");Serial.print(minCellVoltage, DEC);
    
    //Serial.println();
    
    Serial.print("Act Current ");Serial.print(actCurrent, DEC);
    Serial.print(" Act Power ");Serial.print(actPower, DEC);
    Serial.print(" Total mAH ");Serial.print(totalmAH, DEC);
    Serial.print(" Total mWH ");Serial.print(totalmWH, DEC);
    
    Serial.print(" WH per KM ");Serial.print(actmWHperKM, DEC);
    
    Serial.print(" Current speed ");Serial.print(currentSpeed);
    Serial.print(" Total KM ");Serial.print(totalKM);
    
    Serial.println();
    lastValueUpdate = millis();
  }
}

/////////////////////////////////////////////////////////////////////////
////tachometer functions

void onHallSensorTriggered(){

  actPower = actCurrent*totalVoltage;

  currentSpeed = tireSize / (millis() - hsLastTriggered) * 3.6;
  totalKM += tireSize/1000/1000;
  
  if(currentSpeed >= 2){
    actmWHperKM = actPower/currentSpeed*1000;
    actmAHperKM = actCurrent/currentSpeed*1000;
    WHperKMValueCount++;
    AHperKMValueCount++;
    calcAvgmWHperKM();
    calcAvgmAHperKM();
  }else{
    timeOut();
  }
  
  hsLastTriggered = millis();
  
  #ifdef simulateOutputs
    Timer1.initialize(hsCycle *1000);
  #endif
}

void saveValuesToEEPROM(){
  EEPROM.put(totalAHEEPROMAddress, totalmAH);
  EEPROM.put(avgmWHperKM,avgmWHperKMEEPROMAddress);
}


//input in km/h, output in ms
long calcTimeForSpeed(float pSpeed){
  float _time = (tireSize/1000000) / pSpeed * 60 * 60 * 1000;
  long retTime = (long)_time;
  return retTime;
}

void timeOut(){
  currentSpeed = 0;
  actmWHperKM = 0;
  actmAHperKM = 0;
}



void calcAvgmWHperKM(){
  avgmWHperKM = (avgmWHperKM*WHperKMValueCount + actmWHperKM)/(WHperKMValueCount+1);
}

void calcAvgmAHperKM(){
  avgmAHperKM = (avgmAHperKM*AHperKMValueCount + actmAHperKM)/(AHperKMValueCount+1);
}

bool isCalled(taskID id)
{
  if(millis()>(task[id].lastUpdate + task[id].period)){
      setLastUpdate(id);
      return true;
  }else{
    return false;
  }
}

void setLastUpdate(taskID id){
  task[id].lastUpdate = millis();
}

/////////////////////////////////////////////////////////////////////////
////input functions

float readCurrent(){
  float analogCurrent = 0;
  #ifdef simulateInputs
    analogCurrent = simulatedCurrentValue;
  #else
    analogCurrent = analogRead(CURRPIN);
  #endif
  float maxCurrent = 30; //in A

  float current = maxCurrent * analogCurrent / 1023;
  return current;
}

float readVoltage(){
  int analogVoltage = 0;
  #ifdef simulateInputs
    analogVoltage =  1023; //should equal cell voltage 3.73V
  #else
    analogVoltage = analogRead(VOLTPIN);
  #endif

  float voltageDividerRatio = 0.11111111111111111111111111111111;//0.06666666666666666666666666666667; //1k : 15k
  
  float voltage = 5*analogVoltage/voltageDividerRatio/1023;
  
  return voltage; 
  
}

























/////////////////////////////////////////////////////////////////////////
////Simulation functions



int spdState = 0;

#ifdef simulateSpeed;
  void alternateSpeed(){
    switch(spdState){
      case 0:
        hsCycle--;
        if(hsCycle <= 100)
          spdState = 1;
        break;
      case 1:
        hsCycle++;
        if(hsCycle >= 5000)
          spdState = 0;
        break;
    }
  }
#endif

#ifdef simulateInputs
  void alternateCurrent(){
    switch(spdState){
      case 0:
        simulatedCurrentValue--;
        if(simulatedCurrentValue <= 0)
          spdState = 1;
        break;
      case 1:
        simulatedCurrentValue++;
        if(simulatedCurrentValue >= 1024)//341)
          spdState = 0;
        break;
    }
  }

  void simulateRealisticCurrent(float pSpeed){
    float maxSensorCur = 30;
    float maxLoadCur = 4.17;
    float maxLoadSpeed = 30;
    float variance = 0.6;

    float estimatedCurrent = maxLoadCur/maxLoadSpeed*pSpeed;
    
    bool addSubstractVariance = (bool)(millis()%2);

    if(currentSpeed == 0){
      addSubstractVariance = true;
      variance = 0.2;
    }
    
    if(addSubstractVariance){
      estimatedCurrent += ((float)(millis()%((long)(variance*1000))))/1000;
    }else{
      estimatedCurrent -= ((float)(millis()%((long)(variance*1000))))/1000;
    }

    if(estimatedCurrent < 0)
      estimatedCurrent = 0;

    simulatedCurrentValue = 1024 * estimatedCurrent/maxSensorCur;
    
  }
#endif


