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
  LCDMenu, 
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
    float data[25];
    
};

taskDefinition task[8];


/////////////////////////////////////////////////////////////////////////
////menu variables

/*class secondLevelMenuItem{
   private:
    
   public:
    String displayText;
    void (*executeButtonInputs)();
};

class firstLevelMenuItem{
  private:
    
  public:
    String displayText;
    secondLevelMenuItem subItems[];
    void (*executeButtonInputs)();
};

firstLevelMenuItem Reset;
  secondLevelMenuItem mi_kmTrip;
  secondLevelMenuItem mi_kmRange;
  secondlevelMenuItem mi_kmTotal;
  secondLevelMenuItem mi_whperkm;

firstLevelMenuItem Batt;
  secondLevelMenuItem mi_ah;
  secondLevelMenuItem mi_vmin;
  secondLevelMenuItem mi_vmax;
  
firstLevelMenuItem VoltCali;
firstLevelMenuItem CurrCali;
firstLevelMenuItem mainMenueItems[] = {Reset, Batt, VoltCali, CurrCali};*/
/*const char string_0[] PROGMEM = "Reset";  
const char string_1[] PROGMEM = "Batt";
const char string_2[] PROGMEM = "Volt Cali";
const char string_3[] PROGMEM = "Curr Cali";

char strBuffer[10];

const char* const mainMenuItems[] PROGMEM = {string_0, string_1, string_2, string_3};
const int mainMenueItemsCount = 4;

const char string_4[] PROGMEM = "kmTr";
const char string_5[] PROGMEM = "kmRg";
const char string_6[] PROGMEM = "kmAl";
const char string_7[] PROGMEM = "Wh/km";

const char* const resetSubItems[] PROGMEM = {string_4, string_5, string_6, string_7};

const char string_8[] PROGMEM = "AH";
const char string_9[] PROGMEM = "Vmin";
const char string_10[] PROGMEM = "Vmax";
const char* const batterySubItems[] PROGMEM = {string_8, string_9, string_10};*/

String mainMenuItems[] = {
  "Reset",
  "Batt",
  "Volt Cali",
  "Curr Cali",
  "NULL"  
};

String resetSubItems[] = {
  "kmTr",
  "kmRg",
  "kmAl",
  "Wh/km",
  "NULL" 
};

String batterySubItems[] = {
  "AH",
  "Vmin",
  "Vmax",
  "NULL"
};

int selIdx = 0;
int blinkIdx = 0;
int selLevel = 0;

bool menu = false;
bool mainMenuFadeIn = false;
long buttonHoldThreshold = 2000;
long buttonPressThreshold = 1000;
int debounceDelay = 100;

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
    bool _actState = false;
    bool _afterHold = false;
    bool _firstRunSinceLastRelease = false;
    int _pin = 0;
  public:
    void setPin(int pin){
      _pin = pin;
    }

    bool getDebouncedState(){
      bool readState = !digitalRead(_pin);

      if(readState != _lastState){
        _lastSignalChangedTime = millis();
      }
      _lastState = readState;

      if((millis()-_lastSignalChangedTime)>debounceDelay)
        _actState = readState;
      
      return _actState;
    }
    
    eButtonState getState()
    {
      if(getDebouncedState()){//button is pressed          

        if(_firstRunSinceLastRelease){
          _awaitButtonHoldOrRelease = true;
          _firstRunSinceLastRelease = false;
          _buttonPressedTime = millis();
        }
        
        if(_awaitButtonHoldOrRelease && (_buttonPressedTime+buttonHoldThreshold) < millis())//button hold detected
        { 
          _awaitButtonHoldOrRelease = false;
          return hold;
        }else{
          return idle;
        }
      }else{ //button is not pressed
        _firstRunSinceLastRelease = true;
        if(_awaitButtonHoldOrRelease){ //button was released
            _awaitButtonHoldOrRelease = false;
            return pressed;
        }else{
          return idle;
        }
      }
      
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
bool fieldVisibility[2][4] = {{true, true, true, true},{true, true, true, true}};

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
float alertVmin = 3.8;
float alertVmax = 4.25;

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

unsigned long lastUpdateTime = 0;

float expectedRange = 0;


/////////////////////////////////////////////////////////////////////////
////hallsensor values
#define HALLSENSORPIN 3
long hsLastTriggered = 0;
long hallsensorTime = 0;



/////////////////////////////////////////////////////////////////////////
////tachometer
float totalKM = 0;
float tripKM = 0;
float rangeKM = 0;
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
  task[LCDMenu].period = 150;
  task[pageSwitch].period = 750;
  task[t_shortTermAVG].period = 60;
  task[t_longTermAVG].period = 24000;
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
  Serial.begin(57600);

  lcd.begin(); 
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
bool wasHold = false;
bool wasPressed = false;
bool wasTrueOnce = false;
/*eButtonState lastState1 = idle;
eButtonState lastState2 = idle;*/
eButtonState button1state;
eButtonState button2state;

void loop() {
  
  button1state = button1.getState();
  button2state = button2.getState();

  /*if(button1.getDebouncedState())
    wasTrueOnce = true;*/

  /*if(lastState1 == hold && button1state == idle){
    lastState1 = idle;
  }

  if(lastState1 == hold){
    button1state = idle;
  }else{
    lastState1 = button1state;
  }*/
  /*if(button2state == hold)
    menu = false;*/
  
  /*if(button1state == hold)
      wasHold = true;
    if(button1state == pressed)
      wasPressed = true;
    if(button1state == idle)
      LCDWriteRight("idle", 2);
    
    if(wasHold)
      LCDWriteRight("hold", 0);
    if(wasPressed){
      LCDWriteRight("pressed", 1);
    if(wasTrueOnce)
      LCDWriteRight("true", 3);

    if(isCalled(pageSwitch))
    {
      wasHold = false;
      wasPressed = false;
      wasTrueOnce = false;
    }
  }*/
  
  
      
      
  if(!menu){  
    if(button1state == hold){
      menu = true;
      mainMenuFadeIn = true;
    }
    
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
  }else{
    showMenu(button1state, button2state);
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
    if(cellVoltage < alertVmin || cellVoltage > alertVmax)
      blinkOnField(0,0);
    else
      fieldVisibility[0][0] = true;
  
    if(fieldVisibility[0][0])
      LCDWriteLeft(cutString(String(cellVoltage), 4) + " Volt", 0);
      
    if(minCellVoltage < alertVmin || minCellVoltage > alertVmax)
      blinkOnField(0,1);
    else
      fieldVisibility[0][1] = true;
      
    if(fieldVisibility[0][1]) 
      LCDWriteLeft(cutString(String(minCellVoltage), 4) + " Vmin", 1);
    if(fieldVisibility[0][2])
      LCDWriteLeft(cutString(String((int)batteryPercentage), 3) + "% Batt", 2);
    if(fieldVisibility[0][3])
      LCDWriteLeft(cutString(String(actShortTermWHperKM/1000), 4) + " W/km", 3);
    
    if(fieldVisibility[1][0]);
      LCDWriteRight(cutString(String(currentSpeed), 4) + " km/h", 0);
    if(fieldVisibility[1][1])
      LCDWriteRight(cutString(String(tripKM), 4) + " km  ", 1);
    if(fieldVisibility[1][2])
      LCDWriteRight(cutString(String((int)expectedRange), 4) + " kmRg", 2);
    if(fieldVisibility[1][3])
      LCDWriteRight(cutString(String(actLongTermWHperKM/1000), 4) + " AW/k", 3);
}

void showLcdPage2()
{
    if(fieldVisibility[0][0])
      LCDWriteLeft(cutString(String(actCurrent),4) + " A   ", 0);
    if(fieldVisibility[0][1])
      LCDWriteLeft(cutString(String((int)actmAHperKM), 4) + " mA/k", 1);
    if(fieldVisibility[0][2])
      LCDWriteLeft(cutString(String((int)actShortTermAHperKM), 4) + " mA/k", 2);
    if(fieldVisibility[0][3])
      LCDWriteLeft(cutString(String((int)actLongTermAHperKM), 4) + " mA/k", 3);

    if(fieldVisibility[1][0])
      LCDWriteRight(String(millis()),0);
    if(fieldVisibility[1][1])
      LCDWriteRight(cutString(String(actPower), 4) + " Watt", 1);
    if(fieldVisibility[1][2])
      LCDWriteRight(cutString(String((int)totalmAH), 4) + " mAh ", 2);
    if(fieldVisibility[1][3])
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
    setInvisibleFields();
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

  if(!menu){
    actPower = actCurrent*totalVoltage;

    currentSpeed = tireSize / (millis() - hsLastTriggered) * 3.6;
    totalKM += tireSize/1000/1000;
    tripKM += tireSize/1000/1000;
    rangeKM += tireSize/1000/1000;
    
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
////menu

void blinkLCD(){
    lcd.noBacklight();
    delay(100);
    lcd.backlight();
    delay(100);
    lcd.noBacklight();
    delay(100);
    lcd.backlight();
    delay(100);
}

int testState = 0;
int testCount = 0;
int subMainMenuIdx = 0;
int subResetMenuIdx = 0;
int subBattMenuIdx = 0;

void showMenu(eButtonState button1state, eButtonState button2state){
  if(button1state == pressed)
    scrollUp();
  else if(button2state == pressed)
    scrollDown(getStringArrayLength(mainMenuItems));

  if(mainMenuFadeIn && button1state == idle){
    mainMenuFadeIn = false;
  }


  //switch between menus
  if(button2state == hold)
  {
    selIdx = 0;
    selLevel--;
    if(selLevel < 0)
    {
      selLevel = 0;
      setAllFieldsVisible();
      menu = false;
      return;
    }
  }

  
  if(selLevel == 0)
  {
    if(!mainMenuFadeIn && button1state == hold){
      subMainMenuIdx = selIdx;
      selIdx = 0;
      selLevel++;
    }
  }
  else if(selLevel >= 1)
  {
    switch(subMainMenuIdx){
      case 0:
        if(selLevel == 1 && button1state == hold){
          subResetMenuIdx = selIdx;
          selIdx = 0;
          selLevel++;
          lcd.clear();
        }
        else if(selLevel >= 2){
          switch(subResetMenuIdx){
            case 0:
              editTripKM();
              break;
            case 1:
              editRangeKM();
              break;
            case 2:
              editTotalKM();
              break;
            case 3:
              if(button1state == hold){
                WHperKMValueCount = 0;
                avgmWHperKM = 0;
              }
              break;
          }
        }
        break;
      case 1:
        if(selLevel == 1 && button1state == hold){
          subBattMenuIdx = selIdx;
          selIdx = 0;
          selLevel++;
          lcd.clear();
        }
        else if(selLevel >= 2){
          switch(subBattMenuIdx){
            case 0:
              editBattCapacity();
              break;
            case 1:
              editAlertVmin();
              break;
            case 2:
              editAlertVmax();
              break;
          }
        }
        break;
      case 2:
        break;
      case 3:
        break;
    }
  }

  //display menu items
  if(isCalled(LCDMenu)){
    if(selLevel == 0)
    {
      showMainMenu();
    }
    else if(selLevel >= 1)
    {
      switch(subMainMenuIdx){
        case 0:
          if(selLevel == 1)
            showResetSubMenu();
          else if(selLevel >= 2){
            switch(subResetMenuIdx){
              case 0:
                LCDWriteLeft(String(tripKM) + " km", 0);
                break;
              case 1:
                LCDWriteLeft(String(rangeKM) + " km", 0);
                break;
              case 2:
                LCDWriteLeft(String(totalKM) + " km", 0);
                break;
              case 3:
                LCDWriteLeft(String(avgmWHperKM/1000) + " Wh/km", 0);
                break;
            }
          }
          break;
        case 1:
          if(selLevel == 1)
            showBattSubMenu();
          else if(selLevel >= 2){
            switch(subBattMenuIdx){
              case 0:
                LCDWriteLeft(String(totalBatteryCapacity) + " mAh", 0);
                break;
              case 1:
                LCDWriteLeft(String(alertVmin) + " Volt", 0);
                break;
              case 2:
                LCDWriteLeft(String(alertVmax) + " Volt", 0);
                break;
            }
          }
          break;
        case 2:
          break;
        case 3:
          break;
      }
    }
    if(selLevel < 2)
      blinkOnField(0,blinkIdx);
    else
      setAllFieldsVisible();
  }
  setInvisibleFields();
}

void editTotalKM(){
  if(button1state == pressed)
    totalKM += 10;
  if(button2state == pressed && totalKM > 10)
    totalKM -= 10;
  if(button1state == hold)
    totalKM = 0;
}

void editTripKM(){
  if(button1state == pressed)
    tripKM += 1;
  if(button2state == pressed && tripKM > 1)
    tripKM -= 1;
  if(button1state == hold)
    tripKM = 0;
}

void editRangeKM(){
  if(button1state == pressed)
    rangeKM += 1;
  if(button2state == pressed && rangeKM > 1)
    rangeKM -= 1;
  if(button1state == hold)
    rangeKM = 0;
}

void editBattCapacity(){
  if(button1state == pressed)
    totalBatteryCapacity += 100;
  if(button2state == pressed && totalBatteryCapacity > 100)
    totalBatteryCapacity -= 100;
}

void editAlertVmin(){
  if(button1state == pressed)
    alertVmin += 0.05;
  if(button2state == pressed && alertVmin > 0.05)
    alertVmin -= 0.05;
}

void editAlertVmax(){
  if(button1state == pressed)
    alertVmax += 0.05;
  if(button2state == pressed && alertVmax > 0.05)
    alertVmax -= 0.05;
}

void writeMenuItems(String arr[])
{
  int itemCount = getStringArrayLength(arr);
  
  fieldVisibility[0][blinkIdx] = true;
  
  if((selIdx+lcdHeight)<itemCount){
    blinkIdx = 0;
    for(int i=selIdx;i<(selIdx+lcdHeight);i++){
      if(fieldVisibility[0][i-selIdx])
        LCDWriteLeft(arr[i], i-selIdx);
    }
  }else{ //this are the last items
    blinkIdx = selIdx+lcdHeight-itemCount;
    if(getStringArrayLength(arr) <= 4)
      blinkIdx -= lcdHeight - getStringArrayLength(arr);
    
    for(int i=0;i<lcdHeight;i++){
      if(fieldVisibility[0][i]){
        if(getStringArrayLength(arr) > 4){
          LCDWriteLeft(arr[getStringArrayLength(arr)-1-lcdHeight], i);
        }else if(getStringArrayLength(arr) <= 4 && i<getStringArrayLength(arr)){
          LCDWriteLeft(arr[i], i);
        }else{
          LCDWriteLeft("", i);
        }
      }
    }
    /*for(int i=itemCount-1;i>(itemCount-lcdHeight-1);i--)
    {
      if(itemCount>=lcdHeight){
        if(fieldVisibility[0][i-itemCount+lcdHeight])
          LCDWriteLeft(arr[i], i-itemCount+lcdHeight);
      }else{
        if(fieldVisibility[0][i-itemCount+lcdHeight])
          LCDWriteLeft(arr[i], i-itemCount+lcdHeight);
      }
    }*/
  }
}

String battMenuValueArr[2] = {"NULL", "NULL"};
void showBattSubMenu(){
  writeMenuItems(batterySubItems);
  switch(selIdx){
    case 0:
      battMenuValueArr[0] = String(totalBatteryCapacity);
      showSubItems(battMenuValueArr);
      break;
    case 1:
      battMenuValueArr[0] = String(alertVmin);
      showSubItems(battMenuValueArr);
      break;
    case 2:
      battMenuValueArr[0] = String(alertVmax);
      showSubItems(battMenuValueArr);
      break;
  }
}

String resetMenuValueArr[2] = {"NULL", "NULL"};
void showResetSubMenu(){
  writeMenuItems(resetSubItems);
  switch(selIdx){
    case 0:
      resetMenuValueArr[0] = String(tripKM);
      showSubItems(resetMenuValueArr);
      break;
    case 1:
      resetMenuValueArr[0] = String(rangeKM);
      showSubItems(resetMenuValueArr);
      break;
    case 2:
      resetMenuValueArr[0] = String(totalKM);
      showSubItems(resetMenuValueArr);
      break;
    case 3:
      resetMenuValueArr[0] = String(avgmWHperKM/1000);
      showSubItems(resetMenuValueArr);
      break;
  }
}



void showMainMenu(){
  writeMenuItems(mainMenuItems);
  switch(selIdx){
    case 0: //reset
      showSubItems(resetSubItems);
      break;
    case 1: //batt
      showSubItems(batterySubItems);
      break;
    case 2: //volt cali
      showSubItems(NULL);
      break;
    case 3: //curr cali
      showSubItems(NULL);
      break;
  }
  
}

void showSubItems(String arr[]){
  int subItemsCount = lcdHeight;
  bool createEmptyLines = false;
  if(arr != NULL){
    if(getStringArrayLength(arr)<lcdHeight){
      subItemsCount = getStringArrayLength(arr);
      createEmptyLines = true;
    }

    for(int i=0;i<subItemsCount;i++){
      LCDWriteRight(arr[i], i);
    }
  }else{
    createEmptyLines = true;
  }
  
  if(createEmptyLines){
    for(int i=getStringArrayLength(arr);i<lcdHeight;i++){
      LCDWriteRight("", i);
    }
  }
}

int getStringArrayLength(String arr[]){
  if(arr == NULL)
    return 0;
  int count = 0;
  while(arr[count] != "NULL")
    count++;

  return count;
}

void scrollDown(int itemCount){
  if(selIdx<itemCount-1)
    selIdx++;
}

void scrollUp(){
  if(selIdx>0)
    selIdx--;
}

bool blinkFieldState[2][4] = {{false,false,false,false},{false,false,false,false}};
int blinkRatio = 1; // X:1
int currentBlinkCount[2][4] = {{0,0,0,0},{0,0,0,0}};

void blinkOnField(int x, int y){
  if(blinkFieldState[x][y]){
    currentBlinkCount[x][y]++;
    if(currentBlinkCount[x][y] == blinkRatio){
      blinkFieldState[x][y] = false;
      currentBlinkCount[x][y] = 0;
    }
    fieldVisibility[x][y] = true;
  }else{
    blinkFieldState[x][y] = true;
    fieldVisibility[x][y] = false;
  }
}

void setAllFieldsVisible(){
  for(int i = 0;i<2;i++){
    for(int a = 0;a<4;a++){
      fieldVisibility[i][a] = true;
    }
  }
}

void setInvisibleFields(){
  for(int a=0;a<4;a++){
    if(!fieldVisibility[0][a])
      LCDWriteLeft("",a);

    if(!fieldVisibility[1][a])
      LCDWriteRight("", a);
  }
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

