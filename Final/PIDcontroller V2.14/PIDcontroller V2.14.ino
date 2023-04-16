#include <Preferences.h>
Preferences flashMem;

#include <ESP32Encoder.h>

// external sensor
#include <OneWire.h>
#include <DallasTemperature.h>
const int oneWireBus = 16;     
OneWire oneWire(oneWireBus);
DallasTemperature sensors(&oneWire);

// internal sensor
#include <SHTSensor.h>
#include <arduino-sht.h>

#include <RTClib.h>
RTC_DS1307 DS1307;

float currentSensorTemperature = 0;

#include <Wire.h>
#include <LiquidCrystal_I2C.h>

#include "A0Symbols.h"


static const unsigned int LCDy = 2;
static const unsigned int LCDx = 16;

static const char* flashNameSpace = "PIDsettings";


LiquidCrystal_I2C lcd(0x27, LCDx, LCDy);

const int encoderSwitchPin = 5;
const unsigned int holdDelay = 500;  //how long does the button have to be pressed to be registered as held


SHTSensor sht;
ESP32Encoder encoder;


static const unsigned long REFRESH_INTERVAL = 5000;  // ms
static unsigned long lastRefreshTime = 0;
int currentMenu = 0;
int lastSwtichState = 0;
bool drawFrames = true;
int flashReadingDelay = 10;



//caches
String stringCache = "";   //used for displaying cached variables
String stringCache2 = "";  //used for displaying name of variable
bool boolCache = false;
bool boolCache2 = false;
bool boolCache3 = false;
unsigned long uLongCache = 0;
unsigned long uLongCache2 = 0;
unsigned int uIntCache = 0;
int64_t int64Cache = 0;
int64_t int64Cache2 = 0;
int64_t int64Cache3 = 0;
int64_t int64Cache4 = 0;
int64_t int64Cache5 = 0;
int sourceMenuCache = 0;
int intCache = 0;
char charCache = 0;
String usableCharactersCache = ""; //dont use for anything else

DateTime RTC_Time = DateTime(2000,1,1,0,0,0);

bool editorIsActive = false;
bool editorjustFinished = false;
byte editorIntCache = 0;
byte editorIntCache2 = 0;

bool popupIsActive = false;
byte popupIntCache =  0;
byte popupIntCache2 = 0;
long popupLongCache  = 0;
int popupEncoderCache = 0;
String popupStringCache = "";

//float testVariable1 = 0;
//String testVariable2 = "";
bool settingsPickMode = false;
bool switchSettingsPickMode = false;
float Kp = 0;    //saved in "Kp"
float Ki = 0;    //saved in "Ki"
float Kd = 0;    //saved in "Kd"
float Km = 0;    //saved in "Km" - multiplier
float OnOffHeadroom = 1;
bool PIDsettings[] = {0,0,0,0,0,0};  //saved in "s0" - "s4"
//0=MUST be 1
//1=heating / cooling mode
//2=PID / ON-OFF
//3=normal/hybrid  does nothing if PID is off
//4=Print PID value to serial
//5=internal/external tempSensor

int lastEncoderCount = 0;
unsigned long lastMove = 0;
bool backlightState = true;


float ControllerOutput = 0;

unsigned long uLongCacheSwitchHold = 0;
unsigned long uLongCache60Sec = 60000;
unsigned long uLongCache5Sec = 5000;
unsigned long uLongCache1Sec = 1000;

bool internalSensorFailed = false;

float targetTemp = 0;
float manualTemp = 0;
bool secRefresh5 = false;

bool autoMode = false;


uint32_t PWMfrequency = 0;
bool controllerSettingChanged = true;

bool sleepMode = false;

int registeredSymbols[8];
unsigned int registeredSymbols_counter = 0;


String lcdOutput[] = { "", "" };
bool encoderSwitch = false;
bool encoderSwitchHold = false;
bool encoderSwitchHoldOnce = false;
unsigned int lcdSymbols[LCDx * LCDy];

#include "ComplementaryFunc.h"

PlannedRecordObject PROcache("unnamed", 0, 0, 0, 0);  //cache for creating or editing Planned Records



void setup(){
  Serial.begin(9600);
  Serial.println("<STARTUP>");
  lcd.begin(); // Initialize the LCD
  lcd.backlight();
  lastMove = millis();
  DS1307.begin();
  Wire.begin();
  if (sht.init()) {
    Serial.print("internal senzor init(): success\n");
  } else {
    Serial.print("internal senzor init(): failed\n");
  }
  sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM);
  lcd.clear();
  ESP32Encoder::useInternalWeakPullResistors = UP;
  encoder.attachHalfQuad(2, 4);
  encoder.setCount(0);
  pinMode(encoderSwitchPin, INPUT_PULLUP);


  if (lastSwtichState != digitalRead(encoderSwitchPin)) {  //switch pin change
    if (digitalRead(encoderSwitchPin) == 1) {              //it changed to off
      if(!encoderSwitchHold){    //the button was not held
        encoderSwitch = true;
      }             
    }
    else{
      encoderSwitchHoldOnce = false;
      uLongCache = millis();
    }
  }
  
  encoderSwitchHold = false;
  if ((millis() - uLongCacheSwitchHold) > holdDelay) {
    if (digitalRead(encoderSwitchPin) == 0) {
      if(encoderSwitchHold){encoderSwitchHoldOnce = false;  }
      else{                 encoderSwitchHoldOnce = true;  }
      encoderSwitchHold = true;
    }
  }
  lastSwtichState = digitalRead(encoderSwitchPin);

  flashMem.begin(flashNameSpace, true); 
  Serial.println("Kp=" + String(flashMem.getFloat("Kp", -1)));
  Serial.println("Ki=" + String(flashMem.getFloat("Ki", -1)));
  Serial.println("Kd=" + String(flashMem.getFloat("Kd", -1)));
  Serial.println("Km=" + String(flashMem.getFloat("Km", -1)));
  Serial.println("Headroom=" + String(flashMem.getFloat("Headroom", -1)));
  Serial.println("PWMfreq="  + String(flashMem.getFloat("PWMfreq" , -1)));
  Serial.println("PIDsettings=");
  Serial.print(flashMem.getBool("s0", false));
  Serial.print(flashMem.getBool("s1", false));
  Serial.print(flashMem.getBool("s2", false));
  Serial.print(flashMem.getBool("s3", false));
  Serial.print(flashMem.getBool("s4", false));
  Serial.println(flashMem.getBool("s5", false));
  flashMem.end(); 
  refreshVariablesFromMem(true);

  RTC_Time = DS1307.now();
  
  pullPRregister();

  PID_lastMillis = millis();

  ledcSetup(1,100,8);

  ledcAttachPin(27, 1);
  delay(20);

  processAutoMode();
  Serial.println("</STARTUP>");
}

void loop() {
  //ALLWAYS ACTIVE-----------------------------------------------------------------------------------
  
  unsigned int cappedEncoderValue  = (encoder.getCount() * -1);
  
  lcdOutput[0] = "";
  lcdOutput[1] = "";
  for (int y = 0; y < 32; y++) {
    lcdSymbols[y] = 0 - 1;
  }


  if(lastEncoderCount!=cappedEncoderValue){
    lastEncoderCount = cappedEncoderValue;
    lastMove = millis();
  }

  //     click handler
  encoderSwitch = false;


  if (lastSwtichState != digitalRead(encoderSwitchPin)) {  //switch pin change
  lastMove = millis();
    if (digitalRead(encoderSwitchPin) == 1) {              //it changed to off
      if(!encoderSwitchHold){    //the button was not held
        encoderSwitch = true;
      }             
    }
    else{ 
      uLongCacheSwitchHold = millis();
    }
  }
  encoderSwitchHoldOnce = false;
  
  if ((millis() - uLongCacheSwitchHold) > holdDelay) {
    if (digitalRead(encoderSwitchPin) == 0) {
      if(encoderSwitchHold){encoderSwitchHoldOnce = false;  } //only pulses once
      else{                 encoderSwitchHoldOnce = true;  }
      encoderSwitchHold = true;                               //holds one if button remains held
    }
    else{encoderSwitchHold = false;}
  }
  else{encoderSwitchHold = false;}
  lastSwtichState = digitalRead(encoderSwitchPin);


  // SLEEP MODE
  if(millis()-lastMove>60000){
    if(backlightState){
      backlightState = false;
      lcd.noBacklight();
      sleepMode = true;
      lcd.clear();
    }
  }
  else{
    if(!backlightState){
      sleepMode = false;
      backlightState=true;
      lcd.backlight();
    }
  }


  //    click handler end
  
  // 5 second refresh
  secRefresh5 = false;
  if (millis() - uLongCache5Sec >= 5000) {
    if(millis() - uLongCache5Sec >= 6000){
      Serial.println("<Info>5SecRefresh is " + String(millis() - uLongCache5Sec-5000)+ "ms behind</Info>");
      uLongCache5Sec=millis();
    }
    else{
      uLongCache5Sec += 5000;
    }
    secRefresh5 = true;
    

    sht.readSample();
    sensors.requestTemperatures();

    if(!PIDsettings[5]){
      currentSensorTemperature = sht.getTemperature();
      if(currentSensorTemperature != currentSensorTemperature){ // if the sensor isn't connected, it responds with -undefined- and it cannot be equal to itself
        Serial.println("<ERR>no sensor</ERR>");
        sht.init();
        sht.setAccuracy(SHTSensor::SHT_ACCURACY_MEDIUM);
        if(!internalSensorFailed){delay(200);}
        if(sht.getTemperature() != currentSensorTemperature  && !internalSensorFailed){ //second check and checks if it allready did that
          popupHandler(currentMenu, "internal temp sensor is broken or disconnected");
          internalSensorFailed = true;        
        }
      }
      else{
        if(internalSensorFailed){ // sensor works again (cannot happed)
          internalSensorFailed = false;
        }
      }
    }
    else{
      if(sensors.getTempCByIndex(0) == -127){ //external sensor is not installed
        sensors.requestTemperatures(); 
        delay(200);
        if(sensors.getTempCByIndex(0) == -127){ //second try if it responds
        currentSensorTemperature = sht.getTemperature();
        popupHandler(currentMenu, "external sensor is disconnected");
        PIDsettings[5] = false;
        }
        else{currentSensorTemperature = sensors.getTempCByIndex(0);}
      }
      else{currentSensorTemperature = sensors.getTempCByIndex(0);}
    }

    
  }
  bool secRefresh1 = false;

  // 1 second refresh
  if (millis() - uLongCache1Sec >= 1000) {
    RTC_Time = DS1307.now();
    secRefresh1 = true;
    if(millis() - uLongCache1Sec >= 1500){
      Serial.println("<Info>1SecRefresh is " + String(millis() - uLongCache1Sec-1000)+ "ms behind</Info>");
      uLongCache1Sec=millis();
    }
    else{
      uLongCache1Sec += 1000;
    }
    processAutoMode();
    if(autoMode){
      targetTemp = ((double) savedPlannedRecord.temperature) /2.00;
    }
    else{
      targetTemp = manualTemp;
    }

    ControllerOutput = 0;

    if(controllerSettingChanged){
      controllerSettingChanged = false;
      if(PIDsettings[2]){ledcSetup(1, 100, 8);}
      else{              ledcSetup(1, PWMfrequency, 8);}     
    }

    bool printOutPID = false;

    if(PIDsettings[4] && millis()-uLongCache60Sec > 10000){ //PrintPID - outputs current PID values
    uLongCache60Sec = millis();
    Serial.print("<OUT>");
    printOutPID = true;
    }

    if(PIDsettings[0]){ // 0 must be allways one

      int maxStr = 255;
      if(PIDsettings[2]){ //On-Off
        if((currentSensorTemperature+OnOffHeadroom)<targetTemp){ //HEATING
          ControllerOutput = maxStr;
        }
        if((currentSensorTemperature-OnOffHeadroom)>targetTemp){ //COOLING
          ControllerOutput = -maxStr;
        }
      }
      else{ //PID
        ControllerOutput = MyPID(currentSensorTemperature, targetTemp, Kp, Ki, Kd, maxStr, printOutPID);
      }


    }

    if(ControllerOutput>0&&PIDsettings[1]){ControllerOutput=0;} //output is positive and cooling mode enabled
    if(ControllerOutput<0&&!PIDsettings[1]){ControllerOutput=0;} //output is negative and heating mode enabled

    if(printOutPID){
        Serial.println(",controllerOutput:"+String(ControllerOutput)+",currentTemp:"+String(currentSensorTemperature)+",targetTemp:"+String(targetTemp)+"</OUT>");
    }

    ledcWrite(1,255 - ((int)  ControllerOutput));
  }
  String tempString = "";


  
  
  //ALLWAYS ACTIVE-----------------------------------------------------------------------------------



  //MENUS:
  //0-MainScreen
  //1-MainMenu
  if(sleepMode){
      drawFrames = false;
      if(secRefresh5){
        lcd.setCursor(0,0);
        String timeOutput ="";
        if(RTC_Time.hour() < 10){timeOutput+="0";}
        timeOutput += String(RTC_Time.hour()) + ":";
        if(RTC_Time.minute()<10){timeOutput+="0";}
        timeOutput +=  String(RTC_Time.minute());
        lcd.print(timeOutput);
        lcd.setCursor(0,1);
        lcd.print(String(currentSensorTemperature)+ "->" + String(targetTemp));        
      }
      if(encoderSwitch){
        if(currentMenu = 20){ //fixes the viewer freezing when sleep mode gets activated
          uIntCache = 560;
        }
        sleepMode = false;
      }
    }
  else{
  if(popupIsActive){
    drawFrames = true;
    if(popupStringCache.length() > LCDx){
      if(millis()-popupLongCache>400){
      popupLongCache = millis();
      popupIntCache=(popupIntCache+1)%(popupStringCache.length()-LCDx+3);
      }
      setStringAt(0, -popupIntCache+1, popupStringCache);
    }
    else{
      setStringAt(0,0,popupStringCache);
    }
    setStringAt(1, 6, ">OK<");
    if(encoderSwitch){
      currentMenu = popupIntCache2;
      popupIsActive = false;
      Serial.println(sht.readSample(), DEC);
      encoder.setCount(popupEncoderCache);
    }}
  else{
  if(editorIsActive){
    setStringAt(0, 0, stringCache2);
    setStringAt(1, 0, "=");
    setStringAt(1, 1, stringCache);
    int editingCursorPosition = constrainEncoder(int64Cache3);

    if (stringCache.length() != int64Cache3) {
      if(stringCache.length() > int64Cache3){stringCache.remove(int64Cache3);}
      if(stringCache.length() < int64Cache3){for (int i = 0; i < int64Cache3 - stringCache.length(); i++) {stringCache.concat(charCache);}}
      } //add placeholder characters to get to desired length
    if (boolCache) {
      if (millis() - uLongCache > 100) {  //blink timer
        boolCache2 = !boolCache2;
        uLongCache = millis();
      }
      if (boolCache2) {  
        if (int64Cache2 != encoder.getCount()) {int64Cache2 = encoder.getCount();} //dont blink if cycling through options
        else {setSymbolAt(1,1+(uIntCache), 15);}//blink
      }

      //change the value to something else
      stringCache.setCharAt(uIntCache, usableCharactersCache.charAt(constrainEncoder(usableCharactersCache.length())));


      /*if (cappedEncoderValue % 11 == 10) {
        stringCache.setCharAt(uIntCache, char(46));
      } else {
        stringCache.setCharAt(uIntCache, char((cappedEncoderValue % 11) + 48));
      }*/


    } else {setStringAt(1, 1 + (editingCursorPosition), "_");}

    if (encoderSwitchHoldOnce) {
      encoder.setCount(0);
      editorIsActive = false;
      editorjustFinished = true;
      }

    if (encoderSwitch) {
      if (boolCache) {  //browsing mode < editing mode
        encoder.setCount(int64Cache);
      } else {                                                                    //browsing mode > editing mode
        uIntCache = editingCursorPosition;                                        //for augmented cursor position
        int64Cache = encoder.getCount();                                          //for cursor position

        
        for (int i=0; i<usableCharactersCache.length(); i++){
          if(usableCharactersCache.charAt(i) == stringCache.charAt(uIntCache)){encoder.setCount(i*(0-1));}
        }
        

        /*if (stringCache.charAt(uIntCache) == 46) { encoder.setCount(-10); }       //if its a decimal point
        else {encoder.setCount((stringCache.charAt(uIntCache) - 48) * (0 - 1));}  //if its anything else*/
      }

      boolCache = !boolCache;
    }
    }
  else{
  switch (currentMenu) {
    case -2:
      { // testing
      int custom_cappedEncoderValue = cappedEncoderValue%256;
      ledcWrite(1,custom_cappedEncoderValue);
      setStringAt(0,0,String(custom_cappedEncoderValue));

      if(encoderSwitchHoldOnce){
        currentMenu = 0;
        encoder.setCount(0);
      }
      break;
      }
    case -1:
      {
      if(editorjustFinished == false){ // used to detect first inticialization of power saving mode (editorJustFinished is rarely used and thus very likely to be false)
        editorjustFinished = true;
        lcd.clear();
        uIntCache = cappedEncoderValue;
      }
      if(!boolCache2 || secRefresh5){
        lcd.setCursor(0,0);
        String timeOutput ="";
        if(RTC_Time.hour() < 10){timeOutput+="0";}
        timeOutput = String(RTC_Time.hour()) + ":";
        if(RTC_Time.minute()<10){timeOutput+="0";}
        timeOutput +=  String(RTC_Time.minute());
        lcd.print(timeOutput);
        lcd.setCursor(0,1);
        lcd.print(String(currentSensorTemperature)+ "->" + String(targetTemp));

        //Shows if its heating/cooling or neutral
        lcd.setCursor(15,1);
        int tempInt = 0;
        if(ControllerOutput >0){lcd.createChar(0, arrowUpSymbol); lcd.write(0);       tempInt = 17;}else{//heating
        if(ControllerOutput <0){lcd.createChar(0, arrowDownSymbol); lcd.write(0);     tempInt = 18;}     //cooling
        else{                   lcd.createChar(0, blockInMiddleSymbol); lcd.write(0); tempInt = 19;}}    //neutral
        registeredSymbols[0] = tempInt; //fixes symbol library
      }
      /*if(cappedEncoderValue!=uIntCache){
        currentMenu = int64Cache;
        editorjustFinished = false;
      }*/
      if(encoderSwitch){
        currentMenu = sourceMenuCache;
        editorjustFinished = false;
      }
      break;}
    case 0:
      {                       //---------------0 - MAIN SCREEN---------------------------------
        int custom_cappedEncoderValue = constrainEncoder(4);
        drawFrames = true;
        char tempChar = 223;  //°
        tempString = String(round((currentSensorTemperature * 10)) / 10);
        tempString.remove(4, 1);

        setSymbolAt(0, 1, 5);  //temp
        setStringAt(0, 2, tempString);
        setSymbolAt(0, 6, 1);  //celsius
        
        if(autoMode){
          setStringAt(0,15,"A");
        }
        else{
          setStringAt(0,15,"M");
        }

        int absoluteTargetTemp;
        if(targetTemp<0){
          setSymbolAt(1, 1, 23);
          absoluteTargetTemp = -targetTemp;
        }
        else{
          setSymbolAt(1, 1, 16);
          absoluteTargetTemp = targetTemp;
        }
        tempString = String(round((absoluteTargetTemp * 10)) / 10);
        tempString.remove(4, 1);

        setStringAt(1, 2, tempString);
        setSymbolAt(1, 6, 1);  //celsius

        if(!PIDsettings[5]){
          tempString = String(round(sht.getHumidity()));
          tempString.remove(2, 3);
          setSymbolAt(0, 10, 3);
          setStringAt(0, 11, tempString + "%");
        }

        {
        String currentItemName = "";
        switch(custom_cappedEncoderValue){
          case 0:{
            currentItemName = ">Pref";
            break;
          }
          case 1:{
            currentItemName = ">Auto";
            break;
          }
          case 2:{
            currentItemName = ">Mode";
            break;
          }
          case 3:{
            currentItemName = ">Exit";
            break;
          }
          case 4:{
            currentItemName = ">TEST";
            break;
          }
        }
        setStringAt(1,10,currentItemName);
        }

        if(encoderSwitchHoldOnce){
          tempSettingHandler(0);
        }

        if(encoderSwitch){
          switch(custom_cappedEncoderValue){
            case 0:{
              currentMenu = 1;
              encoder.setCount(0);
              refreshVariablesFromMem(true);
              switchSettingsPickMode = false;
              settingsPickMode = false;
              int64Cache = 0;
              break;
            }
            case 1:{
              encoder.setCount(0);
              currentMenu = 2;
              break;
            }
            case 2:{
              autoMode = !autoMode;
              break;
            }
            case 3:{
              powerSavingModeHandler();
              break;
            }
            case 4:{
              nextAutoChange = RTC_Time;
              break;
            }
            default:{
              currentMenu = cappedEncoderValue;
            }
          }
        }
        break;
      }  //---------------0 - END---------------------------------------


    case 1:
      {  //---------------1 - Variable editor  ---------------------------------
      
        drawFrames = true;
        //editor result catch
        if(editorjustFinished){
          editorjustFinished = false;
          switch(editorIntCache){
            case 1:{
              Kp = stringCache.toFloat();
              break;
            }
            case 2:{
              Ki = stringCache.toFloat();
              break;
            }
            case 3:{
              Kd = stringCache.toFloat();
              break;
            }
            case 4:{
              Km = stringCache.toFloat() /100.0;
              break;
            }
            case 5:{
              if(PIDsettings[2]){OnOffHeadroom = stringCache.toFloat();}
              else{              PWMfrequency  = stringCache.toFloat();}
              
              break;
            }
            case 6:{  //time
              // example: 26.4.2016 9:10:11 > 2016, 4, 26, 9, 10, 11
              String tempString = "";
              int hour =-1;
              int minute =-1;
              int second =-1;
              byte currentIndex = 0;
              bool validSyntax = true;
              stringCache+=":";
              Serial.println("<Debug>");
              for(int i=0; i<stringCache.length();i++){
                if(!validSyntax){
                  break;
                }
                if(stringCache.charAt(i) == 58){
                  Serial.println("found : at" + String(i));
                  if(tempString.length()==0){
                    validSyntax = false;
                    break;
                  }
                  switch(currentIndex){
                    case 0:{
                      hour = tempString.toInt();
                      if(hour<0||hour>24){
                        Serial.println("hour: " + String(hour));
                        validSyntax = false;
                      }
                      break;
                    } 
                    case 1:{
                      minute = tempString.toInt();
                      if(minute<0||minute>60){
                        Serial.println("minute: " + String(minute));
                        validSyntax = false;
                      }
                      break;
                    } 
                    case 2:{
                      second = tempString.toInt();
                      if(second<0||second>60){
                        Serial.println("second: " + String(second));
                        validSyntax = false;
                      }
                      break;
                    } 
                    default:{
                      Serial.println("something went wrong with parsing time, " + String(stringCache) +" " + String(currentIndex));
                      validSyntax = false;
                      break;
                    }
                  }
                  tempString = "";
                  currentIndex++;
                }
                else{
                  tempString+=stringCache.charAt(i);
                }
              }
              if(currentIndex!=3){
                Serial.println("length");
                validSyntax = false;
              }
              Serial.println(stringCache);
              Serial.println(String(hour) + ":" + String(minute) + ":" + String(second));
              if(!validSyntax){popupHandler(currentMenu, "wrong time syntax");}
              else{
                DateTime PreparedDateTime(RTC_Time.year(), RTC_Time.month(), RTC_Time.day(), hour, minute, second); 
                if(PreparedDateTime.isValid()){
                  DS1307.adjust(PreparedDateTime);
                }
                else{
                  Serial.println("invalid datetime");
                  popupHandler(currentMenu, "invalid datetime");
                }
              }
              Serial.println("</Debug>");
              break;
            }
            case 7:{  //date
              // example: 26.4.2016 9:10:11 > 2016, 4, 26, 9, 10, 11
              String tempString = "";
              int year =-1;
              int month =-1;
              int day =-1;
              byte currentIndex = 0;
              bool validSyntax = true;
              stringCache+=".";
              Serial.println("<Debug>");
              for(int i=0; i<stringCache.length();i++){
                if(!validSyntax){
                  break;
                }
                if(stringCache.charAt(i) == 46){
                  Serial.println("found : at" + String(i));
                  if(tempString.length()==0){
                    validSyntax = false;
                    break;
                  }
                  switch(currentIndex){
                    case 2:{
                      year = tempString.toInt();
                      if(year<0||year>9999){
                        Serial.println("year: " + String(year));
                        validSyntax = false;
                      }
                      break;
                    } 
                    case 1:{
                      month = tempString.toInt();
                      if(month<1||month>12){
                        Serial.println("month: " + String(month));
                        validSyntax = false;
                      }
                      break;
                    } 
                    case 0:{
                      day = tempString.toInt();
                      if(day<1||day>31){
                        Serial.println("day: " + String(day));
                        validSyntax = false;
                      }
                      break;
                    } 
                    default:{
                      Serial.println("something went wrong with parsing time, " + String(stringCache) +" " + String(currentIndex));
                      validSyntax = false;
                      break;
                    }
                  }
                  tempString = "";
                  currentIndex++;
                }
                else{
                  tempString+=stringCache.charAt(i);
                }
              }
              if(currentIndex!=3){
                Serial.println("length");
                validSyntax = false;
              }
              if(validSyntax){
                DateTime PreparedDateTime(year, month, day, RTC_Time.hour(), RTC_Time.minute(), RTC_Time.second());          
                if(PreparedDateTime.isValid()){
                  Serial.println("valid date");
                  DS1307.adjust(PreparedDateTime);
                }
                else{
                  Serial.println("invalid datetime");
                  popupHandler(currentMenu, "invalid datetime");
                }
              }
              else{popupHandler(currentMenu, "wrong time syntax");}

              Serial.println("</Debug>");
              break;
            }
            default:{
              Serial.println("<Info>editor in page 1 (preferences), returned invalid index" + String(editorIntCache)+"</Info>");
              break;
            }
          }
        }

        //local variables
        int numberOfElements = 13;
        int PIDsettingOffsetIndex = 3;
        int custom_cappedEncoderValue = constrainEncoder(numberOfElements);
        int outputEncoderValue = 0;
        String emptyPH = "---";
        char degreeChar = 223;

        //page input
        String storedPages[] = {"error",emptyPH,emptyPH,emptyPH,emptyPH,emptyPH,emptyPH,emptyPH,emptyPH,emptyPH,emptyPH,emptyPH,emptyPH,emptyPH};
        storedPages[0] = "Kp=" + String(Kp);
        storedPages[1] = "Ki=" + String(Ki);
        storedPages[2] = "Kd=" + String(Kd);
        storedPages[3] = "Km=" + String(Km);
        storedPages[4] = "mode=" + getStringFromLib(PIDsettings[1], 0);
        storedPages[5] = "control=" + getStringFromLib(PIDsettings[2], 1);
        if(PIDsettings[2]){
          storedPages[6] = "Headroom=" + String(OnOffHeadroom) + degreeChar + "C"; //"output=" + getStringFromLib(PIDsettings[3], 2);
        }
        else{
          storedPages[6] = "PWMfreq=" + String(PWMfrequency) + "hz"; 
        }
        storedPages[7] = "PrintOut=" + getStringFromLib(PIDsettings[4], 3);
        storedPages[8] = "Sensor=" + getStringFromLib(PIDsettings[5], 4);
        storedPages[9] = "Time=" +  String(RTC_Time.hour()) + ":" + String(RTC_Time.minute()) + ":" + String(RTC_Time.second());
        storedPages[10] = "Date=" + String(RTC_Time.day()) + "." + String(RTC_Time.month()) + "." + String(RTC_Time.year());
        storedPages[11] = "reset PID cache";
        storedPages[12] = "factory default";


        //bool editor handler
        if(switchSettingsPickMode){ //switch state and encoder value handler
          switchSettingsPickMode = false;
          if(!settingsPickMode){ //from standart
            if(storedPages[int64Cache] == emptyPH){
              Serial.println("<Debug>user tried to edit a disabled variable " + String(int64Cache-PIDsettingOffsetIndex) + " with curr value: " + String(PIDsettings[int64Cache-PIDsettingOffsetIndex]) + "</Debug>");
              settingsPickMode = false;
            }
            else{
            uIntCache = encoder.getCount();
            encoder.setCount(PIDsettings[int64Cache-PIDsettingOffsetIndex]);
            settingsPickMode = true;
            Serial.println("<Debug>editing var number " + String(int64Cache-PIDsettingOffsetIndex) + " with curr value: " + String(PIDsettings[int64Cache-PIDsettingOffsetIndex])+"</Debug>");
            }}
          else{encoder.setCount(uIntCache); settingsPickMode = false; Serial.println("<Debug>value of var number " + String(int64Cache-PIDsettingOffsetIndex)  + " changed to: " + String(PIDsettings[int64Cache-PIDsettingOffsetIndex]) + "</Debug>"); encoderSwitch = false;} //back to standart 
        }
        else{
          if(!settingsPickMode){int64Cache = custom_cappedEncoderValue;}//standart
          else{outputEncoderValue = constrainEncoder(2); PIDsettings[int64Cache-PIDsettingOffsetIndex] = outputEncoderValue;}//setting
        }
        

        //if(storedPages[int64Cache] == "empty"){encoder.setCount(encoder.getCount()-1);}
        setStringAt(0,1,storedPages[int64Cache]);                
        if(int64Cache != sizeof(storedPages)){setStringAt(1,1,storedPages[int64Cache+1]); }
        setStringAt(0,0,">");


        if(encoderSwitchHoldOnce){
          saveScreenHandler(0);
          break;
        }

        if(encoderSwitch){
          if(settingsPickMode){
            switchSettingsPickMode = true;
            break;
          }
          else{
          switch (custom_cappedEncoderValue) {
            case 0:{
              editorHandler(String(Kp),"proportional", 10, 48, "0123456789.-", 1);
              break;
            }
            case 1:{
              editorHandler(String(Ki),"integral", 10, 48, "0123456789.-", 2);
              break;
            }
            case 2:{
              editorHandler(String(Kd),"derival", 10, 48, "0123456789.-", 3);
              break;
            }
            case 3:{
              editorHandler(String(Km*100.0),"multiplier / 100", 10, 48, "0123456789.-", 4);
              break;
            }
            case 4:{
              switchSettingsPickMode = true;
              break;
            }
            case 5:{
              switchSettingsPickMode = true;
              break;
            }
            case 6:{
              if(PIDsettings[2]){
                String tempString22 = "temp headroom  C";
                tempString22[14] = 223;
                editorHandler(String(OnOffHeadroom),tempString22, 10, 48, "0123456789.", 5);
              }
              else{
                editorHandler(String(PWMfrequency),"PWM frequency hz", 10, 32, "0123456789 ", 5);
              }
              break;
            }
            case 7:{
              switchSettingsPickMode = true;
              break;
            }
            case 8:{
              switchSettingsPickMode = true;
              break;
            }
            case 9:{
              editorHandler(String(RTC_Time.hour()) + ":" + String(RTC_Time.minute()) + ":" + String(RTC_Time.second()),"TIME (H:M:S)", 8, 32, "0123456789:", 6);
              break;
            }
            case 10:{
              editorHandler(String(RTC_Time.day()) + "." + String(RTC_Time.month()) + "." + String(RTC_Time.year()),"DATE (D.M.Y)", 10, 32, "0123456789.", 7);
              break;
            }            

            case 11:{
              PID_lastErr = 0;
              PID_lastMillis = millis();
              PID_cumulativeErr = 0;
              popupHandler(1, "PID cache cleared");
              break;
            }           
            case 12:{
              popupHandler(12, "WARNING - YOU ARE ABOUT TO ERASE ALL SETTINGS");
              break;
            } 
            default:{
              Serial.println("<Debug>wrong select ID<Debug>");
              break;
            }
          }
        }}
        break;
      }  //---------------99 - END---------------------------------------

    case 2:
      {  //---------------1 - MAIN MENU---------------------------------
        encoder.getCount();  //encoder responds wrong if not called here 
        drawFrames = true;
        int custom_cappedEncoderValue = constrainEncoder(4);
        setStringAt(0,0,"planned events");
        switch(custom_cappedEncoderValue){
          case 0:{
            setStringAt(1,1,"view");
            break;
          }
          case 1:{
            setStringAt(1,1,"edit");
            break;
          }
          case 2:{
            setStringAt(1,1,"add");
            break;
          }
          case 3:{
            setStringAt(1,1,"remove");
            break;
          }
          default:{
            encoder.setCount(0);
            Serial.print("<Debug>wrong tab in menu 2</Debug>");
            break;
          }
        }
        setStringAt(1, 0, ">");

        scrollComponentSpecific(custom_cappedEncoderValue, 4, 1);

        if(encoderSwitch){
          int64Cache4 = custom_cappedEncoderValue;
          switch(custom_cappedEncoderValue){
            case 0:{ //view
              currentMenu = 20;
              encoder.setCount(0);
              uIntCache = 560;
              break;
            }
            case 1:{ //edit
              currentMenu = 20;
              encoder.setCount(0);
              uIntCache = 560;
              break;
            }
            case 2:{ //add
              currentMenu = 221;
              boolCache3 = false;
              break;
            }
            case 3:{ //remove
              currentMenu = 20;
              encoder.setCount(0);
              uIntCache = 560;            
              break;
            }
            default:{
              encoder.setCount(0);
              Serial.print("<Debug>wrong selected tab in menu 2</Debug>");
              break;
            }
          }
        }

        if(encoderSwitchHoldOnce){
          currentMenu = 0;
          encoder.setCount(0);
          lcd.clear();
          lcd.print("saving values");
          pushPRRegister();          
          drawFrames = false;
          nextAutoChange = RTC_Time;
          break;
        }
        break;
      }  //---------------1 - END---------------------------------------

    case 20:
      {  //---------------1 - MAIN MENU--------------------------------
        if(registeredPlannedRecords[32] == 0){
          popupHandler(2, "no entries found");
          break;
        }
        if(drawFrames){
          int64Cache = registeredPlannedRecords[32];
          if(int64Cache%2==1){int64Cache++;}
        }

        int custom_cappedEncoderValue = constrainEncoder(int64Cache);
        int currentPage = custom_cappedEncoderValue/2;
        
        if(uIntCache!=currentPage){ // encoder moved to another page
        uIntCache = currentPage;
        encoder.pauseCount();

        //PlannedRecordObject PRarray[] = {PlannedRecordObject("---", 0, 0, 0, 0), PlannedRecordObject("---", 0, 0, 0, 0)};
        String PRarray[] = {"---", "---"};


        byte foundEntries = 0;
        byte displayedEntries = 0;
        Serial.print("<PRviewSearch>\nsearching for " +String(registeredPlannedRecords[32])+ " entries");

        flashMem.begin(flashNameSpace, true);         
        for(int pos=0; pos<32; pos++){
          if(displayedEntries>1||foundEntries>=registeredPlannedRecords[32]){break;}
          Serial.print("\n"+String(pos));

          if(registeredPlannedRecords[pos]>0){
            Serial.print("*");
            for(int bit=0; bit<8; bit++){

              if(bitRead(registeredPlannedRecords[pos], bit)){
                if(displayedEntries>1){break;}
                foundEntries++;
                Serial.print(" " + String(bit));

                if(foundEntries > (currentPage*2)){       
                  String charTempCache = getPRfromMemNAME(pos, bit);
                  PRarray[displayedEntries] = "";
                  Serial.print("-{");
                  for(int i=0; i<10; i++){
                    PRarray[displayedEntries]+=charTempCache[i];
                    Serial.print(PRarray[displayedEntries][i]);
                  }
                  Serial.print("}");
                  if(displayedEntries == 0){
                    int64Cache2 = (pos*8)+bit;
                  }
                  else{
                    int64Cache3 = (pos*8)+bit;                    
                  }

                  displayedEntries++;                  
                }

              }
              
            }
          }
          
        }
        flashMem.end();
        if(displayedEntries==1){
        int64Cache3 = -1;
        }

        lcd.clear();
        lcd.setCursor(1,0);
        lcd.print(PRarray[0]);
        lcd.setCursor(1,1);
        lcd.print(PRarray[1]);
        lcd.setCursor(13, 0);
        lcd.print(String(currentPage+1));
        lcd.setCursor(13,1);
        lcd.print(String(int64Cache/2));
        encoder.resumeCount();

        Serial.println("\n</PRviewSearch>");
        }

        lcd.setCursor(0, cappedEncoderValue%2    ); lcd.print(">");
        lcd.setCursor(0, (cappedEncoderValue+1)%2); lcd.print(" ");        
        
        drawFrames = false;
        if(encoderSwitchHoldOnce){
          currentMenu=2;
          encoder.setCount(0);
          break;
        }
        if(encoderSwitch){
          bool PROcacheLoaded = false;
          if(cappedEncoderValue%2 == 0){
            PROcache = getPRfromMem(int64Cache2/8, int64Cache2%8);       
            PROcacheLoaded = true;   
          }
          else{
            if(int64Cache3!=-1){
              PROcache = getPRfromMem(int64Cache3/8, int64Cache3%8);
              PROcacheLoaded = true;
            }
          }
          if(PROcacheLoaded){
            if(cappedEncoderValue%2 == 0){int64Cache5 = int64Cache2;} // save currently loaded PRs position to int64Cache5
            else{int64Cache5 = int64Cache3;}
            Serial.println("<Info>" + String(PROcache.objectDetails()) + "</Info>");         
            switch(int64Cache4){ // different operation switch
              case 0:{ // view operation
                currentMenu = 222;      
                settingsPickMode = true;
                int64Cache = false;
                boolCache = false;   
                encoder.setCount(0);       
                break;
              }
              case 1:{ // edit operation
                currentMenu = 221;
                boolCache3 = false;
                encoder.setCount(0);
                break;
              }
              case 3:{ // remove operation
                currentMenu = 223;
                encoder.setCount(0);
                break;
              }
              default:{
                Serial.println("<Debug>unhandled operation at 20" + String(int64Cache4)+"</Debug>");
                popupHandler(2, "unhandled operation error" + String(int64Cache4));
                break;
              }
            } 
          }
        }

        break;
      }  //---------------1 - END---------------------------------------
    case 21:
      {
        drawFrames = true;
        byte LASTnumOfEntries = registeredPlannedRecords[32];
        lcd.clear();
        lcd.setCursor(0,0);
        lcd.print("saving...           ");
        lcd.setCursor(1,1);
        lcd.print("                    ");      
        if(PROcache.writeToFlashMem(int64Cache5/8, int64Cache5%8)){
          Serial.println("<Info>successfully edited PR on pos " + String(int64Cache5/8) + " " + String(int64Cache5%8) + " " + PROcache.objectDetails()+"</Info>");
        }
        else{
          Serial.println("<ERR>error while saving edited PR to memory<ERR>");
          popupHandler(2,"error while saving to memory");
        }
        registeredPlannedRecords[32] = LASTnumOfEntries;
        encoder.setCount(0);
        uIntCache = 560; 
        currentMenu = 20;
        break;      
      }
    case 221:
      {
      drawFrames = true;
      if(editorjustFinished){
          editorjustFinished = false;
          switch(editorIntCache){
            case 1:{
              Serial.print("<Debug>name: " + stringCache);
              for(int i=0;i<10;i++){
                PROcache.name[i] = stringCache.charAt(i);
                Serial.print(stringCache.charAt(i));
              }
              Serial.println("</Debug>");
              boolCache3 = true;
              break;
            }
            default:{
              Serial.println("<Debug>editor in page 22 (add new planned record), returned invalid index" + String(editorIntCache)+"</Debug>");
              break;
            }            
          }
      }

  
      if(!boolCache3){ //set name
        switch(int64Cache4){
          case 2:{
            String unnamedString  = "unnamed" + String(registeredPlannedRecords[32]) + "   ";
            for(int i=0; i<10;i++){
              PROcache.name[i] = unnamedString[i];
            }
          }
          default:{
            break;
          }
        }
        editorHandler(PROcache.name,"set name", 10,32," abcdefghijklmnopqrstuvwxyz 1234567890 .-/()",1);
      }
      else{
        currentMenu = 222;
        settingsPickMode = true;
        int64Cache = false;
        boolCache = false;
        encoder.setCount(0);
      }
      break;
      }

    case 222:
      {
      drawFrames = true;
        if(settingsPickMode){
          int64Cache = cappedEncoderValue%16;
          
        }
        
        //setStringAt(0,0,"20:00-21:10  25");
        byte Shour10 =   PROcache.startTime/1000;
        byte Shour1  =  (PROcache.startTime/100)%10;
        byte Sminute10 =(PROcache.startTime%100)/10;
        byte Sminute1 =  PROcache.startTime%10;
        byte Ehour10 =   PROcache.endTime/1000;
        byte Ehour1  =  (PROcache.endTime/100)%10;
        byte Eminute10 =(PROcache.endTime%100)/10;
        byte Eminute1 =  PROcache.endTime%10;
        double PROtemp = ((double)PROcache.temperature)/2.0;
        setStringAt(0,0, String(Shour10) + String(Shour1) + ":" + String(Sminute10) + String(Sminute1) + "-" + String(Ehour10) + String(Ehour1) + ":" + String(Eminute10) + String(Eminute1));
        if(PROtemp<0){
          if(PROtemp<-9.5){
            setStringAt(0,11, String(PROtemp));
          }else{
            setStringAt(0,12,String(PROtemp));
          }
        }
        else{
          if(PROtemp>9.5){
            setStringAt(0,12, String(PROtemp));
          }else{
            setStringAt(0,13,String(PROtemp));
          }
        }
        
        if(PROcache.temperature%2!=0){
          setSymbolAt(0,14,21);
        }else{
          setSymbolAt(0,14,22);
        }


        
        setSymbolAt(0,15,1);
        setStringAt(1,1,"M T W T F S S");
        for(int i=0; i<7; i++){
          if(bitRead(PROcache.daysOfWeek,i)){
            setSymbolAt(1,2+(i*2),11);
          }
          else{
            setSymbolAt(1,2+(i*2),13);
          }
        }

        bool selectedBlink = false;
        if(int64Cache4 != 0){ //do not blink if just viewing
        if(settingsPickMode){
          if(millis()-uLongCache > 600){
          if(millis()-uLongCache > 1200){uLongCache = millis();}
          selectedBlink = true;
          }          
        }
        else{
          if(millis()-uLongCache > 200){
          if(millis()-uLongCache > 400){uLongCache = millis();}
          selectedBlink = true;
          }
        }
        
        if(cappedEncoderValue!=uIntCache){
          uIntCache = cappedEncoderValue;
          if(settingsPickMode){ //blink while browsing
            selectedBlink = true;
            uLongCache = 0;
          }
          else{//do not blink while browsing
            selectedBlink = false;
            uLongCache = millis();
          }
        }
        }

      if(int64Cache4 != 0){ // do not let the user go to editing mode if just viewing
      if(encoderSwitch){
        if(settingsPickMode){ //from picking to setting
          settingsPickMode = false;
          int64Cache2 = encoder.getCount();
        }
        else{
          boolCache = false;
          settingsPickMode = true;
          encoder.setCount(int64Cache2);
        }
      }
      }
    
        

        switch(int64Cache){
          case 0:{ //Shour10
            if(!settingsPickMode){
              if(!boolCache){ boolCache = true; encoder.setCount(-Shour10);} //if the pick mode just switched, set current encoder value to current value
              else{PROcache.startTime = (PROcache.startTime%1000) + 1000*(constrainEncoder(3));} 
            }
            if(selectedBlink){ setStringAt(0,0," ");}
            break;
          }
          case 1:{ //Shour1
            if(!settingsPickMode){
              if(!boolCache){ boolCache = true; encoder.setCount(-Shour1);} //if the pick mode just switched, set current encoder value to current value
              else{PROcache.startTime = (PROcache.startTime-(((PROcache.startTime/100)%10)*100)) + 100*(constrainEncoder(10));} 
            }
            if(selectedBlink){ setSymbolAt(0,1,15);}

            break;
          }
          case 2:{ //Sminute10
            if(!settingsPickMode){
              if(!boolCache){ boolCache = true; encoder.setCount(-Sminute10);} //if the pick mode just switched, set current encoder value to current value
              else{PROcache.startTime = (PROcache.startTime-(((PROcache.startTime/10)%10)*10)) + 10*(constrainEncoder(10));} 
            }
            if(selectedBlink){ setSymbolAt(0,3,15);}
            break;

          }
          case 3:{ //Sminute1
            if(!settingsPickMode){
              if(!boolCache){ boolCache = true; encoder.setCount(-Sminute1);} //if the pick mode just switched, set current encoder value to current value
              else{PROcache.startTime = (PROcache.startTime-(PROcache.startTime%10)) + (constrainEncoder(10));} 
            }
            if(selectedBlink){ setSymbolAt(0,4,15);}
            break;

          }
          case 4:{ //Ehour10
            if(!settingsPickMode){
              if(!boolCache){ boolCache = true; encoder.setCount(-Ehour10);} //if the pick mode just switched, set current encoder value to current value
              else{PROcache.endTime = (PROcache.endTime%1000) + 1000*(constrainEncoder(3));} 
            }
            if(selectedBlink){ setSymbolAt(0,6,15);}
            break;

          }
          case 5:{ //Ehour1
            if(!settingsPickMode){
              if(!boolCache){ boolCache = true; encoder.setCount(-Ehour1);} //if the pick mode just switched, set current encoder value to current value
              else{PROcache.endTime = (PROcache.endTime-(((PROcache.endTime/100)%10)*100)) + 100*(constrainEncoder(10));} 
            }
            if(selectedBlink){ setSymbolAt(0,7,15);}
            break;

          }
          case 6:{ //Eminute10
            if(!settingsPickMode){
              if(!boolCache){ boolCache = true; encoder.setCount(-Eminute10);} //if the pick mode just switched, set current encoder value to current value
              else{PROcache.endTime = (PROcache.endTime-(((PROcache.endTime/10)%10)*10)) + 10*(constrainEncoder(10));} 
            }
            if(selectedBlink){ setSymbolAt(0,9,15);}
            break;

          }
          case 7:{ //Eminute1
            if(!settingsPickMode){
              if(!boolCache){ boolCache = true; encoder.setCount(-Eminute1);} //if the pick mode just switched, set current encoder value to current value
              else{PROcache.endTime = (PROcache.endTime-(PROcache.endTime%10)) + (constrainEncoder(10));} 
            }
            if(selectedBlink){
              setSymbolAt(0,10,15);
            }
            break;

          }
          case 8:{ //temperature
            if(!settingsPickMode){
              if(!boolCache){boolCache = true; encoder.setCount(-(PROtemp*2));}
              else{
                if((-encoder.getCount())<127 || encoder.getCount()<127){
                  encoder.setCount(0);
                }
                PROcache.temperature = (-encoder.getCount())%127;
                }
            }
            if(selectedBlink){
              setSymbolAt(0,14,15);
              setSymbolAt(0,13,15);
              setSymbolAt(0,12,15);
            }
            break;

          }
          case 9:{ //monday
            if(!settingsPickMode){
              /*
              if(!boolCache){ boolCache = true; encoder.setCount(bitRead(PROcache.daysOfWeek, 0));} //if the pick mode just switched, set current encoder value to current value
              else{bitWrite(PROcache.daysOfWeek,0,cappedEncoderValue%2);}*/
              bitWrite(PROcache.daysOfWeek,0,!bitRead(PROcache.daysOfWeek,0));
              settingsPickMode = true;
            }
            if(selectedBlink){
              setSymbolAt(1,2,15);
            }
            break;

          }
          case 10:{ //tuesday
            if(!settingsPickMode){
              bitWrite(PROcache.daysOfWeek,1,!bitRead(PROcache.daysOfWeek,1));
              settingsPickMode = true;
            }
            if(selectedBlink){
              setSymbolAt(1,4,15);
            }
            break;

          }
          case 11:{ //Wednesday
            if(!settingsPickMode){
              bitWrite(PROcache.daysOfWeek,2,!bitRead(PROcache.daysOfWeek,2));
              settingsPickMode = true;
            }
            if(selectedBlink){
              setSymbolAt(1,6,15);
            }
            break;

          }
          case 12:{ //Thursday
            if(!settingsPickMode){
              bitWrite(PROcache.daysOfWeek,3,!bitRead(PROcache.daysOfWeek,3));
              settingsPickMode = true;
            }
            if(selectedBlink){
              setSymbolAt(1,8,15);
            }
            break;

          }
          case 13:{ //Friday
            if(!settingsPickMode){
              bitWrite(PROcache.daysOfWeek,4,!bitRead(PROcache.daysOfWeek,4));
              settingsPickMode = true;
            }
            if(selectedBlink){
              setSymbolAt(1,10,15);
            }
            break;

          }
          case 14:{ //Saturday
            if(!settingsPickMode){
              bitWrite(PROcache.daysOfWeek,5,!bitRead(PROcache.daysOfWeek,5));
              settingsPickMode = true;
            }
            if(selectedBlink){
              setSymbolAt(1,12,15);
            }
            break;

          }
          case 15:{ //Sunday
            if(!settingsPickMode){
              bitWrite(PROcache.daysOfWeek,6,!bitRead(PROcache.daysOfWeek,6));
              settingsPickMode = true;
            }
            if(selectedBlink){
              setSymbolAt(1,14,15);
            }
            break;

          }

          default:{
            Serial.println("<Debug>unidentified index in page 22 (add new planned record), " + String(int64Cache)+"<Debug>");
          }
          
        }


      if(encoderSwitchHoldOnce){
        {
        DateTime PreparedDateTime(RTC_Time.year(), RTC_Time.month(), RTC_Time.day(), PROcache.startTime/100, PROcache.startTime%100, RTC_Time.second());          
        if(PreparedDateTime.isValid() || int64Cache4 == 0){
          Serial.println("<Info>start time valid<Info>");          
        }
        else{
          Serial.println("<Info>start time invalid<Info>");
          popupHandler(222, "start time invalid");
          break;   
        }}
        {
        DateTime PreparedDateTime(RTC_Time.year(), RTC_Time.month(), RTC_Time.day(), PROcache.endTime/100, PROcache.endTime%100, RTC_Time.second());  
        if(PreparedDateTime.isValid() || int64Cache4 == 0){
          Serial.println("<Info>end time valid<Info>");          
        }
        else{
          Serial.println("<Info>end time invalid<Info>");
          popupHandler(222, "end time invalid");
          break;   
        }}
        

        
        switch(int64Cache4){
          case 0:{ //viewing
            uIntCache = 560; 
            currentMenu = 20;
            encoder.setCount(0);
            break;            
          }
          case 1:{ //editing
            currentMenu = 223;
            encoder.setCount(0);
            Serial.println(PROcache.objectDetails());
            break;         
          }
          case 2:{ //adding
            currentMenu = 223;
            encoder.setCount(0);
            Serial.println(PROcache.objectDetails());
            break;
          }
          default:{
            Serial.println("<Debug>unhandled operation at 222<Debug>" + String(int64Cache4));
            popupHandler(2, "unhandled operation error" + String(int64Cache4));
            break;
          }
        }
        
      }
      break;
      }

    case 223:
      {
      drawFrames = true;
      int custom_cappedEncoderValue = cappedEncoderValue%2;
      if(int64Cache4 == 3){
        setStringAt(0, 0, "delete from mem?");
      }
      else{
      setStringAt(0, 2, "save to mem?");
      }
      if(custom_cappedEncoderValue == 1){setStringAt(1, 2, ">yes<");}
      else{                              setStringAt(1, 2, "|yes|");}
      if(custom_cappedEncoderValue == 0){setStringAt(1, 9, ">no<");}
      else{                              setStringAt(1, 9, "|no|");}

      if(encoderSwitch){
        switch(int64Cache4){
        case 1: {  //editing
        if(custom_cappedEncoderValue){
          currentMenu = 21;
        }
        else{
          encoder.setCount(0);
          uIntCache = 560;
          currentMenu = 20;
        }
        break;
        }
        case 2: {  //adding new
        if(custom_cappedEncoderValue){
          lcd.setCursor(0,0);
          lcd.print("saving...           ");
          lcd.setCursor(1,1);
          lcd.print("                    ");
          if(!PROcache.saveToFlashMem()){
            popupHandler(222,"failed saving to memory");
            encoder.setCount(0);
          }     
          else{
            currentMenu = 2;
            encoder.setCount(0);
          }   
        }        
        else{
          currentMenu = 2;
          encoder.setCount(0);       
        }
        break;}
        case 3: { //removing
          if(custom_cappedEncoderValue){
            currentMenu = 23;
            encoder.setCount(0);
          }
          else{
            currentMenu = 20;
            uIntCache = 560;
            encoder.setCount(0);   
          }
          break;
        }
        default:{
          Serial.println("<Debug>unhandled operation at 223" + String(int64Cache4)+"</Debug>");
          popupHandler(2, "unhandled operation error" + String(int64Cache4));
          break;
        }
        }
      }

        
      break;
      }
    case 23:
      {
      drawFrames = true;
      bitWrite(registeredPlannedRecords[int64Cache5/8], int64Cache5%8, 0);
      registeredPlannedRecords[32]--;
      Serial.println("<Info>successfully deleted PR from pos " + String(int64Cache5/8) + " " + String(int64Cache5%8) + " " + PROcache.objectDetails()+"<Info>");
      encoder.setCount(0);
      uIntCache = 560; 
      currentMenu = 20;
      break;
      }
    case 3:
      {  //---------------1 - MAIN MENU--------------------------------
        drawFrames = true;
        setStringAt(0, 0, "Menu 3");
        if (encoderSwitch) {
          setStringAt(1,5,"click");
        }
        if (encoderSwitchHold){
          setStringAt(1,5,"hold");
        }
        if (encoderSwitchHoldOnce){
          setStringAt(1,10,"once");
        }
        break;
      }  //---------------1 - END---------------------------------------


    case 4:
      {  //---------------1 - MAIN MENU---------------------------------
        drawFrames = true;
        setStringAt(0, 0, "Menu 4");
        if (encoderSwitch) {
          currentMenu = 452452;
          encoder.setCount(0);
        }
        break;
      }  //---------------1 - END---------------------------------------

    case 11:
      {  //---------------1 - MAIN MENU---------------------------------
        drawFrames = true;
        int custom_cappedEncoderValue = cappedEncoderValue%2;
        setStringAt(0, 2, "save values?");
        if(custom_cappedEncoderValue == 1){setStringAt(1, 2, ">yes<");}
        else{                              setStringAt(1, 2, "|yes|");}
        if(custom_cappedEncoderValue == 0){setStringAt(1, 9, ">no<");}
        else{                              setStringAt(1, 9, "|no|");}
        
        if (encoderSwitch) {
          if(custom_cappedEncoderValue == 1){
            currentMenu = int64Cache;
            flashMem.begin(flashNameSpace, false);
            encoder.setCount(0);
            flashMem.putFloat("Kp", Kp);
            flashMem.putFloat("Ki", Ki);
            flashMem.putFloat("Kd", Kd);
            flashMem.putFloat("Km", Km);
            flashMem.putFloat("Headroom", OnOffHeadroom);
            flashMem.putFloat("PWMfreq",PWMfrequency);
            flashMem.putBool("s0", 1); // repairs corrupted settings
            for(int i=1; i<sizeof(PIDsettings);i++){
              String stringHelper = "s" + String(i);
              flashMem.putBool(stringHelper.c_str(), PIDsettings[i]);
            }
            flashMem.end();
          }
          else{
            currentMenu = int64Cache;
            encoder.setCount(0);
            Serial.println("<Info>changes canceled</Info>");
          }
          
        }
        break;
      }  //---------------1 - END---------------------------------------
    case 12:
      {  //---------------1 - MAIN MENU---------------------------------
        drawFrames = true;
        int custom_cappedEncoderValue = cappedEncoderValue%2;
        setStringAt(0, 0, "delete settings?");
        if(custom_cappedEncoderValue == 1){setStringAt(1, 2, ">yes<");}
        else{                              setStringAt(1, 2, "|yes|");}
        if(custom_cappedEncoderValue == 0){setStringAt(1, 9, ">no<");}
        else{                              setStringAt(1, 9, "|no|");}
        
        if (encoderSwitch) {
          if(custom_cappedEncoderValue == 1){
            currentMenu = 1;
            flashMem.begin(flashNameSpace, false);
            Serial.println("<Info>DELETING SETTINGS</Info>");
            flashMem.clear();
            flashMem.end();

            for(int i=0;i<sizeof(registeredPlannedRecords);i++){
              registeredPlannedRecords[i] = 0;
            }
            pushPRRegister();

            popupHandler(0, "Restart device to delete settings ");
          }
          else{
            currentMenu = 1;
            encoder.setCount(0);
            Serial.println("<Info>canceled</Info>");
          }
          
        }
        break;
      }  //---------------1 - END---------------------------------------
    case 101:
      {
      drawFrames = true;
      float custom_cappedEncoderValue = constrain((((float)encoder.getCount())/2)*(0-1), 0-30, 120);

      char tempChar = 223;  //°
      tempString = String(round((currentSensorTemperature * 10)) / 10);
      tempString.remove(4, 1);

      setSymbolAt(0, 1, 5);  //temp
      setStringAt(0, 2, tempString);
      setSymbolAt(0, 6, 1);  //celsius

      tempString = String(round(sht.getHumidity()));
      tempString.remove(2, 3);
      setSymbolAt(0, 10, 3);
      setStringAt(0, 11, tempString + "%");


      setStringAt(1, 6 + ((cappedEncoderValue % 4) * 2), "<");

      setStringAt(1,0,"targetTemp=" + String(custom_cappedEncoderValue));
      if(encoderSwitch){
        manualTemp = custom_cappedEncoderValue;
        encoder.setCount(int64Cache);
        currentMenu = int64Cache2;
      }
      break;
      }
    
    default:
      {  //---------------X - ERROR---------------------------------------
        Serial.println("<Debug>wrong menu ID (" + String(currentMenu) + ")</Debug>");
        lcd.clear();
        currentMenu = 0;
        break;
      }  //---------------X - END---------------------------------------
    }}}}
  if(drawFrames){
     RefreshDisplay(lcdOutput, lcdSymbols);
    }
  //delay(50);
}