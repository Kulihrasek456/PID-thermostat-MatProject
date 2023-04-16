#include "Arduino.h"
byte registeredPlannedRecords[33];

class PlannedRecordObject {
private:  
public:
  char name[10];
  int8_t temperature;  //only needs to be -127 to 127
  byte daysOfWeek;     // Ne Po Ut Str Ct Pa So
  int startTime;       // created from hours*100 + minutes (hhmm)
  int endTime;

  PlannedRecordObject(char name[10], int8_t temperature, byte daysOfWeek, int startTime, int endTime) {
    this->temperature = temperature;
    for (int i = 0; i < 10; i++) {
      this->name[i] = name[i];
    }
    this->daysOfWeek = daysOfWeek;
    this->startTime = startTime;
    this->endTime = endTime;
  }

  bool saveToFlashMem() {
    Serial.println("<PROsave>\nsaving to memory...");
    bool spaceInMem = false;
    byte designatedPos = 0;
    byte designatedBit = 0;
    for (int p = 0; p < 32; p++) {  //checks if there are some empty spaces in memory
      if (registeredPlannedRecords[p] != 255) {
        spaceInMem = true;
        for (int b = 0; b < 8; b++) {
          if (bitRead(registeredPlannedRecords[p], b) == 0) {
            designatedPos = p;
            designatedBit = b;
            Serial.println("found empty space in " + String(p) + " " + String(b) + "\n</PROsave>");
            break;
          }
        }

        break;
      }
    }
    if (spaceInMem) {
      return writeToFlashMem(designatedPos, designatedBit);
    } else {
      Serial.println("no space in memory\n</PROsave>");
      return false;
    }
  }

  bool writeToFlashMem(byte designatedPos, byte designatedBit) {
    String tempString2 = "pr" + String((designatedPos * 8) + designatedBit);
    Serial.println("<PROwrite>\nsaving PlannedRecord to " + String(designatedPos) + " " + String(designatedBit) + " " + this->objectDetails());
    //String tempString2 = "pr1";

    flashMem.begin(flashNameSpace, false);

    Serial.print("name");  //name
    for (int i = 0; i < 10; i++) {
      String stringHelper = tempString2 + "name" + String(i);
      Serial.print(" " + String(i) + ":");
      flashMem.putChar(stringHelper.c_str(), this->name[i]);
      delay(flashReadingDelay);
      Serial.print(this->name[i]);
    }

    Serial.println("\ntemperature");  //temperature
    String stringHelper = tempString2 + "temp";
    flashMem.putBytes(stringHelper.c_str(), (void*)&this->temperature, sizeof(temperature));
    delay(flashReadingDelay);

    Serial.print("daysOfWeek");  //daysOfWeek
    stringHelper = tempString2 + "dayOW";
    flashMem.putBytes(stringHelper.c_str(), (void*)&this->daysOfWeek, sizeof(daysOfWeek));
    delay(flashReadingDelay);

    Serial.println("\nstart time");  //start Time
    stringHelper = tempString2 + "sTime";
    flashMem.putInt(stringHelper.c_str(), this->startTime);
    delay(flashReadingDelay);

    Serial.println("end time");  //end Time
    stringHelper = tempString2 + "eTime";
    flashMem.putInt(stringHelper.c_str(), this->endTime);
    delay(flashReadingDelay);

    flashMem.end();
    bitWrite(registeredPlannedRecords[designatedPos], designatedBit, 1);  //marks place in memory as registered
    registeredPlannedRecords[32] += 1;                                    //adds one to number of registered plan records

    Serial.println("</PROwrite>");
    return true;
  }

  String objectDetails() {  // {name[10], temperature, daysOfWeek[8], StartTime, EndTime}
    String result = "{";
    for (int i = 0; i < sizeof(this->name); i++) {
      result += this->name[i];
    }
    result += ", " + String(temperature) + ", ";
    for (int i = 0; i < 8; i++) {
      result += bitRead(daysOfWeek, i);
    }
    result += "-" + String(daysOfWeek);
    result += ", " + String(startTime) + ", " + String(endTime) + "}";
    return result;
  }
  String getName() {
    return String(this->name);
  }
};

PlannedRecordObject getPRfromMem(byte designatedPos, byte designatedByte, boolean debugMode) {

  if (debugMode) { Serial.println("<PROget>\ngetting PlannedRecord from memory at " + String(designatedPos) + " " + String(designatedByte)); }


  if (bitRead(registeredPlannedRecords[designatedPos], designatedByte) == 1) {
    char name[10];
    int8_t temperature;
    byte daysOfWeek;
    int startTime;
    int endTime;

    String tempString2 = "pr" + String((designatedPos * 8) + designatedByte);

    flashMem.begin(flashNameSpace, true);

    if (debugMode) { Serial.print("name"); }  //name
    for (int i = 0; i < sizeof(name); i++) {
      String stringHelper = tempString2 + "name" + String(i);
      if (debugMode) { Serial.print(String(i)); }
      name[i] = flashMem.getChar(stringHelper.c_str(), 0);
      delay(flashReadingDelay);
    }

    if (debugMode) { Serial.println("\ntemperature"); }  //temperature
    String stringHelper = tempString2 + "temp";
    flashMem.getBytes(stringHelper.c_str(), (PlannedRecordObject*)&temperature, sizeof(temperature));
    delay(flashReadingDelay);

    if (debugMode) { Serial.print("daysOfWeek"); }  //daysOfWeek
    stringHelper = tempString2 + "dayOW";
    flashMem.getBytes(stringHelper.c_str(), (PlannedRecordObject*)&daysOfWeek, sizeof(daysOfWeek));
    delay(flashReadingDelay);

    if (debugMode) { Serial.println("\nstart time"); }  //start Time
    stringHelper = tempString2 + "sTime";
    startTime = flashMem.getInt(stringHelper.c_str(), 0);
    delay(flashReadingDelay);

    if (debugMode) { Serial.println("\end time"); }  //end Time
    stringHelper = tempString2 + "eTime";
    endTime = flashMem.getInt(stringHelper.c_str(), 0);
    delay(flashReadingDelay);


    flashMem.end();

    PlannedRecordObject goodObj(name, temperature, daysOfWeek, startTime, endTime);
    if(debugMode) {Serial.println("</PROget>");}
    return goodObj;
  } else {
    Serial.println("empty sector" + String(designatedPos) + " " + String(designatedByte) + "\n</PROget>");
    PlannedRecordObject errorObj("error", 0, 0, 0, 0);
    return errorObj;
  }
}

PlannedRecordObject getPRfromMem(byte designatedPos, byte designatedBit) {
  return getPRfromMem(designatedPos, designatedBit, false);
}


String getPRfromMemNAME(byte designatedPos, byte designatedBit) {
  flashMem.begin(flashNameSpace, true);
  char name[10];
  String tempString2 = "pr" + String((designatedPos * 8) + designatedBit);

  for (int i = 0; i < sizeof(name); i++) {
    String stringHelper = tempString2 + "name" + String(i);
    name[i] = flashMem.getChar(stringHelper.c_str(), 0);
    delay(flashReadingDelay);
  }
  flashMem.end();
  return name;
}

int getPRfromMemSTIME(byte designatedPos, byte designatedBit) {
  flashMem.begin(flashNameSpace, true);
  int startTime;
  String tempString2 = "pr" + String((designatedPos * 8) + designatedBit);

  String stringHelper = tempString2 + "sTime";
  startTime = flashMem.getInt(stringHelper.c_str(), 0);
  delay(flashReadingDelay);

  flashMem.end();
  return startTime;
}

int getPRfromMemETIME(byte designatedPos, byte designatedBit) {
  flashMem.begin(flashNameSpace, true);
  int endTime;
  String tempString2 = "pr" + String((designatedPos * 8) + designatedBit);

  String stringHelper = tempString2 + "eTime";
  endTime = flashMem.getInt(stringHelper.c_str(), 0);
  delay(flashReadingDelay);

  flashMem.end();
  return endTime;
}

int8_t getPRfromMemTEMP(byte designatedPos, byte designatedBit) {
  flashMem.begin(flashNameSpace, true);
  int8_t temperature;
  String tempString2 = "pr" + String((designatedPos * 8) + designatedBit);

  String stringHelper = tempString2 + "temp";
  flashMem.getBytes(stringHelper.c_str(), (PlannedRecordObject*)&temperature, sizeof(temperature));
  delay(flashReadingDelay);

  flashMem.end();
  return temperature;
}

byte getPRfromMemDOW(byte designatedPos, byte designatedBit) {
  flashMem.begin(flashNameSpace, true);
  byte daysOfWeek;
  String tempString2 = "pr" + String((designatedPos * 8) + designatedBit);

  String stringHelper = tempString2 + "dayOW";
  flashMem.getBytes(stringHelper.c_str(), (PlannedRecordObject*)&daysOfWeek, 1);
  delay(flashReadingDelay);
  flashMem.end();
  return daysOfWeek;
}

DateTime nextAutoChange = RTC_Time;
bool autoModeNeedsReload = true;
PlannedRecordObject savedPlannedRecord = PlannedRecordObject("error",0,0,0,0);

bool searchForValidPR() {
  Serial.println("\n--searching for new planned records--");

  DateTime RTC_Time = DS1307.now();

  int currentTime = (RTC_Time.hour() * 100) + RTC_Time.minute();
  int numOfEntries = registeredPlannedRecords[32];
  int foundEntries = 0;
  bool savedEntry = false;
  bool stillSearching = true;

  savedPlannedRecord = PlannedRecordObject("error",0,0,9999,0);

  for (byte pos = 0; pos < 32; pos++) {  //search through every pos
    if (registeredPlannedRecords[pos] > 0 && stillSearching) {
      Serial.print("\npos" + String(pos) + "=");
      for (byte bit = 0; bit < 8; bit++) {
        if (numOfEntries > foundEntries) {
          Serial.print("|" + String(bit));
          if (bitRead(registeredPlannedRecords[pos], bit)) {
            Serial.print(":");
            foundEntries++;
            if (bitRead(getPRfromMemDOW(pos, bit), (RTC_Time.dayOfTheWeek() + 6) % 7)) {Serial.print("W");
            int StartTIME = getPRfromMemSTIME(pos, bit);
            if ((StartTIME <= currentTime)) { /* || (lastStartTIME>StartTIME)*/         Serial.print("S");
            int EndTIME = getPRfromMemETIME(pos, bit);
            if (EndTIME > currentTime) {                                               Serial.print("E:");
                if(StartTIME<savedPlannedRecord.startTime){
                  savedPlannedRecord = getPRfromMem(pos,bit); 
                }
                savedEntry = true;
                                                                                        Serial.print(savedPlannedRecord.temperature);
            }}}
          }

        } else {
          Serial.println("\nInfo - no entries left: " + String(foundEntries) + "/" + String(registeredPlannedRecords[32]));
          stillSearching = false;
          break;
        }
      }
    }
  }
  flashMem.end();
  Serial.println("\n--SEARCH ENDED--\n" + savedPlannedRecord.objectDetails());
  return savedEntry;
}

void processAutoMode() {
  if (nextAutoChange.secondstime() <= RTC_Time.secondstime() && registeredPlannedRecords[32] > 0) {
    Serial.println("<AutoSearch>");
    autoMode = searchForValidPR();
    if(autoMode){
      int hour = savedPlannedRecord.endTime /100;
      int minute = savedPlannedRecord.endTime%100;      
      nextAutoChange = DateTime(RTC_Time.year(), RTC_Time.month(), RTC_Time.day(), hour, minute, RTC_Time.second());
    }
    else{
        nextAutoChange = DateTime(RTC_Time.year(), RTC_Time.month(), RTC_Time.day(), RTC_Time.hour(), RTC_Time.minute()+1, RTC_Time.second());

    }
    Serial.println("Next autoMode update in " + String(nextAutoChange.secondstime()-RTC_Time.secondstime()) +" seconds\n</AutoSearch>");
  }
}
 
void pushPRRegister() {
  encoder.pauseCount();
  flashMem.begin(flashNameSpace, false);
  Serial.println("<PRregPush>\npushing planned records register to memory");
  for (int i = 0; i < sizeof(registeredPlannedRecords); i++) {
    String stringHelper = "PRreg" + String(i);
    Serial.print(String(i));
    flashMem.putBytes(stringHelper.c_str(), (void*)&registeredPlannedRecords[i], sizeof(registeredPlannedRecords[i]));
    delay(flashReadingDelay);
  }
  Serial.println("\n</PRregPush>");
  flashMem.end();
  encoder.resumeCount();
}

void pullPRregister() {
  encoder.pauseCount();
  flashMem.begin(flashNameSpace, true);
  Serial.println("<PRregPull>\npulling planned records register from memory");

  byte cacheArray[33];

  for (int i = 0; i < 33; i++) {
    cacheArray[i] = 0;
  }

  byte numOfPRs = 0;
  for (int i = 0; i < sizeof(registeredPlannedRecords); i++) {
    String stringHelper = "PRreg" + String(i);
    Serial.print(String(i));
    flashMem.getBytes(stringHelper.c_str(), (void*)&cacheArray[i], sizeof(registeredPlannedRecords[i]));
    delay(flashReadingDelay);
    if (cacheArray[i] > 0 && i < 32) {
      for (int b = 0; b < 8; b++) {
        if (bitRead(cacheArray[i], b) == 1) {
          Serial.print("*");
          numOfPRs++;
        }
      }
    }
  }
  Serial.println();

  if (numOfPRs != cacheArray[32]) {
    Serial.println("\ncorrupted PRregister numOfPRs:" + String(numOfPRs) + " should be:" + String(cacheArray[32]) + "\nfixing numOfPRs");
    cacheArray[32] = numOfPRs;
  }

  for (int i = 0; i < sizeof(registeredPlannedRecords); i++) {
    registeredPlannedRecords[i] = cacheArray[i];
  }

  Serial.println("</PRregPull>");
  flashMem.end();
  encoder.resumeCount();
}

void RefreshDisplay(String lcdOutput_copy[], unsigned int[LCDx * LCDy]) {
  for (int y = 0; y < LCDy; y++) {
    int outputLenght = lcdOutput_copy[y].length();
    if (outputLenght < LCDx) {  //add to needed length
      for (int j = 0; j < LCDx - outputLenght; j++) {
        lcdOutput_copy[y] += " ";
      }
    }
    String tempStringCache = "";
    for (int x = 0; x < lcdOutput_copy[y].length(); x++) {
      if (lcdSymbols[x + (y * LCDx)] == -1) {  //plain text in celle
        lcd.setCursor(x, y);
        lcd.print(lcdOutput_copy[y][x]);
      } else {  //custom symbol in cell
        //lcd.write(lcdSymbols[x+(y*LCDx)]);
        lcd.write(lcdSymbols[x + (y * LCDx)]);
      }
    }
  }
}

void setStringAt(int line, int pos, String text) {
  String source = lcdOutput[line];
  int sourceLength = source.length();
  int textLength = text.length();
  int neededLength = text.length() + pos;
  String result = source;
  if (neededLength > sourceLength) {
    for (int i = 0; i < neededLength - sourceLength; i++) {
      result += " ";
    }
  }

  for (int a = 0; a < textLength; a++) {
    //result[a+pos] = text[a];
    result.setCharAt(a + pos, text[a]);
  }
  lcdOutput[line] = result;
}

void editorHandler(String variableValue, String variableName, int inputLength, char placeHolderChar, String useableCharacters, int inputID) {
  stringCache2 = variableName;  //name of the variable (displayed on top of screen)
  stringCache = variableValue;  //converted version of the variable
  boolCache = false;
  boolCache2 = false;
  uLongCache = 0;
  uIntCache = 0;
  int64Cache = 0;
  int64Cache2 = 0;
  int64Cache3 = map(inputLength, 0, 15, 0, 15);  //max lenght of the input
  charCache = placeHolderChar;                   //which character will be used to fill to desired length
  usableCharactersCache = useableCharacters;     //a string containing allowed characters
  editorIsActive = true;
  editorIntCache = inputID;  //id which can be used to catch edited result
}

bool refreshVariablesFromMem(bool reload) {  //if given true it reverts the variables to stored values and if something changed, returns true
  encoder.pauseCount();
  flashMem.begin(flashNameSpace, true);
  bool readError = false;


  static const int flashReadingDelay = 100;
  if (reload) {
    Serial.println("<VarReload>\nloading values from memory and rewriting them");
  } else {
    Serial.println("checking for updates");
  }
  drawFrames = false;
  lcd.clear();
  lcd.print("loading values");
  bool changed = false;
  {
    float temp = flashMem.getFloat("Kp", -1);
    delay(flashReadingDelay);  //flash memory is slow
    if (temp < 0) {
      if(!readError){Serial.print("<ERR>"); readError = true;}      
      Serial.println("read error Kp=" + String(temp));

      changed = true;
    } else {
      if (temp != Kp) {
        if (reload) { Kp = temp; }
        changed = true;
      }
    }
  }
  {
    float temp = flashMem.getFloat("Ki", -1);
    delay(flashReadingDelay);  //flash memory is slow
    if (temp < 0) {
      if(!readError){Serial.print("<ERR>"); readError = true;}   
      Serial.println("read error Ki=" + String(temp));
      changed = true;
    } else {
      if (temp != Ki) {
        if (reload) { Ki = temp; }
        changed = true;
      }
    }
  }
  {
    float temp = flashMem.getFloat("Kd", -1);
    delay(flashReadingDelay);  //flash memory is slow
    if (temp < 0) {
      if(!readError){Serial.print("<ERR>"); readError = true;}   
      Serial.println("read error Kd=" + String(temp));
      changed = true;
    } else {
      if (temp != Kd) {
        if (reload) { Kd = temp; }
        changed = true;
      }
    }
  }
  {
    float temp = flashMem.getFloat("Km", -1);
    delay(flashReadingDelay);  //flash memory is slow
    if (temp < 0) {
      if(!readError){Serial.print("<ERR>"); readError = true;}   
      Serial.println("read error Km=" + String(temp));
      changed = true;
    } else {
      if (temp != Km) {
        if (reload) { Km = temp; }
        changed = true;
      }
    }
  }
  {
    float temp = flashMem.getFloat("Headroom", -1);
    delay(flashReadingDelay);  //flash memory is slow
    if (temp < 0) {
      if(!readError){Serial.print("<ERR>"); readError = true;}   
      Serial.println("read error OnOffHeadroom=" + String(temp));
      changed = true;
    } else {
      if (temp != OnOffHeadroom) {
        if (reload) { OnOffHeadroom = temp; }
        changed = true;
      }
    }
  }
  {
    float temp = flashMem.getFloat("PWMfreq", -1);
    delay(flashReadingDelay);  //flash memory is slow
    if (temp < 0) {
      if(!readError){Serial.print("<ERR>"); readError = true;}   
      Serial.println("read error PWMfrequency=" + String(temp));
      changed = true;
    } else {
      if (temp != PWMfrequency) {
        controllerSettingChanged = true;
        if (reload) { PWMfrequency = temp; }
        changed = true;
      }
    }
  }

  bool temp = flashMem.getBool("s0", false);
  delay(flashReadingDelay);  //flash memory is slow
  if (temp) {
    for (int i = 0; i < sizeof(PIDsettings); i++) {
      String stringHelper = "s" + String(i);
      temp = flashMem.getBool(stringHelper.c_str(), false);
      delay(flashReadingDelay);  //flash memory is slow
      if (temp != PIDsettings[i]) {
        if (reload) { PIDsettings[i] = temp; }
        changed = true;
        if(i==2){
          controllerSettingChanged = true;
        }
      }
    }
  } else {
    if(!readError){Serial.print("<ERR>"); readError = true;}   
    Serial.print("read error PIDsettings: {");
    for (int i = 0; i < sizeof(PIDsettings); i++) { Serial.print(PIDsettings[i], DEC); }
    Serial.println("}");
    changed = true;
  }

  if(readError){Serial.println("</ERR>");}

  flashMem.end();


  encoder.resumeCount();

  Serial.println("</VarReload>");
  return changed;
}

void saveScreenHandler(int sourceMenu) {  //reverts the variables to stored values and if something changed, returns true
  if (refreshVariablesFromMem(false)) {
    currentMenu = 11;
    encoder.setCount(0);
    int64Cache = sourceMenu;
  } else {
    currentMenu = sourceMenu;
  }
}

void tempSettingHandler(int sourceMenu) {
  currentMenu = 101;                //menu of temperature settings
  int64Cache = encoder.getCount();  //stores value of encoder before change
  int64Cache2 = sourceMenu;         //stores last menu to retun to (useless in this case but it could fix some bugs)
  encoder.setCount(targetTemp * -2);
}

void popupHandler(int sourceMenu, String Message) {
  popupIntCache = 0;            //used to store last index if the displayed message needs scrolling
  popupIntCache2 = sourceMenu;  //stores last menu to return to
  popupStringCache = Message;
  popupEncoderCache = encoder.getCount();
  popupIsActive = true;  //activates popup window
}

void powerSavingModeHandler() {
  sleepMode = true;
  drawFrames = false;
  lcd.clear();
}


void setSymbolAt(int line, int pos, unsigned int symbolID) {
  setStringAt(line, pos, "%");
  bool symbolISregistered = false;
  for (int i = 0; i < 8; i++) {
    if (symbolID == registeredSymbols[i]) {
      symbolISregistered = true;
      lcdSymbols[pos + (line * LCDx)] = i;
    }
  }

  if (!symbolISregistered) {

    switch (symbolID) {
      case 0:{ lcd.createChar(registeredSymbols_counter % 8, errorSymbol);                  break;}
      case 1:{ lcd.createChar(registeredSymbols_counter % 8, celsiusSymbol);                break;}
      case 2:{ lcd.createChar(registeredSymbols_counter % 8, celsiusSymbol_inv);            break;}
      case 3:{ lcd.createChar(registeredSymbols_counter % 8, humiditySymbol);               break;}
      case 4:{ lcd.createChar(registeredSymbols_counter % 8, humiditySymbol_inv);           break;}
      case 5:{ lcd.createChar(registeredSymbols_counter % 8, temperatureSymbol);            break;}
      case 6:{ lcd.createChar(registeredSymbols_counter % 8, temperatureSymbol_inv);        break;}
      case 7:{ lcd.createChar(registeredSymbols_counter % 8, returnSymbol);                 break;}
      case 8:{ lcd.createChar(registeredSymbols_counter % 8, returnSymbol_inv);             break;}
      case 9:{ lcd.createChar(registeredSymbols_counter % 8, settingsSymbol);               break;}
      case 10:{lcd.createChar(registeredSymbols_counter % 8, settingsSymbol_inv);           break;}
      case 11:{lcd.createChar(registeredSymbols_counter % 8, saveSymbol);                   break;}
      case 12:{lcd.createChar(registeredSymbols_counter % 8, saveSymbol_inv);               break;}
      case 13:{lcd.createChar(registeredSymbols_counter % 8, cancelSymbol);                 break;}
      case 14:{lcd.createChar(registeredSymbols_counter % 8, cancelSymbol_inv);             break;}
      case 15:{lcd.createChar(registeredSymbols_counter % 8, thinUnderlineSymbol);          break;}
      case 16:{lcd.createChar(registeredSymbols_counter % 8, pathTopDownLeftSymbol);        break;}
      case 17:{lcd.createChar(registeredSymbols_counter % 8, arrowUpSymbol);                break;}
      case 18:{lcd.createChar(registeredSymbols_counter % 8, arrowDownSymbol);              break;}
      case 19:{lcd.createChar(registeredSymbols_counter % 8, blockInMiddleSymbol);          break;}
      case 20:{lcd.createChar(registeredSymbols_counter % 8, arrowDoubleSymbol);            break;}
      case 21:{lcd.createChar(registeredSymbols_counter % 8, pointFiveSymbol);              break;}
      case 22:{lcd.createChar(registeredSymbols_counter % 8, pointZeroSymbol);              break;}
      case 23:{lcd.createChar(registeredSymbols_counter % 8, pathTopDownLeftSymbolNegative);break;}
      default:{lcd.createChar(registeredSymbols_counter % 8, errorSymbol);                  break;}
    }
    registeredSymbols[registeredSymbols_counter % 8] = symbolID;
    registeredSymbols_counter++;

    /*
    //15,12,3,5,4,8,6,0 bool=1 45 5
    for (int k = 0; k < 8; k++) { Serial.print(String(registeredSymbols[k]) + ","); }
    Serial.print(" bool=");
    Serial.print(symbolISregistered, DEC);
    Serial.println(" " + String(registeredSymbols_counter) + " " + String(registeredSymbols_counter % 8));*/
    lcdSymbols[pos + (line * LCDx)] = registeredSymbols_counter % 8;  //musi byt index ulozenyho symbolu
  }
}


String getStringFromLib(int index, int_fast16_t nameArrayIndex) {  //essentially mimics enum by letting you create an array of strings and returning a string on given index

  switch (nameArrayIndex) {
    case 0:
      {
        String premadeNameArray[] = { "HEATER", "COOLER" };
        if (index > sizeof(premadeNameArray) || index < 0) {
          return "err1";
        } else {
          return premadeNameArray[index];
        }
      }
    case 1:
      {
        String premadeNameArray[] = { "PID", "ON-OFF" };
        if (index > sizeof(premadeNameArray) || index < 0) {
          return "err1";
        } else {
          return premadeNameArray[index];
        }
      }
    case 2:
      {
        String premadeNameArray[] = { "NORMAL", "HYBRID" };
        if (index > sizeof(premadeNameArray) || index < 0) {
          return "err1";
        } else {
          return premadeNameArray[index];
        }
      }
    case 3:
      {
        String premadeNameArray[] = { "Off", "On" };
        if (index > sizeof(premadeNameArray) || index < 0) {
          return "err1";
        } else {
          return premadeNameArray[index];
        }
      }
    case 4:
      {
        String premadeNameArray[] = { "INTERNAL", "EXTERNAL" };
        if (index > sizeof(premadeNameArray) || index < 0) {
          return "err1";
        } else {
          return premadeNameArray[index];
        }
      }
    default: return "err2"; break;
  }
}

//PID Controller variables
float PID_lastErr = 0;
float PID_lastMillis = 0;
float PID_cumulativeErr = 0;



float MyPID(float currPos, float target, float Kp_input, float Ki_input, float Kd_input, float maxStrength, bool printOut) {

  float err = target - currPos;

  float maxStrengthPos = maxStrength;
  float maxStrengthNeg = -maxStrength;

  float thoreticalPID_cumulativeERR = PID_cumulativeErr + (err / (millis() - PID_lastMillis)) * Ki_input;

  if (thoreticalPID_cumulativeERR < maxStrengthPos && thoreticalPID_cumulativeERR > maxStrengthNeg) {
    PID_cumulativeErr = thoreticalPID_cumulativeERR;
  }

  float P = err * Kp_input;

  float I = PID_cumulativeErr;
  if ((I * err) < 0) {
    I = I * 0.1;
  }

  float D = ((err - PID_lastErr) / (millis() - PID_lastMillis)) * Kd_input * 1000;



  if (printOut) {
    Serial.print(",P:" + String(P) + ",I:" + String(I) + ",D:" + String(D));
  }

  PID_lastErr = err;
  PID_lastMillis = millis();
  return constrain((P + I + D) * Km, maxStrengthNeg, maxStrengthPos);
}

void scrollComponentSpecific(int currentElement, int numberOfElements, int line) {
  byte outputVariant = 0;
  numberOfElements = numberOfElements - 1;
  if (currentElement < numberOfElements) {
    if (currentElement > 0) { outputVariant = 2; }  //middle
    else {
      outputVariant = 1;
    }
  }                            //start
  else { outputVariant = 3; }  //end

  switch (outputVariant) {
    case 1:
      {
        if (line == -1) {
          setSymbolAt(LCDy - 1, LCDx - 1, 18);
        } else {
          setSymbolAt(line, LCDx - 1, 18);
        }
        break;
      }
    case 2:
      {
        if (line == -1) {
          setSymbolAt(0, LCDx - 1, 17);  //up
          setSymbolAt(LCDy - 1, LCDx - 1, 18);
        }  //down
        else { setSymbolAt(line, LCDx - 1, 20); }
        break;
      }
    case 3:
      {
        if (line == -1) {
          setSymbolAt(0, LCDx - 1, 17);
        } else {
          setSymbolAt(line, LCDx - 1, 17);
        }
        break;
      }
  }
}
void scrollComponent(int currentElement, int numberOfElements) {
  scrollComponentSpecific(currentElement, numberOfElements, -1);
}

unsigned int constrainEncoder(int numOfElements, int specialInput){
  int result = 0;
  int input = specialInput;
  if(input<0){
    result = (numOfElements-1)+((input+1)%numOfElements);
  }
  else{
    result = input%numOfElements;
  }
  return result;
}

unsigned int constrainEncoder(int numOfElements){
  return constrainEncoder(numOfElements, -encoder.getCount());
}

