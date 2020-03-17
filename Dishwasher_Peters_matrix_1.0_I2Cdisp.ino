
//Peter Nikolov's Dishwasher v.1.0.
//Programer: Peter Nikolov.
//Based on Rivera_1.0 by Pedro Rivera and the work of steve8428 - Thank you.
//A dishwasher controller with different wash programs defined via a matrix of constants,
//also includes a Custom programme - allows user selection of custom temperature and wash time, and constructs a washing cycle to fulfill the custom parameters.
//Using temperature control thermistor 10k and a potentiometer 50k for program selector, 6-7 relays control, opto-isolator adviceable.
//Licensed under Creative Commons.

// #include <LiquidCrystal.h>
#include <Wire.h>
#include "rgb_lcd.h"
#include <math.h>

rgb_lcd lcd;

//Pin assignment
//LiquidCrystal lcd(8, 9, 10, 11, 12, 13);
#define inletValve 4 //water inlet valve
#define heater 5 //heater 
#define washPump 6 //washpump
#define drainPump 7 //drain pump
#define DetergentSolen 8 //detergent dispenser solenoid 
#define RinseAidSolen 9 //rinse aid dispenser solenoid
#define buzzer 10 //buzzer/beeper 
#define regenSolen A5 //regeneration solenoid 
#define startBtn 11 //start/pause/resume/reset button
#define doorBtn 12 //door button
//#define errorSens 11 //overfill sensor error
#define keySelect A0 //select key
#define fillSens A1 //fill sensor
#define tempSensor A3 //temperature sensor



//constants declaration
const String ProgramVersion = "1.0";
const int NumberOfPrograms = 9;
const String ProgramNames[] = {"Select", "Intensive Wash", "Normal Wash", "Eco Wash", "Fast Wash", "Express Strong", "Cold Wash", "Rinse Only", "Custom"}; // Names of programs
const byte HighestTemperatures[] = {0, 65, 60, 48, 36, 60, 5, 20, 65}; // Highest allowed temperatures for programs in degrees Celsius
const byte ExpectedDurations[] =  {0, 120, 100, 80, 40, 50, 55, 20, 125}; // Expected durations of programs in minutes
const int KeySelectAnalogValues[] =  {0, 150, 300, 450, 600, 750, 850, 950, 1050}; // Analog boundaries of the programs for the selector analog input
const byte MatrixStructure[] =  {0, 1, 1, 0, 1, 1, 1, 1, 0}; // 1 = matrix structure; 0 = custom program
const byte PreWashDurations[] =  {0, 12, 2, 8, 0, 0, 0, 0, 15}; // Pre-wash durations per programs in minutes; 0 = no Pre-wash
const byte WashDurations[] =  {0, 25, 25, 11, 10, 20, 25, 0, 40}; // Wash durations per programs in minutes; 0 = no Wash
const byte Rinse1Durations[] =  {0, 6, 6, 6, 6, 6, 6, 10, 14}; // Rinse 1 durations per programs in minutes; 0 = no Rinse 1
const byte Rinse2Durations[] =  {0, 6, 6, 2, 2, 2, 6, 0, 11}; // Rinse 2 durations per programs in minutes; 0 = no Rinse 2
const byte ClearRinseDurations[] =  {0, 10, 10, 10, 10, 10, 10, 0, 20}; // Clear Rinse durations per programs in minutes; 0 = no Clear Rinse
const byte DryDurations[] =  {0, 11, 11, 11, 2, 2, 1, 1, 12}; // Drying durations per programs in minutes; 0 = no Drying
const int debounceDelay = 20; // delay to count as button pressed - 20 milliseconds
const int R_debounceDelay = 5000; // delay to count as reset - 5 seconds
const int OverheatLimit = 72; // Temperature limit to count for overheating

//variables declaration
volatile int doorBtnState = HIGH;
volatile byte pause = false;
volatile unsigned long timeStopped = 0;      // Record time program was paused
volatile unsigned long pauseTime = 0;        //record how long the program was paused for
volatile byte Prepauseheater;
volatile byte PrepausewashPump;
volatile byte PrepauseDetergentSolen;
volatile byte PrepausedrainPump;
volatile byte PrepauseinletValve;
volatile byte PrepauseRinseAidSolen;
volatile byte PrepauseregenSolen;
volatile int faultCode = 0;
unsigned long TotalPeriodStart, TotalPeriodElapsed;
unsigned long periodStart, periodElapsed;
unsigned long TotalFillTime = 0, CurrentFillStart = 0;
int ExpDuration = 0; //Expected program duration in minutes
double tempArray[25];
byte arrayIndex = 0;
int lcdKeyMenu = 0;
int selKeyIN = 0;
unsigned long CustomTemp = 0;
unsigned long CustomDuration = 0;
double CustomCoefficient;
int fillSensState = 0;
int tempLimit = 0;
int x = 0;
int RawADC = 240; //temporary temperature around 0 degrees Celsius - to be removed
String ProgramName = "                ";
String SubCycleName = "                ";
//-----On/pause/restart/reset Button reference ------
int startBtnState = HIGH;
int O_buttonState = HIGH;
int O_lastButtonState = HIGH;
unsigned long lastDebounceTime = 0;
int onButtonCount = 0;                //switch case statement to control what each button push does



//General actions


void TotalTimeElapsed() {  // Total time counter with countdown time display on screen
  TotalPeriodElapsed = (millis() - TotalPeriodStart - pauseTime);
  lcd.setCursor(13, 0);
  lcd.print(ExpDuration - (TotalPeriodElapsed / 60000));
  lcd.print("' ");
}

void timeElapsed() { // Time counter
  periodElapsed = (millis() - periodStart);
  TotalTimeElapsed();
  onOffFun();
}

void waitXmsec(unsigned long WaitDuration) { // Wait X milliseconds
  unsigned long waitStart = millis();
  while ((millis() - waitStart) < WaitDuration) {
    timeElapsed();
    TotalTimeElapsed();
    onOffFun();
  }
}

void waitThreeSec() {  // Wait 3 seconds
  waitXmsec (3000);
}

double waterTemp() { // Water temperature measurement - subroutine taken from www.neonsquirt.com/
  if (arrayIndex > 23) { //which in its turn was taken from Arduino Playground
    arrayIndex = 0;
  }
  else {
    arrayIndex++;
  }
  double Temp; // The Thermistor2 "Simple Code"
  //int RawADC = analogRead(tempSensor);
  if (digitalRead(heater) == HIGH) { // temp
    RawADC++; // temp
  } // temp
  else { // temp
    if (RawADC > 350) {
      RawADC--; //  temp
    }
  } // temp
  delay(50); // temp
  Temp = log(((10240000 / RawADC) - 10000));
  Temp = 1 / (0.001129148 + (0.000234125 + (0.0000000876741 * Temp * Temp )) * Temp );
  Temp = Temp - 273.15; //convert kelvin to celsius
  tempArray[arrayIndex] = Temp;
  Temp = 0;
  for (int i = 0; i < 24; i++) {
    Temp += tempArray[i];
  }
  return (Temp / 25); // return the average temperature of the array
}


void dispTemp() { // Display current temperature on screen
  lcd.setCursor(12, 1);
  lcd.print(int(waterTemp()));
  lcd.print((char)223);
  lcd.print("C ");
}


// Program flow actions


void(* resetFunc) (void) = 0; // Declaration of reset function


void Finish() { // Washing programme finished
  Serial.println(F("Finish started.")); //temp
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Dishes are clean."));
  lcd.setCursor(0, 1);
  lcd.print(F("Press 'Off' btn."));
  tone (buzzer, 700, 150); // Buzz for 150 milliseconds with frequency 700 Hz
  delay (400);
  tone (buzzer, 700, 150); // Buzz for 150 milliseconds with frequency 700 Hz
  delay (400);
  tone (buzzer, 700, 250); // Buzz for 250 milliseconds with frequency 700 Hz
  Serial.print(F("Programme actual duration: ")); //temp
  Serial.print(TotalPeriodElapsed / 60000);
  Serial.println(F(" minutes.")); //temp
  while (true) {
    onOffFun();
  };
  Serial.println(F("Finish ended.")); //temp
}


void actDoorBtn() { // Door swich action
  Serial.println(F("actDoorBtn started.")); //temp
  pauseFun();
  Serial.println(F("Machine paused due to open door.")); //temp
  doorBtnState = digitalRead(doorBtn);
  while (doorBtnState == LOW && pause == true) {
    doorBtnState = digitalRead(doorBtn);
  }
  Serial.println(F("Machine resumed after door closed.")); //temp
  resumeFun();   //resume program
  Serial.println(F("actDoorBtn ended.")); //temp
}

void stopFun() {  // Stop all parts of the machine
  Serial.println(F("stopFun started.")); //temp
  digitalWrite(inletValve, LOW);
  digitalWrite(heater, LOW);
  digitalWrite(washPump, LOW);
  digitalWrite(drainPump, LOW);
  digitalWrite(DetergentSolen, LOW);
  digitalWrite(RinseAidSolen, LOW);
  digitalWrite(regenSolen, LOW);
  digitalWrite(buzzer, LOW);
  Serial.println(F("stopFun ended.")); //temp
}

void pauseFun() { // Pause - stop all wash processes and raise pause flag
  //Serial.println("pauseFun started."); //temp
  if (pause == false) {
    tone (buzzer, 600, 500); // Buzz for 500 milliseconds with frequency 700 Hz
    pause = true;
    Serial.println(F("Pause started.")); //temp
    timeStopped = millis();
    Prepauseheater = digitalRead(heater);
    PrepausewashPump = digitalRead(washPump);
    PrepauseDetergentSolen = digitalRead(DetergentSolen);
    PrepausedrainPump = digitalRead(drainPump);
    PrepauseinletValve = digitalRead(inletValve);
    PrepauseRinseAidSolen = digitalRead(RinseAidSolen);
    PrepauseregenSolen = digitalRead(regenSolen);
    stopFun();
    lcd.setCursor(0, 1);
    lcd.print(F("Paused          "));
  }
  // Serial.println("pauseFun ended."); //temp
}

void resumeFun() { // Resume wash processes after pause
  Serial.println(F("resumeFun started.")); //temp
  lcd.setCursor(0, 1);
  lcd.print(F("Resuming...     "));
  Serial.println(F("Resumed from pause.")); //temp
  tone (buzzer, 800, 150); // Buzz for 150 milliseconds with frequency 700 Hz
  delay (400);
  tone (buzzer, 800, 300); // Buzz for 300 milliseconds with frequency 700 Hz
  pause = false;
  pauseTime = millis() - timeStopped;
  digitalWrite(heater, Prepauseheater);
  digitalWrite(washPump, PrepausewashPump);
  digitalWrite(DetergentSolen, PrepauseDetergentSolen);
  digitalWrite(drainPump, PrepausedrainPump);
  digitalWrite(inletValve, PrepauseinletValve);
  digitalWrite(RinseAidSolen, PrepauseRinseAidSolen);
  digitalWrite(regenSolen, PrepauseregenSolen);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(ProgramName);
  lcd.setCursor(0, 1);
  lcd.print(SubCycleName);
  Serial.println(F("resumeFun ended.")); //temp
}


void resetFun() { // Reset wash, drain washer and restart machine
  Serial.println(F("resetFun started.")); //temp
  lcd.clear();
  lcd.home (); // go home
  lcd.print(F("Machine reset...     "));
  tone (buzzer, 700, 1000); // Buzz for 1000 milliseconds with frequency 700 Hz
  stopFun();  // Stop all devices
  delay(2000); //Wait for 2 seconds
  actDrain(); // Drain
  pause = false;
  timeStopped = 0;
  pauseTime = 0;
  delay(2000); //Wait for 2 seconds
  // Restart machine
  Serial.println(F("Restarting machine...")); //temp
  resetFunc (); // Restart machine
}


void actErrorSens() { // Error sensor - to be developed
  Serial.println(F("actErrorSens started.")); //temp
  cli();
  stopFun();
  lcd.setCursor(0, 0);
  lcd.print(F("ERROR - DISCONNECT"));
  lcd.setCursor(0, 1);
  lcd.print(F("THE DISHWASHER FROM SOCKET"));
  while (true) {};
  Serial.println(F("actErrorSens ended.")); //temp
}



void errorFun() { // Error encountered - stop all wash processes and show fault code
  Serial.println(F("errorFun started.")); //temp
  tone (buzzer, 600, 3000); // Buzz for 3 seconds with frequency 600 Hz
  stopFun(); // Stop all wash processes
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("ERROR Fault:    "));
  lcd.setCursor(12, 0);
  lcd.print(faultCode);
  lcd.setCursor(0, 1);
  lcd.print(F("Machine halted.  "));
  String faultDescription;
  switch (faultCode) {
    case 20:
      faultDescription = F("20: Over-heating - temperature exceeds ");
      faultDescription = faultDescription + OverheatLimit ;
      faultDescription = faultDescription +  F(" degrees Celsius. All wash processes stopped.");
      break;
    case 21:
      faultDescription = F("21: Water temperature not reached after 25 minutes of heating - possible heater malfunction. Water drained. All wash processes stopped.");
      break;
    case 30:
      faultDescription = F("30: Heater ON while wash pump OFF - possible wash pump malfunction or heater does not turn off. All wash processes stopped. There may be water in the machine.");
      break;
    case 40:
      faultDescription = F("40: Water level high after 3 minutes of draining - possible drain pump malfunction or water outlet blocked. All wash processes stopped. There may be water in the machine.");
      break;
    case 50:
      faultDescription = F("50: Total filling time since last drain exceeds 4 minutes - possible water leakage. Water drained. All wash processes stopped.");
      break;
    case 51:
      faultDescription = F("51: Water level low after 3 minutes of filling - check water supply, possible inlet valve malfunction, fill sensor malfunction or water leakage. Water drained. All wash processes stopped.");
      break;
    default:
      faultDescription = F(": Unspecified error.");
      faultDescription = faultCode + faultDescription;
      break;
  }
  Serial.print(F("ERROR! Fault code = "));
  Serial.println(faultDescription);
  while (true) {};
  /* Fault codes general groups:
    ======= Fault Codes ====================
    10 - 19 - inlet valve faults;
    20 - 29 - heater circuit faults;
    30 - 39 - wash pump circuit faults;
    40 - 49 - drain pump circuit faults;
    50 - 59 - water leakage faults;
    60 - 69 - water level sensor circuit faults;
    70 - 79 - temperature sensor circuit faults;
    80 - 89 - detergent solenoid circuit faults;
    90 - 99 - rinse aid solenoid circuit faults;
    100 - 109 - control electronics faults;
    110 - 119 - general faults.
    ========================================
  */
  Serial.println(F("errorFun ended.")); //temp
}



int readKey() { // Read programs selector key
  selKeyIN = analogRead(keySelect); //read the value from the sensor
  // tone (buzzer, selKeyIN, 1000); // Buzz for 1 second and frequency according to selector
  for (int i = 0; i < NumberOfPrograms; i++) {
    if (selKeyIN < KeySelectAnalogValues[i]) return i;
  }
  return 0;
}


void selMenu() { // Select a program
  Serial.println(F("selMenu started.")); //temp
  lcd.setCursor(0, 0);
  lcd.print(F("Select a program"));
  lcd.setCursor(0, 1);
  lcd.print(F("and press Start"));
  while (onOffFun() != 1) {
    lcdKeyMenu = readKey(); // read key
    lcd.setCursor(0, 1);
    lcd.print(">");
    lcd.print(ProgramNames[lcdKeyMenu]);
    lcd.print(F("                  "));
  }
  TotalPeriodStart = millis();
  timeStopped = 0;
  pauseTime = 0;
  pause = false;
  Serial.println(F("selMenu ended.")); //temp
}



int onOffFun () { // On/Pause/Resume/Reset Button
  if (digitalRead(doorBtn) == LOW) actDoorBtn();
  do { // Start loop to check for "Paused" state
    startBtnState = digitalRead(startBtn);
    if (startBtnState != O_lastButtonState) { //meaning button changed state
      lastDebounceTime = millis();
    }
    if ((startBtnState == LOW) && ((millis() - lastDebounceTime) >= R_debounceDelay)) { // button held down long enough to count as Reset
      O_lastButtonState = startBtnState;
      onButtonCount = 4;
      resetFun(); // Reset the machine.
      onButtonCount = 0;
      return onButtonCount;
    }
    if ((millis() - lastDebounceTime) >= debounceDelay) { // button held down long enough to count as normal push
      if (startBtnState != O_buttonState) {
        O_buttonState = startBtnState;
        if (O_buttonState == HIGH) { //If button has been pushed for long enough time, and then released, increment onButtonCount by 1
          onButtonCount++;
          tone (buzzer, 700, 100); // Buzz for 100 milliseconds with frequency 700 Hz
        }
      }
    }
    O_lastButtonState = startBtnState;
    switch (onButtonCount) { //At startup onButtonCount will == 0
      case 1:              //Button pushed so onButtoncount == 1, start wash cycle, or continue unchanged if already onButtoncount == 1
        return onButtonCount;
        break;
      case 2:             //Button pushed again during wash so call pause function
        pauseFun();
        break;
      case 3:             // button pushed for 3rd time so call restartFun but take 1 down from onButtonCount
        resumeFun();
        onButtonCount = 1;
        break;
      case 4:             // button pushed for so long as to count as reset
        resetFun();
        onButtonCount = 0;
    }
  }
  while (onButtonCount == 2);
  return onButtonCount;
}

//Washing actions


void actDrain() { // Drain machine
  Serial.println(F("    actDrain started.")); //temp
  TotalTimeElapsed();
  lcd.setCursor(0, 1);
  lcd.print(F("Draining...     "));
  actHeaterOFF();                      // Heater OFF
  digitalWrite(washPump, LOW);         // Wash pump OFF
  waitThreeSec();
  //fillSensState = digitalRead(fillSens); //temporarily removed - to be reused
  if (fillSensState == HIGH) {         // Fill sensor HIGH - start draining
    periodStart = millis();
    timeElapsed();
    digitalWrite(drainPump, HIGH);     // Drain pump ON
    //fillSensState = digitalRead(fillSens); //temporarily removed - to be reused
    while (fillSensState == HIGH) {
      timeElapsed();
      if (periodElapsed >= 180000) {   // Check if draining continuing for more than 3 minutes
        digitalWrite(drainPump, LOW);     // Drain pump OFF
        faultCode = 40; // Fault code 40: Water level sensor does not go LOW after 3 minutes of draining - water outlet blocked or drain pump malfunction.
        errorFun();
      }
      digitalWrite(drainPump, HIGH);     // Drain pump ON
      // fillSensState = digitalRead(fillSens); // temporarily removed - to be reused
      waitXmsec (5000); //temp - to be removed
      fillSensState = LOW; //temp - to be removed
      onOffFun();
    }
  }
  digitalWrite(drainPump, HIGH);      // Drain pump ON
  waitXmsec (5000);                   // Wait 5 sec
  digitalWrite(drainPump, LOW);       // Drain pump OFF
  TotalFillTime = 0;  // Initialize fill timer
  lcd.setCursor(0, 1);
  lcd.print(F("            "));
  Serial.println(F("    actDrain ended.")); //temp
}


void actFill() { // Fill water
  Serial.println(F("    actFill started.")); //temp
  int HeaterStatus = digitalRead(heater); // Save heater state
  int WashPumpStatus = digitalRead(washPump); // Save washPump state
  TotalTimeElapsed();
  // fillSensState = digitalRead(fillSens); //temporarily removed - to be reused
  // fillSensState = 0; //temp - to be removed
  if (fillSensState == 0) {          // Fill sensor LOW - start filling
    lcd.setCursor(0, 1);
    lcd.print(F("Filling...  "));
    dispTemp ();
    onOffFun ();
    actHeaterOFF();                    // Heater OFF
    periodStart = millis();
    timeElapsed();
    CurrentFillStart = millis();
    digitalWrite(inletValve, HIGH);    // Inlet valve ON
    waitXmsec (7000);                  // Wait 7 sec
    digitalWrite(washPump, HIGH);      // Wash pump ON
    digitalWrite(inletValve, HIGH);    // Inlet valve ON
    //fillSensState = digitalRead(fillSens); //temporarily removed - to be reused
    while (fillSensState == LOW) {
      timeElapsed();
      onOffFun ();
      if (TotalFillTime + ((millis() - CurrentFillStart)) >= 240000) {   // Check if total fill time exceeds 4 minutes
        actHeaterOFF(); // Heater OFF
        digitalWrite(washPump, LOW);   // Wash pump OFF
        digitalWrite(inletValve, LOW); // Inlet valve OFF
        faultCode = 50; // Fault code 50: Total filling time since last drain exceeds 4 minutes - water leakage.
        actDrain();
        errorFun();
      }
      if (periodElapsed >= 180000) {   // Check if filling continuing for more than 3 minutes
        actHeaterOFF(); // Heater OFF
        digitalWrite(washPump, LOW);   // Wash pump OFF
        digitalWrite(inletValve, LOW); // Inlet valve OFF
        faultCode = 51; // Fault code 51: Water level sensor does not go HIGH after 3 minutes of filling - water leakage or inlet valve or sensor malfunction.
        actDrain();
        errorFun();
      }
      digitalWrite(washPump, HIGH);    // Wash pump ON
      digitalWrite(inletValve, HIGH);  // Inlet valve ON
      // fillSensState = digitalRead(fillSens); // temporarily removed - to be reused
      waitXmsec (5000); //temp - to be removed
      fillSensState = HIGH; //temp - to be removed
      onOffFun();
      dispTemp ();
    }
    digitalWrite(washPump, HIGH);      // Wash pump ON
    digitalWrite(inletValve, HIGH);    // Inlet valve ON
    waitXmsec (5000);                  // Wait 5 sec
    digitalWrite(inletValve, LOW);     // Inlet valve OFF
    TotalFillTime += ((millis() - CurrentFillStart));  // Increase total fill timer with current fill duration
    Serial.print(F("    Total fill time = "));  // temp - to be removed
    Serial.println(TotalFillTime);  // temp - to be removed
    CurrentFillStart = 0;
    RawADC = 340; //temporary temperature - around 9.7 degrees C - to be removed
    digitalWrite(washPump, WashPumpStatus);  // Restore wash pump state
    if (HeaterStatus == HIGH) {
      actHeaterON(); // Restore heater state
    }
    else {
      actHeaterOFF();
    }
    lcd.setCursor(0, 1);
    lcd.print(SubCycleName);
  }
  dispTemp ();
  onOffFun();
  // LastFillTime = millis();           // Reset fill timer
  Serial.println("    actFill ended.");  //temp
}

void actCheckFillLevel() { // Check current water level
  // fillSensState = digitalRead(fillSens); //temporarily removed - to be reused
  // fillSensState = HIGH; //temp - to be removed
  if (fillSensState == LOW) {  // Fill sensor LOW
    Serial.println(F("    Water level low during wash - refilling.")); //temp
    actFill();     // Refill
  }
  timeElapsed();
  dispTemp();
  onOffFun();
}


void actHeaterON() { // Turn heater on
  actCheckFillLevel();
  onOffFun();
  dispTemp ();
  if (digitalRead (heater) == LOW) {
    digitalWrite (heater, HIGH);
    Serial.println(F("     actHeaterON executed.")); //temp
  }
}


void actHeaterOFF() { // Turn heater off
  onOffFun();
  dispTemp ();
  if (digitalRead(heater) == HIGH) {
    digitalWrite(heater, LOW);
    Serial.println(F("     actHeaterOFF executed.")); //temp
  }
}


void actCheckForHeatAlarms(int TargetTemperature, unsigned long MaxDuration) {  // Check for heat alarms
  onOffFun();
  dispTemp ();
  actCheckFillLevel();
  if (waterTemp() >= OverheatLimit) { // Over heating
    actHeaterOFF(); // Heater OFF
    faultCode = 20; // Foult code 20: Over-heating. Heater does not turn off.
    errorFun();
  }
  if (digitalRead(heater) == HIGH && digitalRead(washPump) == LOW) {  // Heater ON while pump OFF
    actHeaterOFF(); // Heater OFF
    faultCode = 30; // Foult code 30: Heater ON while wash pump OFF - wash pump does not turn on or heater does not turn off.
    errorFun();
  }
  if (MaxDuration > 0) {
    if ((periodElapsed >= (MaxDuration)) && (waterTemp() < TargetTemperature)) {
      actHeaterOFF(); // Heater OFF
      faultCode = 21; // Water temperature not reached after MaxDuration (25 minutes) of heating. Heater does not work.
      actDrain();
      errorFun();
    }
  }
  // Code for checking of other heater failure may be introduced here
  onOffFun();
  dispTemp ();
}

void actKeepTemp(unsigned int KeepTemperature, unsigned int KeepDuration, byte Detergent) { // Reach and maintain given temperature for given time, with or without detergent dispensing
  Serial.print(F("    actKeepTemp started at ")); //temp
  Serial.print(KeepTemperature); //temp
  Serial.print(F(" degrees, for ")); //temp
  Serial.print(KeepDuration); //temp
  Serial.print(F(" minutes, Detergent: " )); //temp
  Serial.println(Detergent); //temp
  dispTemp();
  TotalTimeElapsed();
  actCheckFillLevel();
  digitalWrite(washPump, HIGH); // Wash pump ON
  periodStart = millis(); // Starting heating water to reach desired temperature
  timeElapsed();
  while ((periodElapsed < 1500000) && (waterTemp() < (KeepTemperature)) && digitalRead(washPump) == HIGH) { // Absolute maximum duration of heat rising is 25 minutes
    actHeaterON(); // Heater ON
    actCheckForHeatAlarms(KeepTemperature, 1500000); // check for heat alarms
    actCheckFillLevel(); // check water fill level
    dispTemp();
    timeElapsed();
    onOffFun();
  }  // Temperature reached
  actCheckForHeatAlarms(KeepTemperature, 1500000);  //check also for heater failure
  Serial.println(F("    Requested temperature reached, starting temperature maintenance for requested time.")); //temp
  actCheckFillLevel(); // check water fill level
  if (Detergent == true) {  // After temperature is reached, dispense detergent if requested
    digitalWrite(DetergentSolen, HIGH); // Dispense detergent if requested
    Serial.println(F("    DetergentSolen put to ON.")); //temp
    waitXmsec (3000);
    digitalWrite(DetergentSolen, LOW);
    Serial.println(F("    DetergentSolen put to OFF.")); //temp
  }
  actCheckFillLevel(); // check water fill level
  periodStart = millis(); // Starting temperature maintenance
  timeElapsed();
  while (periodElapsed < (KeepDuration * 60000)) {
    actCheckForHeatAlarms(KeepTemperature, (KeepDuration + 5) * 60000); // check for heat alarms - parameters meant to not check for heater failure
    if ((waterTemp() < (KeepTemperature - 2)) && digitalRead(washPump) == HIGH) {
      actHeaterON(); // Heater ON
    }
    if (waterTemp() > (KeepTemperature + 2)) {
      actHeaterOFF(); // Heater OFF
    }
    actCheckFillLevel(); // check water fill level
    dispTemp();
    timeElapsed();
    onOffFun();
    // Serial.print("Maintaining temperature - current: " ); //temp
    // Serial.println(waterTemp()); //temp
  }
  actHeaterOFF(); // Heater OFF
  digitalWrite(washPump, LOW); // Wash pump OFF
  actCheckFillLevel(); // check water fill level
  dispTemp();
  timeElapsed();
  waitThreeSec();
  Serial.println("    actKeepTemp ended."); //temp
}


void actWashCycle(unsigned int WashCycleTemperature, unsigned int WashCycleDuration, byte Detergent, byte RinseAid, String SubCycleName1) { // Basic wash cycle engine
  Serial.print(F("   actWashCycle '"));
  Serial.print(SubCycleName1);
  Serial.print(F("' started at ")); //temp
  Serial.print(WashCycleTemperature); //temp
  Serial.print(F(" degrees C, for ")); //temp
  Serial.print(WashCycleDuration); //temp
  Serial.print(F(" minutes, Detergent: " )); //temp
  Serial.print(Detergent); //temp
  Serial.print(F(" , RinseAid: " )); //temp
  Serial.println(RinseAid); //temp
  SubCycleName = SubCycleName1;
  dispTemp();
  TotalTimeElapsed();
  actFill();                    // Fill
  lcd.setCursor(0, 1);
  lcd.print(SubCycleName);
  waitThreeSec();
  digitalWrite(washPump, HIGH); // Wash pump ON
  timeElapsed();
  actCheckFillLevel();
  dispTemp();
  onOffFun();
  actKeepTemp(WashCycleTemperature, WashCycleDuration, Detergent); // Reach and maintain temperature for period
  actHeaterOFF(); // Heater OFF
  actCheckFillLevel();
  waitThreeSec();
  if (RinseAid == true) { // Perform rinse aid finish if requested
    digitalWrite(washPump, HIGH);  // Wash pump ON
    if ((waterTemp() < WashCycleTemperature) && digitalRead(washPump) == HIGH) {
      actHeaterON();               // Heater ON if applicable
    }
    digitalWrite(RinseAidSolen, HIGH); // Rinse Aid solenoid ON
    waitXmsec (60000);             // Wait 1 minute
    digitalWrite(RinseAidSolen, LOW);  // Rinse Aid solenoid OFF
    waitXmsec (3000);              // Wait 3 seconds
    digitalWrite(RinseAidSolen, HIGH); // Rinse Aid solenoid ON
    waitXmsec (60000);             // Wait 1 minute
    digitalWrite(RinseAidSolen, LOW);  // Rinse Aid solenoid OFF
    actKeepTemp(WashCycleTemperature, 1, 0);  // Wash at desired temperature for 1 minute
    digitalWrite(washPump, HIGH);  // Wash pump ON
    actHeaterOFF();                // Heater OFF
    waitXmsec (60000);             // Wait 1 minute
  }
  digitalWrite(washPump, LOW); // Wash pump OFF
  waitThreeSec();
  actDrain();  // Drain
  timeElapsed();
  dispTemp();
  onOffFun();
  Serial.println(F("   actWashCycle ended.")); //temp
}


void actDry(byte DryDur) { // Dry sub-cycle
  Serial.println(F("   actDry started.")); //temp
  TotalTimeElapsed();
  lcd.setCursor(0, 1);
  SubCycleName = "Drying... ";
  lcd.setCursor(0, 1);
  lcd.print(SubCycleName);
  waitXmsec (120000);              // Wait 2 minutes
  //digitalWrite(regenSolen, HIGH);  // Regeneration solenoid ON
  actDrain();                      // Drain
  lcd.setCursor(0, 1);
  lcd.print(SubCycleName);
  waitXmsec (60000);               // Wait 1 minute
  digitalWrite(inletValve, HIGH);  // Inlet valve ON
  waitXmsec (1000);                // Wait 1 second
  digitalWrite(inletValve, LOW);   // Inlet valve OFF
  waitXmsec (3000);                // Wait 3 seconds
  digitalWrite(inletValve, HIGH);  // Inlet valve ON
  waitXmsec (1000);                // Wait 1 second
  digitalWrite(inletValve, LOW);   // Inlet valve OFF
  //digitalWrite(regenSolen, LOW);   // Regeneration solenoid OFF
  actDrain();                      // Drain
  lcd.setCursor(0, 1);
  lcd.print(SubCycleName);
  waitXmsec (DryDur * 60000);      // Wait arbitrary time according to parameter
  actDrain();                      // Drain
  Serial.println(F("   actDry ended.")); //temp
}



//Washing sub-cycles


void wMatrixCycle(String ProgramName, byte PreWashDuration, byte WashDuration,  byte Rinse1Duration, byte Rinse2Duration, byte ClearRinseDuration, byte DryDuration, byte ExpDuration, byte tempLimit ) { // Main cycle engine
  Serial.println(F("  wMatrixCycle started.")); //temp
  Serial.print(F("  Program: '")); //temp
  Serial.print(ProgramName); //temp
  Serial.print(F("', temperature: ")); //temp
  Serial.print(tempLimit); //temp
  Serial.print(F(", exp. duration: ")); //temp
  Serial.print(ExpDuration); //temp
  Serial.print(F(" , Prewash dur.: " )); //temp
  Serial.print(PreWashDuration); //temp
  Serial.print(F(" , Wash dur.: " )); //temp
  Serial.print(WashDuration); //temp
  Serial.print(F(" , Rinse 1 dur.: " )); //temp
  Serial.print(Rinse1Duration); //temp
  Serial.print(F(" , Rinse 2 dur.: " )); //temp
  Serial.print(Rinse2Duration); //temp
  Serial.print(F(" , Clear Rinse dur.: " )); //temp
  Serial.print(ClearRinseDuration); //temp
  Serial.print(F(" , Dry dur.: " )); //temp
  Serial.println(DryDuration); //temp
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(ProgramName);
  timeElapsed();
  actDrain();  // Initial drain
  waitThreeSec();
  dispTemp();
  timeElapsed();
  if (PreWashDuration > 0) {
    actWashCycle(min(36, tempLimit), PreWashDuration, 0, 0, "Prewash...    ");  // Prewash without detergent
  };
  waitThreeSec();
  dispTemp();
  timeElapsed();
  waitThreeSec();
  if (WashDuration > 0) {
    actWashCycle(tempLimit, WashDuration, 1, 0, "Washing...    ");  // Wash with detergent
  };
  waitThreeSec();
  if (Rinse1Duration > 0) {
    actWashCycle(min(15, tempLimit), Rinse1Duration, 0, 0, "Rinsing...    ");  // Rinse 1
  };
  waitThreeSec();
  timeElapsed();
  if (Rinse2Duration > 0) {
    actWashCycle(min(15, tempLimit), Rinse2Duration, 0, 0, "Rinsing...    ");  // Rinse 2
  };
  waitThreeSec();
  timeElapsed();
  if (ClearRinseDuration > 0) {
    actWashCycle(min(48, tempLimit), ClearRinseDuration, 0, 1, "Clear Rinse...    ");  // Clear Rinse with Rinse Aid
  };
  waitThreeSec();
  if (DryDuration > 0) {
    actDry(DryDuration);  // Dry
  }
  Serial.println(F("  wMatrixCycle ended.")); //temp
}


//Washing programs


void wMatrixProgram(byte ProgramIndex) {  // Matrix program engine
  Serial.println(F(" wMatrixProgram started.")); //temp
  ProgramName = ProgramNames[ProgramIndex]; // Program name
  byte PreWashDuration = PreWashDurations[ProgramIndex]; // Pre-wash duration
  byte WashDuration = WashDurations[ProgramIndex]; // Wash duration
  byte Rinse1Duration = Rinse1Durations[ProgramIndex]; // Rinse 1 duration
  byte Rinse2Duration = Rinse2Durations[ProgramIndex]; // Rinse 2 duration
  byte ClearRinseDuration = ClearRinseDurations[ProgramIndex]; // Clear Rinse duration
  byte DryDuration = DryDurations[ProgramIndex]; // Drying duration
  ExpDuration = ExpectedDurations[ProgramIndex]; // Expected duration
  tempLimit = HighestTemperatures[ProgramIndex]; // Max temperature
  if ( MatrixStructure[ProgramIndex] == 0) { // Program is not matrix - exiting procedure
    return;
  }
  wMatrixCycle (ProgramName, PreWashDuration, WashDuration, Rinse1Duration, Rinse2Duration, ClearRinseDuration, DryDuration, ExpDuration, tempLimit);
  Finish();
  Serial.println(F(" wMatrixProgram ended.")); //temp
}


void wEconom() {  // Eco programme
  Serial.println(F(" wEconom started.")); //temp
  ExpDuration = ExpectedDurations[3]; //Expected duration for Eco Wash
  tempLimit = HighestTemperatures[3]; // Max temperature for Eco Wash
  ProgramName = ProgramNames[3];
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(ProgramName); // name of Eco Wash program
  SubCycleName = "Washing... ";
  lcd.print(SubCycleName);
  actDrain(); // Initial drain
  waitThreeSec();
  actWashCycle(36, 8, 0, 0, "Prewash...    ");  // Prewash without detergent
  waitThreeSec();
  actFill();  // Fill
  waitThreeSec();
  actCheckFillLevel();
  actKeepTemp(41, 10, 1);  // Wash at 41 degrees for 10 minutes with detergent
  actKeepTemp(tempLimit, 1, 0); // Conitinue to wash at temperature tempLimit for 1 minute without detergent
  waitThreeSec();
  actDrain();  // Drain
  waitThreeSec();
  timeElapsed();
  actWashCycle(15, 6, 0, 0, "Rinsing...     ");  // Request wash cycle without detergent
  timeElapsed();
  onOffFun();
  waitThreeSec();
  dispTemp();
  actWashCycle (48, 1, 0, 1, "Clear rinse...    ");  // Wash at 48 degrees Celsius for 1 minute and finish with Rinse Aid
  waitThreeSec();
  timeElapsed();
  actDry(11); //  Dry
  Finish();
  Serial.println(F(" wEconom ended.")); //temp
}


void wCustom() {  // Custom programme - allows user selection of custom temperature and wash time, and constructs a washing cycle to fulfill the custom parameters
  Serial.println(F(" wCustom started.")); //temp
  ExpDuration = ExpectedDurations[8]; //Expected duration for Custom Wash
  tempLimit = HighestTemperatures[8]; // Max temperature for Custom Wash
  ProgramName = ProgramNames[8];
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(ProgramName); // name of program
  lcd.setCursor(0, 1);
  lcd.print(F("Temperature:      "));
  onButtonCount = 0; // initialize selector to select maximum temperature
  while (onOffFun() != 1) {
    selKeyIN = analogRead(keySelect);
    CustomCoefficient = selKeyIN / 1024.0;
    CustomTemp = (tempLimit * CustomCoefficient / 5.0)  ; // read custom temperature
    CustomTemp = CustomTemp * 5;
    if (CustomTemp < 5) CustomTemp = 5;
    lcd.setCursor(12, 1);
    lcd.print(CustomTemp);
    lcd.print((char)223);
    lcd.print("C   ");
  }
  lcd.setCursor(0, 1);
  lcd.print(F("Duration:          "));
  onButtonCount = 0; // initialize selector to select maximum duration
  while (onOffFun() != 1) {
    selKeyIN = analogRead(keySelect);
    CustomCoefficient = selKeyIN / 1024.0;
    CustomDuration = (ExpDuration * CustomCoefficient / 5.0)   ; // read custom duration
    CustomDuration = CustomDuration * 5;
    if (CustomDuration < 10) CustomDuration = 10;
    lcd.setCursor(12, 1);
    lcd.print(CustomDuration);
    lcd.print("'   ");
  }
  tone (buzzer, 700, 300); // Buzz for 300 milliseconds with frequency 700 Hz - selected custom programme starting
  ExpDuration = CustomDuration;
  TotalPeriodStart = millis();
  timeStopped = 0;
  pauseTime = 0;
  pause = false;
  Serial.print(F(" Selected custom temperature: "));
  Serial.print(CustomTemp);
  Serial.print(" degrees C, custom duration: ");
  Serial.print(CustomDuration);
  Serial.println(" minutes.");
  String ProgramName = "Custom            ";
  byte PreWashDuration = (PreWashDurations[8] * CustomCoefficient / 5.0); // Pre-Wash duration, divider 5.0 makes it less important
  PreWashDuration = PreWashDuration * 5;
  byte WashDuration = (WashDurations[8] * CustomCoefficient / 1.0); // Wash duration, divider 1.0 makes it very important
  WashDuration = WashDuration * 1;
  byte Rinse1Duration = (Rinse1Durations[8] * CustomCoefficient / 1.0); // Rinse 1 duration, divider 1.0 makes it very important
  Rinse1Duration = Rinse1Duration * 1;
  byte Rinse2Duration = (Rinse2Durations[8] * CustomCoefficient / 4.0); // Rinse 2 duration, divider 4.0 makes it less important
  Rinse2Duration = Rinse2Duration * 4;
  byte ClearRinseDuration = (ClearRinseDurations[8] * CustomCoefficient / 2.0); // Clear Rinse duration, divider 2.0 makes it more important
  ClearRinseDuration = ClearRinseDuration * 2;
  byte DryDuration = (DryDurations[8] * CustomCoefficient / 4.0); // Drying duration, divider 4.0 makes it less important
  DryDuration = DryDuration * 4;
  wMatrixCycle (ProgramName, PreWashDuration, WashDuration, Rinse1Duration, Rinse2Duration, ClearRinseDuration, DryDuration, CustomDuration, CustomTemp);
  Finish();
  Serial.println(F(" wCustom ended.")); //temp
}

void setup() {
  lcd.begin(16, 2);
  lcd.clear();
  lcd.home (); // go home
  Serial.begin(9600); // opens serial port, sets data rate to 9600 bps
  Serial.println(F("setup started.")); //temp
  for (int i = 0; i < 25; i++) tempArray[i] = 21; //first read temperature
  pinMode(heater, OUTPUT);
  pinMode(washPump, OUTPUT);
  pinMode(DetergentSolen, OUTPUT);
  pinMode(regenSolen, OUTPUT);
  pinMode(RinseAidSolen, OUTPUT);
  pinMode(drainPump, OUTPUT);
  pinMode(inletValve, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(startBtn, INPUT_PULLUP);
  pinMode(doorBtn, INPUT_PULLUP);
  // pinMode(errorSens, INPUT);
  pinMode(keySelect, INPUT);
  pinMode(fillSens, INPUT);
  pinMode(tempSensor, INPUT);
  digitalWrite(inletValve, LOW);
  digitalWrite(heater, LOW);
  digitalWrite(washPump, LOW);
  digitalWrite(drainPump, LOW);
  digitalWrite(DetergentSolen, LOW);
  digitalWrite(RinseAidSolen, LOW);
  digitalWrite(regenSolen, LOW);
  digitalWrite(fillSens, LOW);
  digitalWrite(buzzer, LOW);
  // attachInterrupt(digitalPinToInterrupt(doorBtn), actDoorBtn, FALLING); // Not used at this time - door switch is currently checked via polling together with the On/Off button
  // attachInterrupt(digitalPinToInterrupt(3), actErrorSens, FALLING); // For future use
  Serial.println("setup ended."); //temp
}


void loop() {
  Serial.println(F("loop started.")); //temp
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(F("Welcome! Arduino "));
  lcd.setCursor(0, 1);
  lcd.print(F("Dishwasher "));
  lcd.print(ProgramVersion);
  delay(4000);
  lcd.clear();
  Serial.println(F("Starting program selector.")); //temp
  selMenu();
  lcdKeyMenu = readKey(); // read key
  onOffFun();
  Serial.print(F("Programme '"));
  Serial.print(ProgramNames[lcdKeyMenu]);
  Serial.println(F("' selected.")); //temp
  delay(300);
  tone (buzzer, 700, 300); // Buzz for 300 milliseconds with frequency 700 Hz - selected programme starting
  switch (lcdKeyMenu) {
    case 1:
      wMatrixProgram(lcdKeyMenu);
      break;
    case 2:
      wMatrixProgram(lcdKeyMenu);
      break;
    case 3:
      wEconom(); // Eco program - not matrix
      break;
    case 4:
      wMatrixProgram(lcdKeyMenu);
      break;
    case 5:
      wMatrixProgram(lcdKeyMenu);
      break;
    case 6:
      wMatrixProgram(lcdKeyMenu);
      break;
    case 7:
      wMatrixProgram(lcdKeyMenu);
      break;
    case 8:
      wCustom(); // Custom program - not matrix
      break;
  }
  Serial.println(F("loop ended.")); //temp
}
