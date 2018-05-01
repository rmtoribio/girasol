// ESta version añade el watchdog para reiniciarse en caso de que se cuelgue.
// ESta version es super reducida.
// corregidos los agujeros de calcAzimut cuando m = tsunrise +120 de versiones previas.
// De hecho... solo contempla dos casos... que sea de dia o no sea de dia.


// Atmel MEGA328P AW1519 program 32K, sram 2K, eeprom 1K.
#include <avr/pgmspace.h>
#include <avr/wdt.h>
#include "Wire.h"
#include <HMC5883L.h>

const String PROMT= "Girasol2.1> ";

#define DS3231_I2C_ADDRESS 0x68
#define MiddayMin 720
#define MiddayAng 180
#define Middyear  182
#define sStarting  0
#define sNormal    1
#define sManual    2
#define sHurrican  3
#define sHailing   4
#define sCheking   5
#define sAlarm     6
#define sExecute   7
#define forward    0
#define backward   1
#define stoped     2
#define ZeroDegree 0
#define NinetyDegree 1023

#define forwardAzimutPIN    7
#define backwardAzimutPIN   6
#define forwardTiltPIN      5
#define backwardTiltPIN     4
#define AnemometerPulsePIN  2
#define SextantAnalogPinX   A0
#define SextantAnalogPinY   A1
#define SextantAnalogPinZ   A2

String         inputString; 
String         outputString;
String         Value;
int            i;      
int static     iTmpValue       = 0;
int static     sstate          = sStarting; // system state
int static     pstate          = sStarting; // previus sytem state
int static     sElevation      = 72;
int static     wElevation      = 30;
int static     solarDay        = 180;
int static     solarHour       = 12;
int static     solarMinute     = 00;
int static     solarSecond     = 00;
int static     solarWeek       = 00;
int static     sextant         = 0;
int static     compass         = 0;
int static     CompassOffset   = 195;
int static     azimuth         = 179;
int static     elevation       = 45;
int static     deltaA          = 5;
int static     deltaE          = 10;
int static     month           = 1; //we only need to do conversion.
int static     year            = 1;
int static     day             = 1;
short          thermal         = 0;
short          fracThermal     = 0;
long           PulseSample     = 0;
int            TiltX           = 0;
int            TiltY           = 0;
int            TiltZ           = 0;

HMC5883L       compassHW;
Vector         VectorDirection; 
float          heading = 0;
float          fTmpValue = 0;
float          tsunrise  = 0;
float          tsunset   = 0;
float          asunrise  = 0;
float          asunset   = 0;


boolean        comanndPending    = false;
boolean        motorsService     = true;
boolean        formaterr         = false;

const int WeekMinutesFromMidday[] = {
  230,230,231,234,239,245,253,261,271,280,
  292,303,314,326,337,347,357,363,375,383,
  389,393,397,398,399,397,395,391,386,380,
  374,368,362,355,348,342,336,329,323,315,
  308,300,294,285,277,269,260,254,247,240,
  235,232
};

const int WeekAngleFromMidday[] = {
  53,54,55,56,67,68,70,72,74,76,
  78,85,92,94,96,98,100,106,108,109,
  115,116,117,117,117,117,117,117,116,115,
  114,108,106,100,98,96,94,87,85,83,
  81,79,72,65,64,62,61,60,59,58,
  53,53
};

const int MonthElevationAtMidday[] = {
  28,31,40,51,63,71,75,71,63,51,
  40,31
};


// Convert binary coded decimal to normal decimal numbers & v.v.
byte decToBcd(byte val) { return( (val/10*16) + (val%10) );}
byte bcdToDec(byte val) { return( (val/16*10) + (val%16) );}


void setup() {
  wdt_disable();
  Serial.begin(9600);
  inputString.reserve(32);
  outputString.reserve(64);
  Wire.begin(); // set II2C subsystem.
  pinMode(forwardAzimutPIN,  OUTPUT);
  pinMode(backwardAzimutPIN, OUTPUT);
  pinMode(forwardTiltPIN,    OUTPUT);
  pinMode(backwardTiltPIN,   OUTPUT);
  
  digitalWrite(forwardAzimutPIN,  HIGH);
  digitalWrite(backwardAzimutPIN, HIGH);
  digitalWrite(forwardTiltPIN,    HIGH);
  digitalWrite(backwardTiltPIN,   HIGH);
  
  Serial.print(F("Initialize HMC5883L... "));
  while (!compassHW.begin())
  {
    Serial.print(F("Error: no compass"));
    delay(5000);
  }
  compassHW.setRange(HMC5883L_RANGE_1_3GA);
  compassHW.setMeasurementMode(HMC5883L_CONTINOUS);
  compassHW.setDataRate(HMC5883L_DATARATE_30HZ);
  compassHW.setSamples(HMC5883L_SAMPLES_8);
  compassHW.setOffset(0, 0);
  wdt_enable(WDTO_250MS);
}

void printStatus(){
      wdt_disable();
      Serial.println(F("Sensors Values:"));
      Serial.print  (F( "  compass...........: "));Serial.println(compass);
      Serial.print  (F( "  CompassOffset.....: "));Serial.println(CompassOffset);
      Serial.println(F("Calculed Sun position:"));
      Serial.print  (F( "  azimuth...........: "));Serial.println(azimuth);
      Serial.print  (F( "  elevation.........: "));Serial.println(elevation);
      Serial.print  (F( "  deltaA............: "));Serial.println(deltaA);
      Serial.print  (F( "  deltaE............: "));Serial.println(deltaE);
      Serial.println(F("Sun Domain configuratio:"));
      Serial.print  (F("   Date..............: "));Serial.print("20");Serial.print(year);
                                                   Serial.print("-");Serial.print(month);
                                                   Serial.print("-");Serial.println(day);
      Serial.print  (F( "  solarDay..........: "));Serial.println(solarDay);
      Serial.print  (F( "  solarHour.........: "));Serial.println(solarHour);
      Serial.print  (F( "  solarMinute.......: "));Serial.println(solarMinute);
      Serial.print  (F( "  solarWeek.........: "));Serial.println((solarDay/7)+1);
      Serial.print  (F("   time of sunrise...: "));Serial.println(tsunrise);
      Serial.print  (F("   time of sunset....: "));Serial.println(tsunset);
      Serial.print  (F("   azimuth sunrise...: "));Serial.println(asunrise);
      Serial.print  (F("   azimtth sunset....:" ));Serial.println(asunset);
      Serial.println(F("Stored sun path in midday offset:"));
      Serial.println(F( "Week   ->  Minutes, Angle Arc"));
        for ( i=0; i < 52; i++ ) { 
          Serial.print(i+1); Serial.print(F("week  ->  "));
          Serial.print(WeekMinutesFromMidday[i]);Serial.print(",");
          Serial.print(WeekAngleFromMidday[i]);Serial.println(";");
        }  
     wdt_enable(WDTO_250MS);  
 } 

void ReadSextant(){
    float ltilt = 0;
    TiltX = analogRead(SextantAnalogPinX);
    TiltY = analogRead(SextantAnalogPinY);
    TiltZ = analogRead(SextantAnalogPinZ); 

    if ( TiltZ >  374 ) { ltilt = 45 ;}  
    if ( TiltZ <= 374 ) { ltilt = 55 - 1.40 * (TiltZ - 367) ;}
    if ( TiltZ <= 367 ) { ltilt = 65 - 0.83 * (TiltZ - 355) ;} 
    if ( TiltZ <= 355 ) { ltilt = 75 - 0.76 * (TiltZ - 342) ;}
    if ( TiltZ <= 342 ) { ltilt = 80 - 0.80 * (TiltZ - 338) ;}
    sextant =(int)(ltilt + 0.5);  
}

void ReadCompass(){
  
    VectorDirection = compassHW.readNormalize();
    // +50 calibra el error de construcción dentro de la caja. 
    heading = atan2(VectorDirection.YAxis+50, VectorDirection.XAxis);
    compass = heading / 0.01745; //de radianes a grados.
    compass = CompassOffset + compass; 
    compass = compass % 360;

}

void calcSolarDay ()
{
  solarDay =0;
   // waterfall sum (note lack of "break" sentence)
   switch (month){ 
   case 12 : solarDay += 30;
   case 11 : solarDay += 31;
   case 10 : solarDay += 30;
   case  9 : solarDay += 31;
   case  8 : solarDay += 31;
   case  7 : solarDay += 30;
   case  6 : solarDay += 31;
   case  5 : solarDay += 30;
   case  4 : solarDay += 31;
   case  3 : solarDay += 28;
             if (year == 20) solarDay++;
             if (year == 24) solarDay++;
             if (year == 28) solarDay++;
             if (year == 32) solarDay++;
             if (year == 36) solarDay++;
             if (year == 40) solarDay++;
             if (year == 44) solarDay++;
             if (year == 48) solarDay++;
             if (year == 52) solarDay++;
             if (year == 56) solarDay++;
             if (year == 60) solarDay++;
             if (year == 64) solarDay++;
             if (year == 68) solarDay++;
             if (year == 72) solarDay++;
             if (year == 76) solarDay++;
             if (year == 80) solarDay++;
             if (year == 84) solarDay++;
             if (year == 88) solarDay++;
             if (year == 92) solarDay++;
             if (year == 96) solarDay++;
   case  2 : solarDay += 31;
   case  1 : solarDay += day;
   }
}

void WriteTime()
{
  // sets time and date data to DS3231
  Wire.beginTransmission(DS3231_I2C_ADDRESS);
  Wire.write(0);                      // set next input to start at the seconds register
  Wire.write(00);                     // set seconds
  Wire.write(decToBcd(solarMinute));  // set minutes
  Wire.write(decToBcd(solarHour));    // set hours
  Wire.write(decToBcd(1));            // set day of week (1=Sunday, 7=Saturday)
  Wire.write(decToBcd(day));          // set date (1 to 31)
  Wire.write(decToBcd(month));        // set month
  Wire.write(decToBcd(year));         // set year (0 to 99)
  Wire.endTransmission();
}

void ReadTime()
{
    Wire.beginTransmission(DS3231_I2C_ADDRESS);
    Wire.write(0x00);                       // set DS3231 register pointer to 00h
    Wire.endTransmission();
    Wire.requestFrom(DS3231_I2C_ADDRESS, 7);
    // request seven bytes of data from DS3231 starting from register 00h
    solarSecond = bcdToDec(Wire.read() & 0x7f);
    solarMinute = bcdToDec(Wire.read());
    solarHour   = bcdToDec(Wire.read() & 0x3f);
    day         = bcdToDec(Wire.read()); //d of week, We dont need it, overwriten.
    day         = bcdToDec(Wire.read());
    month       = bcdToDec(Wire.read());
    year        = bcdToDec(Wire.read());
    
    Wire.beginTransmission(DS3231_I2C_ADDRESS);
    Wire.write(0x11); // set DS3231 register pointer to 11h Temperature
    Wire.endTransmission();
    Wire.requestFrom(DS3231_I2C_ADDRESS, 2);
    thermal = Wire.read();
    fracThermal = Wire.read();
    calcSolarDay();
}

int calcAzimuth (){
  
  float azi;
  tsunrise = MiddayMin - (float)WeekMinutesFromMidday[solarDay/7];
  tsunset  = MiddayMin + (float)WeekMinutesFromMidday[solarDay/7];
  asunrise = MiddayAng - (float)WeekAngleFromMidday[solarDay/7];
  asunset  = MiddayAng + (float)WeekAngleFromMidday[solarDay/7];

  float m  = (float)(solarHour * 60 + solarMinute); //minute of day

  // Durante el dia...
  if (  tsunrise < m < tsunset ) {
    azi = asunrise + (asunset - asunrise) / (tsunset - tsunrise)  *  (m - tsunrise);
    return (int)(azi + 0.5) ;
  }
  // Durante la noche...
  else {
    return asunrise;
  }
}

int calcElevation (){
  float elev;

  elev = MonthElevationAtMidday[month-1]/2 ;  // Por la mañana
  if ( solarHour > 10 ) { elev= MonthElevationAtMidday[month-1] ; }  // Al medio dia
  if ( solarHour > 14 ) { elev= MonthElevationAtMidday[month-1]/2 ; } // Por la tarde

  i = (int)(elev + 0.5);
  if ( i < 45 ) return 45; // mechanic limit.
  if ( i > 75 ) return 75; // mechanic limit.
  return i ; 
} 

byte moveAzimutMotor ( byte acction) {
  
   if ( acction == forward ) {
     digitalWrite(forwardAzimutPIN,  LOW);
     digitalWrite(backwardAzimutPIN, HIGH);
   }

   if ( acction == backward ) {
     digitalWrite(forwardAzimutPIN,  HIGH);
     digitalWrite(backwardAzimutPIN, LOW);
   }

   if ( acction == stoped ) {
     digitalWrite(forwardAzimutPIN,  HIGH);
     digitalWrite(backwardAzimutPIN, HIGH);
   }
}

byte moveTiltMotor ( byte acction ) {
    
    if ( acction == forward ) { 
      digitalWrite(forwardTiltPIN,    LOW);
      digitalWrite(backwardTiltPIN,   HIGH);
    }
   
    if ( acction == backward ) {
      digitalWrite(forwardTiltPIN,    HIGH);
      digitalWrite(backwardTiltPIN,   LOW);
    }
   
    if ( acction == stoped ) {
      digitalWrite(forwardTiltPIN,    HIGH);
      digitalWrite(backwardTiltPIN,   HIGH);
    } 
}

void printInfo (){
       Serial.print(F("DATE: "));
       Serial.print("20");Serial.print(year);Serial.print(" ");
       Serial.print("D");Serial.print(solarDay);Serial.print(" ");
       Serial.print(" (week");Serial.print(solarDay/7);Serial.print(F(") H.Solar:"));
       Serial.print(solarHour);Serial.print("h");
       Serial.print(solarMinute);Serial.print("m");
       Serial.print(solarSecond);Serial.println("s ");
       Serial.print(F("  -- Compass/Azimuth: "));   Serial.print(compass);Serial.print("/");Serial.println(azimuth);
       Serial.print(F("  -- Sextant/Elevation: ")); Serial.print(sextant);Serial.print("/");Serial.println(elevation);
       Serial.print(F("  -- Ma"));Serial.print(digitalRead(forwardAzimutPIN));Serial.println(digitalRead(backwardAzimutPIN));
       Serial.print(F("  -- Mt"));Serial.print(digitalRead(forwardTiltPIN));Serial.println(digitalRead(backwardTiltPIN));
}


void serialEvent() {
  while (Serial.available()) {
    char inChar = (char)Serial.read();
    if (inChar == '\n') {
      comanndPending = true;
    }
    else {
      inputString += inChar;
    }
  }
}

//# # # # # #   M A I N   L O O P  # # # # # 

void loop() {

  printInfo();

  // Cada bucle tarda 3segundos en ejecutarse.
  // entre tanto resetea el watchdog.
  // Cuidado! este es el periodo minimo que funcionaran los motores.
  // si es muy largo puede que se pase de la posicion correcta.

  
  for (i=0;i<30;i++) {
    wdt_reset();
    delay(100);
  }
  
  ReadTime();
  ReadCompass();
  ReadSextant();
  
  switch(sstate) {

  case sStarting :
    pstate = sStarting;
    Serial.println(F("Start System..."));
    printStatus(); 
    sstate = sNormal;
    break;      
    
  case sNormal :
    pstate = sNormal;

    azimuth  = calcAzimuth();
    elevation = calcElevation();

    if      ( compass < (azimuth - deltaA) )   { moveAzimutMotor (forward); }
    else if ( compass > (azimuth + deltaA) )   { moveAzimutMotor (backward);}
    else                                       { moveAzimutMotor (stoped);  }
    
    if      ( sextant < (elevation - deltaE))  { moveTiltMotor   (forward); }
    else if ( sextant > (elevation + deltaE))  { moveTiltMotor   (backward);}
    else                                       { moveTiltMotor   (stoped);  }

    sstate = sNormal;
    if (comanndPending) { sstate = sExecute;}
  break; 
 
  case sExecute :

    // Estos son los comando definidos:
    // set date 2018-02-05 18:33
    // set compass 90
    // set sextant 45
    // set deltaA 5
    // set deltaE 5
    // set compassoffset 20
    
    if (inputString.startsWith("set date ")) {
      //i.e: set date 2018-02-25 18:33
      Value = inputString.substring(11,12);
      iTmpValue = Value.toInt();
      year = iTmpValue;
      
      Value = inputString.substring(14,15);
      iTmpValue = Value.toInt();
      month = iTmpValue; 
      
      Value = inputString.substring(17,18);
      iTmpValue = Value.toInt();
      day = iTmpValue;  
 
      Value = inputString.substring(20,21);
      iTmpValue = Value.toInt();
      solarHour = iTmpValue;

      Value = inputString.substring(23,24);
      iTmpValue = Value.toInt();
      solarMinute = iTmpValue;
 
      WriteTime();
    }
    
    if (inputString.startsWith("set compass ")) {
      Value = inputString.substring(12, inputString.length());
      iTmpValue = Value.toInt();
      compass = iTmpValue;
      Serial.println(inputString);
    }

    if (inputString.startsWith("set sextant ")) {
      Value = inputString.substring(12,13);
      iTmpValue = Value.toInt();
      sextant = iTmpValue;
    }

    if (inputString.startsWith("set deltaA ")) {
      Value = inputString.substring(11,12);
      iTmpValue = Value.toInt();
      deltaA = iTmpValue;
    }
    
    if (inputString.startsWith("set deltaE ")) {
      Value = inputString.substring(11, inputString.length());
      iTmpValue = Value.toInt();
      deltaA = iTmpValue;
    }
    
    if (inputString.startsWith("set compassoffset ")) {
      Value = inputString.substring(18, inputString.length());
      iTmpValue = Value.toInt();
      CompassOffset = iTmpValue;
    }

    if (inputString.startsWith("status")) {
      printStatus();
    } 
    
    inputString = "";
    comanndPending = false;
    sstate = pstate;
    //NOTE: pstate = sstate, could be inf.loop when sstate == sExecute.
    //      So, pstate = sNormal is less harmfull.
    if (comanndPending) { pstate = sNormal; sstate = sExecute;} 
    break; //sExecute
  }
}

