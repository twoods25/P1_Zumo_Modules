#include <Wire.h>     // inkluderer bibliotek
#include <Zumo32U4.h> // inkluderer bibliotek

Zumo32U4Motors motors;           // inkluderer motorer
Zumo32U4LineSensors lineSensors; // inkluderer linjesensorer
Zumo32U4OLED OLED;               // inkluderer OLED-skærm

#define NUM_SENSORS 3                         // Definerer antal sensorer
unsigned short lineSensorValues[NUM_SENSORS]; // Gemmer målingerne i en array
int Direction = 0;                            // Retning - bruges til cases
int subDirection = 2;                         // Subretning - bruges til cases

//                                 L     C     R
// Hvis det er hvid er det under = 1000 , 600 , 1000
unsigned short LR_ThreshHold = 1000; // Tærskelværdi for hvid venstre sensor
unsigned short C_ThreshHold = 600;   // Tærskelværdi for hvid midterste sensor

unsigned short speed = 100; // sætter hastighed (unsigned short = 0 til 65535)

// Her bruges struct af forskellige situationer for sensorene til at bestemme hvad robotten skal gøre.
  struct setBooleans
{
  // Boolean værdier for forskellige situationer af sensorsignaler med en reprentativt navn. (Gør koden lettere at læse)
  bool S0Sort1Hvid2Hvid = ((lineSensorValues[Direction] > LR_ThreshHold) && (lineSensorValues[1] < C_ThreshHold) && (lineSensorValues[subDirection] < LR_ThreshHold));
  bool S0Hvid1Hvid2Hvid = ((lineSensorValues[Direction] < LR_ThreshHold) && (lineSensorValues[1] < C_ThreshHold) && (lineSensorValues[subDirection] < LR_ThreshHold));
  bool S0Sort1Sort2Sort = ((lineSensorValues[Direction] > LR_ThreshHold) && (lineSensorValues[1] > C_ThreshHold) && (lineSensorValues[subDirection] > LR_ThreshHold));
  // Dette gør if sætningerne senere nemmere at læse og forstå.
};

// Funktion til debugging - printer sensorværdier til serial monitor
void printReadingsToSerial()
{
  // Et char array til at holde den formaterede streng. Tallet 80 er valgt, da vi ikke har flere end 80 tegn i vores serial monitor.
  char buffer[80]; // opretter tegn-array med plads til 80 tegn
  // %4d betyder et heltal med minimum bredde på 4 tegn, \n betyder ny linje.
  sprintf(buffer, "%4d %4d %4d\n", lineSensorValues[0], lineSensorValues[1], lineSensorValues[2]); // sprintf() er en funktion der formaterer en streng og gemmer den i et char array.
  Serial.print(buffer);                                                                            // Printer Arrayet til serial monitor
}
// Funktion til at læse sensorsignalerne og gemme dem i arrayet lineSensorValues[]. QTR_EMITTERS_ON betyder at sensorerne er tændt.
void readLineSensors()
{
  lineSensors.read(lineSensorValues, QTR_EMITTERS_ON); // læser sensorerne og gemmer værdierne i arrayet lineSensorValues[NUM_SENSORS].
}
// Funktion til at køre robotten fremad.
void forward()
{
  motors.setSpeeds(speed, speed); // kør med defineret hastighed
}
// Funktion til at dreje robotten til venstre.
void turnLeft()
{
  motors.setSpeeds(-speed, speed); // drej til venstre med defineret hastighed
}
// Funktion til at dreje robotten til højre.
void turnRight()
{
  motors.setSpeeds(speed, -speed); // drej til højre med defineret hastighed
}

// Funktion til at vælge retning baseret på sensorsignalerne.
void chooseDirection()
{
  do // Do while loop for at præsentere valgmuligheder indtil en linje er fundet.
  {
    readLineSensors();                       // Læs sensorsignalerne.
    forward();                               // Kør fremad mens der læses.
    if (lineSensorValues[0] > LR_ThreshHold) // Hvis venstre sensor ser linjen.
    {
      Direction = 0;    // Sæt retning til venstre.
      subDirection = 2; // Sæt subretning til højre.
      return;           // Afslut funktionen.
    }
    else if (lineSensorValues[2] > LR_ThreshHold) // Hvis højre sensor ser linjen.
    {
      Direction = 2;    // Sæt retning til højre.
      subDirection = 0; // Sæt subretning til venstre.
      return;           // Afslut funktionen.
    }
  } while (1); // Fortsæt indtil en linje er fundet.
}

// Funktion til at følge linjen baseret på sensorsignalerne.
void followDirection()
{
  if (setBooleans().S0Sort1Hvid2Hvid) // Hvis sensorens 'direction' ser sort og midterste og subsensor ser hvid.
  // Navnet "S0Sort1Hvid2Hvid" er dynamisk og kunne også gælde for "S0Hvid1Hvid2Sort" afhængigt af retningen.
  {
    forward(); // Kør fremad.
  }
  else if (setBooleans().S0Hvid1Hvid2Hvid) // Hvis alle sensorer ser hvid.
  {
    while (lineSensorValues[Direction] < LR_ThreshHold) // Mens sensor direction ser hvid.
    {
      readLineSensors();  // Læs sensorsignalerne.
      if (Direction == 0) // Hvis retning er venstre.
        turnLeft();       // Drej til venstre.
      else
        turnRight(); // Ellers drej til højre.
    }
  }
  else if (setBooleans().S0Sort1Sort2Sort) // Hvis alle sensorer ser sort.
  {
    while (lineSensorValues[Direction] > LR_ThreshHold) // Mens sensor direction ser sort.
    {
      readLineSensors();  // Læs sensorsignalerne.
      if (Direction == 0) // Hvis retning er venstre.
        turnRight();      // Drej til højre.
      else
        turnLeft(); // Ellers drej til venstre.
    }
  }
}

// Hovedprogrammet setup() køres én gang ved opstart.
void setup()
{
  lineSensors.initThreeSensors(); // initialiserer de tre sensorer
  Serial.begin(9600);             // starter serial kommunikation
  chooseDirection();              // vælg retning
}
// Hovedprogrammet loop() køres gentagne gange indtil robotten slukkes.
void loop()
{
  readLineSensors();       // læs sensorerne
  printReadingsToSerial(); // print til serial monitor
  switch (Direction)       // vælg handling baseret på retning
  {
  case 0:
    followDirection(); // hvis venstre sensor ser linjen: følg linjen
    break;
  case 2:
    followDirection(); // hvis højre sensor ser linjen: følg linjen
    break;
  default:
    forward();                                           // hvis ingen linje er fundet: kør lige ud
    Serial.println("No line detected, moving forward."); // Brugt til debugging
  }
}
