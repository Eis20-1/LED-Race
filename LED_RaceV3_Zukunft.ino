
//AUFBAU//
//Der Datenanschluss des LED-Streifens ist mit dem PWM-Pin 6 verbunden. Für Spieler 1 dient der Taster an Pin 9 zum Beschleunigen
//und Pin 10 als temporaerer Boost. Bei Spieler 2 dient der Taster an Pin 11 zum Beschleunigen und der Taster an Pin 12 als temporaerer Boost.
//Der Piezo wird mit Pin 5 verbunden.

//Als LED-Streifen-Typ kommt der WS2812B zum Einsatz. Es wird kein problemfreier Betrieb garantiert, falls ein anderer LED-Streifen verwendet werden sollten.

//Aenderungen bei der Pinbelegung sollten bitte auch im Code vermerkt werden!!!


#include <Adafruit_NeoPixel.h>      //benoetigte Bibliothek fuer LED-Streifen

#ifdef __AVR__
#include <avr/power.h> // Required for 16 MHz Adafruit Trinket
#endif

#include "pitches.h"                //benoetigte Bibliothek fuer alle Toene

//veraenderbare Variablen
//------------------------------------------------------------------------------------------
#define NUMPIXELS 245            //Anzahl aller LEDs, welche auf dem LED-Streifne vom Arduino angesprochen werden.

int trackend = 192;              // Ende der Normalen Strecke
int interruptionstart = 112;   // Pixel auf der Normalstrecke, wo die Unterbrechung beginnt.
int interruptionend = 168;       // Pixel auf der Normalstrecke, wo die Unterbrechung endet.


float maxSpeed = 0.7;           //Maximales Tempo: Jedes Tempo von 0.00 bis 1.00 ist vom Programmcode fehlerfrei verarbeitbar. Fuer den Boost sollte aber ein empfohlener Puffer von mindestens 0.2 festgehalten werden.
float difficultness = 5;       //Schwierigkeitsgrad fuer das Beschleunigen: Werte von 0 - 15 koennen hier eingetragen werden. Je hoeher der Wert desto schwieriger wird das Erreichen der Maximalgeschwindigkeit.

int movingUpwards1_Begin = 119;       //Ort, an dem die Steigung beginnt.
int movingUpwards1_End = 135;         //Ort, an dem die Steigung endet.
int movingDownwards1_Begin = 145;     //Ort, an dem das Gefälle beginnt.
int movingDownwards1_End = 161;       //Ort, an dem das Gefälle endet.

int movingUpwards2_Begin = 192;       //Ort, an dem die 2. Steigung beginnt.
int movingUpwards2_End = 208;         //Ort, an dem die 2. Steigung endet.
int movingDownwards2_Begin = 210;     //Ort, an dem das 2. Gefälle beginnt.
int movingDownwards2_End = 221;       //Ort, an dem das 2. Gefälle endet.

int movingUpwards3_Begin = 192;       //Ort, an dem die 3. Steigung beginnt.
int movingUpwards3_End = 208;         //Ort, an dem die 3. Steigung endet.
int movingDownwards3_Begin = 210;     //Ort, an dem das 3. Gefälle beginnt.
int movingDownwards3_End = 221;       //Ort, an dem das 3. Gefälle endet.


int laps = 5;                   //Max. Anzahl an Runden, die zum Erreichen des Ziels notwendig sind.

byte devSettings = 0;           //Entwicklertools: 0 - Aus, 1 - Ein

long points = 1000;          //Punktezahl welche mit dem Faktor verrechnet wird

//------------------------------------------------------------------------------------------
//LCD-Display
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
LiquidCrystal_I2C lcd(0x27, 20, 4);
//------------------------------------------------------------------------------------------
//Punktesystem
long schleifen = 0;           //Anzahl der Schleifen in denen der Wert erhöht wird
float xP1 = 0.000;                //Variable des Spieler 1 , welche durch den SpeedP1 erhöht wird
String stringxP1 = String(xP1, 3);
float xP2 = 0.000;                //Variable des Spieler 2 , welche durch den SpeedP2 erhöht wird
String stringxP2 = String(xP2, 3);
float faktorP1 = 0.000;           //Errechneter Faktor zum errechnen der Punktzahl von Spieler 1
String stringfaktorP1 = String(faktorP1, 3);
float faktorP2 = 0.000;           //Errechneter Faktor zum errechnen der Punktzahl von Spieler 1
String stringfaktorP2 = String(faktorP2, 3);
long PointsP1 = 0;            // Punktzahl Spieler 1
long PointsP2 = 0;            // Punktzahl Spieler 2
//------------------------------------------------------------------------------------------
#define melodyPin 6                                         //Datenpin des Piezos bzw. Lautsprechers
#define PIN 11                                               //Datenpin des LED-Streifens

//Pin-Belegung der Taster fuer Spieler 1
const int player1AccButton = 2;
const int player1BoostButton = 3;

//Pin-Belegung der Taster fuer Spieler 2
const int player2AccButton = 4;
const int player2BoostButton = 5;

const int LED = 0;                                        // Datenpin zum Ansteuern einer LED
int randomNum = 0;                                        //Variable fuer einen zufaellig definierten Pixel, an dem einer der Wuermer die Unterbrechung triggern soll.


//    Spieler 1
int P1firstLED_int = 0;                                 //Variable auf der die Position des ersten Spielers als ganze Zahl gespeichert wird

float P1firstLED = -2;                              //Variable für die Position des 1. Pixels beim Wurm
float P1secondLED = -3;                             //Variable für die Position des 2. Pixels beim Wurm
float P1thirdLED = -4;                              //Variable für die Position des 3. Pixels beim Wurm
float P1fourthLED = -5;                             //Variable für die Position des 4. Pixels beim Wurm
float P1fifthLED = -6;                              //Variable für die Position des 5. Pixels beim Wurm.

//    Spieler 2
int P2firstLED_int = 0;                                  //Variable auf der die Position des zweiten Spielers als ganze Zahl gespeichert wird

float P2firstLED = -2;                               //Variable für die Position des 1. Pixels beim Wurm
float P2secondLED = -3;                              //Variable für die Position des 2. Pixels beim Wurm
float P2thirdLED = -4;                               //Variable für die Position des 3. Pixels beim Wurm
float P2fourthLED = -5;                              //Variable für die Position des 4. Pixels beim Wurm
float P2fifthLED = -6;                               //Variable für die Position des 5. Pixels beim Wurm


//----------------------------------------------------------------------------------------------

float taktzukunft= 1400;
float zukunft[]={NOTE_DS4, taktzukunft/4, NOTE_FS4, taktzukunft/4, 0, taktzukunft/16, NOTE_FS4, taktzukunft/32, 0, taktzukunft/32,  NOTE_FS4, taktzukunft/32, 0, taktzukunft/32, NOTE_FS4, taktzukunft/32, 0, taktzukunft/32, 0, taktzukunft/16, NOTE_GS4, taktzukunft/4, 0, taktzukunft/16, NOTE_B4, taktzukunft/4, 0, taktzukunft/8, NOTE_C5, taktzukunft/5.2, 0, taktzukunft/16, NOTE_C5, taktzukunft/5.2, 0, taktzukunft/16, NOTE_CS5, taktzukunft/5.2, NOTE_DS5, taktzukunft/5.2};


int ar;

//_____________________________________________________________________________________________________________________________________________________________S_E_T_U_P


Adafruit_NeoPixel pixels(NUMPIXELS, PIN, NEO_GRB + NEO_KHZ800); //Deklarierung des LED-Stripes mit Anzahl der LEDs (Arg.1), angeschlossene Pin (Arg. 2). Arg. 3 und 4 sind vorgegeben

void setup() { //Wird ein Mal ausgefuehrt
  // These lines are specifically to support the Adafruit Trinket 5V 16 MHz.
  // Any other board, you can remove this part (but no harm leaving it):
#if defined(__AVR_ATtiny85__) && (F_CPU == 16000000)
  clock_prescale_set(clock_div_1);
#endif
  //Serial.begin(9600); //Zum Beobachten und ueberpruefen verschiedener Werte
  pinMode(player1AccButton, INPUT); //Taster wird als Input deklariert
  pinMode(player2AccButton, INPUT);
  pinMode(player1BoostButton, INPUT);
  pinMode(player2BoostButton, INPUT);
  pinMode(melodyPin, OUTPUT);
  pinMode(LED, OUTPUT);                     //

  Wire.begin(); //beginn der I2C Verbindung der beiden Arduinos
  lcd.init();   //Hinzufügen des LCD-Displays
  lcd.backlight();
  lcd.clear();
  pixels.begin(); //Intialisiert LED-Strip
  pixels.clear(); //Laesst saemtliche LEDS ausschalten (0,0,0)
  pixels.show(); //Zeigt die Veraenderung (hier:pixels.clear();) an
  randomSeed(analogRead(0)); //Sorgt fuer komplett zufaellige Wert. Ohne Verwendung dieser Zeile, waeren die Werte von dem Befehl "random" nicht zufaellig

  randomNum = random(0, trackend);
  LCDMenu();

  ar = sizeof(zukunft) / sizeof(float);
}


//_____________________________________________________________________________________________________________________________________________________________L_O_O_P

//Variablen für den "loop"
//---------------------------------

//Tasterzustand
int startRace = 0;

int switchStateP1 = 0;
int prevSwitchStateP1 = 0;
int switchStateP2 = 0;
int prevSwitchStateP2 = 0;

byte switchStateItemP1 = 0;
byte switchStateItemP2 = 0;
byte prevSwitchStateItemP1 = 0;
byte prevSwitchStateItemP2 = 0;


void loop() {


  switchStateP1 = digitalRead(player1AccButton);//Liest den Wert von Pin 10 (button1) und speichert diesen im switchStateP1 ab.
  switchStateP2 = digitalRead(player2AccButton); //Liest den Wert von Pin 11 (button2) und speichert diesen im switchStateP2 ab.
  switchStateItemP1 = digitalRead(player1BoostButton);
  switchStateItemP2 = digitalRead(player2BoostButton);

/* if(devSettings == 1){
  switchStateP1 = 1;
  switchStateP2 = 1;
  switchStateItemP1 = 1;
  switchStateItemP2 = 1;
}*/

  if (startRace == 0) {
    menuBackground();
    skins();
    skinmode();
    LCDMenu();
  }
  startSeq();
  if (startRace == 2) {
    pointSystem();
    skins();
    movement();
    LCD();
  }
  if (startRace == 3) {
    LCDDisplay();

    if (switchStateP1 == HIGH && switchStateP2 == HIGH && switchStateItemP1 == HIGH && switchStateItemP2 == HIGH) {
      startRace = 0;
      lcd.clear();
    }
  }

  prevSwitchStateP1 = switchStateP1;                        //Der vorherige Wert soll den aktuellen Wert der Variable annehmen
  prevSwitchStateP2 = switchStateP2;
  prevSwitchStateItemP1 = switchStateItemP1;
  prevSwitchStateItemP2 = switchStateItemP2;

  P2firstLED_int = (int) P2firstLED + 1;          //Die Position des roten Spielers wird als ganze Zahl auf P2firstLED_int gespeichert
  P1firstLED_int = (int) P1firstLED + 1;        //Die Position des blauen Spielers wird als ganze Zahl auf P1firstLED_int gespeichert

  //pixels.setPixelColor(trackend, pixels.Color(255, 255, 255));
  pixels.show();
}

//_____________________________________________________________________________________________________________________________________________________________S_T_A_R_T_S_E_Q_U_E_N_Z

int startCounter;                               //Aktiver Pixel, welcher nach und nach den LED-Streifen während der Ampelsequenz mit der roten Ampelfarbe auffüllt.
int prepForStartSeq;                            //Countdown zum Starten des Rennens.

void startSeq() {
  if (switchStateP1 == HIGH && switchStateP2 == HIGH && prepForStartSeq < 300 && startRace == 0) { //Werden die Starttaster (button1 & button2) gedrueckt gehalten und der Zaehler zum Setzen der Startsequenz "prepForStartSeq" ist kleiner als der Wert 300,
    prepForStartSeq++;//dann soll dieser Zaehler solange um +1 erhöht werden und
    delay(2);                                                                                       //und dazwischen soll eine Verzögerung von 10ms liegen (300 x 10ms = 3000ms - damit muessen beide Taster zusammen 3 Sekunden gedrueckt werden, sodass die Startsequenz eingeleitet wird.)
  }
  else if (prepForStartSeq > 0 && prepForStartSeq < 300) {   //Werden die Taster vor der Einleitung des Startvorgangs losgelassen, sodass der Wert des Zaehlers zwischen 0 und 300 liegt,
    prepForStartSeq = 0;                                                                 //dann soll der Zaehler wieder auf 0 gesetzt werden.
  }

  if (prepForStartSeq == 300 && startRace == 0) {
    startRace = 1;
  }

  if (startRace == 1) {
    int start_melody[] = {
      NOTE_B3, NOTE_B3, NOTE_B3, NOTE_GS5
    };
    //Tondauer
    int start_tempo[] = {
      2, 2, 2, 1
    };

    if (startCounter < trackend) {  // Wenn startCounter kleiner als NUMPIXELS ist
      pixels.setPixelColor(startCounter, pixels.Color(250, 0, 0));  // soll der aktuelle Wert von startCounter (LED) rot erleuchten
      startCounter++; // anschließend wird startCounter um den Wert 1 erhoeht
      delay(2); // Wartezeit von 10/1000 ms
    }

    if (startCounter == trackend) {  // = wenn alle LEDs aufleuchten
      pixels.clear();// sollen alle LEDs erloeschen
      pixels.show(); // Dient der Anzeige der Veraenderungen aus der vorigen Zeile
      delay(250); // Dient der besseren Visualisierung des Erlöschen aller LEDs

      int size = sizeof(start_melody) / sizeof(int); // bestimmt die Groesse der Melodie. Sie ist vier Toene gross.
      for (int thisNote = 0; thisNote < size; thisNote++) { // so lange nicht alle Noten abgespielt wurden

        if (thisNote ==  0 || thisNote == 1) { // wenn die aktuelle Note die erste (0) oder die zweite Note (1) ist (die Noten werden ab 0 hochgezaehlt)
          pixels.fill(pixels.Color(200, 0, 0)); //soll der gesamte LED-Streifen rot erleuchten
          pixels.show(); // Dient der Anzeige der Veraenderungen aus der vorigen Zeile
          lcd.clear();
          lcd.setCursor(3, 1);
          lcd.print("Auf die Plaetze");
        }
        if (thisNote ==  2) { // wenn die aktuelle Note die dritte Note (2) ist
          pixels.fill(pixels.Color(200, 110, 0)); // soll der gesamte LED-Streifen gelb erleuchten
          pixels.setBrightness(100); //setzt die Helligkeit des Streifens auf 100 runter.
          pixels.show(); // dient der Anzeige der Veraenderungen aus der vorigen Zeile
          lcd.clear();
          lcd.setCursor(7, 1);
          lcd.print("Fertig");
        }
        if (thisNote ==  3) { // wenn die aktuelle Note die vierte Note (3) ist
          pixels.fill(pixels.Color(0, 200, 0)); // soll der gesamte LED-Streifen gruen erleuchten
          pixels.setBrightness(250); //Setzt die Helligkeit wieder auf 250.
          pixels.show(); // dient der Anzeige der Veraenderungen aus der vorigen Zeile

          lcd.clear();
          lcd.setCursor(6, 1);
          lcd.print("Loooooos");
        }

        int noteDuration = 1000 / start_tempo[thisNote]; // Zur Berechnung der Dauer der Tonwiedergabe wird eine Sekunde (= 1000 ms) durch die in start_tempo definierte Tondauer dividiert. (halbe Note = 1000/2, ganze Note = 1000/1)

        buzz(melodyPin, start_melody[thisNote], noteDuration); // spielt die Note fuer die angegebene Tondauer ueber das Piezo-Element ab
        int pauseBetweenNotes = noteDuration * 1.30; // um die Toene voneinander unterscheiden zu koennen, wird die Tondauer + 30%  als Pause zwischen den einzelnen Noten als pauseBetweenNotes eingefuegt
        pixels.clear(); // alle LEDs erloeschen
        pixels.show(); // Dient der Anzeige der Veraenderungen aus der vorigen Zeile
        delay(pauseBetweenNotes); // Pause zwischen den Toenen
        LCDClear();
      }
      startRace = 2; // setStart wird auf 1 gesetzt, damit das Rennen beginnt.
    }
  }
}


//_____________________________________________________________________________________________________________________________________________________________P_I_E_Z_O_-_T_O_N_E_R_Z_E_U_G_U_N_G

void buzz(int targetPin, long frequency, long length) { // Funktion zum Abspielen der einzelnen Toene
  long delayValue = 1000000 / frequency / 2; // 1000000 Mikrosekunden (= 1 Sekunde) geteilt durch die abzuspielende Note geteilt durch 2 (da zwei Phasen waehrend eines Zyklus (Oeffnungs- und Schliessungsphase)
  long numCycles = frequency * length / 1000; // Anzahl der Zyklen = Note (= Zyklen pro Sekunde) * Notenlaenge / 1000
  for (long i = 0; i < numCycles; i++) {  // so lange die kalkulierte Zeit nicht erreicht ist
    digitalWrite(targetPin, HIGH); //faehrt die Membran des Piezo-Elementes aus und laesst ihn den Ton abspielen
    delayMicroseconds(delayValue); // errechnete Wartezeit
    digitalWrite(targetPin, LOW); //faehrt die Membran des Piezo-Elementes wieder ein
    delayMicroseconds(delayValue); // errechnete Wartezeit
  }
}


//_____________________________________________________________________________________________________________________________________________________________U_N_T_E_R_F_U_N_K_T_I_O_N_E_N___Z_U_R___R_E_N_N_A_N_I_M_A_T_I_O_N

void movement() { //Automatisierung beider Farbe und der Items

  P1_ACC();        //Beschleunigungs- & Bremssequenz fuer Blau
  P2_ACC();         //Beschleunigungs- & Bremssequenz fuer Rot
  maxSpeedP1();   //Sequenz fuer die farbliche Veraenderung vom blauen Spieler bei Höchstgeschwindigkeit
  maxSpeedP2();    //Sequenz fuer die farbliche Veraenderung vom roten Spieler bei Höchstgeschwindigkeit
  movementP1(); //Spielsequenz fuer Blau
  movementP2(); //Spielsequenz fuer Rot
  interruption();  //Alternative Route
  LEDON();  // Ansteuern der LED während der Unterbrechung
  boost();
  //gravity();

  //delay(1);  //Verzögerung von 1ms

  //pixels.show();
}



//_____________________________________________________________________________________________________________________________________________________________B_E_S_C_H_L_E_U_N_I_G_U_N_G_/_B_R_E_M_S_U_N_G_-_S_P_I_E_L_E_R_1

float P1_speed = 0.0;                                //Grundgeschwindigkeit von Spieler 1
float P1_Add = 0.02;                                 //Wert fuer die Zu- bzuw. Abnahme der Geschwindigkeit

float P1_BrakeValue = 0;                             //Variable fuer die aktuelle Bremskraft des ersten Spielers.
float prevP1BrakeValue = 0;                         //Variable fuer die Bremskraft bei keinem Tempo fuer den ersten Spieler.
float P1_difficulty = 0;                             //Variable zum Ausrechnen der Bremskraft beim ersten Spieler.


void P1_ACC() {                                  //Beschleunigungs- & Bremssequenz fuer Blau
  if (switchStateP1 != prevSwitchStateP1 || devSettings == 1) {
    if (switchStateP1 == HIGH && P1_speed < 0.7) {
      P1_speed = P1_speed + P1_Add; //dann soll das Tempo von Blau um "P1_Add" zunehmen
      prevP1BrakeValue = 0; //und "prevP1BrakeValue" soll auf 0 gesetzt werden
    }
  }
  if (prevP1BrakeValue < 15) {                                    //15 x 1ms = 15ms -> nach 15ms ohne Beruehrung soll das Tempo langsam (Es sind nicht ganz 15ms)
    prevP1BrakeValue++;                                          //wieder sinken. Dazu wird hier so lange prevP1BrakeValue hochgezaehlt, bis diese Variable den Wert 15 erreicht.
  }

  //Verhältnis von der aktuellen Geschwindigkeit zur maximalen Geschwindigkeit für Bremsvariable

  P1_difficulty = P1_speed / maxSpeed;
  //Je nachdem, wie schnell man ist, desto schwieriger soll es sein, das Tempo zu halten. Da prevP1BrakeValue = 15 deutlich zu leicht zum Tempo halten ist, soll
  //mit steigender Geschwindigkeit diese Variable bis zu 6 runtergehen. Hier entsteht eine Differenz von 9.

  P1_BrakeValue = 15 - (difficultness * P1_difficulty); //Die neuere "P1_BrakeValue" soll sich dann aus "prevP1BrakeValue" (15) und der Prozentzahl von "P1_difficulty" ergeben.

  //Trigger der Bremsvariable

  if (prevP1BrakeValue >= P1_BrakeValue && prevP1BrakeValue < P1_BrakeValue + 1 && P1_speed > 0 || prevP1BrakeValue == 15 && P1_speed > 0) {               //Wenn prevP1BrakeValue gleich 15 erreicht und das Tempo von noch blau größer als 0 ist, dann
    prevP1BrakeValue = 0;                                       //soll prevP1BrakeValue auf 0 gesetzt werden und
    P1_speed = P1_speed - P1_Add;
  }
  else if (prevP1BrakeValue == 15) {                             //Trifft die vorherige Bedingung nicht zu und NUR prevP1BrakeValue ist gleich 5,
    prevP1BrakeValue = 0;                                       //dann soll prevP1BrakeValue gleich 0 sein.
  }
  if (P1_speed < 0) {
    P1_speed = 0;
  }

}


//_____________________________________________________________________________________________________________________________________________________________B_E_S_C_H_L_E_U_N_I_G_U_N_G_/_B_R_E_M_S_U_N_G_-_S_P_I_E_L_E_R_2

float P2_speed = 0.0;                                 //Grundgeschwindigkeit fuer Spieler 2
float P2_Add = 0.02;                                  //Wert fuer die Zu- bzw. Abnahme der Geschwindigkeit

float P2_BrakeValue = 0;                              //Variable fuer die aktuelle Bremskraft des zweiten Spielers.
float prevP2BrakeValue = 0;                          //Variable fuer die Bremskraft bei keinem Tempo fuer den zweiten Spieler.
float P2_difficulty = 0;                              //Variable zum Ausrechnen der Bremskraft beim zweiten Spieler.

void P2_ACC() {                                  //Beschleunigungs- & Bremssequenz fuer Rot
  if (switchStateP2 != prevSwitchStateP2 || devSettings == 1) {
    if (switchStateP2 == HIGH && P2_speed < 0.7) {
      P2_speed = P2_speed + P2_Add; //dann soll das Tempo von Rot um P2_Add zunehmen
      prevP2BrakeValue = 0; //und prevP2BrakeValue soll auf 0 gesetzt werden.
    }
  }

  if (prevP2BrakeValue < 15) {                                   //15 x1ms = 15ms -> nach 30ms ohne Beruehrung soll das Tempo langsam
    prevP2BrakeValue++;                                         //wieder sinken. Dazu wird hier prevP2BrakeValue so lange hochgezaehlt, bis prevP2BrakeValue den Wert 15 erreicht.
  }

  //Verhältnis von der aktuellen Geschwindigkeit zur maximalen Geschwindigkeit für Bremsvariable

  P2_difficulty = P2_speed / maxSpeed;
  //Je nachdem, wie schnell man ist, desto schwieriger soll es sein, das Tempo zu halten. Da prevP1BrakeValue = 15 deutlich zu leicht zum Tempo halten ist, soll
  //mit steigender Geschwindigkeit diese Variable bis zu 10 runtergehen. Hier entsteht eine Differenz von 5.

  P2_BrakeValue = 15 - (difficultness * P2_difficulty);

  //Serial.println(P2_BrakeValue);
  //Serial.print(" - ");

  //Trigger der Bremsvariable

  if (prevP2BrakeValue >= P2_BrakeValue && prevP2BrakeValue < P2_BrakeValue + 1 && P2_speed > 0 || prevP2BrakeValue == 15 && P2_speed > 0) {                //Wenn prevP2BrakeValue gleich 15 erreicht und das Tempo von Rot noch größer als 0 ist, dann
    prevP2BrakeValue = 0;                                       //soll prevP2BrakeValue auf 0 gesetzt werden und
    P2_speed = P2_speed - P2_Add;                //das Tempo von Rot soll um P2_Add abnehmen.
  }
  else if (prevP2BrakeValue == 15) {                             //Trifft die vorherige Bedingung nicht zu und NUR prevP2BrakeValue ist gleich 15,
    prevP2BrakeValue = 0;                                       //dann soll prevP2BrakeValue gleich 0 sein.
  }
  if (P2_speed < 0) {
    P2_speed = 0;
  }

}


//_____________________________________________________________________________________________________________________________________________________________O_P_T_I_S_C_H_E___A_E_N_D_E_R_U_N_G___V_O_N___W_U_R_M___B_E_I___M_A_X_T_E_M_P_O_-_S_1

int saturationColor[] = {255, 0, 200, 100};           //Saettigung - Array fuer unterschiedlich kraeftige Farben

int player1Color[] = {240, 180, 120, 120};            //Hue - Farbdarstellung des ersten Spielers
int colorNumPlayer1 = 0;                              //Variable zur Auswahl der Farbe im Array von "player1Color[]".

int saturationPl1 = 0;                                //Variable zur Auswahl der Farbkraft im Array von "saturationColor[]".
int saturationHead1 = 0;                              //Variable zur Auswahl der Farbkraft im Array von "saturationColor[]".  - Kopf des Wurms


void maxSpeedP1() { //Funktion zur Farbveränderung des Wurms bei verschiedenen Tempo
  if (P1_speed >= 0.6) { //Ist das Tempo des Wurms groesser als 0,6, dann
    saturationHead1 = 1;
  }

  //else if (P1_speed > 0 && P1_speed < 0.2) { //Befindet sich das Tempo zwischen 0 und 0,2, dann
  //saturationHead1 = 0;
  //}
}


//_____________________________________________________________________________________________________________________________________________________________D_A_R_S_T_E_L_L_U_N_G_-_W_U_R_M___S_P_I_E_L_E_R_1


int P1_lap = 0;                                      //Abgeschlossene Rundenanzahl von Spieler 1
int P1prevFirstLED_int = 0;                             //Variable, welche als Trigger für die aktuelle Rundenzahl dient.

int playerBright[] = {170, 100, 50, 30};              //Helligkeit - Array fuer unterschiedlich helle Pixel beim Wurm

int blinkReps = 0;            //Variable auf die die Anzahl der Blinkwiderholungen gespeichert werden.


void movementP1() { //Spielsequenz fuer den blauen Spieler

  pixels.setPixelColor(P1firstLED, pixels.ColorHSV(player1Color[colorNumPlayer1] * 182, saturationColor[saturationHead1], playerBright[0])); //Laesst die erste LED des blauen Spielers mit 100%-iger Helligkeit aufleuchten
  pixels.setPixelColor(P1secondLED, pixels.ColorHSV(player1Color[colorNumPlayer1] * 182, saturationColor[saturationPl1], playerBright[1])); //Laesst die zweite LED des blauen Spielers mit %-iger Helligkeit aufleuchten
  pixels.setPixelColor(P1thirdLED, pixels.ColorHSV(player1Color[colorNumPlayer1] * 182, saturationColor[saturationPl1], playerBright[2])); //Laesst die dritte LED des blauen Spielers mit 30%-iger Helligkeit aufleuchten
  pixels.setPixelColor(P1fourthLED, pixels.ColorHSV(player1Color[colorNumPlayer1] * 182, saturationColor[saturationPl1], playerBright[3])); //Laesst die vierte LED des blauen Spielers mit 10%-iger Helligkeit aufleuchten
  pixels.setPixelColor(P1fifthLED, pixels.ColorHSV(0 * 182, 0, 0));
  P1firstLED += P1_speed; //Erhoehung des Wertes von P1firstLED um den Wert P1_speed
  P1secondLED = P1firstLED - 1; // LED 2-4 des Wurm, sind abhängig von LED1
  P1thirdLED = P1firstLED - 2;
  P1fourthLED = P1firstLED - 3;
  P1fifthLED = P1firstLED - 4;


  if (P1firstLED_int == (trackend + 1)) { //Befindet sich der erste Pixel am Ende der Strecke, wird er an den Anfang zurück gesetzt
    P1firstLED = 0;
  }
  if (P1_lap > 0) {             //Der schweif des Wurms(Pixel 2-5 werden außer am Anfang des Spiels hinterher gezogen.
    if (P1firstLED_int == 1) {
      P1secondLED = trackend;
      P1thirdLED = trackend - 1;
      P1fourthLED = trackend - 2;
      P1fifthLED = trackend - 3;
    }
    if (P1firstLED_int == 2) {
      P1thirdLED = trackend;
      P1fourthLED = trackend - 1;
      P1fifthLED = trackend - 2;
    }
    if (P1firstLED_int == 3) {
      P1fourthLED = trackend;
      P1fifthLED = trackend - 1;
    }
    if (P1firstLED_int == 4) {
      P1fifthLED = trackend;
    }
  }


  //Rundenzaehler
  if (P1firstLED_int != P1prevFirstLED_int) { //Wenn P1firstLED ungleich dem vorigen Wert ist
    if (P1firstLED_int == trackend) { //Diese if-Schleife wird ausgefuehrt, wenn der blaue Spieler seine erste Runde geschafft hat
      P1_lap++; //Anzahl der beendeten Runden des blauen Spielers wird um den Wert 1 erhoeht
    }
  }

  //Alle Runden abgeschlossen
  if (P1_lap >= laps) { //If-Schleife wird ausgefuehrt, wenn die gesamte Rundenanzahl des blauen Spielers groesser oder gleich der maximalen Rundenanzahl ist
    if (blinkReps < 5) { //und die Variable "blinkReps" kleiner als 3 ist 
      blinkReps++; //dann soll die Variable hochzählen.
      pixels.fill(pixels.ColorHSV(player1Color[colorNumPlayer1] * 182, saturationColor[saturationPl1], 50)); //Gesamte Rennbahn soll blau aufleuchten
      pixels.show(); //Zeigt die Veraenderungen der vorigen Zeile an
      delay(300); //Verzoegerung von 300/1000 ms, bevor der naechste Schritt ausgefuehrt wird
      pixels.clear();  //Laesst alle LEDs ausschalten
      pixels.show();  //Zeigt die Veraenderungen der vorigen Zeile an
      delay(300); //Verzoegerung von 300/1000 ms, bevor der naechste Schritt ausgefuehrt wird
      lcd.clear();
      lcd.setCursor(6, 1);
      lcd.print("Spieler 1");
      lcd.setCursor(4, 2);
      lcd.print("hat Gewonnen");

    }
    if (blinkReps >= 5) {   //Blinkt der LED-Stripe zum 5ten mal Blau, wird reset() ausgeführt
      reset();
      Endseq();
    }

  }

  P1prevFirstLED_int = P1firstLED_int; //Setzt den aktuellen Wert von P1firstLED (Rundenzaehler des blauen Spielers) in prevFirstP1LED fuer den eventuellen naechsten Durchgang
}


//_____________________________________________________________________________________________________________________________________________________________O_P_T_I_S_C_H_E___A_E_N_D_E_R_U_N_G___V_O_N___W_U_R_M___B_E_I___M_A_X_T_E_M_P_O_-_S_2


int player2Color[] = {1, 1, 40, 300};                 //Hue - Farbdarstellung des zweiten Spielers
int colorNumPlayer2 = 0;                              //Variable zur Auswahl der Farbe im Array von "player2Color[]".

int saturationPl2 = 0;                                //Variable zur Auswahl der Farbkraft im Array von "saturationColor[]".
int saturationHead2 = 0;                              //Variable zur Auswahl der Farbkraft im Array von "saturationColor[]".  - Kopf des Wurms

void maxSpeedP2() { //Funktion zur Farbveraenderung des roten Wurms bei erhoehtem Tempo
  if (P2_speed >= 0.6) { //Ist der Wurm schneller als 0,6,
    saturationHead2 = 1;
  }


  //else if (P2_speed > 0 && P2_speed < 0.2) { //Liegt das Tempo zwischen 0 und 0,2,
  // saturationNum = 0;
  // }
}

//_____________________________________________________________________________________________________________________________________________________________D_A_R_S_T_E_L_L_U_N_G_-_W_U_R_M___S_P_I_E_L_E_R_2

int P2_lap = 0;                                       //Abgeschlossene Rundenanzahl von Spieler 2
int P2prevFirstLED_int = 0;                              //Variable, welche als Trigger für die aktuelle Rundenzahl dient.

void movementP2() { //Spielsequenz fuer den roten Spieler

  pixels.setPixelColor(P2firstLED, pixels.ColorHSV(player2Color[colorNumPlayer2] * 182, saturationColor[saturationHead2], playerBright[0]));
  pixels.setPixelColor(P2secondLED, pixels.ColorHSV(player2Color[colorNumPlayer2] * 182, saturationColor[saturationPl2], playerBright[1]));
  pixels.setPixelColor(P2thirdLED, pixels.ColorHSV(player2Color[colorNumPlayer2] * 182, saturationColor[saturationPl2], playerBright[2]));
  pixels.setPixelColor(P2fourthLED, pixels.ColorHSV(player2Color[colorNumPlayer2] * 182, saturationColor[saturationPl2], playerBright[3]));
  pixels.setPixelColor(P2fifthLED, pixels.ColorHSV(0 * 182, 0, 0));
  P2firstLED += P2_speed; //Erhoehung des Wertes von P2firstLED um den Wert 1
  P2secondLED = P2firstLED - 1;   // LED 2-4 des Wurm, sind abhängig von LED1
  P2thirdLED = P2firstLED - 2;
  P2fourthLED = P2firstLED - 3;
  P2fifthLED = P2firstLED - 4;


  if (P2firstLED_int == (trackend + 1)) {     //Befindet sich der erste Pixel am Ende der Strecke, wird er an den Anfang zurück gesetzt
    P2firstLED = 0;
  }
  if (P2_lap > 0) {                      //Der schweif des Wurms(Pixel 2-5 werden außer am Anfang des Spiels hinterher gezogen.
    if (P2firstLED_int == 1) {
      P2secondLED = trackend;
      P2thirdLED = trackend - 1;
      P2fourthLED = trackend - 2;
      P2fifthLED = trackend - 3;
    }
    if (P2firstLED_int == 2) {
      P2thirdLED = trackend;
      P2fourthLED = trackend - 1;
      P2fifthLED = trackend - 2;
    }
    if (P2firstLED_int == 3) {
      P2fourthLED = trackend;
      P2fifthLED = trackend - 1;
    }
    if (P2firstLED_int == 4) {
      P2fifthLED = trackend;
    }
  }


  //Rundenzaehler aktualisieren
  if (P2firstLED_int != P2prevFirstLED_int) { //Wenn P2firstLED ungleich dem vorigen Wert ist
    if (P2firstLED_int == trackend) { //Diese if-Schleife wird ausgefuehrt, wenn der rote Spieler seine erste Runde geschafft hat
      P2_lap++; //Anzahl der beendeten Runden des roten Spielers wird um den Wert 1 erhoeht
    }
  }


  //Alle Runden abgeschlossen
  if (P2_lap >= laps) { //If-Schleife wird ausgefuehrt, wenn die gesamte Rundenanzahl des roten Spielers groesser oder gleich der maximalen Rundenanzahl ist
    if (blinkReps < 5) { //For-Schleife wird zwei mal ausgefuehrt
      blinkReps++;
      pixels.fill(pixels.ColorHSV(player2Color[colorNumPlayer2] * 182, saturationColor[saturationPl2], 50)); //Gesamte Rennbahn soll rot aufleuchten
      pixels.show(); //Zeigt die Veraenderungen der vorigen Zeile an
      delay(300); //Verzoegerung von 300/1000 ms, bevor der naechste Schritt ausgefuehrt wird
      pixels.fill(pixels.Color(0, 0, 0)); //Laesst alle LEDs ausschalten
      pixels.show(); //Zeigt die Veraenderungen der vorigen Zeile an
      delay(300); //Verzoegerung von 300/1000 ms, bevor der naechste Schritt ausgefuehrt wird
      lcd.clear();
      lcd.setCursor(6, 1);
      lcd.print("Spieler 2");
      lcd.setCursor(4, 2);
      lcd.print("hat Gewonnen");

    }
    if (blinkReps >= 5) {    //Blinkt der LED-Stripe zum 5ten mal Blau, wird reset() ausgeführt
      reset();
      Endseq();
    }


  }

  P2prevFirstLED_int = P2firstLED_int; //Setzt den aktuellen Wert von P2firstLED (Rundenzaehler des roten Spielers) in prevFirstP2LED fuer den eventuellen naechsten Durchgang
}


//_____________________________________________________________________________________________________________________________________________________________A_L_T_E_R_N_A_T_I_V_S_T_R_E_C_K_E

//Varibalen für interruption
//-----------------------------------

//Variablen zur Eingrenzung von der alternativen Streckez
int halflap = laps / 2;                                   //Variable, die als ungefährer Zeitpunkt zum Ausloesen der Unterbrechung dient.
int detourbeginn = trackend + 1;                          //Beginn der Unterbrechung(wird bei "interrupt()" automatisch definiert)
int detourend = NUMPIXELS - 12;                           //Ende der Unterbrechung(wird bei "interrupt()" automatisch definiert)

int interrupt = 0;                                        //0/1 - Variable, welche dem Wurm sagt, ob er die alternative Strecke nutzen soll oder nicht.
int interruptGo = 0;                                      //Falls die Unterbrechung durch den Wurm getriggeP2_ werden sollte, sorgt die Variable dafuer, dass sich kein weiterer Wurm gerade in der Unterbrechung befindet, wenn diese ausgeloest wird.

int fillColor1 = (interruptionend - interruptionstart) / 2 + interruptionstart;
int fillColor2 = fillColor1;


void interruption() {

  if ( (P1_lap == halflap)) {
    if ( P1firstLED_int == randomNum) {   //Wenn die hälfte der Runden vollendet sind und sich der blaue Pixel auf einem zufällug definierten Pixel befindet,
      interruptGo = 1;                  //wird interruptGo auf 1 gesetzt
    }
  }
  else if ( (P2_lap == halflap)) {     //Wenn die hälfte der Runden vollendet sind und sich der rote Pixel auf einem zufällug definierten Pixel befindet,
    if ( P2firstLED_int == randomNum) {     //wird interruptGo auf 1 gesetzt
      interruptGo = 1;
    }
  }

  if ( interruptGo == 1) {
    if ( (P1firstLED_int) < (interruptionstart - 4) || (P1firstLED_int ) > (interruptionend + 4)) { //Befindet sich werder der rote, noch der blaue Wurm in dem Unterbrechungsbereich,
      if ( (P2firstLED_int) < (interruptionstart - 4) || (P2firstLED_int ) > (interruptionend + 4)) { //wird interrupt auf 1 gesetzt
        interrupt = 1;
      }
    }
  }

  if (interrupt == 1) {                       //Dieser Teil sorgt dafür, dass der blaue Wurm beim erreichen der Unterbrechung, flüssig in die Unterbrechung über geht
    if ( P1firstLED_int == interruptionstart) {
      P1firstLED = detourbeginn;
    }
    if (P1firstLED_int == detourbeginn + 1) {
      P1secondLED = interruptionstart - 2 ;
      P1thirdLED = interruptionstart - 3;
      P1fourthLED = interruptionstart - 4;
      P1fifthLED = interruptionstart - 5;
    }
    if (P1firstLED_int == detourbeginn + 2) {
      P1thirdLED = interruptionstart - 2;
      P1fourthLED = interruptionstart - 3;
      P1fifthLED = interruptionstart - 4;
    }
    if (P1firstLED_int == detourbeginn + 3) {
      P1fourthLED = interruptionstart - 2;
      P1fifthLED = interruptionstart - 3;
    }
    if (P1firstLED_int == detourbeginn + 4) {
      P1fifthLED = interruptionstart - 2;
    }


    if ( P1firstLED_int == detourend) {         //Dieser Teil sorgt dafür, dass der blaue Wurm beim erreichen des Endes der Unterbrechung,
      P1firstLED = interruptionend;         //flüssig in die auf die Normalstrecke zurück geht
    }
    if ( P1firstLED_int == interruptionend + 1) {
      P1secondLED = detourend - 1;                                 //vorher -2
      P1thirdLED = detourend - 2;                                  //vorher -3
      P1fourthLED = detourend - 3;                                 //vorher -4
      P1fifthLED = detourend - 4;                                  //vorher -5
    }
    if ( P1firstLED_int == interruptionend + 2) {
      P1thirdLED = detourend - 1;                                  //vorher -2
      P1fourthLED = detourend - 2;                                 //vorher -3
      P1fifthLED = detourend - 3;                                  //vorher -4
    }
    if ( P1firstLED_int == interruptionend + 3) {
      P1fourthLED = detourend - 1;                                  //vorher -2
      P1fifthLED = detourend - 2;                                   //vorher -3
    }
    if ( P1firstLED_int == interruptionend + 4) {
      P1fifthLED = detourend - 1;                                   //vorher -2 -> Fehler mit letztem Pixel behoben
    }


    if ( P2firstLED_int == interruptionstart) {   //Dieser Teil sorgt dafür, dass der rote Wurm beim erreichen der Unterbrechung, flüssig in die Unterbrechung über geht
      P2firstLED = detourbeginn;
    }
    if (P2firstLED_int == detourbeginn + 1) {
      P2secondLED = interruptionstart - 2 ;
      P2thirdLED = interruptionstart - 3;
      P2fourthLED = interruptionstart - 4;
      P2fifthLED = interruptionstart - 5;
    }
    if (P2firstLED_int == detourbeginn + 2) {
      P2thirdLED = interruptionstart - 2;
      P2fourthLED = interruptionstart - 3;
      P2fifthLED = interruptionstart - 4;
    }
    if (P2firstLED_int == detourbeginn + 3) {
      P2fourthLED = interruptionstart - 2;
      P2fifthLED = interruptionstart - 3;
    }
    if (P2firstLED_int == detourbeginn + 4) {
      P2fifthLED = interruptionstart - 2;
    }


    if ( P2firstLED_int == detourend) {           //Dieser Teil sorgt dafür, dass der blaue Wurm beim erreichen des Endes der Unterbrechung,
      P2firstLED = interruptionend;               //flüssig auf die Normalstrecke zurück geht
    }
    if ( P2firstLED_int == interruptionend + 1) { // Dies Funktioniert dann darüber, indem die anderen LED´s 
      P2secondLED = detourend - 1;                // an die erste LED angepasst werden.
      P2thirdLED = detourend - 2;
      P2fourthLED = detourend - 3;
      P2fifthLED = detourend - 4;
    }
    if ( P2firstLED_int == interruptionend + 2) {
      P2thirdLED = detourend - 1;
      P2fourthLED = detourend - 2;
      P2fifthLED = detourend - 3;
    }
    if ( P2firstLED_int == interruptionend + 3) {
      P2fourthLED = detourend - 1;
      P2fifthLED = detourend - 2;
    }
    if ( P2firstLED_int == interruptionend + 4) {
      P2fifthLED = detourend - 1;
    }

    //---------------Unterbrechung wird rot markiert------------------------

    pixels.setPixelColor(fillColor1, pixels.ColorHSV(1 * 257, 255, 30));
    pixels.setPixelColor(fillColor2, pixels.ColorHSV(1 * 257, 255, 30));


    if (fillColor1 < interruptionend - 1) {
      fillColor1++;
    }
    if (fillColor2 > interruptionstart - 1) {
      fillColor2--;
    }
  }
}

//_____________________________________________________________________________________________________________________________________________________________B_O_O_S_T_F_U_N_K_T_I_O_N

byte Boost_P1_ = 0;                             //Boost-Verfügbarkeit - 0 = kein Boost & 1 = Boost verfügbar
int Boost_P1_Time = 0;                          //Dauer des Boosts von Spieler Blau
float Boost_P1_Fuel = 0;                        //Fülle des Boost von Spieler Blau
int Boost_P1_Load = 0;                          //Variable zum Aufladen des Boost von Spieler Blau

byte Boost_P2_ = 0;
int Boost_P2_Time = 0;
float Boost_P2_Fuel = 0;
int Boost_P2_Load = 0;

int chargingTime = 200;                       //Aufladezeit als Grenzwert für beide Spieler
float boostTime = 10;                         //Boostdauer als Grenzwert für beide Spieler

float pulseSpeed = 0.25;
float pulsePixelP11 = 0;
float pulsePixelP12 = 0;
float pulsePixelP13 = 0;

float pulsePixelP21 = 0;
float pulsePixelP22 = 0;
float pulsePixelP23 = 0;

//Variablen zum Anzeigen des Boosts
int boostBright = 200;                                //Helligkeit der einzelnen Pixel
int boostBrightP1 = 200;                              //Der Boost macht den Spieler bei voller Ladung auf sich aufmerksam.
int boostBrightP2 = 200;                              //Da die Spielerfarbe varriiert, muss die Helligkeit fuer eine bessere Darstellung angepasst werden.

void boost() {
  //Boost-Spieler Blau
  //-------------------------------------------------------------------------------------------------
  if (Boost_P1_ == 1 && Boost_P1_Time < boostTime) {  //Boost aktiviert für Spieler 1
    P1_speed = 0.8;
    Boost_P1_Time ++;
    pulsePixelP11 = 0;
    pulsePixelP12 = 0;
    pulsePixelP13 = 0;

    if (Boost_P1_Fuel > 0) {
      Boost_P1_Fuel -= 6 / boostTime;
      pixels.setPixelColor(Boost_P1_Fuel + detourend, pixels.ColorHSV(0 * 182, 0, 0));
    }
  }
  else if (Boost_P1_Time >= boostTime) {                                      //Wenn Boostdauer um, Werte zurücksetzen
    Boost_P1_ = 0;
    Boost_P1_Time = 0;
  }

  else if (Boost_P1_Load < chargingTime) {                                  //Aufladezeit pro Pixel
    Boost_P1_Load++;
  }
  else if (Boost_P1_Fuel < 6) {                                             //6 Pixel zur Darstellung
    Boost_P1_Load = 0;
    pixels.setPixelColor(Boost_P1_Fuel + detourend, pixels.ColorHSV(player1Color[colorNumPlayer1] * 182, saturationColor[saturationPl1], boostBright));
    Boost_P1_Fuel++;
  }
  //Serial.println(pulsePixelP11 + detourend);
  if (Boost_P1_Fuel >= 5.90) {                                                                 //Pulsartige Darstellung bei vollem Boost              - eigentlich "== 6" anstatt ">= 5.90"
    //Kommazahlen scheinen Probleme mit Prüfen auf Gleichheit zu haben

    if (pulsePixelP11 < Boost_P1_Fuel - 0.1) {                                                                                             //Normalerweise "Boost_P1_Fuel" ohne "-0.1", jedoch rundet der LED-Streifen bislang ohne Erkenntnis
      pixels.setPixelColor(pulsePixelP11 + detourend, pixels.ColorHSV(player1Color[colorNumPlayer1] * 182, 100, boostBrightP1));           //auf den 7. Pixel auf.
      pulsePixelP11 += pulseSpeed;

    }
    if (pulsePixelP12 < Boost_P1_Fuel - 0.1 && pulsePixelP11 > 1) {
      pixels.setPixelColor(pulsePixelP12 + detourend, pixels.ColorHSV(player1Color[colorNumPlayer1] * 182, 100, boostBrightP1));
      pulsePixelP12 += pulseSpeed;

    }
    if (pulsePixelP13 < Boost_P1_Fuel - 0.1 && pulsePixelP12 > 1) {
      pixels.setPixelColor(pulsePixelP13 + detourend, pixels.ColorHSV(player1Color[colorNumPlayer1] * 182, saturationColor[saturationPl1], boostBright));
      pulsePixelP13 += pulseSpeed;
    }
    else if (pulsePixelP13 >= Boost_P1_Fuel - 0.1) {
      pulsePixelP11 = 0;
      pulsePixelP12 = 0;
      pulsePixelP13 = 0;
    }

  }

  if (Boost_P1_Fuel >= 6 && switchStateItemP1 == HIGH) {                 //Boost verfügbar machen
    Boost_P1_ = 1;
  }

  //Boost-Spieler Rot
  //-------------------------------------------------------------------------------------------------
  if (Boost_P2_ == 1 && Boost_P2_Time < boostTime) {  //Boost aktiviert für Spieler 2
    P2_speed = 0.8;
    Boost_P2_Time ++;
    pulsePixelP21 = 0;
    pulsePixelP22 = 0;
    pulsePixelP23 = 0;

    if (Boost_P2_Fuel > 0) {
      Boost_P2_Fuel -= 6 / boostTime;
      pixels.setPixelColor(Boost_P2_Fuel + detourend + 6, pixels.ColorHSV(0 * 182, 0, 0));
    }
  }
  else if (Boost_P2_Time >= boostTime) {                                      //Wenn Boostdauer um, Werte zurücksetzen
    Boost_P2_ = 0;
    Boost_P2_Time = 0;
  }

  else if (Boost_P2_Load < chargingTime) {       //Aufladezeit pro Pixel
    Boost_P2_Load++;
  }
  else if (Boost_P2_Fuel < 6) {             //6 Pixel zur Darstellung
    Boost_P2_Load = 0;
    pixels.setPixelColor(Boost_P2_Fuel + detourend + 6, pixels.ColorHSV(player2Color[colorNumPlayer2] * 182, saturationColor[saturationPl2], boostBright));
    Boost_P2_Fuel++;
  }

  if (Boost_P2_Fuel >= 5.90) {                                                                 //Pulsartige Darstellung bei vollem Boost         - siehe Unterfunktion für Blau

    if (pulsePixelP21 < Boost_P2_Fuel - 0.1) {
      pixels.setPixelColor(pulsePixelP21 + detourend + 6, pixels.ColorHSV(player2Color[colorNumPlayer2] * 182, 100, boostBrightP2));
      pulsePixelP21 += pulseSpeed;
    }
    if (pulsePixelP22 < Boost_P2_Fuel - 0.1 && pulsePixelP21 > 1) {
      pixels.setPixelColor(pulsePixelP22 + detourend + 6, pixels.ColorHSV(player2Color[colorNumPlayer2] * 182, 100, boostBrightP2));
      pulsePixelP22 += pulseSpeed;

    }
    if (pulsePixelP23 < Boost_P2_Fuel - 0.1 && pulsePixelP22 > 1) {
      pixels.setPixelColor(pulsePixelP23 + detourend + 6, pixels.ColorHSV(player2Color[colorNumPlayer2] * 182, saturationColor[saturationPl2], boostBright));
      pulsePixelP23 += pulseSpeed;
    }
    else if (pulsePixelP23 >= Boost_P2_Fuel - 0.1) {
      pulsePixelP21 = 0;
      pulsePixelP22 = 0;
      pulsePixelP23 = 0;
    }

  }

  if (Boost_P2_Fuel >= 6 && switchStateItemP2 == HIGH) {                 //Boost verfügbar machen
    Boost_P2_ = 1;
  }
}


//_____________________________________________________________________________________________________________________________________________________________G_R_A_V_I_T_A_T_I_O_N


void gravity() {
  /*
  int movingUpwards2_Begin = 192;       //Ort, an dem die 2. Steigung beginnt.
  int movingUpwards2_End = 208;         //Ort, an dem die 2. Steigung endet.
  int movingDownwards2_Begin = 210;     //Ort, an dem das 2. Gefälle beginnt.
  int movingDownwards2_End = 221;       //Ort, an dem das 2. Gefälle endet.

  //pixels.setPixelColor(movingUpwards1_Begin, pixels.Color(139, 0, 100));
  //pixels.setPixelColor(movingUpwards1_End, pixels.Color(139, 0, 100));

//pixels.setPixelColor(movingDownwards1_Begin, pixels.Color(139, 0, 100));
//pixels.setPixelColor(movingDownwards1_End, pixels.Color(139, 0, 100));
  //gravityauf Funktion Blau
  if (P1firstLED_int >= movingUpwards1_Begin && P1firstLED_int <= movingUpwards1_End && P1_speed > 0.04) { //0.04 als Unterstützung beim gravityauf-Fahren
    P1_speed = P1_speed - P1_Add / 4;
  }


  //gravityauf Funktion Rot
  if (P2firstLED_int >= movingUpwards1_Begin && P2firstLED_int <= movingUpwards1_End && P2_speed > 0.04) {
    P2_speed = P2_speed - P2_Add / 4;
  }



  //gravityab Blau
  if (P1firstLED_int >= movingDownwards1_Begin && P1firstLED_int <= movingDownwards1_End && P1_speed < 0.74) {
    P1_speed = P1_speed + P1_Add / 2;
  }

  //gravityab Rot
  if (P2firstLED_int >= movingDownwards1_Begin && P2firstLED_int <= movingDownwards1_End && P1_speed < 0.74) {
    P2_speed = P2_speed + P2_Add / 2;
  }
  //---------------------------------------------------------------------------------------------------------------------
  /*
    //gravityauf2 Funktion Blau
    if (P1firstLED_int >= movingUpwards2_Begin && P1firstLED_int <= movingUpwards2_End && P1_speed > 0.04) { //0.04 als Unterstützung beim gravityauf-Fahren
      P1_speed = P1_speed - P1_Add / 4;
    }


    //gravityauf2 Funktion Rot
    if (P2firstLED_int >= movingUpwards2_Begin && P2firstLED_int <= movingUpwards2_End && P2_speed > 0.04) {
      P2_speed = P2_speed - P2_Add / 4;
    }



    //gravityab2 Blau
    if (P1firstLED_int >= movingDownwards2_Begin && P1firstLED_int <= movingDownwards2_End && P1_speed < 0.74) {
      P1_speed = P1_speed + P1_Add / 2;
    }

    //gravityab2 Rot
    if (P2firstLED_int >= movingDownwards2_Begin && P2firstLED_int <= movingDownwards2_End && P1_speed < 0.74) {
      P2_speed = P2_speed + P2_Add / 2;
    }


    //---------------------------------------------------------------------------------------------------------------------

    //gravityauf3 Funktion Blau
    if (P1firstLED_int >= movingUpwards3_Begin && P1firstLED_int <= movingUpwards3_End && P1_speed > 0.04) { //0.04 als Unterstützung beim gravityauf-Fahren
      P1_speed = P1_speed - P1_Add / 4;
    }


    //gravityauf3 Funktion Rot
    if (P2firstLED_int >= movingUpwards3_Begin && P2firstLED_int <= movingUpwards3_End && P2_speed > 0.04) {
      P2_speed = P2_speed - P2_Add / 4;
    }



    //gravityab3 Blau
    if (P1firstLED_int >= movingDownwards3_Begin && P1firstLED_int <= movingDownwards3_End && P1_speed < 0.74) {
      P1_speed = P1_speed + P1_Add / 2;
    }

    //gravityab3 Rot
    if (P2firstLED_int >= movingDownwards3_Begin && P2firstLED_int <= movingDownwards3_End && P1_speed < 0.74) {
      P2_speed = P2_speed + P2_Add / 2;
    }*/
}

//_____________________________________________________________________________________________________________________________________________________________M_E_N_U_E_-_H_I_N_T_E_R_G_R_U_N_D_A_N_I_M_A_T_I_O_N

int prevDemoPixel;
float demoPixel1;
float demoPixel2 = -1;
float demoPixel3 = -2;
float demoPixel4 = -3;

float demoPixelSpeed = 0.5;

float demoPixelSpeed1 = demoPixelSpeed;           //Alle Pixel sür die Neue Hintergrundanimation werden hier auf die gleiche Geschwindigkeit gesetzt 
float demoPixelSpeed2 = demoPixelSpeed;
float demoPixelSpeed3 = demoPixelSpeed;
float demoPixelSpeed4 = demoPixelSpeed;

int brightness1 = 230;                            //Die vier verschiedenen Helligkeiten werrden deklariert
int brightness2 = 150;
int brightness3 = 70;
int brightness4[] = {0, 0, 40, 40, 40, 40};

int brightnessVal4 = 0;

int hue;

void menuBackground() {
  pixels.setPixelColor(demoPixel1, pixels.ColorHSV(hue * 182, 255, brightness1));                   //Farbeinstellungen für die Hintergrundanimation im Menü
  pixels.setPixelColor(demoPixel2, pixels.ColorHSV(hue * 182, 255, brightness2));                   //Farbeinstellungen für die Hintergrundanimation im Menü
  pixels.setPixelColor(demoPixel3, pixels.ColorHSV(hue * 182, 255, brightness3));                   //Farbeinstellungen für die Hintergrundanimation im Menü
  pixels.setPixelColor(demoPixel4, pixels.ColorHSV(hue * 182, 255, brightness4[brightnessVal4]));   //Farbeinstellungen für die Hintergrundanimation im Menü
  demoPixel1 += demoPixelSpeed1;                                                                    //Die Variablen demoPixel und demoPixelSpeed von 1 bis 4 werden gleichgesetzt
  demoPixel2 += demoPixelSpeed2;
  demoPixel3 += demoPixelSpeed3;
  demoPixel4 += demoPixelSpeed4;
  

  if (prevDemoPixel != demoPixel1) {          // Wenn der demo Pixel den gleichen Wert hat wie prevDemoPixel
    if (demoPixelSpeed4 > 0) {                // und wenn demoPixelSpeed4 eine größere geschwindigkeit als 0 ist
      brightnessVal4 = random(6);             // Dann soll brightnessVal4 eine zufallszahl von 0-6 zugewiesen werden
    }
    else {
      brightnessVal4 = 0;                     // Andernfalls soll diese Variable auf den Wert 0 gesetzt werden.
    }

    if (hue < 360) {                          // Wenn hue einen Wert kleiner als 360 hat, dann soll jedesmal wenn die schleife
      hue++;                                  // ausgeführt wird 
    }
    else {
      hue = 0;
    }
  }

  if (demoPixel1 > trackend || demoPixel1 < 1 && demoPixelSpeed1 == -demoPixelSpeed) {
    demoPixelSpeed1 = demoPixelSpeed1 * -1;
  }
  if (demoPixel2 > trackend || demoPixel2 < 1 && demoPixelSpeed2 == -demoPixelSpeed) {
    demoPixelSpeed2 = demoPixelSpeed2 * -1;
  }
  if (demoPixel3 > trackend || demoPixel3 < 1 && demoPixelSpeed3 == -demoPixelSpeed) {
    demoPixelSpeed3 = demoPixelSpeed3 * -1;
  }
  if (demoPixel4 > trackend || demoPixel4 < 1 && demoPixelSpeed4 == -demoPixelSpeed) {
    demoPixelSpeed4 = demoPixelSpeed4 * -1;
  }


  prevDemoPixel = demoPixel1;
}

//_____________________________________________________________________________________________________________________________________________________________F_A_R_B_W_E_C_H_S_E_L_M_O_D_U_S__V_O_N___W_U_R_M

//Variablen für "skinmode"

float skinP1 = NUMPIXELS - 12;                        //Platzierung des Spielers auf der Boostanzeige
float skinP2 = NUMPIXELS - 6;                         //Dient als optische Darstellung des Farbwechsels vor Beginn der Rennens.

int setSkinP1 = 1;
int setSkinP2 = 1;
int colorNumMax = 4;                                  //Anzahl der maximal einstellbaren Farben pro Spieler



void skinmode() {

  pixels.setPixelColor(skinP1, pixels.ColorHSV(player1Color[colorNumPlayer1] * 182, saturationColor[saturationPl1], boostBright));
  pixels.setPixelColor(skinP2, pixels.ColorHSV(player2Color[colorNumPlayer2] * 182, saturationColor[saturationPl2], boostBright));
  //pixels.show();

  skinP1 = skinP1 + 0.3;
  skinP2 = skinP2 + 0.3;

  if (switchStateItemP1 != prevSwitchStateItemP1) {
    if (switchStateItemP1 == HIGH) {
      if (setSkinP1 > 1) {
        colorNumPlayer1++;
        skinP1 = NUMPIXELS - 12;
        setSkinP1 = 1;
        if (colorNumPlayer1 == colorNumMax) {
          colorNumPlayer1 = 0;
        }
      }
    }
  }


  if (switchStateItemP2 != prevSwitchStateItemP2) {
    if (switchStateItemP2 == HIGH) {
      if (setSkinP2 > 1) {
        colorNumPlayer2++;
        skinP2 = NUMPIXELS - 6;;
        setSkinP2 = 1;
        if (colorNumPlayer2 == colorNumMax) {
          colorNumPlayer2 = 0;
        }
      }
    }
  }

  if (skinP1 > NUMPIXELS - 6) {
    skinP1 = NUMPIXELS - 12;
    setSkinP1 ++;
  }
  if (skinP2 > NUMPIXELS + 1) {
    skinP2 = NUMPIXELS - 6;
    setSkinP2 ++;
  }
}


//_____________________________________________________________________________________________________________________________________________________________V_E_R_S_C_H_I_E_D_E_N_E___F_A_R_B_E_N


void skins() {
  saturationPl1 = 0;
  saturationPl2 = 0;
  saturationHead1 = 0;
  saturationHead2 = 0;
  boostBrightP1 = 200;
  boostBrightP2 = 200;

  if (colorNumPlayer2 == 1) {
    saturationPl2 = 3;
    saturationHead2 = 3;
    boostBrightP2 = 0;

  }
  if (colorNumPlayer1 == 3) {
    saturationPl1 = 2;
    saturationHead1 = 2;
  }

}

//_____________________________________________________________________________________________________________________________________________________________P_U_N_K_T_E_S_Y_S_T_E_M

void pointSystem() {
  if (P1_lap <  laps && P2_lap < laps) {
    schleifen++;
    xP1 += P1_speed;
    xP2 += P2_speed;
  }
  else {
    faktorP1 = xP1 / schleifen;
    faktorP2 = xP2 / schleifen;
  }
}


//_____________________________________________________________________________________________________________________________________________________________Z_U_R_U_E_C_K_S_E_T_Z_E_N
int LCD2 = 1;
void reset() { //Zurücksetzten der gesamten Variablen auf die Anfangswerte, damit eine neues Spiel gestartet werden kann
  lcd.clear();
  LCD2 = 1;
  P2_lap = 0;
  P1_lap = 0;
  blinkReps = 0;



  switchStateP2 = 0;
  prevSwitchStateP2 = 0;
  P2firstLED = -2;
  P2secondLED = -3;
  P2thirdLED = -4;
  P2fourthLED = -5;
  P2fifthLED = -6;
  P2prevFirstLED_int = 0;

  switchStateP1 = 0;
  prevSwitchStateP1 = 0;
  P1firstLED = -2;
  P1secondLED = -3;
  P1thirdLED = -4;
  P1fourthLED = -5;
  P1fifthLED = -6;
  P1prevFirstLED_int = 0;

  P1_speed = 0.0;

  P2_speed = 0.0;

  interrupt = 0;
  interruptGo = 0;

  randomNum = random(0, trackend);

  Boost_P1_ = 0;
  Boost_P1_Time = 0;
  Boost_P1_Fuel = 0;
  Boost_P1_Load = 0;

  Boost_P2_ = 0;
  Boost_P2_Time = 0;
  Boost_P2_Fuel = 0;
  Boost_P2_Load = 0;

  prepForStartSeq = 0;
  startRace = 3;
  startCounter = 0;
  fillColor1 = (interruptionend - interruptionstart) / 2 + interruptionstart;
  fillColor2 = fillColor1;


  colorNumPlayer1 = 0;
  colorNumPlayer2 = 0;

}
//_____________________________________________________________________________________________________________________________________________________________D_I_S_P_L_A_Y_-_A_N_Z_E_I_G_E

void LCDDisplay() {               //Displayaanzeige am Ende des Spiels(Punkteanzeige)
  PointsP1 = points * faktorP1;   //Errechnung der Punkte für Spieler 1
  PointsP2 = points * faktorP2;   //Errechnung der Punkte für Spieler 2
  lcd.setCursor(0, 0);
  lcd.print("Punktzahl Spieler 1:");
  lcd.setCursor(0, 1);
  lcd.print(PointsP1);
  lcd.setCursor(0, 2);
  lcd.print("Punktzahl Spieler 2:");
  lcd.setCursor(0, 3);
  lcd.print(PointsP2);
}
int LCD1 = 1; //Variable für das LCD-Display
void LCDClear() {     //Display Anzeige damit die Alten Werte für die neue Runde überschrieben werden
  schleifen = 0;      //Zurücksetzen der benötigten Variablen für die Errechnung der Punkte
  xP1 = 0;
  xP2 = 0;
  faktorP1 = 0;
  faktorP2 = 0;
  PointsP1 = 0;
  PointsP2 = 0;
  LCD1 = 1;
  lcd.clear();
}
void LCDMenu() {                    // LCD Anzeige vor dem Start des ersten Spiels
  if (LCD2 == 1) {
    lcd.setCursor(1, 0);
    lcd.print("Herzlich Willkommen");
    lcd.setCursor(4, 2);
    lcd.print("Waehle deine");
    lcd.setCursor(4, 3);
    lcd.print("Spielerfarbe");
    //lcd.setCursor(0,3);
    //lcd.print();
    LCD2 = 0;
  }
}


void LCD() {                      //LCD Anzeige während des Spiels
  if (LCD1 == 1) {
    lcd.clear();
    lcd.setCursor(1, 1);
    lcd.print("www.vw-led-race.de");
    LCD1 = 0;

  }
}

void LEDON() {                    // LED welche beim Start der Abkürzung angeht
  if (interrupt == 1) {
    LED == HIGH;
  }
  else {
    LED == LOW;
  }
}
void Endseq(){
for (int x=0; x<ar;x=x+2){
  tone(6,zukunft[x],zukunft[x+1]);
  delay(zukunft[x+1]);
}
}
