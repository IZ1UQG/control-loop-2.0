/*

    28.03.2015

    This is a fork of Alain De Carolis, WW3WW, FT-817 Automatic Loop Tuner (https://code.google.com/p/ft-817-automatic-loop/)

    This progam uses a modified version of the qrptracker library
    written by Bruce Gordon Robertson, VE9QRP

    released under the GPL V3 license

    I have added a 16x2 LCD display with some infos of the rig an the possibility to work with a digital interface and
    this tuner.



*/

#include <SoftwareSerial.h>
#include "FT817.h"
#include <PWMServo.h>
#include <EEPROM.h>

//i2c lcd
#include <Wire.h>
//#include <LCD.h>
#include <LiquidCrystal_I2C.h>

#define I2C_ADDR    0x27

LiquidCrystal_I2C lcd(0x27, 20, 4);
//i2c lcd

const int addr = 0x50;            // Indirizzo EEPROM per il salvataggio della posizione corrente del condensatore

int n = LOW;
int encoder0PinALast = LOW;

const int encoder0PinA = 3;    // Encoder PinA
const int encoder0PinB = 4;    // Encoder PinB
const int stepTime = 100;      // Misura la rotazione dell'encoder lenta o veloce
const int bigStep = 10;        // gradi durante la rotazione veloce --> orig era 10!!
const int smallStep = 1;       // gradi durante la rotazione lenta
int step;

const int rxPin = 8;            //  Seriale pin
const int txPin = 7;            //  Seriale pin
const int relaisPin = 0;        //  Pin per relè per commutare DataIN/DataOUT

const int buttonPin = 0;       // 5 pulsante di sintonizzazione - premi per sintonizzare
int buttonState = 0;           // pulsante di sintonizzazione

const int ledPin = 0;         // 12 tuning led - ON quando la sintonizzazione è andata a buon fine
const int ledPinON = 0;       // 11 led sintonizzato - ON quando sintonizzato OK - lampeggia per errori
const int servoPin = 9;        // Questo DEVE essere 9 quando si usa PWMServo lib
const int buzzerPin = 0;      // 10 alcuni effetti acustici
const int servoTimeout = 2000; // 2 secondi inutilizzati e il servo si staccherà

unsigned long servoTime;       // usato per decidere quando possiamo staccare il servo
unsigned long encoderTime;     // utilizzato per determinare la velocità di rotazione dell'ecoder

int pos;                       // Variabile helper posizione servo
int swr;                       // Variabile helper SWR
unsigned long frq;             // Frequenz dal rig
const int minPos = 1;          // Posizione minima raggiungibile dal Servo
const int maxPos = 650;        // Posizione massima raggiungibile dal Servo

PWMServo myservo;
FT817 rig;

void setup() {

  lcd.begin (16, 2);
lcd.init();
  lcd.backlight();
  lcd.home (); // go home
  lcd.print("Arduino ML Tuner");
  lcd.setCursor(1, 1);
  lcd.print("----IZ1UQG----");
  delay(1500);
  init_screen();

  //pinmode input
  pinMode(encoder0PinA, INPUT);
  pinMode(encoder0PinB, INPUT);
  pinMode(buttonPin, INPUT);

  //pinmode output
 // pinMode(relaisPin, OUTPUT);
 // pinMode(ledPin, OUTPUT);
 // pinMode(ledPinON, OUTPUT);

  //internal pullup
 // digitalWrite(relaisPin, HIGH);
 // digitalWrite(ledPinON, HIGH);

  Serial.begin(9600);
  SoftwareSerial mySerial(rxPin, txPin);
  rig.assignSerial(mySerial);


  pos = EEPROM.read(addr);
  myservo.attach(servoPin);
  myservo.write(pos);
  Serial.print("Servo posizionato a ");
  Serial.println(pos);
  lcd_update_pos();

  lcd.setCursor(5, 0);
  lcd.print("Un");
  servoTime = millis();

}

void loop() {

  rig.begin(9600);

  frq = rig.getFreqMode();
  lcd_update_freq();


  if (( myservo.attached() ) && (millis() > (servoTime + servoTimeout))) {
    myservo.detach();
    Serial.println(" Scollegare il servo ");
    EEPROM.write(addr, pos);
    Serial.println(EEPROM.read(addr));
    lcd_update_pos();
  }

  buttonState = digitalRead(buttonPin);

  if (buttonState == HIGH) {
    tune();
  }

  n = digitalRead(encoder0PinA);

  if ((encoder0PinALast == LOW) && (n == HIGH)) {

    if (! myservo.attached() ) {
      myservo.attach(servoPin);
      myservo.write(pos);
    }

    if (millis() - encoderTime > stepTime) {
      step = smallStep;
    } else {
      step = bigStep;
    }

    encoderTime = millis();

    if (digitalRead(encoder0PinB) == LOW) {

      if ( pos >= maxPos ) {
        pos = maxPos;
        singleFlash();
      }
      else {
        pos += step;
      }

    } else { // digitalRead(encoder0PinB) == HIGH

      if ( pos <= minPos ) {
        pos = minPos;
        singleFlash();
      }
      else {
        pos -= step;
      }

    }

    Serial.print(" Posizione selezionata dall'Encoder: ");
    Serial.println (pos);
    lcd_update_pos();
    myservo.write(pos);
    servoTime = millis();

  }
  encoder0PinALast = n;

}

void tune() {

  boolean tuned = 0;
  unsigned int quadrant = 0;
  unsigned long sweepTime;
  unsigned int quadrantSize = 9;  // 15  - 9
  unsigned quadrantTime = 150;  // 150  - 200

  Serial.println("At Tune...");

  byte mode = rig.getMode();
  Serial.print("The Radio is in mode ");
  Serial.println(mode);

  rig.setMode(FT817_MODE_FM);

  rig.setPTTOn();
  delay(300);

  swr = rig.getSWR();

  Serial.print("Initial SWR ");
  Serial.println(swr);
  lcd_update_swr();

  if (! rig.getTXSuccess()) {
    rig.setPTTOff();
    delay(300);
    rig.setMode(mode);
    Serial.println("Errore: la radio non può trasmettere!");

    lcd.setCursor(0, 3);
  lcd.println("Err: RTX no TX!");
  
    errorBeep();
    return;
  }

  if (swr <= 2) {   //was 3
    rig.setPTTOff();
    delay(300);
    rig.setMode(mode);
    Serial.println("Già sintonizzato");
    successBeep();
    return;
  }

  Serial.println("Spostare il condensatore");
  if (! myservo.attached() ) {
    myservo.attach(servoPin);
    servoTime = millis();
    lcd.setCursor(0, 2);
  lcd.println("SpostO il condensatore");
    
  }


  myservo.write(minPos);
  servoTime = millis();


  delay(1500);

  Serial.println("Avvio dei campioni SWR");

  for ( pos = minPos; pos <= maxPos; pos += quadrantSize ) {

    quadrant++;

    Serial.print("Inserimento quadrante ");
    Serial.print(quadrant);
    Serial.print(": ");
    Serial.print(quadrantSize * (quadrant -  1) + minPos);
    Serial.print("-");
    Serial.println(quadrantSize * quadrant + minPos);

    myservo.write(pos);

    servoTime = millis();

    for (sweepTime = servoTime; sweepTime <= servoTime + quadrantTime; sweepTime = millis()) {

      // Stop if a button is pressed
      if ( digitalRead(buttonPin) == HIGH ) {
        errorBeep();
        break;
      }

      swr = rig.getSWR();
      Serial.print(" SWR: ");
      Serial.println(swr);
      lcd_update_swr();

      if (swr <= 0) {

        Serial.println(" SWR 0 Trovato!");
        tuned = 1;
        break;

      }

    }

    if (tuned) {
      Serial.print("Tunable in quadrant ");
      Serial.println(quadrant);

      // Fine Tuning
      // Go back two quadrants to find the right position
      tuned = 0;
      for (pos = quadrantSize * quadrant + minPos;
           pos >= quadrantSize * ( quadrant - 2) + minPos;
           pos--) {

        myservo.write(pos);
        Serial.print("Trying pos ");
        Serial.print(pos);
        lcd_update_pos();
        delay(50);
        swr = rig.getSWR();
        Serial.print(" SWR: ");
        Serial.println(swr);
        lcd_update_swr();

        if (swr <= 1) {
          Serial.print("Tuned to pos ");
          Serial.println(pos);
          lcd_update_pos();
          lcd_update_swr();
          tuned = 1;
          break;
        }

      }

      break;

    }

  }

  // The End
  // back to RX
  rig.setPTTOff();
  delay(300);
  // Switch back to original mode
  rig.setMode(mode);
  Serial.print("Mode tuned: ");
  Serial.println(mode);

  if (tuned) {
    successBeep();
  } else {
    errorBeep();
  }

}

void singleFlash() {

  digitalWrite(ledPinON, LOW);
  delay(300);
  digitalWrite(ledPinON, HIGH);

}

void errorBeep() {

  digitalWrite(ledPinON, LOW);
  delay(300);
  digitalWrite(ledPinON, HIGH);
  delay(300);
  digitalWrite(ledPinON, LOW);
  delay(300);
  digitalWrite(ledPinON, HIGH);
  delay(300);
  tone(buzzerPin, 800, 500);

}

void successBeep() {

  digitalWrite(ledPin, HIGH);

  tone(buzzerPin, 600, 500);
  digitalWrite(ledPin, HIGH);
  delay(100);
  digitalWrite(ledPin, LOW);

  tone(buzzerPin, 800, 500);
  digitalWrite(ledPin, HIGH);
  delay(100);
  digitalWrite(ledPin, LOW);

  tone(buzzerPin, 1000, 500);
  digitalWrite(ledPin, HIGH);
  delay(100);

  digitalWrite(ledPin, LOW);

}

void init_screen()
{
  lcd.clear();
  lcd.setCursor (0, 0);
  lcd.print("SWR: ");
  lcd.setCursor (5, 0);
  lcd.print("   ");

  lcd.setCursor(9, 0);
  lcd.print("Pos: ");
  lcd.print("   ");

  lcd.setCursor(0, 1);
  lcd.print("FRQ: ");
  lcd.print("    ");
}

void lcd_update_pos()
{
  lcd.setCursor (13, 0);
  if (pos < 100) {
    lcd.print(" ");
  }
  if (pos < 10) {
    lcd.print(" ");
  }
  lcd.print (pos);
}

void lcd_update_freq()
{
  lcd.setCursor (4, 1);
  //t_frq = frq/100;
  if (frq < 1000000) {
    lcd.print(" ");
  }
  if (frq < 100000) {
    lcd.print(" ");
  }
  if (frq < 10000) {
    lcd.print(" ");
  }

  lcd.print((frq / 100), DEC);
  lcd.print(",");
  lcd.print((frq) % 100, DEC);
  lcd.print(" kHz");
  if ((frq) % 100 < 10)
  {
    lcd.setCursor (15, 1);
    lcd.print(" ");
  }
}

void lcd_update_swr()
{
  lcd.setCursor(5, 0);
  lcd.print(swr);
}
