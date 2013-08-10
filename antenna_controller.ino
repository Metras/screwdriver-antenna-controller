/*
    Screwdriver antenna controller based on arduino
    Version 0.1
    

    Copyright (C) 2013  Dave Stoll, N9SLE dave.stoll@gmail.com

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/
#include <EEPROM.h>
#include <SoftwareSerial.h>

// set up global static variables
const int DspPin = 0;
const int mem1 = 1;
const int mem2 = 2;
const int mem3 = 3;
const int mem4 = 4;
const int mem5 = 5;

const int setBtn = 6;
const int upBtn = 7;
const int dnBtn = 8;

const int okBtn = 9;
const int clrBtn = 10;

const int reedSw = 12;

const int motor0 = A0;
const int motor1 = A1;
const int voltMeter = A2;

// global non-static variables
SoftwareSerial mySerial = SoftwareSerial(255, DspPin);
int turnsCount=0, lastTurnsCount=0;

long debounceButtonDelay = 50;    // the debounce time; increase if the output flickers
long debounceReedDelay = 5;    // the debounce time; increase if the output flickers
float voltageConstant = 0.004887585532746823;  // multiply analog value by this to get input voltage


//This function will write a 2 byte integer to the eeprom at the specified address and address + 1
void EEPROMWriteInt(int p_address, int p_value) {
   byte lowByte = ((p_value >> 0) & 0xFF);
   byte highByte = ((p_value >> 8) & 0xFF);

   EEPROM.write(p_address, lowByte);
   EEPROM.write(p_address + 1, highByte);
}

//This function will read a 2 byte integer from the eeprom at the specified address and address + 1
unsigned int EEPROMReadInt(int p_address) {
   byte lowByte = EEPROM.read(p_address);
   byte highByte = EEPROM.read(p_address + 1);

   return ((lowByte << 0) & 0xFF) + ((highByte << 8) & 0xFF00);
}

void resetMotor() {
  /* drop all input bias to motor control MOSFETs. 
     this will eliminate possibility of any short circuit */
  digitalWrite(motor0, LOW);
  digitalWrite(motor1, LOW);
  return;
}

void motorUp() {
  // wire 0 to +12v
  digitalWrite(motor0, HIGH);
  return;
}

void motorDown() {
  // wire 1 to +12v
  digitalWrite(motor1, HIGH);
  return;
}

void updateDisplay() {
  mySerial.write(12);
  mySerial.print("Screwdriver Ctrl");
  mySerial.write(148);
  mySerial.print("Ant Pos: ");
  mySerial.print(turnsCount);
  return;
}  

void writeVoltage(float volts) {
  mySerial.write(12);
  mySerial.print("Going up...");
  mySerial.write(148);
  mySerial.print("Volts: ");
  mySerial.print(volts);
  return;
}

void setup() {
  // setup Parallax LCD on output pin 12
  pinMode(DspPin, OUTPUT);
  digitalWrite(DspPin, HIGH);
  mySerial.begin(9600);
  delay(100);
  mySerial.write(12);                 // Clear             
  mySerial.write(22);                 // Cursor Off, No Blink             
  mySerial.write(17);                 // Turn backlight on
  delay(5);                           // Required delay
  mySerial.print("Booting....");  // First line
  mySerial.write(13);
  mySerial.print("v0.1 de N9SLE");   // Second line
  mySerial.write(212);                // Quarter note
  mySerial.write(220);                // A tone
  delay(3000);                        // show the boot message for a few seconds
  
  // all momentary inputs will use built-in pullup and to activate
  // will pull down to ground.  This eliminates need for current limiting
  // resistor and simplifies the circuitry.
  pinMode(mem1, INPUT_PULLUP);
  pinMode(mem2, INPUT_PULLUP);
  pinMode(mem3, INPUT_PULLUP);
  pinMode(mem4, INPUT_PULLUP);
  pinMode(mem5, INPUT_PULLUP);
  
  pinMode(setBtn, INPUT_PULLUP);
  
  pinMode(upBtn, INPUT_PULLUP);
  pinMode(dnBtn, INPUT_PULLUP);
  
  pinMode(okBtn, INPUT_PULLUP);
  pinMode(clrBtn, INPUT_PULLUP);
  
  pinMode(reedSw, INPUT_PULLUP);
  
  pinMode(motor0, OUTPUT);
  pinMode(motor1, OUTPUT);
  
  updateDisplay();
}

void loop() {
  int reedStat = digitalRead(reedSw);
  int mem1Stat = digitalRead(mem1);
  int mem2Stat = digitalRead(mem2);
  int mem3Stat = digitalRead(mem3);
  int mem4Stat = digitalRead(mem4);
  int mem5Stat = digitalRead(mem5);
  
  int up = digitalRead(upBtn);
  int dn = digitalRead(dnBtn);
  
  int clr = digitalRead(clrBtn);
  
  float voltage = analogRead(voltMeter);
  
  // reed switch activated
  if (reedStat == LOW) {
    long start = millis();
    while (digitalRead(reedSw) == LOW) { } // do nothing until releasing button
    if ((millis() - start) > debounceReedDelay) { // only increase counter if reed actually hit
      turnsCount++;
      updateDisplay();
    }
  } 
  
  // up button pressed
  if (up == LOW) {
    long start = millis();
    while (digitalRead(upBtn) == LOW) { 
      while ((millis() - start) <= debounceButtonDelay) { }
      voltage = analogRead(voltMeter) * voltageConstant;
      writeVoltage(voltage);
      delay(500);
         
    } 
  
  }
  
}
    