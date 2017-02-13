/* Mono Peak Meter

   Scrolling peak audio level meter in the Arduino Serial Monitor

   Audio input needs to connect to pin 16 (A2).  The signal range is 0 to 1.2V.
   See the documentation in the Audio System Design Tool for the recommended
   circuit to connect an analog signal.

   This example code is in the public domain
*/
// https://forum.pjrc.com/threads/25532-ADC-library-update-now-with-support-for-Teensy-3-1

#include <Audio.h>
#include <Wire.h>

#include <SPI.h>
#include <SD.h>

#include <ADC.h>

ADC *adc = new ADC(); // adc object


// GUItool: begin automatically generated code
AudioInputAnalog         adc1;           //xy=164,95
AudioAnalyzePeak         peak1;          //xy=317,123
AudioConnection          patchCord1(adc1, peak1);
// GUItool: end automatically generated code



#define ana16 A16
#define ana17 A16
#define ana18 A16
#define ana18 A16

#define R_LED 5
#define G_LED 4
#define B_LED 3

#define A_BRIGHT A7
#define A_SPEED A0
#define A_RANGE A1
#define A_HUE A6

void setup() {
  AudioMemory(4);
  Serial.begin(9600);

  pinMode(A6, INPUT);
  pinMode(A7, INPUT);


  pinMode(A16, INPUT);
  pinMode(A17, INPUT);
  pinMode(A18, INPUT);
  pinMode(A19, INPUT);

  pinMode(A6, INPUT);
  pinMode(A7, INPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);


//  adc->setReference(ADC_REF_3V3, ADC_1);
//  adc->setAveraging(12, ADC_1); // set number of averages
//  adc->setResolution(8, ADC_1); // set bits of resolution
//  adc->setConversionSpeed(ADC_VERY_LOW_SPEED, ADC_1); // change the conversion speed
//  adc->setSamplingSpeed(ADC_VERY_LOW_SPEED, ADC_1); // change the sampling speed
//  adc->setReference(ADC_REF_DEFAULT, ADC_1);
//  //adc->setReference(ADC_REF_3V3,ADC_0);


  pinMode(R_LED, OUTPUT); // PWM_G
  pinMode(B_LED, OUTPUT); // PWM_B
  pinMode(G_LED, OUTPUT); // PWM_R

  // start dark
  analogWrite(R_LED, 0);
  analogWrite(G_LED, 0);
  analogWrite(B_LED, 0);

}

// for best effect make your terminal/monitor a minimum of 31 chars wide and as high as you can.

elapsedMillis fps;

void loop() {
  if (fps > 24) {
    if (peak1.available()) {
    //if (true) {
      fps = 0;

      int bright = 0;
      //bright = bright/4;
      char peak = peak1.read() * 128;

      //char peak = 0;
      analogWrite(R_LED, peak);
      analogWrite(B_LED, 128 - peak);


      int monoPeak = peak / 4;

      Serial.print(analogRead(A16));
      Serial.print("|");
      Serial.print(analogRead(A17)); // HUE
      Serial.print("|");
      Serial.print(analogRead(A18)); //SPEED
      Serial.print("|");
      Serial.print(analogRead(A19));
      Serial.print("|");
      for (int cnt = 0; cnt < monoPeak; cnt++) {
        Serial.print(">");
      }
      Serial.println();
    }
  }
}
