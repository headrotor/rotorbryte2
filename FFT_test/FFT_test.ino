// FFT Test for Analog Mic
//
// Use PRJC's Audio library to compute audio spectrum analysis
// from an analog mic input on ADC0 while taking an analog read
// sample from a pin on ADC1 using Pedvide's ADC library.
// Download the latest copy at:
// https://github.com/pedvide/ADC
//
// Compute a 1024 point Fast Fourier Transform (spectrum analysis)
// on audio connected to the Left Line-In pin.  By changing code,
// a synthetic sine wave can be input instead.
//
// The first 40 (of 512) frequency analysis bins are printed to
// the Arduino Serial Monitor.  Viewing the raw data can help you
// understand how the FFT works and what results to expect when
// using the data to control LEDs, motors, or other fun things!
//
// This example code is in the public domain.


#include <Audio.h>
#include <Wire.h>
#include <SPI.h>
#include <SD.h>

#include <ADC.h>

// CREATE THE ADC OBJECT FIRST, BEFORE ALL AUDIO STUFF!!
ADC *adc = new ADC(); // adc object
const int readPin = 29; // ADC1, Choose a pin the can be accessed by the ADC *NOT* being used to stream audio.
// Currently this must be a pin on ADC1, but has been suggested as a fix for the audio library.

// Create the Audio components.  These should be created in the
// order data flows, inputs/sources -> processing -> outputs
// Using adafruit mic-amp, edit library to turn-off internal voltage reference in
// inputAnalog.cpp

AudioInputAnalog       analogAudioIn(2); //Pin 15, mic pin, currently must be on ADC0
//AudioAnalyzeFFT1024    myFFT;
AudioAnalyzeFFT256    myFFT;



// Connect either the live input or synthesized sine wave
AudioConnection patchCord1(analogAudioIn, 0, myFFT, 0);


#define R_LED 5
#define G_LED 4
#define B_LED 3

#define A_BRIGHT A7
#define A_SPEED A0
#define A_RANGE A1
#define A_HUE A6


void setup() {
  // Audio connections require memory to work.  For more
  // detailed information, see the MemoryAndCpuUsage example
  AudioMemory(12);


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

  // Configure the window algorithm to use
  //myFFT.windowFunction(AudioWindowHanning1024);
  myFFT.windowFunction(NULL);
  ///  344 sps /24 fps = 14
  myFFT.averageTogether(14);

  // Create a synthetic sine wave, for testing
  // To use this, edit the connections above
  //sinewave.amplitude(0.8);
  //sinewave.frequency(1034.007);


  adc->setReference(ADC_REF_1V2, ADC_1);
  //adc->setReference(ADC_REF_1V2, ADC_0);
  adc->setAveraging(32, ADC_1); // set number of averages
  adc->setResolution(10, ADC_1); // set bits of resolution
  adc->setConversionSpeed(ADC_VERY_LOW_SPEED, ADC_1); // change the conversion speed
  adc->setSamplingSpeed(ADC_VERY_LOW_SPEED, ADC_1); // change the sampling speed


  pinMode(R_LED, OUTPUT); // PWM_G
  pinMode(B_LED, OUTPUT); // PWM_B
  pinMode(G_LED, OUTPUT); // PWM_R

  // start dark
  analogWrite(R_LED, 0);
  analogWrite(G_LED, 0);
  analogWrite(B_LED, 0);

}

int value = 0;

float bass = 0.0;
float midr = 0.0;
float treb = 0.0;

float thresh = 0.002;
float smooth = 0;

void loop() {
  float b, m, t, n;
  int i;
  int c;



  if (myFFT.available()) {
    value = adc->analogRead(readPin, ADC_1); // read a new value, will return ADC_ERROR_VALUE if the comparison is false.

    b = 0;
    c = 0;
    for (i = 0; i < 1; i++) {

      n = myFFT.read(i);
      if (n > thresh) {
        b += n;
      }
      c++;
    }
    b = b / float(c);

    m = 0;
    c = 0;
    for (; i < 10; i++) {
      n = myFFT.read(i);
      if (n > thresh) {
        m += n;
      }
      c++;
    }
    m = m / float(c);

    t = 0;
    c = 0;
    for (; i < 40; i++) {
      n = myFFT.read(i);
      if (n > thresh) {
        t += n;
      }
      t += myFFT.read(i);
      c++;
    }
    t = t / float(c);



    // each time new FFT data is available
    // print it all to the Arduino Serial Monitor


    Serial.print("Pin: ");
    Serial.print(readPin);
    Serial.print(" = ");
    Serial.print(value, DEC); //assuming 3.3v reference voltage
    //    Serial.print(" bass=");
    //    Serial.print(bass);
    //    Serial.print(" mid=");
    //    Serial.print(midr);
    //    Serial.print(" treb=");
    //    Serial.print(treb);

    bass = (smooth * bass + (1 - smooth) * b);
    treb = (smooth * treb + (1 - smooth) * t);
    midr = (smooth * midr + (1 - smooth) * m);
    analogWrite(R_LED, bass * 10 * value);
    analogWrite(G_LED, midr * 10 * value);
    analogWrite(B_LED, treb * 10 * value);


    Serial.print("v   FFT: ");
    for (i = 0; i < 40; i++) {
      n = myFFT.read(i);
      if (n >= 0.008) {
        Serial.print(n);
        Serial.print(" ");
      }
      else {
        Serial.print("  -  "); // don't print "0.00"
      }
    }
    Serial.print("\n");
  }
}

