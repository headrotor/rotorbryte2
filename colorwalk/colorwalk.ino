
// https://forum.pjrc.com/threads/25532-ADC-library-update-now-with-support-for-Teensy-3-1

#include <ADC.h>

#include <Audio.h>
#include <Wire.h>
#include <FastLED.h>
#define NUM_LEDS 1
// for hsv




CRGB rgb;
CHSV hsv;
//phase of sinusoidal motion
float omega = 0;

#include <SPI.h>
#include <SD.h>


ADC *adc = new ADC(); // adc object

//pot_bright=1 pot_speed=70 pot_hue=176 pot_range=9
//hue=183 sat=255 val=255
//omega=6.10
//pot_bright=1 pot_speed=69 pot_hue=176 pot_range=9
//hue=183 sat=255 val=255
//omega=6.12


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

// up/down buttons
#define BUT0 (0)
#define BUT1 (1)

#define A_BRIGHT A7
#define A_SPEED A0
#define A_RANGE A1
#define A_HUE A6

int pot_bright = 0;
int pot_speed = 0;
int pot_range = 0;
int pot_hue = 0;


#define SERVO_MIN 1350
#define SERVO_MAX 1650

int but0 = 1;
int but1 = 1;

#include <Servo.h>


//Servo servoA;  // create servo object to control a servo
//Servo servoB;  // create servo object to control a servo

int servoA = 18;
int servoB = 19;

int posA = 1505;    // variable to store the servo position
int posB = 1505;    // variable to store the servo position


// for time-averaging peaks
float peak_smooth;
// smoothing constant
float alpha = 0.98;



void setup() {
  AudioMemory(4);
  Serial.begin(9600);

  //servoA.attach(18);  // attaches the servo on pin 9 to the servo object
  //servoB.attach(19);  // attaches the servo on pin 9 to the servo object

  pinMode(servoA, OUTPUT);
  pinMode(servoB, OUTPUT);
  digitalWrite(servoA, LOW);
  digitalWrite(servoB, LOW);

  pinMode(A16, INPUT);
  pinMode(A17, INPUT);
  pinMode(A18, INPUT);
  pinMode(A19, INPUT);

  pinMode(A6, INPUT);
  pinMode(A7, INPUT);
  pinMode(A0, INPUT);
  pinMode(A1, INPUT);

  pinMode(BUT0, INPUT_PULLUP);
  pinMode(BUT1, INPUT_PULLUP);

  adc->setReference(ADC_REF_3V3, ADC_1);
  adc->setAveraging(12, ADC_1); // set number of averages
  adc->setResolution(8, ADC_1); // set bits of resolution
  adc->setConversionSpeed(ADC_VERY_LOW_SPEED, ADC_1); // change the conversion speed
  adc->setSamplingSpeed(ADC_VERY_LOW_SPEED, ADC_1); // change the sampling speed
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


elapsedMillis fps;
elapsedMillis servo_fps;


void loop() {
  if (fps > 24) {
    fps = 0;
    get_buts();
    print_buts();


    get_pots();

    float peak;
    bool audio = true;

    if (audio) {
      // advance phase based on audio input
      peak = peak1.read();



      alpha = float(pot_range) / 255.0;

      if (peak > 0.05) { // iuf above noise floor
        peak_smooth = alpha * peak + (1 - alpha) * peak_smooth;
      }
      // decay so we can detect zero input
      peak_smooth = 0.97 * peak_smooth;
      Serial.print("peak_smooth=");
      Serial.println(peak_smooth);
      Serial.print("omega=");
      Serial.println(omega);
      //omega +=  float(pot_speed - 1) / 3000.0;
      if (peak_smooth > 0.15) {
        omega +=  float(pot_hue) * peak_smooth / 500.0;
      }
      else {
        // Advance phase linearly
        omega +=  float(pot_speed) / 3000.0;
      }
    }


    if (omega > TWO_PI) {
      omega -= TWO_PI;
    }

    hsv.h = pot_hue + int((pot_range - 1) * cos(omega));
    hsv.s = 255 - int(peak_smooth * 10);
    hsv.v = 255;
    hsv2rgb_rainbow(hsv, rgb);

    if (true) {
      if (peak_smooth > 0.02) {
        int bright = int(20 * peak_smooth * pot_bright);
        if (bright > 255) {
          bright = 255;
        }
        rgb.nscale8_video(bright);
      } else {
        rgb.nscale8_video(pot_bright);
      }
    }
    //rgb.nscale8_video(int(10 * peak_smooth * pot_bright));
    //peak = min(10 * peak_smooth, pot_bright);
    //rgb.nscale8_video(int(peak));

    //rgb.nscale8_video(pot_bright);

    //print_pots();
    //print_rgb();
    show_led();
  }

  // horible hack: servo lib interferes with AD so just bitbang them
  if (servo_fps > 20) {
    // handle servos once every 20ms
    get_buts();
    // use buttons to increment and decrement servo position
    if (but1) {
      posA += 1;
      if (posA > SERVO_MAX)
        posA = SERVO_MAX;
    } else if (but0) {
      posA -= 1;
      if (posA < SERVO_MIN)
        posA = SERVO_MIN;
    }
    Serial.print("posA=");
    Serial.println(posA);
    servo_fps = 0;
    digitalWrite(servoA, HIGH);
    digitalWrite(servoB, HIGH);
    delayMicroseconds(posA);
    digitalWrite(servoA, LOW);
    digitalWrite(servoB, LOW);
  }
}


void get_pots() {
  pot_speed = analogRead(A18);
  pot_bright = analogRead(A16);
  pot_hue = analogRead(A17);
  pot_range = analogRead(A19);

}

void get_buts() {
  // return 1 if button depressed
  but0 = 1 - digitalRead(BUT0);
  but1 = 1 - digitalRead(BUT1);
}


void print_buts() {
  if (but0)
    Serial.println("but0");
  if (but1)
    Serial.println("but1");
}


void print_hsv() {
  Serial.print("hue=");
  Serial.print(hsv.h);
  Serial.print(" sat=");
  Serial.print(hsv.s);
  Serial.print(" val=");
  Serial.println(hsv.v);

}
void print_rgb() {
  Serial.print("R=");
  Serial.print(rgb.r);
  Serial.print(" G=");
  Serial.print(rgb.g);
  Serial.print(" B=");
  Serial.println(rgb.b);

}

void print_pots() {

  Serial.print("pot_bright=");
  Serial.print(pot_bright);
  Serial.print(" pot_speed=");
  Serial.print(pot_speed);
  Serial.print(" pot_hue=");
  Serial.print(pot_hue);
  Serial.print(" pot_range=");
  Serial.print(pot_range);
  //for (int cnt = 0; cnt < monoPeak; cnt++) {
  //  Serial.print(">");
  //}
  Serial.println();
}

void show_led() {

  analogWrite(R_LED, rgb.red);
  analogWrite(G_LED, rgb.green);
  analogWrite(B_LED, rgb.blue);
}
