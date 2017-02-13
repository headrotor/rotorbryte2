#include "FastLED.h"
#include <Servo.h>
//local gamma lookup
#include "gamma.h"

//Servo myservo;  // create servo object to control a servo
// a maximum of eight servo objects can be created

int pos = 0;    // variable to store the servo position

// clockwise from leftmost servo/laser
int servo_pins[] = { 19, 18};
int laser_pins[] = { 2, 3, 4};

// number of channels
#define NC 2

Servo servo0;
Servo servo1;

Servo servos[2];

// servos on pins 19 (SRVA) and 18 (SRVB
// R, G, B on pins D2, D3, D4
//Audio on A2


uint8_t laser_val[] = {0, 0, 0, 0, 0, 0, 0, 0};
// increment phase regularly by this amount
uint8_t laser_phase[] =   {0, 0, 0, 0, 0, 0, 0, 0};
// wait this many ticks to increment laser phase
uint16_t laser_period[] = {10, 10, 10, 10, 10, 10, 10 , 10};
// count ticks to update
uint16_t laser_ticks[] = {0, 0, 0, 0, 0, 0, 0, 0};

// add this to for different brightnessess
uint8_t laser_offset[] =   {0x00, 0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70};
#define LASER_MIN 30

float  laser_phase_factor = 1.0;  // scale phase offset, 0 = points in same direction


int push_button = 0;
int led = 13;

// servo motion
uint8_t servo_val[] = {0, 0, 0, 0, 0, 0, 0, 0};

uint8_t servo_phase[] = {0, 0, 0, 0, 0, 0, 0, 0};
// increment phase regularly by this amount

// wait this many ticks to increment servo phase
uint16_t servo_period[] = {10, 10, 10, 10, 10, 10, 10, 10};
// count ticks to update
uint16_t servo_ticks[] = {0, 0, 0, 0, 0, 0, 0, 0};

// add this to point in different directions
uint8_t servo_offset[] =   {0x00, 0x10, 0x20, 0x30, 0x40, 0x50, 0x60, 0x70};

float  servo_phase_factor = 1.0;  // scale phase offset, 0 = points in same direction
// want to vary phase factor from 0 to 1 to 0 to put servos in phase, then out, then in...

#define SERVO_RANGE 60
// global amplitude of servo motion
int servo_amp = SERVO_RANGE;


// STATE MACHINE hash defines for state machine...

#define IDLE_STATE 0  // When no button is pressed and we are all caught up
#define CATCHUP 99 // button is released; catch up to constant phase and 90 servos
#define ACTIVE  1  // pattern 1 for servos and so forth
int state_now = CATCHUP;

int current_servo_pattern = 0;
int current_laser_pattern = 0;

// step through each pattern sequentially;
#define N_LASER_PATTERNS 4
#define N_SERVO_PATTERNS 3


#define R_LED 5
#define G_LED 4
#define B_LED 3

void setup()
{

  Serial.begin(9600);

  servo0.attach(servo_pins[0]);
  servo1.attach(servo_pins[1]);
  servos[0] = servo0;
  servos[1] = servo1;

  //pinMode(6, INPUT); // jumpered to D5
  //pinMode(7, INPUT); // jumpered to D4
  pinMode(R_LED, OUTPUT); // PWM_G
  pinMode(B_LED, OUTPUT); // PWM_B
  pinMode(G_LED, OUTPUT); // PWM_R

  analogWrite(R_LED, 0);
  analogWrite(G_LED, 0);
  analogWrite(B_LED, 0);

  pinMode(led, OUTPUT);
  pinMode(push_button, INPUT_PULLUP);

  digitalWrite(5, LOW);
  digitalWrite(6, LOW);
  digitalWrite(7, LOW);
}

int toggle = 0;
int val = 0;
void loop()
{
  //int button_state;

  return;

  val += 1;
  if (val > 254) {
    toggle = 1 - toggle;
    val = 0;
  }
  if (toggle) {
    analogWrite(R_LED, gammat[val]);
    analogWrite(G_LED, gammat[val]);
    analogWrite(B_LED, gammat[val]);
  } else {
    analogWrite(R_LED, 255 - gammat[val]);
    analogWrite(G_LED, 255 - gammat[val]);
    analogWrite(B_LED, 255 - gammat[val]);
  }

  //    analogWrite(6, 0);
  //    analogWrite(7, 0);

  //    digitalWrite(5,HIGH);
  //    digitalWrite(6,HIGH);
  //    digitalWrite(7,HIGH);
  digitalWrite(13, toggle);
  delay(5);

  //   analogWrite(5,255);
  //    digitalWrite(6,LOW);
  //    digitalWrite(7,LOW);
  //    toggle = 1-toggle;
  //    digitalWrite(13,toggle);

  //  delay(1000);

}

void set_laser_period(int per) {
  // set all lasers to same period
  for (int i = 0; i < NC ; i++) {
    laser_period[i] = per;
    laser_phase[i] = 0;
  }
}

void advance_laser_pattern() {
  // change params to for laser pattern

  // synchronize phase
  for (int i = 0; i < NC ; i++) {
    laser_phase[i] = 0;
  }

  current_laser_pattern++;
  if (current_laser_pattern >= N_LASER_PATTERNS) {
    current_laser_pattern = 0;
  }

  if (current_laser_pattern == 0) {
    // pattern 0, ripple
    set_laser_period(2);
    laser_phase_factor = 0.0;
    Serial.println("laser *0*");
  } else if (current_laser_pattern == 1) {
    // pattern 1
    laser_phase_factor = 1.0;
    set_laser_period(0);
    for (int i = 0; i < 4; i++) {
      laser_offset[i] = 0x10 * i;
      laser_offset[i + 4] = 0x10 * i;
    }
    Serial.println("laser *1*");
  } else if (current_laser_pattern == 2) {
    // pattern 2
    for (int i = 0; i < NC; i++) {
      laser_offset[i] = 0x80 - (0x10 * i);
    }
    laser_phase_factor = 0.5;
    set_laser_period(4);
    Serial.println("laser *2*");
  }
  else if (current_laser_pattern == 3) {
    // pattern 3

    laser_offset[0] = 0;
    laser_offset[7] = 0;

    laser_offset[1] = 0x20;
    laser_offset[6] = 0x20;

    laser_offset[2] = 0x40;
    laser_offset[5] = 0x40;

    laser_offset[3] = 0x60;
    laser_offset[4] = 0x60;

    laser_phase_factor = 1.0;
    set_laser_period(5);
    Serial.println("laser *3*");
  }
}

void advance_servo_pattern() {
  // change params to for laser pattern
  current_servo_pattern++;
  if (current_servo_pattern >= N_SERVO_PATTERNS) {
    current_servo_pattern = 0;
  }
  if (current_servo_pattern == 0) {
    servo_phase_factor = 0.0;
    Serial.println("servo *0*");
  } else if (current_servo_pattern == 1) {
    //pattern 2
    for (int i = 0; i < NC; i++) {
      servo_phase_factor = 1.0;
      servo_offset[i] = 0x10 * i;
    }
    Serial.println("servo *1*");
  } else if (current_servo_pattern == 2) {
    // pattern 3
    servo_phase_factor = 1.0;
    for (int i = 0; i < 4; i++) {
      servo_offset[i] = 0x10 * i;
    }
    servo_offset[4] = 0xB0;
    servo_offset[5] = 0xA0;
    servo_offset[6] = 0x90;
    servo_offset[7] = 0x80;
    Serial.println("servo *2*");

  }

}



void do_delay() {
  // state-sensitive delay function
  if (servo_amp == 0) {
    delay(1);
    return;
  }
  if (servo_amp < SERVO_RANGE) {
    delay(5);
  }
  else {
    delay(1);
  }
}


uint8_t duck_phase(uint8_t phase) {
  // Make a piecewise-linear shape like /\___ where n is the breakpoint
  uint8_t n = 64;

  if (phase < n) { // rising
    return map(phase, 0, n, 0, 255);
  }
  else if (phase < 2 * n) // falling
    return map(phase - n, 0, n, 255, 0);

  return 0;
}



void update_servos() {
  for (int i = 0; i < NC ; i++) {
    servo_ticks[i]++;
    if (servo_ticks[i] > servo_period[i]) {
      servo_phase[i] += 1;
      servo_ticks[i] = 0;
    }
    servo_val[i] = sin8(servo_phase[i] + uint8_t(servo_phase_factor * float(servo_offset[i])));
    servos[i].write(map(servo_val[i], 0, 255, 90 - servo_amp, 90 + servo_amp));
  }
}




