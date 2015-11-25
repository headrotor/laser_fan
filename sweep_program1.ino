
// sweep-program: Teensy 3.1 Arduino program to control 8 servos and 8 PWM laser outputs
// for the kinetic artwork described here: http://www.instructables.com/id/Kinetic-light-work-with-servo-controlled-laser-mod/
// Uses cos8() from the FastLED library http://fastled.io/

#include "FastLED.h"
#include <Servo.h>
//local gamma lookup
#include "gamma.h"



int pos = 0;    // variable to store the servo position

// clockwise from leftmost servo/laser
int servo_pins[] = { 17, 16, 15, 14, 10, 9, 8, 7};
int laser_pins[] = { 23, 22, 21, 20, 6, 5, 4, 3};

// number of channels
#define NC 8



Servo servo0;
Servo servo1;
Servo servo2;
Servo servo3;
Servo servo4;
Servo servo5;
Servo servo6;
Servo servo7;

Servo servos[8];

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


void setup()
{

  Serial.begin(9600);

  // can't do this in a loop for whatever reason :(
  servo0.attach(servo_pins[0]);
  servo1.attach(servo_pins[1]);
  servo2.attach(servo_pins[2]);
  servo3.attach(servo_pins[3]);
  servo4.attach(servo_pins[4]);
  servo5.attach(servo_pins[5]);
  servo6.attach(servo_pins[6]);
  servo7.attach(servo_pins[7]);
  servos[0] = servo0;
  servos[1] = servo1;
  servos[2] = servo2;
  servos[3] = servo3;
  servos[4] = servo4;
  servos[5] = servo5;
  servos[6] = servo6;
  servos[7] = servo7;

  for (int i = 0; i < NC ; i++) {
    pinMode(laser_pins[i], OUTPUT);
  }
  pinMode(led, OUTPUT);
  pinMode(push_button, INPUT_PULLUP);
}


void loop() {
  int button_state;

  // implement state machine
  if (state_now == CATCHUP) {
    // block, ignoring button, until all servos are at 90
    do_catchup();
    state_now = IDLE_STATE;
    //update_lasers_inverse_ripple();
    Serial.println("state = IDLE");

  } else if (state_now == IDLE_STATE) {
    do_idle();
    button_state = digitalRead(push_button);
    if (button_state == LOW) {
      state_now = ACTIVE;
      Serial.println("state = ACTIVE");
      advance_laser_pattern();
      advance_servo_pattern();

    }
  } else if (state_now == ACTIVE) {
    do_active();
    button_state = digitalRead(push_button);
    if (button_state == HIGH) {
      state_now = CATCHUP;
      Serial.println("state = CATCHYUP");
    }
  }
  //digitalWrite(led, button_state);
  do_delay();
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

void do_catchup() {
  // block, catch up all servos to the same phase by reducing servo amp
  while (servo_amp > 0) {
    servo_amp -= 1;
    update_servos();
    do_delay();
  }
}

void do_idle() {
  // no need to do servos, just test pattern for  lasers

  for (int i = 0; i < 4; i++) {
    laser_period[i] = 6 + i;
    laser_period[NC - i - 1] = 6 + i;
  }
  laser_phase_factor = 0;
  update_lasers_idle();
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

void do_active() {
  if (servo_amp < SERVO_RANGE) {
    servo_amp++;
  }
  update_servos();
  update_lasers();

}

void update_lasers() {
  for (int i = 0; i < NC ; i++) {
    laser_ticks[i]++;
    if (laser_ticks[i] > laser_period[i]) {
      laser_phase[i] += 1;
      laser_ticks[i] = 0;
    }
    laser_val[i] = cos8(laser_phase[i] + uint8_t(laser_phase_factor * float(laser_offset[i])));
    analogWrite(laser_pins[i], gammat[map(laser_val[i], 0, 255, 50, 255)]);
  }
}

void update_lasers_idle() {
  for (int i = 0; i < NC ; i++) {
    laser_ticks[i]++;
    if (laser_ticks[i] > laser_period[i]) {
      laser_phase[i] += 1;
      laser_ticks[i] = 0;
    }
    laser_val[i] = sin8(laser_phase[i] + uint8_t(laser_phase_factor * float(laser_offset[i])));
    analogWrite(laser_pins[i], gammat[map(laser_val[i], 0, 255, 50, 255)]);
  }
}

void update_lasers_inverse_ripple() {
  for (int i = 0; i < NC ; i++) {
    laser_ticks[i]++;
    if (laser_ticks[i] > laser_period[i]) {
      laser_phase[i] += 1;
      laser_ticks[i] = 0;
    }
    laser_val[i] = 255 - duck_phase(laser_phase[i] + uint8_t(1.0 * float(laser_offset[i])));
    analogWrite(laser_pins[i], gammat[map(laser_val[i], 0, 255, 100, 255)]);
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




