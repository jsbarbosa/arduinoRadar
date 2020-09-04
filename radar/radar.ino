#include <stdlib.h>
#include <Servo.h>

#define UART_SPEED 115200
#define IDN_COMMAND "*IDN?"
#define GET_ANGLE_DIST "GAD"
#define IDN_ANSWER "Radar-v01"
#define SET_MIN_DISTANCE_COMMAND "SMIN"
#define SET_MAX_DISTANCE_COMMAND "SMAX"
#define STOP_COMMAND "STOP"
#define STOP_SHOOTING "SHOOTING"
#define SHOOT_COMMAND "SHOOT"

// Ultrasonic pins
#define TRIGGER_PIN 3
#define ECHO_PIN 2
#define MAX_DISTANCE 2.0 // meters
#define WAIT_TIME (2 * MAX_DISTANCE * 1e6) / 340.0 // microseconds

// Servo
#define SERVO_PIN 11
#define SERVO_START 750.0
#define SERVO_ANGLE_RATIO 13
#define MAX_ANGLE 130

// Shooter
#define SHOOTER_PIN 10
#define SHOOTER_TIME_THRESHOLD 30
#define SHOOTER_MAX_ANGLE 110

Servo servo;
Servo shooter;

volatile uint8_t current_angle = 0;
volatile unsigned int current_distance = 0, current_min_distance = 0, current_max_distance = 0;
volatile int shooting_timer = -1;
volatile uint8_t angle_step = 1, stop_rotating = 0, stop_shooting = 0;
volatile uint8_t timer = 0;

void setup() {
  Serial.begin(UART_SPEED);
  servo.attach(SERVO_PIN);
  shooter.attach(SHOOTER_PIN);
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(SHOOTER_PIN, OUTPUT);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  setup_timer();
  sei();
}

void loop() {
  interpret_uart();
  current_distance = get_distance();
  if(
      (current_distance >= current_min_distance) &
      (current_distance <= current_max_distance) &
      (shooting_timer == -1) &
      (stop_shooting == 0)
    ) shoot();
}

void setup_timer(void){
  TCCR0A |= (1 << WGM01);    // Set the CTC mode
  OCR0A = 250;            // Set the value for 16ms
  TIMSK0 |= (1 << OCIE0A);   // Set the interrupt request
  TCCR0B = (1 << CS02) | (1 << CS00);    //Set the prescale 1/1024 clock
}

void interpret_uart(void) {
  if (Serial.available()) {
    String received = Serial.readStringUntil('\n');
    if (received == IDN_COMMAND) {
      Serial.println(IDN_ANSWER);
    }
    else if (received == GET_ANGLE_DIST){
      Serial.print(current_angle);
      Serial.print(' ');
      Serial.println(current_distance);
    }
    else if (received == STOP_COMMAND){
      if(stop_rotating == 0) stop_rotating = 1;
      else{
        stop_rotating = 0;
        timer = 0;
      }
    }
    else if (received == STOP_SHOOTING){
      if(stop_shooting == 0) stop_shooting = 1;
      else stop_shooting = 0;
    }
    else if (received == SHOOT_COMMAND){
      shoot();
    }
    else if(received.indexOf(SET_MIN_DISTANCE_COMMAND) >= 0){
      sscanf(received.c_str(), "SMIN%d", &current_min_distance);
    }
    else if(received.indexOf(SET_MAX_DISTANCE_COMMAND) >= 0){
      sscanf(received.c_str(), "SMAX%d", &current_max_distance);
    }
  }
}

int get_distance(void) {
  unsigned long value;

  digitalWrite(TRIGGER_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIGGER_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIGGER_PIN, LOW);

  value = pulseIn(ECHO_PIN, HIGH, WAIT_TIME);
  if(value == 0){
    value = WAIT_TIME;
  }

  return ceil(value * 0.034 / 2);
}

void move_servo(int angle){ 
 servo.writeMicroseconds(angle * SERVO_ANGLE_RATIO + SERVO_START);
}

void make_step(void) {
  move_servo(current_angle);
  current_angle += angle_step;
  if (current_angle == MAX_ANGLE){
    angle_step = -1;
  }
  else if(current_angle == 0){
    angle_step = 1;
  }
}

void shoot(void){
  shooting_timer = 0;
}

ISR(TIMER0_COMPA_vect){
  timer += 1;
  if((timer == 2) & !stop_rotating){
    make_step();
    timer = 0;
  }
  if(shooting_timer >= 0){
    if(shooting_timer == 0) shooter.writeMicroseconds(SHOOTER_MAX_ANGLE * SERVO_ANGLE_RATIO + SERVO_START);
    else if (shooting_timer == SHOOTER_TIME_THRESHOLD){
      shooter.writeMicroseconds(0 * SERVO_ANGLE_RATIO + SERVO_START);
      shooting_timer = -2;
    }
    shooting_timer += 1;   
  }
}
