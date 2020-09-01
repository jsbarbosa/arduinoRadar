#include <stdlib.h>
#include <Servo.h>

#define UART_SPEED 115200
#define IDN_COMMAND "*IDN?"
#define GET_ANGLE_DIST "GAD"
#define IDN_ANSWER "Radar-v01"

// Ultrasonic pins
#define TRIGGER_PIN 3
#define ECHO_PIN 2
#define MAX_DISTANCE 2.0 // meters
#define WAIT_TIME (2 * MAX_DISTANCE * 1e6) / 340.0 // microseconds

// Servo
#define SERVO_PIN 12
#define SERVO_START 750.0
#define SERVO_ANGLE_RATIO 13
#define MAX_ANGLE 130

Servo servo;

volatile uint8_t current_angle = 0;
volatile unsigned int current_distance = 0;
volatile uint8_t angle_step = 1;
volatile uint8_t timer = 0;

void setup() {
  Serial.begin(UART_SPEED);
  servo.attach(SERVO_PIN);
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  setup_timer();
  sei();
}

void loop() {
  interpret_uart();
  current_distance = get_distance();
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

ISR(TIMER0_COMPA_vect){
  timer += 1;
  if(timer == 2){
    make_step();
    timer = 0;
  }
}
