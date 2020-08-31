#include <stdlib.h>
#include <Servo.h>

#define UART_SPEED 9600
#define IDN_COMMAND "*IDN?"
#define IDN_ANSWER "Radar-v01"

// Ultrasonic pins
#define TRIGGER_PIN 3
#define ECHO_PIN 2
#define MAX_DISTANCE 2.0 // meters
#define WAIT_TIME (2 * MAX_DISTANCE * 1e6) / 340.0 // microseconds
#define DETECTION_READINGS 3

// Servo
#define SERVO_PIN 12
#define SERVO_START 750.0
#define SERVO_ANGLE_RATIO 13
#define MAX_ANGLE 130

Servo servo;

volatile uint8_t current_angle = 0;
volatile uint8_t angle_step = 1;

void setup() {
  Serial.begin(UART_SPEED);
  servo.attach(SERVO_PIN);
  pinMode(SERVO_PIN, OUTPUT);
  pinMode(TRIGGER_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);
}

void loop() {
  interpret_uart();

  make_step();
}

void interpret_uart(void) {
  if (Serial.available()) {
    String received = Serial.readStringUntil('\n');
    if (received == IDN_COMMAND) {
      Serial.println(IDN_ANSWER);
    }
  }
}

unsigned long get_median(unsigned long *data, int positions) {
  unsigned long temp;

  int i, j;
  for (i = 0; i < positions - 1; i++) {
    for (j = i + 1; j < positions; j++) {
      if (data[j] < data[i]) {
        temp = data[i];
        data[i] = data[j];
        data[j] = temp;
      }
    }
  }

  if (positions % 2 == 0) {
    return ((data[positions / 2] + data[positions / 2 - 1]) / 2.0);
  }
  else {
    return data[positions / 2];
  }
}

int get_distance(void) {
  uint8_t i;
  unsigned long *values, value, total_time = 0;
  values = (unsigned long *) malloc(sizeof(unsigned long) * DETECTION_READINGS);

  for (i=0; i < DETECTION_READINGS; i++) {
    digitalWrite(TRIGGER_PIN, LOW);
    delayMicroseconds(2);
    digitalWrite(TRIGGER_PIN, HIGH);
    delayMicroseconds(10);
    digitalWrite(TRIGGER_PIN, LOW);

    value = pulseIn(ECHO_PIN, HIGH, WAIT_TIME);
    if(value == 0){
      value = WAIT_TIME;
    }
    total_time += value; 
    values[i] = value;
  }

  value = get_median(values, DETECTION_READINGS);
  free(values);

  delayMicroseconds(WAIT_TIME * DETECTION_READINGS - total_time);

  return int(value * 0.034 / 2);
}

void move_servo(int angle){ 
 servo.writeMicroseconds(angle * SERVO_ANGLE_RATIO + SERVO_START);
}

void make_step(void) {
  move_servo(current_angle);
  Serial.print(current_angle);
  Serial.print(' ');
  Serial.println(get_distance());
  current_angle += angle_step;
  if (current_angle == MAX_ANGLE){
    angle_step = -1;
  }
  else if(current_angle == 0){
    angle_step = 1;
  }
}
