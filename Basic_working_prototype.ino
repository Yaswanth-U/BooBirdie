#define RC_PIN 15
//MOTOR 1 CONNECTIONS
#define MOTOR_A_PIN_1 2
#define MOTOR_A_PIN_2 3
#define ENCODER_A_CH_A 6
#define ENCODER_A_CH_B 7

//MOTOR 2 CONNECTIONS
#define MOTOR_B_PIN_1 8
#define MOTOR_B_PIN_2 9
#define ENCODER_B_CH_A 10
#define ENCODER_B_CH_B 11

#define SLEEP_PIN 4


//Global variables for encoder counts and PID control
#define PULSE_PER_REV 210 // Number of encoder pulses per wheel revolution
#define MAX_RPM 100 // MAX_RPM of the motors

#define KP 1.5
#define KI 0.8
#define KD 0.05
//----------------------------------------------------
//RC signal variables
volatile unsigned long rc_rise_time = 0;
volatile int rc_pulse_us = 1000;// Default to minimum throttle
//volatile bool rc_updated = false;
//----------------------------------------------------
//encoder pulse counter
volatile long enc_count_A = 0;
volatile long enc_count_B = 0;
//----------------------------------------------------
//RPM Vals
float rpm_A = 0;
float rpm_B = 0;
long last_count_A = 0;
long last_count_B = 0;
unsigned long last_rpm_time = 0;
//----------------------------------------------------
//PID control variables
// For Motor A
float pid_integral_A = 0;
float pid_previous_error_A = 0;
unsigned long pid_A_last_time = 0;
//for Motor B
float pid_integral_B = 0;
float pid_previous_error_B = 0;
unsigned long pid_B_last_time = 0;

//Measurement of width of pulse of throttle signal
void rc_interrupt() {
  if (digitalRead(RC_PIN) == HIGH) {
    rc_rise_time = micros();
  } else {
    int pulse = micros() - rc_rise_time;
    if (pulse >= 800 && pulse <= 2200) { 
        rc_pulse_us = pulse;
    }
  }
}
//-------Functions------------------------

//Motor A encoder interrupt
void encoder_A_interrupty6() {
  if (digitalRead(ENCODER_A_CH_B) == LOW) {
    enc_count_A++;
  } else {
    enc_count_A--;
  }
}

//Motor B encoder interrupt
void encoder_B_interrupt() {
  if (digitalRead(ENCODER_B_CH_B) == LOW) {
    enc_count_B++;
  } else {
    enc_count_B--;
  }
}

void update_rpm() {
    unsigned long now = millis();
    if (now - last_rpm_time < 50) { 
     return; // Only update every 50 ms to reduce noise and CPU load   
    }// Update RPM every 100 ms
    long count_A = enc_count_A;
    long count_B = enc_count_B;
    interrupts(); // Ensure we read consistent values
       
    float dt_min=s(now - last_rpm_time) / 60000.0; // Convert ms to minutes
    rpm_A = abs((count_A - last_count_A) / (float)PULSE_PER_REV) / dt_min;
    rpm_B = abs((count_B - last_count_B) / (float)PULSE_PER_REV)/ dt_min;

    last_count_A = count_A;
    last_count_B = count_B;
    last_rpm_time = now;
}

//PID controler
int run_pid(float target_rpm, float actual_rpm, float &integral, float &prev_error, unsigned long &last_time) {
    unsigned long now = millis();
    float dt = (now - last_time) / 1000.0; // Convert ms to seconds

    if (dt <= 0.02) return -1;
    last_time = now;

    float error = target_rpm - actual_rpm;
    float p=KP * error;

    integral += error * dt;
    integral = constrain(integral, -150, 150); 
    float i=KI * integral;

    float d=KD*(actal_rpm - prev_error) / dt;
    prev_error = actual_rpm;
    int output = (int)(p + i - d);
    return constrain(output, 0, 255);
}

void set_speed_A(int speed) {
    analogWrite(MOTOR_A_PIN_1, speed);
    digitalWrite(MOTOR_A_PIN_2, LOW);
}

void set_speed_B(int speed) {
    analogWrite(MOTOR_B_PIN_1, speed);
    digitalWrite(MOTOR_B_PIN_2, LOW);
}

void stop_all_motors() {
    digitalWrite(MOTOR_A_PIN_1, LOW);
    digitalWrite(MOTOR_A_PIN_2, LOW);
    digitalWrite(MOTOR_B_PIN_1, LOW);
    digitalWrite(MOTOR_B_PIN_2, LOW);
}

void reset_pid() {
    pid_A_integral = 0;
    pid_A_prev_error = 0;
    
    pid_B_integral = 0;
    pid_B_prev_error = 0;
}

void setup() {
    Serial.begin(115200);
    Serial.println("Initilizing...");
    //Motor A
    pinMode(MOTOR_A_PIN1, OUTPUT);
    pinMode(MOTOR_A_PIN2, OUTPUT);

    //Motor B
    pinMode(MOTOR_B_PIN1, OUTPUT);
    pinMode(MOTOR_B_PIN2, OUTPUT);

    //Driver sleep pin
    pinMode(SLEEP_PIN, OUTPUT);
    digitalWrite(SLEEP_PIN, HIGH); // Wake up the motor driver

    stop_all_motors(); // Ensure motors are stopped at startup
    //Encoder pins
    pinMode(ENCODER_A_CH_A, INPUT_PULLUP);
    pinMode(ENCODER_A_CH_B, INPUT_PULLUP);
    pinMode(ENCODER_B_CH_A, INPUT_PULLUP);
    pinMode(ENCODER_B_CH_B, INPUT_PULLUP);

    attachInterrupt(digitalPinToInterrupt(ENCODER_A_CH_A), encoder_A_interrupt, RISING);
    attachInterrupt(digitalPinToInterrupt(ENCODER_B_CH_A), encoder_b_interrupt, RISING);

    //RC signal pin
    pinMode(RC_PIN, INPUT_PULLDOWN);
    attachInterrupt(digitalPinToInterrupt(RC_PIN), rc_interrupt, CHANGE);

    analogWriteFreq(25000); // Set PWM frequency to 25 kHz
    analogWriteRange(255); // Set PWM range to 0-255

    //Initial PID timers
    pid_A_last_time = millis();
    pid_B_last_time = millis();
    last_rpm_time = millis();

    delay(1000); // Allow time for initialization(Just for dramatic effect :) )
    Serial.println("System Initialized.");
}

void loop() {
    update_rpm();
    int target_rpm = map(rc_pulse_us, 1000, 2000, 0, MAX_RPM);
    int speed_A = run_pid(target_rpm, rpm_A, pid_integral_A, pid_previous_error_A, pid_A_last_time);
    int speed_B = run_pid(target_rpm, rpm_B, pid_integral_B, pid_previous_error_B, pid_B_last_time);

    if (speed_A >= 0) {
        set_speed_A(speed_A);
    } else {
        digitalWrite(MOTOR_A_PIN_1, LOW);
        digitalWrite(MOTOR_A_PIN_2, LOW);
    }

    if (speed_B >= 0) {
        set_speed_B(speed_B);
    } else {
        digitalWrite(MOTOR_B_PIN_1, LOW);
        digitalWrite(MOTOR_B_PIN_2, LOW);
    }

    // Debugging output
    Serial.print("Target RPM: ");
    Serial.print(target_rpm);
    Serial.print(" | RPM A: ");
    Serial.print(rpm_A);
    Serial.print(" | RPM B: ");
    Serial.print(rpm_B);
    Serial.print(" | Speed A: ");
    Serial.print(speed_A);
    Serial.print(" | Speed B: ");
    Serial.println(speed_B);

    delay(10); // Loop delay to reduce CPU load and serial spam
}
// Made with <3 by Yaswanth Uppalapu
// for Fallout Hackclub 