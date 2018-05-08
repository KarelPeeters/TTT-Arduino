/*                    Servo     Servo2     ServoTimer2      PWMServo
 * SoftwareSerial       59         59           59              59
 * AltSoftSerial        59         59           59              59
 * NeoSWSerial          59         59           59              5
 */

#define LOGBT
//#define LOGSTEP

#include <Arduino.h>
#include <Servo.h>

#define MSG_SPEED 0
#define MSG_STEPPER 1

#define p_left_motor   6      //D pwm
#define p_right_motor  5      //D pwm
#define p_servo        8
#define p_stepper_step 4      //D
#define p_stepper_dir  2      //D

#define ir_threshold 512
#define p_first_ir A0
#define p_second_ir A1

#define bluetooth Serial

Servo servo;

void setup() {
    bluetooth.begin(9600);

    pinMode(p_left_motor, OUTPUT);
    pinMode(p_right_motor, OUTPUT);
    pinMode(p_servo, OUTPUT);
    pinMode(p_stepper_step, OUTPUT);
    pinMode(p_stepper_dir, OUTPUT);

    servo.attach(p_servo);
}

int currentStepperPos = 300;
int targetStepperPos = 300;
int stepperDelay = 500;
long lastStepTaken = 0;
long lastStepPosSent = 0;
boolean lastStepWrite = false;

void handleSpeedTest();

void handleStepper() {
    int delta = targetStepperPos - currentStepperPos;
    int dir;
    if (delta > 0) {
        dir = 1;
        digitalWrite(p_stepper_dir, HIGH);
    } else if (delta < 0) {
        dir = -1;
        digitalWrite(p_stepper_dir, LOW);
    } else {
        return;
    }


    long curr = micros();
    if (curr - lastStepTaken >= stepperDelay) {
        if (lastStepWrite) {
            currentStepperPos += dir;

            if (curr - lastStepPosSent >= 10000 || targetStepperPos == currentStepperPos) {
                lastStepPosSent = curr;
                bluetooth.write(MSG_STEPPER);
                bluetooth.write(currentStepperPos);
                bluetooth.write(currentStepperPos >> 8u);
            }
        }

        lastStepTaken = curr;
        digitalWrite(p_stepper_step, (uint8_t) (!lastStepWrite));
        lastStepWrite = !lastStepWrite;
    }
}

bool firstDetect = false;
bool lastDetect = false;
unsigned long firstDetectMilis = 0;

void handleSpeedTest() {
    int first = analogRead(p_first_ir);
    int second = analogRead(p_second_ir);
    unsigned long time = millis();

    if (first < ir_threshold) {
        if (!firstDetect) {
            firstDetect = true;
            firstDetectMilis = time;
        }
    } else {
        firstDetect = false;
    }

    if (second < ir_threshold) {
        if (!lastDetect) {
            lastDetect = true;
            auto delta = (int) (time - firstDetectMilis);

            bluetooth.write(MSG_SPEED);
            bluetooth.write(delta);
            bluetooth.write(delta >> 8u);
        }
    } else {
        lastDetect = false;
    }
}

void loop() {
    while (bluetooth.available() >= 7) {
        int lm = bluetooth.read();
        int rm = bluetooth.read();
        int serv = bluetooth.read();

        stepperDelay = bluetooth.read() + (((uint16_t) bluetooth.read()) << 8u);
        targetStepperPos = bluetooth.read() + ((uint16_t ) (bluetooth.read()) << 8u);

        analogWrite(p_left_motor, lm);
        analogWrite(p_right_motor, rm);
        servo.write(serv);
    }
    handleStepper();
    handleSpeedTest();
}