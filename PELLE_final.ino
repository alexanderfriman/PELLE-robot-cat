#include <Arduino.h>
#include <Wire.h>
#include <PS4Controller.h>
#include <Adafruit_PWMServoDriver.h>
#include <TCA9548.h>
#include <AS5600.h>

// ==========================
// ======= CONSTANTS ========
// ==========================
#define NUM_MOTORS 8
#define NUM_ENCODERS 8
#define NUM_LEGS 4

const float l1 = 100.0;
const float l2 = 150.0;

float x = 0;
float y = -200;

const float IDLE_X = 0;
const float IDLE_Y = -200;

const float CROUCH_Y = -160;

const float STEP_TIME_MIN = 2.0;  // fastest (seconds per step)
const float STEP_TIME_MAX = 2.0;  // slowest
const float DUTY = 0.7; // % on ground

// ==========================
// ======= MOTOR PINS =======
// ==========================
const int IN1[NUM_MOTORS] = {12,27,32,14,4,25,16,19};
const int IN2[NUM_MOTORS] = {13,33,15,21,5,26,17,18};
const int PWM_CH[NUM_MOTORS] = {0,1,2,3,4,5,6,7};

// ==========================
// ==== HARDWARE OBJECTS ====
// ==========================
Adafruit_PWMServoDriver pwm(0x40);
TCA9548 tca(0x70, &Wire);
AS5600 encoders[NUM_ENCODERS];

// ==========================
// ===== MOTOR CONTROL ======
// ==========================
void driveMotor(uint8_t index, int speed)
{
    speed = constrain(speed, -255, 255);

    if (speed > 0)
    {
        digitalWrite(IN1[index], HIGH);
        digitalWrite(IN2[index], LOW);
    }
    else if (speed < 0)
    {
        digitalWrite(IN1[index], LOW);
        digitalWrite(IN2[index], HIGH);
        speed = -speed;
    }
    else
    {
        digitalWrite(IN1[index], LOW);
        digitalWrite(IN2[index], LOW);
    }

    uint16_t pwmValue = map(speed, 0, 255, 0, 4095);
    pwm.setPWM(PWM_CH[index], 0, pwmValue);
}

// ==========================
// === INVERSE KINEMATICS ===
// ==========================
void IK(float x, float y, float &theta1, float &theta2)
{
    float r = sqrt(x*x + y*y);
    if (r < 0.001f) r = 0.001f;

    float c = (l1*l1 + r*r - l2*l2) / (2*l1*r);
    c = constrain(c, -1, 1);

    float alpha = acos(c);
    float beta = -atan2(y, x)-PI/2;

    theta1 = PI/2 - (beta + alpha);
    theta2 = PI/2 - (alpha - beta);
}

// ==========================
// ===== STEP TRAJECTORY ====
// ==========================
float getStepTime(float period)
{
    return fmod(millis() / 1000.0 * (2*PI / period), 2*PI);
}
void ellipseStep(float t, float &x, float &y)
{
    float a = 40.0;     // step length
    float b = 35.0;     // step height
    float cx = 0.0;
    float cy = -200.0;

    // normalize t → 0..2π
    while (t > 2*PI) t -= 2*PI;
    while (t < 0) t += 2*PI;

    float phase = t / (2*PI);  // 0 → 1

    if (phase < DUTY) // On ground
    {
        float s = phase / DUTY;  // 0 → 1

        x = cx + a * (1 - 2*s);  // move backward
        y = cy;                  // on ground
    }
    else // In air
    {
        float s = (phase - DUTY) / (1 - DUTY); // 0 → 1

        x = cx + a * (-1 + 2*s);   // forward

        // smooth lift (sinusoidal arc)
        y = cy + b * sin(s * PI);
    }
}

// ==========================
// ===== ENCODER READ =======
// ==========================
const float EN_OFFSET[NUM_ENCODERS] = {5.580, 5.500, 5.630, 0.540, 2.200, 6.000, 2.170, 6.000};

float readEncoderRad(uint8_t index)
{
    tca.selectChannel(index);

    uint16_t raw = encoders[index].readAngle();
    float angle = (raw / 4095.0f) * 2.0f * PI;

    angle = angle - EN_OFFSET[index];

    if (angle < 0) angle += 2.0f * PI;

    return angle;
}

// ==========================
// ==== MOVEMENT CONTROL ====
// ==========================
float angleDiff(float target, float current)
{
    float diff = target - current;

    while (diff > PI) diff -= 2*PI;
    while (diff < -PI) diff += 2*PI;

    return diff;
}
void moveToPos(float x, float y, int index)
{
    int index1 = 2 * index;
    int index2 = 2 * index + 1;

    float angle1 = readEncoderRad(index1);
    float angle2 = readEncoderRad(index2);

    float theta1, theta2;
    IK(x, y, theta1, theta2);

    float diff1 = angleDiff(theta1, angle1);
    float diff2 = angleDiff(theta2, angle2);

    static float lastDiff[NUM_MOTORS] = {0};

    float d1 = diff1 - lastDiff[index1];
    float d2 = diff2 - lastDiff[index2];

    float nearZone = 0.05;

    float Kp_down = 90.0;
    float Kd_down = 40.0;
    float Kd_lift = 20.0;   // damping while lifting

    int speed1, speed2;

    //Joint 1
    if (diff1 > 0) // lifting
    {
        float scale = constrain(abs(diff1) / nearZone, 0.0, 1.0);

        speed1 = (230 - Kd_lift * d1) * scale;
    }
    else // lowering
    {
        speed1 = Kp_down * diff1 + Kd_down * d1;
    }

    //Joint 2
    if (diff2 > 0)
    {
        float scale = constrain(abs(diff2) / nearZone, 0.0, 1.0);
        speed2 = (255 - Kd_lift * d2) * scale;
    }
    else
    {
        speed2 = Kp_down * diff2 + Kd_down * d2;
    }

    int hold = 50; // Constant torque

    if (speed1 > 10) speed1 += hold;
    if (speed1 < -10) speed1 -= hold;

    if (speed2 > 10) speed2 += hold;
    if (speed2 < -10) speed2 -= hold;

    speed1 = constrain(speed1, -255, 255);
    speed2 = constrain(speed2, -255, 255);

    driveMotor(index1, speed1);
    driveMotor(index2, speed2);

    lastDiff[index1] = diff1;
    lastDiff[index2] = diff2;
}

int applyDeadzone(int value, int dz) {
    if (abs(value) < dz) return 0;
    return value;
}

// ==========================
// ========= SETUP ==========
// ==========================
void setup()
{
    Serial.begin(115200);

    Wire.begin();

    PS4.begin("e8:9e:b4:ad:8b:4e");

    pwm.begin();
    pwm.setPWMFreq(1600);

    for (int i = 0; i < NUM_MOTORS; i++)
    {
        pinMode(IN1[i], OUTPUT);
        pinMode(IN2[i], OUTPUT);
    }

    tca.begin();

    for (int i = 0; i < NUM_ENCODERS; i++)
    {
        tca.selectChannel(i);
        encoders[i].begin();
    }

    Serial.println("Robot Cat Started");
}

// ==========================
// ========= LOOP ===========
// ==========================
void loop()
{
    //PS4 Controller
    int stickY = 0;
    int deadzone = 10;
    bool currentCross = false;
    static bool lastCross = false;

    if (PS4.isConnected())
    {
        stickY = applyDeadzone(PS4.RStickY(), deadzone);
        currentCross = PS4.Cross();

    }
    if (currentCross && !lastCross) {
        Serial.println("X PRESSED");
    }
    lastCross = currentCross;

    //Movement
    float input = abs(stickY) / 128.0;   // normalize 0 → 1
    float stepTime = STEP_TIME_MAX - input * (STEP_TIME_MAX - STEP_TIME_MIN);
    float t = getStepTime(stepTime); // stage 0-2pi

    float direction = (stickY >= 0) ? 1.0 : -1.0;

    float phaseOffsets[4] = {0, PI/2, PI, 3*PI/2};
    float sideOffsets[4] = {1.0, 1.0, -1.0, -1.0};

    bool isCrouching = currentCross;
    bool isMoving = (stickY != 0) && !isCrouching;

    for(int i = 0; i < NUM_LEGS; i++){
        float lx, ly;

        if (isCrouching)
        {
            //Croutch
            lx = IDLE_X;
            ly = CROUCH_Y;
        }
        else if (isMoving)
        {
            //Walk
            float phase = phaseOffsets[i];
            float side = sideOffsets[i];
            ellipseStep(side * (direction * t + phase), lx, ly);
        }
        else
        {
            //Idle
            lx = IDLE_X;
            ly = IDLE_Y;
        }

        moveToPos(lx, ly, i);
    }

    //Printing
    static unsigned long lastPrint = 0;
    if (millis() - lastPrint > 200) {
        lastPrint = millis();
        Serial.print("Encoders: ");

        for (int i = 0; i < NUM_ENCODERS; i++)
        {
            float angle = readEncoderRad(i);

            
            Serial.print(angle, 3);
            Serial.print("  ");
        }

        Serial.println();

        Serial.print("x: ");
        Serial.print(x);
        Serial.print(" , y: ");
        Serial.println(y);

        float t1, t2;
        IK(x, y, t1, t2);
        Serial.print("Theta1: ");
        Serial.print(t1);
        Serial.print(" , Theta2: ");
        Serial.println(t2);
    }
}

