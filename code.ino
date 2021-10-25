
#include "I2Cdev.h"

#include "MPU6050_6Axis_MotionApps20.h"
//#include "MPU6050.h" // not necessary if using MotionApps include file

// Arduino Wire library is required if I2Cdev I2CDEV_ARDUINO_WIRE implementation
// is used in I2Cdev.h
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
#include "Wire.h"
#endif

// class default I2C address is 0x68
// specific I2C addresses may be passed as a parameter here
// AD0 low = 0x68 (default for SparkFun breakout and InvenSense evaluation board)
// AD0 high = 0x69
MPU6050 mpu;

/* =========================================================================
   NOTE: In addition to connection 3.3v, GND, SDA, and SCL, this sketch
   depends on the MPU-6050's INT pin being connected to the Arduino's
   external interrupt #0 pin. On the Arduino Uno and Mega 2560, this is
   digital I/O pin 2.
   ========================================================================= */


// uncomment "OUTPUT_READABLE_YAWPITCHROLL" if you want to see the yaw/
// pitch/roll angles (in degrees) calculated from the quaternions coming
// from the FIFO. Note this also requires gravity vector calculations.
// Also note that yaw/pitch/roll angles suffer from gimbal lock (for
// more info, see: http://en.wikipedia.org/wiki/Gimbal_lock)
#define OUTPUT_READABLE_YAWPITCHROLL


#define INTERRUPT_PIN 2  // use pin 2 on Arduino Uno & most boards
#define LED_PIN 13 // (Arduino is 13, Teensy is 11, Teensy++ is 6)
bool blinkState = false;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (default is 42 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[64]; // FIFO storage buffer

// orientation/motion vars
Quaternion q;           // [w, x, y, z]         quaternion container
VectorInt16 aa;         // [x, y, z]            accel sensor measurements
VectorInt16 aaReal;     // [x, y, z]            gravity-free accel sensor measurements
VectorInt16 aaWorld;    // [x, y, z]            world-frame accel sensor measurements
VectorFloat gravity;    // [x, y, z]            gravity vector
float ypr[3];           // [yaw, pitch, roll]   yaw/pitch/roll container and gravity vector


// ================================================================
// ===               INTERRUPT DETECTION ROUTINE                ===
// ================================================================

volatile bool mpuInterrupt = false;     // indicates whether MPU interrupt pin has gone high
void dmpDataReady() {
  mpuInterrupt = true;
}


// ================================================================
// ===                      INITIAL SETUP                       ===
// ================================================================
#define MAX_THRUST 170
#define START_THRUST 120
#define LEFT_MOTOR_OFFSET 12
#define RIGHT_MOTOR_OFFSET 12// 25
//#define ERROR_OFFSET -8   // za accSensitivity =- +-2g
#define ERROR_OFFSET -7   // za accSensitivity =- +-8g

#define PIN_POT_KP 0
#define PIN_POT_KD 1


const int pinMotorLeft = 11;
const int pinMotorRight = 10;

float error_prev, error_new, error_diff;
float time_prev, time_new, time_diff;
float thrustLeft;
float thrustRight;
float Kp, Ki, Kd;
float K;

void setup() {
  // join I2C bus (I2Cdev library doesn't do this automatically)
#if I2CDEV_IMPLEMENTATION == I2CDEV_ARDUINO_WIRE
  Wire.begin();
  Wire.setClock(100000); // 400kHz I2C clock. Comment this line if having compilation difficulties
#elif I2CDEV_IMPLEMENTATION == I2CDEV_BUILTIN_FASTWIRE
  Fastwire::setup(100, true);   // 100 intead of 400?
#endif

  Serial.begin(9600);

  // initialize device
  //Serial.println(F("Initializing I2C devices..."));
  mpu.initialize();
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_8); // set acceleration meter to +- 8
  pinMode(INTERRUPT_PIN, INPUT);

  // verify connection
  //Serial.println(F("Testing device connections..."));
  //Serial.println(mpu.testConnection() ? F("MPU6050 connection successful") : F("MPU6050 connection failed"));

  // wait for ready
  //Serial.println(F("\nSend any character to begin DMP programming and demo: "));
  /*while (Serial.available() && Serial.read()); // empty buffer
    while (!Serial.available());                 // wait for data
    while (Serial.available() && Serial.read()); // empty buffer again
  */

  delay(2000);
  // load and configure the DMP
  //Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();

  // make sure it worked (returns 0 if so)
  if (devStatus == 0) {
    // Calibration Time: generate offsets and calibrate our MPU6050
    //mpu.CalibrateAccel(10);
    //mpu.CalibrateGyro(10);
    //mpu.PrintActiveOffsets();
    /*
        mpu.setXGyroOffset(174);
        mpu.setYGyroOffset(-10);
        mpu.setZGyroOffset(56);

        mpu.setXAccelOffset(-1308);
        mpu.setYAccelOffset(-1783);
        mpu.setZAccelOffset(886);
    */
    //+-2g       X Accel  Y Accel  Z Accel   X Gyro   Y Gyro   Z Gyro
    //OFFSETS    -1308,   -1783,     886,     174,     -10,      56

    //+-8        X Accel  Y Accel  Z Accel   X Gyro   Y Gyro   Z Gyro
    //OFFSETS    -1332,   -1721,     894,     176,     -13,      55
    mpu.setXGyroOffset(176);
    mpu.setYGyroOffset(-13);
    mpu.setZGyroOffset(55);

    mpu.setXAccelOffset(-1332);
    mpu.setYAccelOffset(-1721);
    mpu.setZAccelOffset(894);

    // turn on the DMP, now that it's ready
    //Serial.println(F("Enabling DMP..."));
    mpu.setDMPEnabled(true);

    // enable Arduino interrupt detection
    //Serial.print(F("Enabling interrupt detection (Arduino external interrupt "));
    //Serial.print(digitalPinToInterrupt(INTERRUPT_PIN));
    //Serial.println(F(")..."));
    attachInterrupt(digitalPinToInterrupt(INTERRUPT_PIN), dmpDataReady, RISING);
    mpuIntStatus = mpu.getIntStatus();

    // set our DMP Ready flag so the main loop() function knows it's okay to use it
    //Serial.println(F("DMP ready! Waiting for first interrupt..."));
    dmpReady = true;

    // get expected DMP packet size for later comparison
    packetSize = mpu.dmpGetFIFOPacketSize();
  } else {
    // ERROR!
    // 1 = initial memory load failed
    // 2 = DMP configuration updates failed
    // (if it's going to break, usually the code will be 1)
    //Serial.print(F("DMP Initialization failed (code "));
    //Serial.print(devStatus);
    //Serial.println(F(")"));
  }

  // configure LED for output
  pinMode(LED_PIN, OUTPUT);
  pinMode(pinMotorLeft, OUTPUT);
  pinMode(pinMotorRight, OUTPUT);

  Kp = 0.15;
  Kd = 6.45;
  thrustLeft = START_THRUST;
  thrustRight = START_THRUST;
  error_prev = 0;
  time_prev = millis();
  time_new = time_prev;

  mpu.dmpGetQuaternion(&q, fifoBuffer);
  mpu.dmpGetGravity(&gravity, &q);
  mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
  error_new = ypr[2] * 180 / M_PI + ERROR_OFFSET;
}
void calculateK(float error, float error_diff, float time_diff) {

  K = Kp * error + Kd * (error_diff / time_diff);

  //Serial.print(F(", Kp : ")); //Serial.print(Kp);
  //Serial.print(F(", Kd : ")); //Serial.print(Kd);
}

void calculateThrust(float err) {

  if (fabs(err) > 2) { // uvedemo odstupanje

    thrustRight += K;
    thrustLeft -= K;


    if (thrustLeft < LEFT_MOTOR_OFFSET) {
      thrustLeft = LEFT_MOTOR_OFFSET;
    }
    if (thrustRight < RIGHT_MOTOR_OFFSET) {
      thrustRight = RIGHT_MOTOR_OFFSET;
    }
    if (thrustLeft > 255) {
      thrustLeft = 255;
    }
    if (thrustRight > 255) {
      thrustRight = 255;
    }

    //Serial.print(F(", lT : ")); //Serial.print(thrustLeft);
    //Serial.print(F(", rT : ")); //Serial.print(thrustRight);
  }
}


void setThrust(int thrustL, int thrustR) {

  int trueThrustL = map(thrustL, 0, 255, 0, MAX_THRUST);
  int trueThrustR = map(thrustR, 0, 255, 0, MAX_THRUST);
  analogWrite(pinMotorLeft, trueThrustL);
  analogWrite(pinMotorRight, trueThrustR);

  //Serial.print(F(", ML : ")); //Serial.print(trueThrustL);
  //Serial.print(F(", MR : ")); //Serial.print(trueThrustR);
}

float floatMap(float x, float in_min, float in_max, float out_min, float out_max) {
  return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


// ================================================================
// ===                    MAIN PROGRAM LOOP                     ===
// ================================================================

void loop() {
  // if programming failed, don't try to do anything
  if (!dmpReady) return;
  // read a packet from FIFO
  if (mpu.dmpGetCurrentFIFOPacket(fifoBuffer)) { // Get the Latest packet

#ifdef OUTPUT_READABLE_YAWPITCHROLL
    // display Euler angles in degrees
    mpu.dmpGetQuaternion(&q, fifoBuffer);
    mpu.dmpGetGravity(&gravity, &q);
    mpu.dmpGetYawPitchRoll(ypr, &q, &gravity);
    /*
        //Serial.print("ypr\t");
        //Serial.print(ypr[0] * 180 / M_PI);
        //Serial.print("\t");
        //Serial.print(ypr[1] * 180 / M_PI);
        //Serial.print("\t");
        //Serial.println(ypr[2] * 180 / M_PI);
    */

    //Kp = floatMap(analogRead(PIN_POT_KP), 0, 1023, 0, 2);    // changing the Kd value
    //Kd = floatMap(analogRead(PIN_POT_KD), 0, 1023, 0, 100);    // changing the Kd value

    error_prev = error_new;
    error_new = ypr[2] * 180 / M_PI + ERROR_OFFSET;
    //Serial.print(F("Error : ")); //Serial.print(error_new);
    time_new = millis();
    time_diff = time_new - time_prev;
    time_prev = time_new;

    error_diff = error_new - error_prev;
    //Serial.print(F(", dErr : ")); //Serial.print(error_diff);
    //Serial.print(F(", dTime : ")); //Serial.print(time_diff);

    calculateK(error_new, error_diff, time_diff);
    calculateThrust(error_new);
    setThrust(thrustLeft, thrustRight);

    //Serial.println(F(""));

    delay(20);
#endif

    // blink LED to indicate activity
    blinkState = !blinkState;
    digitalWrite(LED_PIN, blinkState);
  }


}
