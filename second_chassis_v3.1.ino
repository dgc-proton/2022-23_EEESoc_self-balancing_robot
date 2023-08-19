/*
********************************************
*** Dave & Anoop Robot "second chassis"  ***
********************************************
*History:
*v1.0      23.03.23 Taking code across from "alt2" version one part at a time, testing and altering as necessary. Competition is next week.
*v1.5      24.03.23 Fixed bug where right wheel wouldn't move - turns out that the ISR was using the timer needed for all PWM pins past pin6.
*v2.0      24.03.23 Robot struggling to balance - wheels too agressive so changed how motor stall & motor speed are implemented, added coasting for motors,
*                   moved set_motor_speed to ISR. Robot can now balance close to indefinately on some ocassions.
*v2.5      25.03.22 Implementing IR control of robot - IRreceiver needs a hardware timer but all are in use, so changing to IRsmallDecoder library using interupt pin.
*                   Found further conflicts between IRsmallDecoder, and both the ISR and the MPU. Modified libraries to resolve.
*v2.6      25.03.23 Now making changes so that turning works better. Moved motot routine call back to main code loop.
*v2.7      26.03.23 Eliminated PID errorsum contrain, coast angle & all motorMin. Tested different MPU chips but all had similar (or worse) drift. Max drift on
*                   this chip ~0.7 deg. Changed PID so that Ki applies to whole sum; several sources site this as easier to manually tune.
*v2.8      28.03.23 Tidied up unused code. Modified wheels with rubber bands. Implemented 'window' (FIFO size 20) for errorSum to limit history. Reverted Ki & Kd calcs
*                   back to origional KJ style (keeping the adjustments made since v1.0 above, but reverting changes made prior to this).
*v3.0      28.03.23 Now balances very well (major contributors since v2.7 being PID algorithm changes & implementing an errorSum 'window', PID values: Kp 12.4, Ki 4.8
*                   Kd 0.05). Now improving PID, turning & forwards tuning. Added map() to compensate for motor  deadband, and a momentary forward lean ability.
*v3.1      29.03.23 Reverted PID back to seperate KpValue, KiValue and KdValue. Increased errorSum window size.
********************************
*
*Inbuilt LED will flash a couple of times quickly after power-on to indicate successful initialisation. If
*unsucessfull it will continually flash and the program will not run.
*ributing
*===> Don't forget to turn off debug statements when not in use! <===
*
*/


/*
***************************************
************  Tuning Area  ************
***************************************
*/
// PWM correction factors (to ensure mototrs run the same speed) - one should be 1, the other <1
#define LMOTOR_CORRECTION_FACTOR 0.78 // !Max value is to be 1
#define RMOTOR_CORRECTION_FACTOR 1.0 // !Max value is to be 1
// Motor stall value, used to adjust motor values accounting for deadband
#define MOTORMIN 60 //68
// Turning factor (needs to be > 1)
#define T_FACTOR 1.35
// Value to add to targetAngle for momentary forward lean
#define FORWARD_LEAN 0.22
// definitions for PID constants & sample time
#define Kp 16.0 //12.2 
#define Ki 12.5//12.5 //9.0 
#define Kd 0.68 //0.68 
#define sampleTime 0.005
//*************************************


// MPU
#define OFFSETS -1526,    1252,    1610,     226,       2,      34  // newer chip 1554,     996,    1926,      95,      11,      26 // older chip: -1526,    1252,    1610,     226,       2,      34
#define MPU6050_DEFAULT_ADDRESS 0x68
#define Three_Axis_Quaternions 3
#define Six_Axis_Quaternions 6  // Default

// left motor definitions
#define LMOTOR_PWM_PIN   3
#define LMOTOR_DIR_PIN_1 4 
#define LMOTOR_DIR_PIN_2 6
// right motor definitions
#define RMOTOR_DIR_PIN_1 7 
#define RMOTOR_DIR_PIN_2 8
#define RMOTOR_PWM_PIN   5

// IR receiver protocol
#define IR_SMALLD_NEC

// Header files & global object creation
#include <math.h>
#include <Arduino.h>
#include <DMP_Image.h>
#include <MPU_ReadMacros.h>
#include <MPU_WriteMacros.h>
#include <Simple_MPU6050.h> // Note: commented out line 35 to eliminate conflict with IRsmallDecoder (load this header file first)
Simple_MPU6050 mpu(Three_Axis_Quaternions); // Setup mpu object
#include <IRsmallDecoder.h> 
IRsmallDecoder irDecoder(2); // setup IR receiving object for pin 2 (needs to be a digital interrupt pin)

// Global variables
volatile float targetAngle = 1.22, prevAngle = 0.0, currentAngle = 0.0; // degrees (target 1.22)
volatile float KpValue, KiValue, KdValue;
volatile float error = 0.0, prevError = 0.0, errorSum = 0.0;
volatile float error_queue[40] = {0}; // for tracking the errorSum 'window' 
volatile int error_queue_counter = 0; // for tracking the errorSum 'window'
float euler[3];         // [psi, theta, phi]    Euler angle container
float eulerDEG[3];         // [psi, theta, phi]    Euler angle degrees container
volatile int motorPower;
static int motorDirection = 1, LmotorP = 0, RmotorP = 0;
static int powerValue = 0, forwardLean = 0;
static float LmotorTurn = 1, RmotorTurn = 1;
irSmallD_t irData; // for IRsmallDecoder

// Function prototypes
void init_interrupt(void);
void setup(void);
void init_MPU(void);
void setMotorSpeed(void);
void convert_readings(int16_t *gyro, int16_t *accel, int32_t *quat);
void read_ir(void);


/******************************
***  Setup code, runs once  ***
*******************************/
void setup() {  
  // Serials comms
  Serial.begin(9600);

  // Set motor pins as output
  pinMode(LMOTOR_DIR_PIN_1, OUTPUT);
  pinMode(LMOTOR_DIR_PIN_2, OUTPUT);
  pinMode(LMOTOR_PWM_PIN, OUTPUT); 
  pinMode(RMOTOR_DIR_PIN_1, OUTPUT);
  pinMode(RMOTOR_DIR_PIN_2, OUTPUT);
  pinMode(RMOTOR_PWM_PIN, OUTPUT); 

  // set inbuilt LED pin (D13) as output to show when MPU initialised
  pinMode(LED_BUILTIN, OUTPUT);

  // Initialise MPU6050 inc DMP
  init_MPU();

  // set motors in forward direction
  digitalWrite(LMOTOR_DIR_PIN_1, LOW);
  digitalWrite(LMOTOR_DIR_PIN_2, HIGH);
  digitalWrite(RMOTOR_DIR_PIN_1, LOW);
  digitalWrite(RMOTOR_DIR_PIN_2, HIGH);

  // Setup ISR
  init_interrupt();  

  // blink LED to show completed
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
  delay(100);
  digitalWrite(LED_BUILTIN, HIGH);
  delay(100);
  digitalWrite(LED_BUILTIN, LOW);
}


/***********************************
***  Main code, runs repeatedly  ***
***********************************/
void loop() {
  // Get readings from mpu DMP; function will poll data then call convert_readings routine  
  mpu.dmp_read_fifo(false);
  // Motor routine 
  setMotorSpeed(); 

  // Cancel any forward lean, then check if any IR commands have been received and action them.
  // Only done once per loop because less time critical.
  if (forwardLean) {
    forwardLean = 0;
    targetAngle -= FORWARD_LEAN;
  }
  read_ir();
  // Add any forward lean
  if (forwardLean) {
    targetAngle += FORWARD_LEAN;
  }

  // DEBUGGING  (for use with Python serial reading & plotting script)
  //Serial.println(String(millis()) + "," + String(currentAngle) + "," + String(prevAngle) + "," + String(error) + "," + String(errorSum)
  //               + "," + String(KpValue) + "," + String(KiValue) + "," + String(KdValue) + "," + String(powerValue));
}


/****************************************
***  Control motor speed & direction  ***
****************************************/
void setMotorSpeed(void)
{
  // Set motor direction according to current and target angles
  if (currentAngle > targetAngle && motorDirection != 1)
  {
    // set motors in forward direction
    digitalWrite(LMOTOR_DIR_PIN_1, LOW);
    digitalWrite(LMOTOR_DIR_PIN_2, HIGH);
    digitalWrite(RMOTOR_DIR_PIN_1, LOW);
    digitalWrite(RMOTOR_DIR_PIN_2, HIGH);
    motorDirection = 1;
  }
  else if (currentAngle < targetAngle && motorDirection != 0)
  {
    // set motors in the reverse direction
      digitalWrite(LMOTOR_DIR_PIN_1, HIGH);
      digitalWrite(LMOTOR_DIR_PIN_2, LOW);  
      digitalWrite(RMOTOR_DIR_PIN_1, HIGH);
      digitalWrite(RMOTOR_DIR_PIN_2, LOW);
      motorDirection = 0;          
  }  

  // Set powervalue
  powerValue = (abs(motorPower));
  powerValue = constrain(powerValue, 0, 170); // constrained to 170 to allow for motor turning factor of upto 50%
  powerValue = map(powerValue, 0, 170, MOTORMIN, 170); // adjust to account for motor stall value
  // If robot falling over cut power to motors
  if (abs(currentAngle) > 50) {
    powerValue = 0;
  }
  LmotorP = ceil((powerValue) * LMOTOR_CORRECTION_FACTOR);
  RmotorP = ceil((powerValue) * RMOTOR_CORRECTION_FACTOR);
  
  // Update powervalue output to motors
  if (motorDirection == 1) {
    analogWrite(LMOTOR_PWM_PIN, ceil(LmotorP * LmotorTurn));
    analogWrite(RMOTOR_PWM_PIN, ceil(RmotorP * RmotorTurn));
  } else {
    analogWrite(LMOTOR_PWM_PIN, LmotorP);
    analogWrite(RMOTOR_PWM_PIN, RmotorP);
  }  
}


/**********************************************************
***  Interrupt Service Routine - calculates motorPower  ***
**********************************************************/
ISR(TIMER1_COMPA_vect)
{
    prevAngle = currentAngle;
    prevError = error; 
    currentAngle = (float) eulerDEG[2];
    
    // calculate error for a window of 40 readings
    error = currentAngle - targetAngle;
    errorSum += error;
    errorSum -= error_queue[error_queue_counter];
    error_queue[error_queue_counter] = error;
    error_queue_counter += 1;
    if (error_queue_counter > 39){
        error_queue_counter = 0;
    }

    //calculate output from Kp, Ki and Kd values
    KpValue = Kp * error;
    KiValue = Ki * errorSum * sampleTime;
    KdValue = Kd * ((currentAngle - prevAngle) / sampleTime);
    motorPower = (KpValue + KiValue + KdValue);
}

/************************************************************************************************
***  Check if IR sensor data available and carry out instructions if they have been received  ***
************************************************************************************************/
void read_ir(void)
{
  // Carry out jobs only if command has been received
  if (irDecoder.dataAvailable(irData)) {
    // Perform actions according to the received command
    if (irData.cmd == 0x46) {
        //Serial.println("up");
        targetAngle += 0.05;        
    } else if (irData.cmd == 0x15) {
        //Serial.println("down");
        targetAngle -= 0.05;        
    } else if (irData.cmd == 0x40) {
        //Serial.println("ok");
        LmotorTurn = 1.0;
        RmotorTurn = 1.0;
    } else if (irData.cmd == 0x44) {
        //Serial.println("left");
        LmotorTurn = 1.0;
        RmotorTurn = T_FACTOR; // Right motor turns faster when going forwards
    } else if (irData.cmd == 0x43) {
        //Serial.println("right");
        RmotorTurn = 1.0;
        LmotorTurn = T_FACTOR; // Left motor turns faster when going forwards
    } else if (irData.cmd == 0x19) {
        //Serial.println("2");
        targetAngle = 0.0;
    } else if (irData.cmd == 0x16) {
        //Serial.println("1");
        forwardLean = 1;
    }
  }
}


/********************************************
***  Convert DMP readings to usable data  ***
********************************************/
void convert_readings(int16_t *gyro, int16_t *accel, int32_t *quat)
{
    Quaternion q;
    VectorFloat gravity;
    float ypr[3] = { 0, 0, 0 };
    mpu.GetQuaternion(&q, quat);  
    mpu.GetGravity(&gravity, &q);
    mpu.GetEuler(euler, &q);
    mpu.ConvertToDegrees(euler, eulerDEG);
}


/******************
***  Setup ISR  ***
******************/
void init_interrupt(void)
{
    cli();                      // disable global interrupts
    TCCR1A = 0;                 // set entire TCCR1A register to 0
    TCCR1B = 0;                 // same for TCCR1B  
    OCR1A = 9999;               // set timer for 5ms
    TCCR1B |= (1 << WGM12);     // turn on CTC mode
    TCCR1B |= (1 << CS11);      // Set CS11 bit for prescaling by 8
    TIMSK1 |= (1 << OCIE1A);    // enable timer compare interrupt
    sei();                      // enable global interrupts
}


/************************
***  Setup MPU & DMP  ***
************************/
void init_MPU(void)
{
    mpu.begin();
    mpu.Set_DMP_Output_Rate_Hz(200);
    mpu.SetAddress(MPU6050_DEFAULT_ADDRESS); 
    mpu.setOffset(OFFSETS);
    mpu.CalibrateGyro();
    mpu.CalibrateAccel();
    mpu.load_DMP_Image();
    mpu.on_FIFO(convert_readings);
}
