// B-ROBOT  SELF BALANCE ROBOT WITH STEPPER MOTORS
// WITA Board (Arduino Leonardo + Wifi module board)
// MPU6050 using DMP processor
// Author: Jose Julio
// Date: 04/10/2013
// Updated: 18/10/2013
// License: GPL v2

// Angle calculations and control part is running at 200Hz from DMP solution
// DMP is using the gyro_bias_no_motion correction method.
// The board needs at least 10-15 seconds to give good values...

// STEPPER MOTOR PINS
// ENABLE PIN: D4
// STEP Motor1: D6 -> PORTD,7
// DIR  Motor1: D8 -> PORTB,4
// STEP Motor2: D12-> PORTD,6
// DIR  Motor2: D13-> PORTC,7
// To control the stepper motors we use Timer1 interrupt running at 25Khz. We control the speed of the motors
// Note: We could not use WITA led because pin 13 is already used for stepper motor control

// Robot servo arm connected to D5
// Distance sensor (sonar) connected to A0 (analog 0)
// Battery monitor (voltage divider) connected to A5

// We use a standard PID control for robot stability
// We have a P control for speed control and a PD control for stability (robot angle)
// 		The output of the control (motor speed) is integrated so it´s really an acceleration

// We control the robot from a WIFI module using OSC standard UDP messages (OSCmini library)
//    fadder1: Throttle 
//    fadder2: Steering 
//    push1: Move arm (and robot raiseup)
//    push2: push up
//    toggle1: autonomous mode
//    PAGE2: PID adjustements

// Robot autonomous mode
//   the robot start walking until reach an obstacle. When find an obstacle, start steering until it find a free way and continue walking


#include <WITA.h> 
#include <OSCmini.h>
#include <Wire.h>
#include <I2Cdev.h>
#include <JJ_MPU6050_DMP_6Axis.h>  // Modified version of the library to work with DMP (see comments inside)

#define DEBUG 0
#define SHUTDOWN_WHEN_BATTERY_OFF 1

#define CLR(x,y) (x&=(~(1<<y)))
#define SET(x,y) (x|=(1<<y))

#define ZERO_SPEED 65535
#define MAX_ACCEL 7

#define MAX_THROTTLE 530
#define MAX_STEERING 136
#define MAX_TARGET_ANGLE 12

// PRO MODE = MORE AGGRESSIVE
#define MAX_THROTTLE_PRO 650
#define MAX_STEERING_PRO 240 
#define MAX_TARGET_ANGLE_PRO 18


//#define I2C_SPEED 100000L
#define I2C_SPEED 400000L
//#define I2C_SPEED 800000L

#define ACCEL_SCALE_G 8192             // (2G range) G = 8192
#define ACCEL_WEIGHT 0.01
#define GYRO_BIAS_WEIGHT 0.005

// MPU6000 sensibility   (0.0609 => 1/16.4LSB/deg/s at 2000deg/s, 0.03048 1/32.8LSB/deg/s at 1000deg/s)
#define Gyro_Gain 0.03048
#define Gyro_Scaled(x) x*Gyro_Gain //Return the scaled gyro raw data in degrees per second

#define RAD2GRAD 57.2957795
#define GRAD2RAD 0.01745329251994329576923690768489

// Default control terms   
#define KP 0.20 // 0.22        
#define KD 26   // 30 28        
#define KP_THROTTLE 0.065  //0.08
#define KI_THROTTLE 0.05

// Control gains for raiseup
#define KP_RAISEUP 0.16
#define KD_RAISEUP 40
#define KP_THROTTLE_RAISEUP 0  // No speed control on raiseup
#define KI_THROTTLE_RAISEUP 0.0

#define SERVO_AUX_NEUTRO 1470

#define ITERM_MAX_ERROR 40   // Iterm windup constants
#define ITERM_MAX 5000

#define OBSTACLE_DISTANCE_MIN 36
#define WALK_DISTANCE_MIN 76

#define BATTERY_WARNING 110  // (11 volts) aprox
#define BATTERY_SHUTDOWN 104 // (10.4 volts)

bool Robot_shutdown=false;   // Robot shutdown flag => Out of battery
uint8_t mode;   // 0: MANUAL MODE   1: autonomous MODE
uint8_t autonomous_mode_status;   // 1: NORMAL STATUS=Walking  2: 
int16_t autonomous_mode_counter;  
int16_t autonomous_mode_distance;
int16_t pushUp_counter;  // for pushUp functionality (experimental)

int16_t distance_sensor;
int16_t battery;

// MPU control/status vars
bool dmpReady = false;  // set true if DMP init was successful
uint8_t mpuIntStatus;   // holds actual interrupt status byte from MPU
uint8_t devStatus;      // return status after each device operation (0 = success, !0 = error)
uint16_t packetSize;    // expected DMP packet size (for us 18 bytes)
uint16_t fifoCount;     // count of all bytes currently in FIFO
uint8_t fifoBuffer[18]; // FIFO storage buffer
Quaternion q;

uint8_t loop_counter;       // To generate a medium loop 40Hz 
uint8_t slow_loop_counter;  // slow loop 2Hz
long timer_old;
long timer_value;
int debug_counter;
float dt;

// class default I2C address is 0x68
MPU6050 mpu;

float angle_adjusted;
float angle_adjusted_Old;

float Kp=KP;
float Kd=KD;
float Kp_thr=KP_THROTTLE;
float Ki_thr=KI_THROTTLE;
float Kp_user=KP;
float Kd_user=KD;
float Kp_thr_user=KP_THROTTLE;
float Ki_thr_user=KI_THROTTLE;
bool newControlParameters = false;
bool modifing_control_parameters=false;
float PID_errorSum;
float PID_errorOld = 0;
float PID_errorOld2 = 0;
float setPointOld = 0;
float target_angle;
float throttle;
float steering;
float max_throttle = MAX_THROTTLE;
float max_steering = MAX_STEERING;
float max_target_angle = MAX_TARGET_ANGLE;
float control_output;
int16_t motor1;
int16_t motor2;

int16_t speed_m[2];           // Actual speed of motors
uint8_t dir_m[2];             // Actual direction of steppers motors
int16_t actual_robot_speed;          // overall robot speed (measured from steppers speed)
int16_t actual_robot_speed_Old;          // overall robot speed (measured from steppers speed)
float estimated_speed_filtered;

uint16_t counter_m[2];        // counters for periods
uint16_t period_m[2][8];      // Eight subperiods 
uint8_t period_m_index[2];    // index for subperiods


// STEPPER MOTOR PINS
// ENABLE PIN: D4
// STEP Motor1: D6 -> PORTD,7
// DIR  Motor1: D8 -> PORTB,4
// STEP Motor2: D12-> PORTD,6
// DIR  Motor2: D13-> PORTC,7


int freeRam () {
  extern int __heap_start, *__brkval; 
  int v; 
  return (int) &v - (__brkval == 0 ? (int) &__heap_start : (int) __brkval); 
}

// DMP FUNCTIONS
// This function defines the weight of the accel on the sensor fusion
// default value is 0x80
// The official invensense name is inv_key_0_96 (??)
void dmpSetSensorFusionAccelGain(uint8_t gain)
{
  // INV_KEY_0_96
  mpu.setMemoryBank(0);
  mpu.setMemoryStartAddress(0x60);
  mpu.writeMemoryByte(0);
  mpu.writeMemoryByte(gain);
  mpu.writeMemoryByte(0);
  mpu.writeMemoryByte(0);
}

// Quick calculation to obtein Phi angle from quaternion solution
float dmpGetPhi() {
   mpu.getFIFOBytes(fifoBuffer, 16); // We only read the quaternion
   mpu.dmpGetQuaternion(&q, fifoBuffer); 
   mpu.resetFIFO();  // We always reset FIFO
    
   //return( asin(-2*(q.x * q.z - q.w * q.y)) * 180/M_PI); //roll
   return (atan2(2*(q.y * q.z + q.w * q.x), q.w * q.w - q.x * q.x - q.y * q.y + q.z * q.z)* RAD2GRAD);
}

// PD implementation. DT is in miliseconds
float stabilityPDControl(float DT, float input, float setPoint,  float Kp, float Kd)
{
  float error;
  float output;

  error = setPoint-input;

  // Kd is implemented in two parts
  //    The biggest one using only the input (sensor) part not the SetPoint input-input(t-2)
  //    And the second using the setpoint to make it a bit more agressive   setPoint-setPoint(t-1)
  output = Kp*error + (Kd*(setPoint - setPointOld) - Kd*(input - PID_errorOld2))/DT;       // + error - PID_error_Old2
  //Serial.print(Kd*(error-PID_errorOld));Serial.print("\t");
  PID_errorOld2 = PID_errorOld;
  PID_errorOld = input;  // error for Kd is only the input component
  setPointOld = setPoint;
  return(output);
}

// P control implementation.
float speedPControl(float input, float setPoint,  float Kp)
{
  float error;

  error = setPoint-input;
 
  return(Kp*error);
}

// PI implementation. DT is in miliseconds
float speedPIControl(float DT, float input, float setPoint,  float Kp, float Ki)
{
  float error;
  float output;

  error = setPoint-input;
  PID_errorSum += constrain(error,-ITERM_MAX_ERROR,ITERM_MAX_ERROR);
  PID_errorSum = constrain(PID_errorSum,-ITERM_MAX,ITERM_MAX);

  output = Kp*error + Ki*PID_errorSum*DT*0.001;
  return(output);
}



// 200ns => 4 instructions at 16Mhz
void delay_200ns()  
{
  __asm__ __volatile__ (
		"nop" "\n\t"
                "nop" "\n\t"
                "nop" "\n\t"
		"nop"); 
}


ISR(TIMER1_COMPA_vect)
{
  counter_m[0]++;
  counter_m[1]++;
  if (counter_m[0] >= period_m[0][period_m_index[0]])
    {
    counter_m[0] = 0;
    if (period_m[0][0]==ZERO_SPEED)
      return;
    if (dir_m[0])
      SET(PORTB,4);  // DIR Motor 1
    else
      CLR(PORTB,4);
    // We need to wait at lest 200ns to generate the Step pulse...
    period_m_index[0] = (period_m_index[0]+1)&0x07; // period_m_index from 0 to 7
    //delay_200ns();
    SET(PORTD,7); // STEP Motor 1
    delayMicroseconds(1);
    CLR(PORTD,7);
    }
  if (counter_m[1] >= period_m[1][period_m_index[1]])
    {
    counter_m[1] = 0;
    if (period_m[1][0]==ZERO_SPEED)
      return;
    if (dir_m[1])
      SET(PORTC,7);   // DIR Motor 2
    else
      CLR(PORTC,7);
    period_m_index[1] = (period_m_index[1]+1)&0x07;
    //delay_200ns();
    SET(PORTD,6); // STEP Motor 1
    delayMicroseconds(1);
    CLR(PORTD,6);
    }
}


// Dividimos en 8 subperiodos para aumentar la resolucion a velocidades altas (periodos pequeños)
// subperiod = ((1000 % vel)*8)/vel;
// Examples 4 subperiods:
// 1000/260 = 3.84  subperiod = 3
// 1000/240 = 4.16  subperiod = 0
// 1000/220 = 4.54  subperiod = 2
// 1000/300 = 3.33  subperiod = 1 
void calculateSubperiods(uint8_t motor)
{
  int subperiod;
  int absSpeed;
  uint8_t j;
  
  if (speed_m[motor] == 0)
    {
    for (j=0;j<8;j++)
      period_m[motor][j] = ZERO_SPEED;
    return;
    }
  if (speed_m[motor] > 0 )   // Positive speed
    {
    dir_m[motor] = 1;
    absSpeed = speed_m[motor];
    }
  else                       // Negative speed
    {
    dir_m[motor] = 0;
    absSpeed = -speed_m[motor];
    }
    
  for (j=0;j<8;j++)
    period_m[motor][j] = 1000/absSpeed;
  // Calculate the subperiod. if module <0.25 => subperiod=0, if module < 0.5 => subperiod=1. if module < 0.75 subperiod=2 else subperiod=3
  subperiod = ((1000 % absSpeed)*8)/absSpeed;   // Optimized code to calculate subperiod (integer math)
  if (subperiod>0)
   period_m[motor][1]++;
  if (subperiod>1)
   period_m[motor][5]++;
  if (subperiod>2)
   period_m[motor][3]++;
  if (subperiod>3)
   period_m[motor][7]++;
  if (subperiod>4)
   period_m[motor][0]++;
  if (subperiod>5)
   period_m[motor][4]++;
  if (subperiod>6)
   period_m[motor][2]++;
  
  // DEBUG
  /*
  if ((motor==0)&&((debug_counter%10)==0)){
    Serial.print(1000.0/absSpeed);Serial.print("\t");Serial.print(absSpeed);Serial.print("\t");
    Serial.print(period_m[motor][0]);Serial.print("-");
    Serial.print(period_m[motor][1]);Serial.print("-");
    Serial.print(period_m[motor][2]);Serial.print("-");
    Serial.println(period_m[motor][3]);
    }
  */  
}


void setMotorSpeed(uint8_t motor, int16_t tspeed)
{
  // WE LIMIT MAX ACCELERATION
  if ((speed_m[motor] - tspeed)>MAX_ACCEL)
    speed_m[motor] -= MAX_ACCEL;
  else if ((speed_m[motor] - tspeed)<-MAX_ACCEL)
    speed_m[motor] += MAX_ACCEL;
  else
    speed_m[motor] = tspeed;
  
  calculateSubperiods(motor);  // We use four subperiods to increase resolution
  
  // To save energy when its not running...
  if ((speed_m[0]==0)&&(speed_m[1]==0))
    digitalWrite(4,HIGH);   // Disable motors
  else
    digitalWrite(4,LOW);   // Enable motors
}

void readControlParameters()
{
  // Parameter initialization (first time we enter page2)
  if ((OSC.page==2)&&(!modifing_control_parameters))
    {
    OSC.fadder1 = 0.5;
    OSC.fadder2 = 0.5;
    OSC.fadder3 = 0.5;
    OSC.fadder4 = 0.0;
    OSC.toggle1 = 0;
    modifing_control_parameters=true;
    }
  // Parameters Mode (page2 controls)
  // User could adjust KP, KD and KP_THROTTLE (fadder1,2,3)
  // Now we need to adjust all the parameters all the times because we don´t know what parameter has been moved
  if (OSC.page==2)
  //if ((OSC.page==2)&&(OSC.toggle1==1))
    {
    Serial.print("Par: ");
    Kp_user = KP*2*OSC.fadder1;
    Kd_user = KD*2*OSC.fadder2;
    Kp_thr_user = KP_THROTTLE*2*OSC.fadder3;
    Ki_thr_user = (KI_THROTTLE+0.1)*2*OSC.fadder4;
    if (OSC.toggle1 == 0)
      {
      max_throttle = MAX_THROTTLE;
      max_steering = MAX_STEERING;
      max_target_angle = MAX_TARGET_ANGLE;
      }
    else
      {
      max_throttle = MAX_THROTTLE_PRO;
      max_steering = MAX_STEERING_PRO;
      max_target_angle = MAX_TARGET_ANGLE_PRO;
      }
    Serial.print(Kp_user);
    Serial.print(" ");
    Serial.print(Kd_user);
    Serial.print(" ");
    Serial.print(Kp_thr_user);
    Serial.print(" ");
    Serial.println(Ki_thr_user);
    
    // Kill robot => Sleep
    while (OSC.toggle2)
      {
      //Reset external parameters
      mpu.resetFIFO();
      PID_errorSum = 0;
      timer_old = millis(); 
      distance_sensor = 150;
      setMotorSpeed(0,0);
      setMotorSpeed(1,0);
      OSC.MsgRead();
      }
        
    newControlParameters = true;
    }
  if ((newControlParameters)&&(!modifing_control_parameters))
    {
    // Reset parameters
    OSC.fadder1 = 0.5;
    OSC.fadder2 = 0.5;
    OSC.toggle1 = 0;
    newControlParameters=false;
    }
}

void readBattery()
{
  battery = (battery*9 + (analogRead(5)/8))/10; // output : Battery voltage*10 (aprox) and filtered
}

void readDistanceSensor()
{
  int aux;
	
  //aux = 20 + ((560 - analogRead(0))*2)/9;   // Quick distance calculation (we don´t need to be accurate)
  //distance_sensor = (distance_sensor*4 + aux)/5;  // Average with previous calculation
  aux = 5 + (analogRead(0)*4)/3;
  distance_sensor = (distance_sensor*9 + aux)/10;   // heavy filtering
}

void autonomousMode()
{
  if (autonomous_mode_status==0)   // Walking
  { 
    if (throttle < 125)
      throttle = throttle + 8;  
    if (throttle > 125) 
      throttle = 125;
    if (distance_sensor < OBSTACLE_DISTANCE_MIN)  // obstacle near robot
      {
      autonomous_mode_status=1;
      autonomous_mode_counter=0;
      }
    // If something stop the robot, we start an obstacle avoiding
    if ((throttle == 125)&&(estimated_speed_filtered < 10))
      {
      autonomous_mode_counter++;
      if (autonomous_mode_counter>200)
        {
        autonomous_mode_status=1;
        autonomous_mode_counter=0;
        }
      }
    }
  else if (autonomous_mode_status==1)  // Obstacle: Waiting to stop
    {
    autonomous_mode_counter++;
    throttle = throttle - 8;
    if (throttle < 0)
      throttle = 0;
    if (autonomous_mode_counter > 250)  // Wait 1 second
      {
      autonomous_mode_status=2;
      autonomous_mode_counter=0;
      autonomous_mode_distance = WALK_DISTANCE_MIN;
      throttle = 0;
      if ((millis() % 2)==0)
        steering = 20;
      else
        steering = -20;
      }
    }
  else if (autonomous_mode_status==2) // Obstacle Steering
    {
    autonomous_mode_counter++;
    if (distance_sensor > autonomous_mode_distance)
      {
      autonomous_mode_status = 3;  // We find a way...
      autonomous_mode_counter = 0;
      steering = -steering;     // rectification in steering
      }
    else if (autonomous_mode_counter > 600)
      {
      autonomous_mode_distance--;
      autonomous_mode_counter = 500;
      }
    //else if (autonomous_mode_counter == 500)  // if we don´t find an exit in 2 seconds we reverse the steering command
    //  {
    //  steering = -steering;
    //  }
    }
  else if (autonomous_mode_status==3) // Small rectification in steering
    {
    autonomous_mode_counter++;
    if (autonomous_mode_counter > 20)  // 0.1 seconds rectification
      {
      autonomous_mode_status = 0;  // We find a way...
      autonomous_mode_counter=0;
      steering = 0;        
      }
    }  
  else
    {
    throttle = 0;
    steering = 0;
    }
}


void setup() 
{ 
  // STEPPER PINS 
  pinMode(4,OUTPUT);  // ENABLE MOTORS
  pinMode(6,OUTPUT);  // STEP MOTOR 1 PORTD,7
  pinMode(8,OUTPUT);  // DIR MOTOR 1
  pinMode(12,OUTPUT); // STEP MOTOR 2 PORTD,6
  pinMode(13,OUTPUT); // DIR MOTOR 2
  digitalWrite(4,HIGH);   // Disbale motors
  
  pinMode(5,OUTPUT);  // Auxiliar servo 
  
  Serial.begin(115200);
  
  //servoAux.attach();
  
  WITA.WifiInit();
  //Serial.println("Wifi configuration...");  // CONFIGURATION ONLY NEEDED FIRST TIME
  //WITA.WifiFactoryReset();
  //WITA.WifiChangeBaudRateFast();
  //WITA.WifiEnableUDP("2222","2223","192.168.1.11");
  //WITA.WifiAP();       // Soft AP mode SSID:"WITA_AP"
  //WITA.LedBlink(3,true);
  
  // Join I2C bus
  Wire.begin();
  // 4000Khz fast mode
  TWSR = 0;
  TWBR = ((16000000L/I2C_SPEED)-16)/2;
  TWCR = 1<<TWEN;
  
  Serial.println("Initializing I2C devices...");
  //mpu.initialize();
  mpu.setClockSource(MPU6050_CLOCK_PLL_ZGYRO);
  mpu.setFullScaleGyroRange(MPU6050_GYRO_FS_2000);
  mpu.setFullScaleAccelRange(MPU6050_ACCEL_FS_2);
  mpu.setDLPFMode(MPU6050_DLPF_BW_20);  //10,20,42,98,188
  mpu.setRate(4);   // 0=1khz 1=500hz, 2=333hz, 3=250hz 4=200hz
  mpu.setSleepEnabled(false);
  
  delay(2000);
  Serial.println(F("Initializing DMP..."));
  devStatus = mpu.dmpInitialize();
  if (devStatus == 0) {
        // turn on the DMP, now that it's ready
        Serial.println(F("Enabling DMP..."));
        mpu.setDMPEnabled(true);
        mpuIntStatus = mpu.getIntStatus();
        dmpReady = true;
        
    } else { // ERROR!
        // 1 = initial memory load failed
        // 2 = DMP configuration updates failed
        // (if it's going to break, usually the code will be 1)
        Serial.print(F("DMP Initialization failed (code "));
        Serial.print(devStatus);
        Serial.println(F(")"));
    }
  
  // Gyro calibration
  // The robot must be steady during initialization
  delay(15000);   // Time to settle things... the bias_from_no_motion algorithm needs some time to take effect and reset gyro bias.

  Serial.print("Free RAM: ");
  Serial.println(freeRam());
  Serial.print("Max_throttle: ");
  Serial.println(max_throttle);
  Serial.print("Max_steering: ");
  Serial.println(max_steering);
  Serial.print("Max_target_angle: ");
  Serial.println(max_target_angle);

  // verify connection
  Serial.println("Testing device connections...");
  Serial.println(mpu.testConnection() ? "MPU6050 connection successful" : "MPU6050 connection failed");
  timer_old = millis();
  
  // Init servos
  Serial.println("Servo initialization...");
  WITA.InitServos();   // Servos initialization
  WITA.Servo(3,SERVO_AUX_NEUTRO);
  
  //We are going to overwrite the Timer1 to use the stepper motors
  
  // STEPPER MOTORS INITIALIZATION
  // TIMER1 CTC MODE
  TCCR1B &= ~(1<<WGM13);
  TCCR1B |=  (1<<WGM12);
  TCCR1A &= ~(1<<WGM11); 
  TCCR1A &= ~(1<<WGM10);

  // output mode = 00 (disconnected)
  TCCR1A &= ~(3<<COM1A0); 
  TCCR1A &= ~(3<<COM1B0); 
  
  // Set the timer pre-scaler
  // Generally we use a divider of 8, resulting in a 2MHz timer on 16MHz CPU
  TCCR1B = (TCCR1B & ~(0x07<<CS10)) | (2<<CS10);

  //OCR1A = 125;  // 16Khz
  //OCR1A = 100;  // 20Khz
  OCR1A = 80;   // 25Khz
  TCNT1 = 0;

  delay(2000);
  
  //Adjust sensor fusion gain
  Serial.println("Adjusting DMP sensor fusion gain...");
  dmpSetSensorFusionAccelGain(0x20);
  
  Serial.println("Initializing Stepper motors...");
  delay(1000);
  TIMSK1 |= (1<<OCIE1A);  // Enable Timer1 interrupt
  digitalWrite(4,LOW);    // Enable stepper drivers
  
  // Little motor vibration to indicate that robot is ready
  for (uint8_t k=0;k<3;k++)
	{
	setMotorSpeed(0,3);
	setMotorSpeed(1,-3);
	delay(150);
	setMotorSpeed(0,-3);
	setMotorSpeed(1,3);
	delay(150);
	}

  distance_sensor = 150;
  battery = 120; 
  
  // OSC initialization
  OSC.fadder1 = 0.5;
  OSC.fadder2 = 0.5;
 
  mpu.resetFIFO();
  timer_old = millis();
  Robot_shutdown = false;
}


// Main loop
void loop() 
{ 
  // If we have no battery, we do nothing...
  #ifdef SHUTDOWN_WHEN_BATTERY_OFF
    if (Robot_shutdown)
      return;
  #endif
  debug_counter++;
  OSC.MsgRead();  // Read UDP OSC messages
  if (OSC.page==1)    // Get commands from user (PAGE1 are user commands: throttle, steering...)
	{
	// Mode Change?   0 : MANUAL  1: AUTONOMOUS (based on toggle1 switch)
        if ((mode == 0)&&(OSC.toggle1))
          {
          mode = 1;
          distance_sensor = 150;
          autonomous_mode_status = 0;
          autonomous_mode_counter = 0;
          Serial.println("AUTO");
          }
        if ((mode==1)&&(OSC.toggle1==0))
          {
          mode = 0;
          distance_sensor = 150;
          Serial.println("MANUAL");
          }
          
        if (mode == 0)
          {
          if ((OSC.fadder1 > 0.45)&&(OSC.fadder1<0.55))   // Small deadband on throttle
            throttle = 0;
          else
	    throttle = (OSC.fadder1-0.5)*max_throttle;
		// We add some exponential on steering to smooth the center band
	  steering = OSC.fadder2-0.5;
          if (steering>0)
            steering = (steering*steering+0.5*steering)*max_steering;
          else
            steering = (-steering*steering+0.5*steering)*max_steering;
          }
        modifing_control_parameters=false;
	}
   
  timer_value = millis();
  // New DMP Orientation solution?
  fifoCount = mpu.getFIFOCount();
  if (fifoCount>=18)
      {
      if (fifoCount>18)  // If we have more than one packet we take the easy path: discard the buffer 
        {
        Serial.println("FIFO RESET!!");
        mpu.resetFIFO();
        return;
        }
	loop_counter++;
        slow_loop_counter++;
    dt = (timer_value-timer_old);
    timer_old = timer_value;
    
    angle_adjusted_Old = angle_adjusted;
    angle_adjusted = dmpGetPhi();
	
    #if DEBUG==8
      Serial.print(throttle);
      Serial.print(" ");
      Serial.print(steering);
      Serial.print(" ");
      Serial.println(mode);
    #endif

    //angle_adjusted_radians = angle_adjusted*GRAD2RAD;
    #if DEBUG==1
      Serial.println(angle_adjusted);
    #endif
    //Serial.print("\t");
    mpu.resetFIFO();  // We always reset FIFO

    if (mode==1)
      autonomousMode();

    // We calculate the estimated robot speed
    // Speed = angular_velocity_of_stepper_motors - angular_velocity_of_robot(angle measured by IMU)
    actual_robot_speed_Old = actual_robot_speed;
    actual_robot_speed = (speed_m[1] - speed_m[0])/2;  // Positive: forward

    int16_t angular_velocity = (angle_adjusted-angle_adjusted_Old)*90.0;     // 90 is an empirical extracted factor to adjust for real units
    int16_t estimated_speed = actual_robot_speed_Old - angular_velocity;     // We use robot_speed(t-1) or (t-2) to compensate the delay
    estimated_speed_filtered = estimated_speed_filtered*0.95 + (float)estimated_speed*0.05;
    #if DEBUG==2
      Serial.print(" ");
      Serial.println(estimated_speed_filtered);
    #endif
    
    //target_angle = (target_angle + speedPControl(estimated_speed_filtered,throttle,Kp_thr))/2.0;   // Some filtering : Average with previous output
    //target_angle = target_angle*0.3 + speedPIControl(dt,estimated_speed_filtered,throttle,Kp_thr,Ki_thr)*0.7;   // Some filtering 
    target_angle = speedPIControl(dt,estimated_speed_filtered,throttle,Kp_thr,Ki_thr); 
    target_angle = constrain(target_angle,-max_target_angle,max_target_angle);   // limited output
    
    #if DEBUG==3
      Serial.print(" ");Serial.println(estimated_speed_filtered);
      Serial.print(" ");Serial.println(target_angle);
    #endif
    
    // New user control parameters ?
    readControlParameters();
	
	if (pushUp_counter>0)  // pushUp mode?
      target_angle = 10;
	  
    // We integrate the output (acceleration)
    control_output += stabilityPDControl(dt,angle_adjusted,target_angle,Kp,Kd);	
    control_output = constrain(control_output,-500,500);   // Limit max output from control
    
        
    // The steering part of the control is injected directly on the output
    motor1 = control_output + steering;
    motor2 = -control_output + steering;   // Motor 2 is inverted
    
	// Limit max speed
    motor1 = constrain(motor1,-500,500);   
    motor2 = constrain(motor2,-500,500);
    
	// Is robot ready (upright?)
    if ((angle_adjusted<74)&&(angle_adjusted>-74))
      {
      if (OSC.push2) // pushUp mode?
        {
        pushUp_counter++;
        WITA.Servo(3,600);  // Move arm forward
        OSC.fadder1=0.5;
        OSC.fadder2=0.5;
        if (pushUp_counter>60) // 0.3 seconds
          {
          // Set motors to 0 => disable steppers => robot 
          setMotorSpeed(0,0);
          setMotorSpeed(1,0);
          // We prepare the raiseup mode  
          Kp = KP_RAISEUP;   // CONTROL GAINS FOR RAISE UP
          Kd = KD_RAISEUP;
          Kp_thr = KP_THROTTLE_RAISEUP;
          control_output = 0;
          estimated_speed_filtered = 0;
          }
        else
          {
          setMotorSpeed(0,motor1);
          setMotorSpeed(1,motor2);
          }
        }
      else
        {
        // NORMAL MODE
        setMotorSpeed(0,motor1);
        setMotorSpeed(1,motor2);
        pushUp_counter=0;
        if (OSC.push1)  // Move arm
          WITA.Servo(3,750);
        else
          WITA.Servo(3,SERVO_AUX_NEUTRO);
      }
      
      if ((angle_adjusted<40)&&(angle_adjusted>-40))
        {
        Kp = Kp_user;  // Default or user control gains
        Kd = Kd_user; 
        Kp_thr = Kp_thr_user;
        Ki_thr = Ki_thr_user;
        }     
      else
        {
        Kp = KP_RAISEUP;   // CONTROL GAINS FOR RAISE UP
        Kd = KD_RAISEUP;
        Kp_thr = KP_THROTTLE_RAISEUP; 
	Ki_thr = KI_THROTTLE_RAISEUP;
        }   
      }
    else   // Robot not ready, angle > 70º
      {
      setMotorSpeed(0,0);
      setMotorSpeed(1,0);
      PID_errorSum = 0;  // Reset PID I term
      Kp = KP_RAISEUP;   // CONTROL GAINS FOR RAISE UP
      Kd = KD_RAISEUP;
      Kp_thr = KP_THROTTLE_RAISEUP;
      Ki_thr = KI_THROTTLE_RAISEUP;
      // Raise up the robot with the servo arm
      if (OSC.push1)
        {
        if (angle_adjusted>0)
          WITA.Servo(3,600);
        else
          WITA.Servo(3,2400);
        }
      else
        WITA.Servo(3,SERVO_AUX_NEUTRO);
      // If we are in good position, we could raise up if we detect an user
      if (angle_adjusted<0)
        {
        readDistanceSensor();
        if (distance_sensor < OBSTACLE_DISTANCE_MIN)
          WITA.Servo(3,2400);
        }
      }
    } // New IMU data
	
  // Medium loop 40Hz
  if (loop_counter >= 5) 
    {
    loop_counter = 0;
    // if we are in autonomous mode we read the distance sensor
    if (mode==1)
        {
        readDistanceSensor();
        #if DEBUG == 5
          Serial.print(distance_sensor);
          Serial.print(" A");
          Serial.println(autonomous_mode_status);
        #endif
        }
    } // Medium loop
      
  if (slow_loop_counter>=99)  // 2Hz
    {
    slow_loop_counter = 0;
    // Read battery status
    readBattery();
    #if DEBUG==6
      Serial.print("B");
      Serial.println(battery);
      Serial.print(" ");
    #endif
    #if DEBUG==7
      Serial.print(distance_sensor);
      Serial.print(" A");
      Serial.println(autonomous_mode_status);
    #endif
    if (battery < BATTERY_SHUTDOWN)
      {
      // Robot shutdown !!!
      #ifdef SHUTDOWN_WHEN_BATTERY_OFF
        Serial.println("LOW BAT!! SHUTDOWN"); 
        Robot_shutdown = true;
        // Disable steppers
        digitalWrite(4,HIGH);   // Disable motors
      #endif	
      }
    else if (battery < BATTERY_WARNING)
      {
      // Battery warning
      // What to do here???
      Serial.println("LOW BAT!!");
      WITA.Servo(3,SERVO_AUX_NEUTRO+300);  // Move arm?
      }
    }  // Slow loop
}
