
#include <SimpleFOC.h>
#include <SPI.h>
#include <SdFat.h>
#include <Adafruit_SPIFlash.h>

#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20649.h>

// For SPI mode, we need a CS pin
#define LSM_CS 7
#define ICM_CS 9

#define POWER_SWITCH 12   // Pin 12 controls power to the camera
#define LED 13

#define PulsesPerRev 2560


// On-board external flash (QSPI or SPI) macros should already
// defined in your board variant if supported
// - EXTERNAL_FLASH_USE_QSPI
// - EXTERNAL_FLASH_USE_CS/EXTERNAL_FLASH_USE_SPI
#if defined(EXTERNAL_FLASH_USE_QSPI)
  Adafruit_FlashTransport_QSPI flashTransport;
#elif defined(EXTERNAL_FLASH_USE_SPI)
  Adafruit_FlashTransport_SPI flashTransport(EXTERNAL_FLASH_USE_CS, EXTERNAL_FLASH_USE_SPI);
#else
  #error No QSPI/SPI flash are defined on your board variant.h !
#endif

Adafruit_SPIFlash flash(&flashTransport);


// file system object from SdFat
FatFileSystem fatfs;
File DataFile;


// objects for the two 6DOF sensor modules
Adafruit_ISM330DHCX ism330dhcx;
Adafruit_ICM20649 icm20649;
sensors_event_t accel, gyro, temp;


// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(11); // 11 pole pairs
// Only certain pins can be used on the Metro M4 - e.g. pin 9 will NOT work
BLDCDriver3PWM driver = BLDCDriver3PWM(10, 5, 6, 8);

Encoder encoder = Encoder(2, 3, PulsesPerRev);

// interrupt routine intialization
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}

// commander communication instance
Commander command = Commander(Serial1);
float P_Base, P_Gain, D_Base, D_Gain, Spin_Rate_Adj;
float Z_Gyro_Cal, Z_Gyro2_Cal;
void onMotor(char* cmd){ command.motor(&motor, cmd); }
void P_Base_Call(char* cmd){ command.scalar(&P_Base, cmd); }
void P_Gain_Call(char* cmd){ command.scalar(&P_Gain, cmd); }
void D_Base_Call(char* cmd){ command.scalar(&D_Base, cmd); }
void D_Gain_Call(char* cmd){ command.scalar(&D_Gain, cmd); }
void Rate_Adj_Call(char* cmd){ command.scalar(&Spin_Rate_Adj, cmd); }
void Z_Gyro_Call(char* cmd){ command.scalar(&Z_Gyro_Cal, cmd); }
void Z_Gyro2_Call(char* cmd){ command.scalar(&Z_Gyro2_Cal, cmd); }

char* my_string; 

uint32_t prev_micros_timer = micros();
uint32_t micros_timer = micros();
uint32_t loop_duration_micros;
uint32_t target_loop_rate_us = 10000;
uint32_t BLDC_loop_duration_us = 333;
uint32_t loop_timer = micros();
uint32_t Runtime = millis();
uint32_t Starttime = micros();
uint32_t Timer1 = micros();
uint32_t Timer2 = micros();
uint32_t PrevConversionTimer = micros();
uint32_t ConversionTimer = micros();
uint32_t ConversionTimeMicros;
uint32_t PrevConversionTimer2 = micros();
uint32_t ConversionTimer2 = micros();
uint32_t ConversionTime2Micros;

int loopcount = 0;
int memorycount = 0;
int retract_count = 0;
int number_of_files = 0;


// Since there are two independent 6DOF sensor modules
// there are "two of everything" with respect to
// accelerometer and gyro varibles

float accel_x_vert, accel_y_vert, accel_z_vert;
float gyro_x_vert, gyro_y_vert, gyro_z_vert;
float accel2_x_vert, accel2_y_vert, accel2_z_vert;
float gyro2_x_vert, gyro2_y_vert, gyro2_z_vert;

float raw_accel_x_offset, raw_accel_y_offset, raw_accel_z_offset;
float raw_accel_x_scaling, raw_accel_y_scaling, raw_accel_z_scaling;
float raw_accel_x_corrected, raw_accel_y_corrected, raw_accel_z_corrected;

float raw_accel2_x_offset, raw_accel2_y_offset, raw_accel2_z_offset;
float raw_accel2_x_scaling, raw_accel2_y_scaling, raw_accel2_z_scaling;
float raw_accel2_x_corrected, raw_accel2_y_corrected, raw_accel2_z_corrected;

float accel_x_norm, accel_y_norm, accel_z_norm, accel_magnitude;
float pitch_rads_from_accel, yaw_rads_from_accel, pitch_degs_from_accel, yaw_degs_from_accel;
float accel_x_old_ewma, accel_y_old_ewma, accel_z_old_ewma;
float accel_x_ewma, accel_y_ewma, accel_z_ewma;
float gyro_x_old_ewma, gyro_y_old_ewma, gyro_z_old_ewma;
float gyro_x_ewma, gyro_y_ewma, gyro_z_ewma;

float accel2_x_norm, accel2_y_norm, accel2_z_norm, accel2_magnitude;
float pitch_rads_from_accel2, yaw_rads_from_accel2, pitch_degs_from_accel2, yaw_degs_from_accel2;
float accel2_x_old_ewma, accel2_y_old_ewma, accel2_z_old_ewma;
float accel2_x_ewma, accel2_y_ewma, accel2_z_ewma;
float gyro2_x_old_ewma, gyro2_y_old_ewma, gyro2_z_old_ewma;
float gyro2_x_ewma, gyro2_y_ewma, gyro2_z_ewma;

float gyro_x_offset, gyro_y_offset, gyro_z_offset;
float gyro2_x_offset, gyro2_y_offset, gyro2_z_offset;

float PID_Setpoint_rads;
float PID_Setpoint_degs;
float rotary_encoder_rads;
float position_error_rads;
float position_error_degs;
float CurrentPositionRads = 0, PreviousPositionRads = 0;
float encoder_offset_rads;

float spin_rate_dps;
float prev_vel_mps = 0, vel_mps = 0, vert_vel_mps = 0;
float prev_alt_m = 0, alt_m = 0;

float spin_rate2_dps;
float prev_vel2_mps = 0, vel2_mps = 0, vert_vel2_mps = 0;
float prev_alt2_m = 0, alt2_m = 0;

float x_angle_rads_increment;
float y_angle_rads_increment;
float z_angle_rads_increment;

float x_angle2_rads_increment;
float y_angle2_rads_increment;
float z_angle2_rads_increment;

// float x_angle_rads_from_simple_intg;  // not needed
// float y_angle_rads_from_simple_intg;  // not needed
float z_angle_rads_from_simple_intg;

// float x_angle2_rads_from_simple_intg;  // not needed
// float y_angle2_rads_from_simple_intg;  // not needed
float z_angle2_rads_from_simple_intg;

// float x_angle_degs_from_simple_intg; // not needed
// float y_angle_degs_from_simple_intg; // not needed
float z_angle_degs_from_simple_intg;

// float x_angle2_degs_from_simple_intg; // not needed
// float y_angle2_degs_from_simple_intg; // not needed
float z_angle2_degs_from_simple_intg;

float Pad_Ref_Yaw_Component_Rads;
float Pad_Ref_Pitch_Component_Rads;

float Pad_Ref_Yaw2_Component_Rads;
float Pad_Ref_Pitch2_Component_Rads;

float Pad_Ref_Yaw_Component_Degs;
float Pad_Ref_Pitch_Component_Degs;

float Pad_Ref_Yaw2_Component_Degs;
float Pad_Ref_Pitch2_Component_Degs;

float bearing_deg_from_intg_z_rotation;
float delta_bearing_deg_from_intg_z_rotation;
float windup_bearing_deg_from_intg_z_rotation;

float bearing2_deg_from_intg_z_rotation;
float delta_bearing2_deg_from_intg_z_rotation;
float windup_bearing2_deg_from_intg_z_rotation;

float tilt_bearing_deg;
float prev_tilt_bearing_deg;
float tilt_bearing_difference;
float delta_tilt_bearing_deg;
float windup_tilt_bearing_deg;
float EWMA_windup_tilt_bearing_deg;

float tilt_bearing2_deg;
float prev_tilt_bearing2_deg;
float tilt_bearing2_difference;
float delta_tilt_bearing2_deg;
float windup_tilt_bearing2_deg;
float EWMA_windup_tilt_bearing2_deg;

float delta_error_signal;
float windup_error_signal;

float delta_error_signal2;
float windup_error_signal2;

float a, b, c, e, tilt, tilt_rads;
float tilt_on_pad;

float a2, b2, c2, e2, tilt2, tilt2_rads;
float tilt2_on_pad;

boolean START = false;
boolean STOP = false;
boolean DEMO = false;
boolean START_FILE = false;
boolean STOP_TEST = false;
boolean ZERO = false;
boolean SKIP = false;
boolean ARM = false;
boolean LAUNCH = false;
boolean APOGEE = false;
boolean COMPLETE = false;
boolean TILT_LIMIT = false;
boolean MOTOR_DISABLE = false;
boolean ISM330DHCX_CONVERSION = false;
boolean START_DATA_LOGGING = false;
boolean toggle;
String Incoming;


uint32_t time_array[2001];
int16_t spin_rate_dps_array[2001];
int16_t lat_accel_mag[2001];
int16_t accel_z_vert_array[2001];
int16_t accel2_z_vert_array[2001];
int16_t vel_mps_array[2001];
uint16_t alt_m_array[2001];
int16_t bearing_deg_from_intg_z_rotation_array[2001];
int16_t bearing2_deg_from_intg_z_rotation_array[2001];
int16_t tilt_degs_array[2001];
int16_t tilt2_degs_array[2001];
int16_t tilt_bearing_deg_array[2001];
int16_t tilt_bearing2_deg_array[2001];
float motor_pos_setpoint_array[2001];
float motor_pos_error_array[2001];


/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/  


#include "9_Subroutines.h"


/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/  


void setup() {

  Serial.begin(115200);  // USB connection
  Serial1.begin(57600);  // Bluetooth connection

  pinMode(POWER_SWITCH, OUTPUT);  // Pin 12 controls power to the camera
  digitalWrite(POWER_SWITCH, LOW);
  
  delay(2000);

  Serial.println("\n\nInitializing Onboard Flash File System\n");
  Serial1.println("\n\nInitializing Onboard Flash File System\n");
  
  // Init external flash
  flash.begin();

  // Init file system on the flash
  fatfs.begin(&flash);

  Serial.println("Filesystem Initialized\n\n");
  Serial1.println("Filesystem Initialized\n\n");
  
  delay(500);


  /*-------------------------------------------------------------------------------------------------*/
  /*-------------------------------------------------------------------------------------------------*/  


  #include "1_Setup_Param.h"


  /*-------------------------------------------------------------------------------------------------*/
  /*-------------------------------------------------------------------------------------------------*/  


  while (!START) {

  Serial1.println("Type START to begin");
  Serial1.println("Type DEMO for accel-only demo");  
  Serial1.println("Type FILE for file operations\n");    
  delay(1000);
  
    while (!START && !DEMO && !START_FILE && Serial1.available() > 0) {   
      // get the new byte:
      char inChar = Serial1.read();
      // add it to the inputString:
      Incoming += inChar;    
      if (inChar == '\n') {
        Serial1.println(Incoming);
        
        if (Incoming.indexOf("START") >= 0) {
          Serial1.println("System Started\n\n\n");
          START = true;
        }

        else if (Incoming.indexOf("DEMO") >= 0) {
          Serial1.println("System Started\n\n\n");
          START = true;
          DEMO = true;          
        }

        else if (Incoming.indexOf("FILE") >= 0) {
          START_FILE = true;
          File_Operations();
        }

        else Serial1.println("No Match\n");
          
        Incoming = "";   
          
      }
           
    }

  }

  while (Serial1.available() > 0) Serial1.read(); // clear buffer

  delay(500);


  /*-------------------------------------------------------------------------------------------------*/
  /*-------------------------------------------------------------------------------------------------*/  


  #include "4_SensorActivate.h"


  /*-------------------------------------------------------------------------------------------------*/
  /*-------------------------------------------------------------------------------------------------*/  


  #include "3_EncoderZero.h"


  /*-------------------------------------------------------------------------------------------------*/
  /*-------------------------------------------------------------------------------------------------*/  


  // Optional code for testing purposes
  // Edit "2_TestingCode.h" as necessary
  // #include "2_TestingCode.h"


  /*-------------------------------------------------------------------------------------------------*/
  /*-------------------------------------------------------------------------------------------------*/  


  #include "6_DemoCode.h"


  /*-------------------------------------------------------------------------------------------------*/
  /*-------------------------------------------------------------------------------------------------*/  


  #include "5_Arm_And_Wait.h"


  /*-------------------------------------------------------------------------------------------------*/
  /*-------------------------------------------------------------------------------------------------*/  


  // The code below is run at the moment the launch is detected

  motor.velocity_limit = 40;
  motor.P_angle.limit = 40; // MUST follow changes to motor.velocity_limit

  motor.PID_velocity.P = 0.50000;
  motor.PID_velocity.I = 5.00000;
  motor.PID_velocity.D = 0.00000;
  motor.LPF_velocity.Tf = 0.005;   // velocity low pass filtering time constant   // smaller means less filtering
  motor.P_angle.P = 20;   // angle loop PID controller parameters   // not used for torque or velocity control
  motor.P_angle.I = 0;
  motor.P_angle.D = 0;  

  // x_angle_rads_from_simple_intg = 0;  // not needed
  // y_angle_rads_from_simple_intg = 0;   // not needed
  z_angle_rads_from_simple_intg = 0;

  // x_angle2_rads_from_simple_intg = 0;  // not needed
  // y_angle2_rads_from_simple_intg = 0;  // not needed
  z_angle2_rads_from_simple_intg = 0;
  
  bearing_deg_from_intg_z_rotation = 0;
  delta_bearing_deg_from_intg_z_rotation = 0;  
  windup_bearing_deg_from_intg_z_rotation = 0;

  bearing2_deg_from_intg_z_rotation = 0;
  delta_bearing2_deg_from_intg_z_rotation = 0;  
  windup_bearing2_deg_from_intg_z_rotation = 0;

  // Record gyro offsets
  gyro_x_offset = gyro_x_ewma;
  gyro_y_offset = gyro_y_ewma;
  gyro_z_offset = gyro_z_ewma;

  gyro2_x_offset = gyro2_x_ewma;
  gyro2_y_offset = gyro2_y_ewma;
  gyro2_z_offset = gyro2_z_ewma;

  // Initialize the gyro-based yaw and pitch using the
  // accelerometer-based yaw and pitch from the pre-launch pad conditions
  // Yaw is the same as the "left to right" tilt on the pad
  // Pitch is same as the "forward to back" tilt on the pad 
  Pad_Ref_Yaw_Component_Rads = yaw_rads_from_accel;
  Pad_Ref_Pitch_Component_Rads = pitch_rads_from_accel;

  Pad_Ref_Yaw2_Component_Rads = yaw_rads_from_accel2;
  Pad_Ref_Pitch2_Component_Rads = pitch_rads_from_accel2;
  
  prev_tilt_bearing_deg = tilt_bearing_deg;
  tilt_bearing_difference = 0;
  delta_tilt_bearing_deg = 0;  
  windup_tilt_bearing_deg = tilt_bearing_deg;

  prev_tilt_bearing2_deg = tilt_bearing2_deg;
  tilt_bearing2_difference = 0;
  delta_tilt_bearing2_deg = 0;  
  windup_tilt_bearing2_deg = tilt_bearing2_deg;
  
  EWMA_windup_tilt_bearing_deg = 0;  // This will allow for a smooth transition at launch
  EWMA_windup_tilt_bearing2_deg = 0;  // This will allow for a smooth transition at launch

  delta_error_signal = 0;  
  delta_error_signal2 = 0;  

  loopcount = 0;

  spin_rate_dps = 0;
  spin_rate2_dps = 0;  

  PID_Setpoint_rads = 0;
  PID_Setpoint_degs = 0;

  motor.target = PID_Setpoint_rads;

  PreviousPositionRads = 0;
  CurrentPositionRads = 0;

  // Initialize variables for estimating velocity and altitude
  // based on the accelerometer data
  // Since launch has already been detected
  // make a guess-timate of the starting initial vel and pos
  // before entering the actual loop below  
  vel_mps = 3 * 9.81 * 0.01;
  alt_m = vel_mps * 0.01;

  vel2_mps = 3 * 9.81 * 0.01;
  alt2_m = vel2_mps * 0.01;

  BLDC_loop_duration_us = 333;

  ISM330DHCX_CONVERSION = false;

  loop_timer = micros();
  micros_timer = micros();
  prev_micros_timer = micros();
  Timer1 = micros();
  Timer2 = micros();
  Starttime = micros();
  Runtime = millis();
   
}


void loop() {

  if ((millis() - Runtime) < 62000) {     // After launch detection, run for just over one minute
 
    if ( (micros() - Timer1) >= 4500 && !ISM330DHCX_CONVERSION) {

      while ( (micros() - Timer1) <= 5000 ) {
        delayMicroseconds(1);
      }   

      ISM330DHCX_CONVERSION = true;
      
      // Get new sensor events   
      ism330dhcx.getEvent(&accel, &gyro, &temp);  // About 200us for a single SPI bus conversion

      PrevConversionTimer = ConversionTimer;
      ConversionTimer = micros();
      ConversionTimeMicros = ConversionTimer - PrevConversionTimer;
    
      // Perform offsets and corrections
      raw_accel_x_corrected = (accel.acceleration.x - raw_accel_x_offset) * raw_accel_x_scaling;
      raw_accel_y_corrected = (accel.acceleration.y - raw_accel_y_offset) * raw_accel_y_scaling;
      raw_accel_z_corrected = (accel.acceleration.z - raw_accel_z_offset) * raw_accel_z_scaling;
    

      // Adjust to "vertical" orientation of the sensor board
      accel_x_vert = -raw_accel_z_corrected;
      accel_y_vert = raw_accel_y_corrected;
      accel_z_vert = raw_accel_x_corrected;
      gyro_x_vert = -(gyro.gyro.z - gyro_z_offset);
      gyro_y_vert = gyro.gyro.y - gyro_y_offset;
      gyro_z_vert = gyro.gyro.x - gyro_x_offset;
  
      gyro_z_vert *= Z_Gyro_Cal;
  
      spin_rate_dps = 57.295779513 * gyro_z_vert;

      if ( abs(spin_rate_dps) >= 1000 ) {
        gyro_z_vert = gyro2_z_vert;
        spin_rate_dps = spin_rate2_dps;
      }

      // Integrate the gyros and make simple calculations to be used later    
      // These are the increments in yaw, pitch and roll in the rocket's reference frame
      // x_angle_rads_increment is the yaw increment
      // y_angle_rads_increment is the pitch increment
      // z_angle_rads_increment is the roll (or spin) increment   
      x_angle_rads_increment = (gyro_x_vert * float(ConversionTimeMicros)) / 1000000.0f;
      y_angle_rads_increment = (gyro_y_vert * float(ConversionTimeMicros)) / 1000000.0f;
      z_angle_rads_increment = (gyro_z_vert * float(ConversionTimeMicros)) / 1000000.0f;
    
      // x_angle_rads_from_simple_intg += x_angle_rads_increment;  // not needed
      // y_angle_rads_from_simple_intg += y_angle_rads_increment;  // not needed
      z_angle_rads_from_simple_intg += z_angle_rads_increment;
      
      // x_angle_degs_from_simple_intg = 57.295779513 * x_angle_rads_from_simple_intg; // not needed
      // y_angle_degs_from_simple_intg = 57.295779513 * y_angle_rads_from_simple_intg; // not needed  
      z_angle_degs_from_simple_intg = 57.295779513 * z_angle_rads_from_simple_intg;
  
      bearing_deg_from_intg_z_rotation = -z_angle_degs_from_simple_intg;

      // Don't touch these next two lines of code
      // They make the data logging of integrated Z rotation independent from
      // corrections made to "z_angle_rads_from_simple_intg" in case of motor stalling
      delta_bearing_deg_from_intg_z_rotation = -57.295779513 * z_angle_rads_increment;
      windup_bearing_deg_from_intg_z_rotation += delta_bearing_deg_from_intg_z_rotation;  // This value is data logged
  
  
      // Determine the components of rocket tilt in the launchpad-based reference frame
      // These are used to calculate both the total tilt and the direction of rocket tilt
      // These component variables are "immune" to rocket "spin" (a.k.a. roll) about the rocket's long axis  
      // Note that these are summation calculations because they used the += operator 
      Pad_Ref_Yaw_Component_Rads += x_angle_rads_increment * cos(z_angle_rads_from_simple_intg)
                                        - y_angle_rads_increment * sin(z_angle_rads_from_simple_intg);  
    
      Pad_Ref_Pitch_Component_Rads += x_angle_rads_increment * sin(z_angle_rads_from_simple_intg)
                                        + y_angle_rads_increment * cos(z_angle_rads_from_simple_intg);
    
  
      Pad_Ref_Yaw_Component_Degs = 57.295779513 * Pad_Ref_Yaw_Component_Rads;
      Pad_Ref_Pitch_Component_Degs = 57.295779513 * Pad_Ref_Pitch_Component_Rads;


      // Rocket tilt calculation      
      a = sin(Pad_Ref_Pitch_Component_Rads);
      b = cos(Pad_Ref_Pitch_Component_Rads);  
      c = b * sin(Pad_Ref_Yaw_Component_Rads);
      e = sqrt( (a*a) + (c*c) );
      tilt_rads = asin(e);
      if (isnan(tilt_rads)) tilt_rads = 1.57079632;  // In case asin(e) is NAN because e > 1 due to floating point errors
      tilt = 57.295779513 * tilt_rads;
      
      // Make corrections for when actual tilt is > 90 degrees
      if (abs(Pad_Ref_Yaw_Component_Degs)<=90 && abs(Pad_Ref_Pitch_Component_Degs)>=90) {
        tilt_rads = 3.14159 - tilt_rads;
        tilt = 180 - tilt;
      }
      else if (abs(Pad_Ref_Yaw_Component_Degs)>=90 && abs(Pad_Ref_Pitch_Component_Degs)<=90) {
        tilt_rads = 3.14159 - tilt_rads;        
        tilt = 180 - tilt;  
      } 


      // The velocity and altitude estimates incorporate tilt for greater accuracy
      // The estimate will be wrong if the accelerometer saturates
      // Make these calculations AFTER calculating the most recent estimate for tilt
      prev_vel_mps = vel_mps;
      prev_alt_m = alt_m;     
      vel_mps = prev_vel_mps + ((accel_z_vert - (9.81 * cos(tilt_rads))) * float(ConversionTimeMicros) / 1000000.0f);
      vert_vel_mps = cos(tilt_rads) * vel_mps;
      alt_m = prev_alt_m + (vert_vel_mps * float(ConversionTimeMicros) / 1000000.0f);         
      
      
      prev_tilt_bearing_deg = tilt_bearing_deg;
      tilt_bearing_deg = 57.295779513 * atan2(Pad_Ref_Yaw_Component_Rads, Pad_Ref_Pitch_Component_Rads);    
  
  
      // The "tilt_bearing_difference" variable below has a range of -359.99999 to +359.99999
      tilt_bearing_difference = tilt_bearing_deg - prev_tilt_bearing_deg;
  
      // Calculate the shortest Delta rotation - Between -179.99999 and +180.00000
      if ( tilt_bearing_difference <= -180 ) delta_tilt_bearing_deg = 360 + tilt_bearing_difference;
      else if ( tilt_bearing_difference <= 0 ) delta_tilt_bearing_deg = tilt_bearing_difference;
      else if ( tilt_bearing_difference <= 180 ) delta_tilt_bearing_deg = tilt_bearing_difference;
      else delta_tilt_bearing_deg = tilt_bearing_difference - 360;
  
      windup_tilt_bearing_deg += delta_tilt_bearing_deg;


      // This next conditional is useful if the rocket happens to fly near-vertical
      if (tilt <= 20) {

        while (windup_tilt_bearing_deg - EWMA_windup_tilt_bearing_deg > 180) windup_tilt_bearing_deg -= 360; 
        
        while (windup_tilt_bearing_deg - EWMA_windup_tilt_bearing_deg < -180) windup_tilt_bearing_deg += 360; 
        
      }


      // For smooth transition off of the rail do not start calculating
      // the EWMA_windup_tilt_bearing_deg until actually off of the rail
      if ( memorycount >= 20 ) {
          if (tilt <= 5) EWMA_windup_tilt_bearing_deg = (0.002 * windup_tilt_bearing_deg) + ( 0.998 * EWMA_windup_tilt_bearing_deg);
          else if (tilt <= 10) EWMA_windup_tilt_bearing_deg = (0.005 * windup_tilt_bearing_deg) + ( 0.995 * EWMA_windup_tilt_bearing_deg);   
          else if (tilt <= 15) EWMA_windup_tilt_bearing_deg = (0.01 * windup_tilt_bearing_deg) + ( 0.99 * EWMA_windup_tilt_bearing_deg); 
          else if (tilt <= 20) EWMA_windup_tilt_bearing_deg = (0.02 * windup_tilt_bearing_deg) + ( 0.98 * EWMA_windup_tilt_bearing_deg);  
          else if (tilt <= 30) EWMA_windup_tilt_bearing_deg = (0.03 * windup_tilt_bearing_deg) + ( 0.97 * EWMA_windup_tilt_bearing_deg);  
          else if (tilt <= 40) EWMA_windup_tilt_bearing_deg = (0.04 * windup_tilt_bearing_deg) + ( 0.96 * EWMA_windup_tilt_bearing_deg);                                 
          else EWMA_windup_tilt_bearing_deg = (0.05 * windup_tilt_bearing_deg) + ( 0.95 * EWMA_windup_tilt_bearing_deg);  
      }


      // Calculate the error signal used as the input to the PID position loop using EWMA_windup_tilt_bearing_deg
      windup_error_signal = bearing_deg_from_intg_z_rotation - EWMA_windup_tilt_bearing_deg;  // units are degrees

      // Temporary substitute code for calibration of the Z gyro gain
      // windup_error_signal = bearing_deg_from_intg_z_rotation;   
  
      PID_Setpoint_degs = windup_error_signal;

      PID_Setpoint_rads = windup_error_signal / 57.295779513;


      //              Perform checks for when to shut the system down  
    
      // Detect apogee conditions any time after 10 seconds
      if ( memorycount >= 1000 && !APOGEE ) {
        if ( (vert_vel2_mps <= 5) || ( (vert_vel2_mps <= 15) && (abs(accel_z_vert / 9.81) >=  15) ) ) APOGEE = true;  // the 2nd condition is e-charge detection
      }
  
      // Detect "unusual" events early in the flight
      if ( memorycount >= 50 && memorycount <= 400 && !TILT_LIMIT ) {
        if ( tilt >= 45 ) TILT_LIMIT = true;
      }
  
      // Detect if tilt exceeds 110 degrees after 4 seconds
      if ( memorycount >= 400 && !TILT_LIMIT ) {
        if ( tilt >= 110 ) TILT_LIMIT = true;
      }
   
      if ( !MOTOR_DISABLE ) {
        if ( APOGEE || TILT_LIMIT ) {
          MOTOR_DISABLE = true;
          motor.disable();
        }
      }
      

      if (memorycount % 3 == 0 && memorycount <= 6000) {
  
        time_array[memorycount/3] = (micros() - Starttime);
        
        spin_rate_dps_array[memorycount/3] = -20 * spin_rate_dps;
        
        lat_accel_mag[memorycount/3] = 1000 * (sqrt(accel_x_vert * accel_x_vert + accel_y_vert * accel_y_vert) / 9.81);      
        
        accel_z_vert_array[memorycount/3] = 1000 * (accel_z_vert / 9.81);

        accel2_z_vert_array[memorycount/3] = 1000 * (accel2_z_vert / 9.81);        
  
        vel_mps_array[memorycount/3] = 10 * vel2_mps;  // max vel is +/- mach 9.5
  
        alt_m_array[memorycount/3] = 10 * alt2_m;  // max alt is 6553 m or 21500 ft
        
        bearing_deg_from_intg_z_rotation_array[memorycount/3] = windup_bearing_deg_from_intg_z_rotation;

        bearing2_deg_from_intg_z_rotation_array[memorycount/3] = windup_bearing2_deg_from_intg_z_rotation;       
        
        tilt_degs_array[memorycount/3] = 100 * tilt;

        tilt2_degs_array[memorycount/3] = 100 * tilt2;       
        
        tilt_bearing_deg_array[memorycount/3] = 10 * EWMA_windup_tilt_bearing_deg;

        tilt_bearing2_deg_array[memorycount/3] = 10 * EWMA_windup_tilt_bearing2_deg;
        
        motor_pos_setpoint_array[memorycount/3] = 57.295779513 * motor.target;  
  
        motor_pos_error_array[memorycount/3] = position_error_degs;   
        
      }

      memorycount++;  // memorycount indexes at 100Hz
   
    }


    if ( (micros() - Timer1) >= 9500 && ISM330DHCX_CONVERSION) {

      while ( (micros() - Timer1) <= 10000 ) {
        delayMicroseconds(1);
      }   
     
      Timer1 = micros();
  
      ISM330DHCX_CONVERSION = false;    

      delayMicroseconds(100);  // Should this be only used when calling the faster lis.getEvent(&accel) ?????????
      
      icm20649.getEvent(&accel, &gyro, &temp);  // About 300us for a single SPI bus conversion

      PrevConversionTimer2 = ConversionTimer2;
      ConversionTimer2 = micros();
      ConversionTime2Micros = ConversionTimer2 - PrevConversionTimer2;

      // Perform offsets and corrections
      raw_accel2_x_corrected = (accel.acceleration.x - raw_accel2_x_offset) * raw_accel2_x_scaling;
      raw_accel2_y_corrected = (accel.acceleration.y - raw_accel2_y_offset) * raw_accel2_y_scaling;
      raw_accel2_z_corrected = (accel.acceleration.z - raw_accel2_z_offset) * raw_accel2_z_scaling;
    

      // Adjust to "vertical" orientation of the sensor board
      accel2_x_vert = -raw_accel2_z_corrected;
      accel2_y_vert = -raw_accel2_x_corrected;
      accel2_z_vert = raw_accel2_y_corrected;
      gyro2_x_vert = -(gyro.gyro.z - gyro2_z_offset);
      gyro2_y_vert = -(gyro.gyro.x - gyro2_x_offset);
      gyro2_z_vert = gyro.gyro.y - gyro2_y_offset;

      gyro2_z_vert *= Z_Gyro2_Cal;

      spin_rate2_dps = 57.295779513 * gyro2_z_vert; 


      // Integrate the gyros and make simple calculations to be used later    
      // These are the increments in yaw, pitch and roll in the rocket's reference frame
      // x_angle_rads_increment is the yaw increment
      // y_angle_rads_increment is the pitch increment
      // z_angle_rads_increment is the roll (or spin) increment    
      x_angle2_rads_increment = (gyro2_x_vert * float(ConversionTime2Micros)) / 1000000.0f;
      y_angle2_rads_increment = (gyro2_y_vert * float(ConversionTime2Micros)) / 1000000.0f;
      z_angle2_rads_increment = (gyro2_z_vert * float(ConversionTime2Micros)) / 1000000.0f;
     
      // x_angle2_rads_from_simple_intg += x_angle2_rads_increment;  // not needed
      // y_angle2_rads_from_simple_intg += y_angle2_rads_increment;  // not needed
      z_angle2_rads_from_simple_intg += z_angle2_rads_increment;

      // x_angle2_degs_from_simple_intg = 57.295779513 * x_angle2_rads_from_simple_intg; // not needed
      // y_angle2_degs_from_simple_intg = 57.295779513 * y_angle2_rads_from_simple_intg; // not needed    
      z_angle2_degs_from_simple_intg = 57.295779513 * z_angle2_rads_from_simple_intg;
  
      bearing2_deg_from_intg_z_rotation = -z_angle2_degs_from_simple_intg;
      delta_bearing2_deg_from_intg_z_rotation = -57.295779513 * z_angle2_rads_increment;  
      windup_bearing2_deg_from_intg_z_rotation += delta_bearing2_deg_from_intg_z_rotation;
  
  
      // Determine the components of rocket tilt in the launchpad-based reference frame
      // These are used to calculate both the total tilt and the direction of rocket tilt
      // These component variables are "immune" to rocket "spin" (a.k.a. roll) about the rocket's long axis  
      // Note that these are summation calculations because they used the += operator 
      Pad_Ref_Yaw2_Component_Rads += x_angle2_rads_increment * cos(z_angle2_rads_from_simple_intg)
                                        - y_angle2_rads_increment * sin(z_angle2_rads_from_simple_intg);  
    
      Pad_Ref_Pitch2_Component_Rads += x_angle2_rads_increment * sin(z_angle2_rads_from_simple_intg)
                                        + y_angle2_rads_increment * cos(z_angle2_rads_from_simple_intg);
    
  
      Pad_Ref_Yaw2_Component_Degs = 57.295779513 * Pad_Ref_Yaw2_Component_Rads;
      Pad_Ref_Pitch2_Component_Degs = 57.295779513 * Pad_Ref_Pitch2_Component_Rads;
  

      // Rocket tilt calculation      
      a2 = sin(Pad_Ref_Pitch2_Component_Rads);
      b2 = cos(Pad_Ref_Pitch2_Component_Rads);  
      c2 = b2 * sin(Pad_Ref_Yaw2_Component_Rads);
      e2 = sqrt( (a2*a2) + (c2*c2) );
      tilt2_rads = asin(e2);
      if (isnan(tilt2_rads)) tilt2_rads = 1.57079632;  // In case asin(e) is NAN because e > 1 due to floating point errors
      tilt2 = 57.295779513 * tilt2_rads;

      // Make corrections for when actual tilt is > 90 degrees
      if (abs(Pad_Ref_Yaw2_Component_Degs)<=90 && abs(Pad_Ref_Pitch2_Component_Degs)>=90) {
        tilt2_rads = 3.14159 - tilt2_rads;
        tilt2 = 180 - tilt2;
      }
      else if (abs(Pad_Ref_Yaw2_Component_Degs)>=90 && abs(Pad_Ref_Pitch2_Component_Degs)<=90) {
        tilt2_rads = 3.14159 - tilt2_rads;        
        tilt2 = 180 - tilt2;  
      }


      // The velocity and altitude estimates incorporate tilt for greater accuracy
      // The estimate will be wrong if the accelerometer saturates
      // Make these calculations AFTER calculating the most recent estimate for tilt
      // Using accel data from the High-G acclerometer and tilt data from the better gyro calculation
      prev_vel2_mps = vel2_mps;
      prev_alt2_m = alt2_m;       
      vel2_mps = prev_vel2_mps + ((accel2_z_vert - (9.81 * cos(tilt_rads))) * float(ConversionTime2Micros) / 1000000.0f);
      vert_vel2_mps = cos(tilt_rads) * vel2_mps;
      alt2_m = prev_alt2_m + (vert_vel2_mps * float(ConversionTime2Micros) / 1000000.0f);  

 
      prev_tilt_bearing2_deg = tilt_bearing2_deg;
      tilt_bearing2_deg = 57.295779513 * atan2(Pad_Ref_Yaw2_Component_Rads, Pad_Ref_Pitch2_Component_Rads);    
  
  
      // The "tilt_bearing_difference" variable below has a range of -359.99999 to +359.99999
      tilt_bearing2_difference = tilt_bearing2_deg - prev_tilt_bearing2_deg;
  
      // Calculate the shortest Delta rotation - Between -179.99999 and +180.00000
      if ( tilt_bearing2_difference <= -180 ) delta_tilt_bearing2_deg = 360 + tilt_bearing2_difference;
      else if ( tilt_bearing2_difference <= 0 ) delta_tilt_bearing2_deg = tilt_bearing2_difference;
      else if ( tilt_bearing2_difference <= 180 ) delta_tilt_bearing2_deg = tilt_bearing2_difference;
      else delta_tilt_bearing2_deg = tilt_bearing2_difference - 360;
  
      windup_tilt_bearing2_deg += delta_tilt_bearing2_deg;


      // This next conditional is useful if the rocket happens to fly near-vertical
      if (tilt2 <= 20) {

        while (windup_tilt_bearing2_deg - EWMA_windup_tilt_bearing2_deg > 180) windup_tilt_bearing2_deg -= 360; 
        
        while (windup_tilt_bearing2_deg - EWMA_windup_tilt_bearing2_deg < -180) windup_tilt_bearing2_deg += 360; 
        
      }


      // For smooth transition off of the rail do not start calculating
      // the EWMA_windup_tilt_bearing2_deg until actually off of the rail
      if ( memorycount >= 20 ) {
          if (tilt2 <= 5) EWMA_windup_tilt_bearing2_deg = (0.002 * windup_tilt_bearing2_deg) + ( 0.998 * EWMA_windup_tilt_bearing2_deg);
          else if (tilt2 <= 10) EWMA_windup_tilt_bearing2_deg = (0.005 * windup_tilt_bearing2_deg) + ( 0.995 * EWMA_windup_tilt_bearing2_deg);   
          else if (tilt2 <= 15) EWMA_windup_tilt_bearing2_deg = (0.01 * windup_tilt_bearing2_deg) + ( 0.99 * EWMA_windup_tilt_bearing2_deg); 
          else if (tilt2 <= 20) EWMA_windup_tilt_bearing2_deg = (0.02 * windup_tilt_bearing2_deg) + ( 0.98 * EWMA_windup_tilt_bearing2_deg);  
          else if (tilt2 <= 30) EWMA_windup_tilt_bearing2_deg = (0.03 * windup_tilt_bearing2_deg) + ( 0.97 * EWMA_windup_tilt_bearing2_deg);  
          else if (tilt2 <= 40) EWMA_windup_tilt_bearing2_deg = (0.04 * windup_tilt_bearing2_deg) + ( 0.96 * EWMA_windup_tilt_bearing2_deg);                                 
          else EWMA_windup_tilt_bearing2_deg = (0.05 * windup_tilt_bearing2_deg) + ( 0.95 * EWMA_windup_tilt_bearing2_deg);  
      }


      // Calculate the error signal used as the input to the PID position loop using EWMA_windup_tilt_bearing_deg
      windup_error_signal2 = bearing2_deg_from_intg_z_rotation - EWMA_windup_tilt_bearing2_deg;  // units are degrees 

      // Temporary substitute code for calibration of the Z gyro gain on the second gyro sensor (if present)
      // windup_error_signal2 = bearing2_deg_from_intg_z_rotation;   

    }


    while ((micros() - loop_timer) < BLDC_loop_duration_us) {
      delayMicroseconds(1);
    }   

    loop_timer = micros();
    prev_micros_timer = micros_timer;
    micros_timer = micros();
    loop_duration_micros = micros_timer - prev_micros_timer;

    // the microsecond timer will rollover about every 1.2 hours    
    if (loop_duration_micros > 1000000) loop_duration_micros = BLDC_loop_duration_us;  // in case of rollover

    // Serial.println(loop_duration_micros);   
   
    // Shut down system if past apogee or if tilt exceeds limits   
    if ( !APOGEE && !TILT_LIMIT ) {

      if ( abs(spin_rate_dps <= 720) ) {
        motor.P_angle.P = P_Base + (P_Gain * abs(spin_rate_dps/360) );
        motor.P_angle.D = D_Base + (D_Gain * abs(spin_rate_dps/360) );
        
      }
      else {
        motor.P_angle.P = P_Base + (P_Gain * 2);
        motor.P_angle.D = D_Base + (D_Gain * 2); 
      }

      // A setpoint modifier based on spin dps is used to offset
      // position errors and gyro integration loop lag 
      motor.target = PID_Setpoint_rads - (Spin_Rate_Adj * (spin_rate_dps/360));        

      motor.loopFOC();
      motor.move();

      /* 
      // Turn on the Commander interface for bench testing and dynamically adjusting P_Base, etc.
      // Command mappings as follows ......
      // 'A', onMotor
      // 'D', P_Base_Call
      // 'E', P_Gain_Call
      // 'F', D_Base_Call
      // 'G', D_Gain_Call
      // 'S', Rate_Adj_Call  // For real-time calibration of the feed forward - maps to Spin_Rate_Adj
      // 'Z', Z_Gyro_Call   // For real-time calibration of the Z gyro - maps to Z_Gyro_Cal
      // 'Y', Z_Gyro2_Call   // For real-time calibration of the second Z gyro - maps to Z_Gyro2_Cal

      motor.monitor();
      command.run();  // Uncomment for debug and determining parameters
      */

      PreviousPositionRads = CurrentPositionRads;
 
      rotary_encoder_rads = encoder.getAngle();

      CurrentPositionRads = rotary_encoder_rads - encoder_offset_rads;
      position_error_rads = PID_Setpoint_rads - CurrentPositionRads;      

      position_error_degs = 57.295779513 * position_error_rads;
    
    } 


    // The next two IF statements reduce the amount of "unwinding"
    // needed if the BLDC motor temporarily stalls
    if ( position_error_degs > 180 && loopcount % 100 == 0 ) {
      z_angle_rads_from_simple_intg += 6.28318531;
    }

    if ( position_error_degs < -180 && loopcount % 100 == 0 ) {
      z_angle_rads_from_simple_intg -= 6.28318531;      
    }

/*
    if ( loopcount % 1000 == 0 ) {
      Serial1.println(position_error_degs);
      Serial1.println(z_angle_rads_from_simple_intg);
      Serial1.println("\n");        
    }
*/

    loopcount++;  // loopcount can increment as fast was 3KHz

/*
    if (loopcount % 200 == 0) {
  
      loopcount = 0;
  
      // Serial1.print(millis() - Runtime); Serial1.print(",  "); 
      // Serial1.print(EWMA_windup_tilt_bearing_deg); Serial1.print(",  ");      
      // Serial1.print(bearing_deg_from_intg_z_rotation); Serial1.print(",  "); 
      // Serial1.print(windup_error_signal); Serial1.print(",  ");
      // Serial1.print(PID_Setpoint_rads); Serial1.print(",  ");
      // Serial1.print(motor.target); Serial1.print(",  ");
      // Serial1.print(rotary_encoder_rads); Serial1.print(",  ");      
      // Serial1.println(position_error_rads);  

      // Serial1.println(spin_rate_dps - spin_rate2_dps, 2);  // useful for calibrating the second gyro
 
    }  
*/

  }  
  
  else {

    COMPLETE = true;  // The "Controlled UP" part flight has finished
    
    Serial1.println("Motor Amplifier OFF\n");
    motor.disable();

    pinMode(POWER_SWITCH, OUTPUT);  // Pin 12 controls power to the camera
    digitalWrite(POWER_SWITCH, LOW);
    Serial1.println("Camera OFF\n");
  
    delay(1000);
    
    Serial1.println("\n\nData Capture Completed\n"); 

    Serial1.println("\n\nWriting Data to Flash\n\n"); 
    Write_Data_To_Flash();
    delay(500);

    Serial1.println("\n\nWriting Data to Flash Again\n\n"); 
    Write_Data_To_Flash();
    delay(500);

    while (Serial1.available() > 0) Serial1.read(); // clear buffer
  
    while (1) {

      Serial1.println("Type FILE for file operations .......\n");
      delay(1000);
      
      toggle = !toggle;    
      digitalWrite(LED, toggle);   

      while (Serial1.available() > 0) {   
        // get the new byte:
        char inChar = Serial1.read();
        // add it to the inputString:
        Incoming += inChar;    
        if (inChar == '\n') {
          Serial1.println(Incoming);
          if (Incoming.indexOf("FILE") >= 0) {
            START_FILE = true;
            File_Operations();  
          }
  
          else Serial1.println("No Match\n");
          
          Incoming = "";        
        }
  
      }
                   
    }
        
  }
  
}


/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/  

/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/  
