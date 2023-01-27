// Basic demo for accelerometer/gyro readings from Adafruit ISM330DHCX

#include <Adafruit_ISM330DHCX.h>
#include <Adafruit_ICM20X.h>
#include <Adafruit_ICM20649.h>
#include <Adafruit_Sensor.h>

// For SPI mode, we need a CS pin
#define LSM_CS 7
#define ICM_CS 9

#define POWER_SWITCH 12   // Pin 12 controls power to the camera and servos

Adafruit_ISM330DHCX ism330dhcx;
Adafruit_ICM20649 icm20649;

uint32_t timer1, timer2, convert_time;

sensors_event_t accel;
sensors_event_t gyro;
sensors_event_t temp;

long loopcount;

// The following values are for Adafruit ISM330DHCX accelerometer 
// bias correction
float raw_accel_x_offset = 0.000;
float raw_accel_y_offset = 0.000;
float raw_accel_z_offset = 0.000;
float raw_accel_x_scaling = 1.000;
float raw_accel_y_scaling = 1.000;
float raw_accel_z_scaling = 1.000;


// The following values are for Adafruit ICM20649 accelerometer 
// bias correction
float raw_accel2_x_offset = 0.000;
float raw_accel2_y_offset = 0.000;
float raw_accel2_z_offset = 0.000;
float raw_accel2_x_scaling = 1.000;
float raw_accel2_y_scaling = 1.000;
float raw_accel2_z_scaling = 1.000;

float raw_accel_x_corrected, raw_accel_y_corrected, raw_accel_z_corrected;
float raw_accel2_x_corrected, raw_accel2_y_corrected, raw_accel2_z_corrected;

float accel_1_EWMA_x, accel_1_EWMA_y, accel_1_EWMA_z;
float accel_2_EWMA_x, accel_2_EWMA_y, accel_2_EWMA_z;
float gyro_1_EWMA_x, gyro_1_EWMA_y, gyro_1_EWMA_z;
float gyro_2_EWMA_x, gyro_2_EWMA_y, gyro_2_EWMA_z;

float weight = 0.90;
float counter_weight;

boolean TOGGLE = false;


void setup() {

  Serial.begin(115200);
  Serial1.begin(57600);  // Bluetooth connection  

  pinMode(LSM_CS, OUTPUT); 
  pinMode(ICM_CS, OUTPUT);   

  pinMode(POWER_SWITCH, OUTPUT);  // Pin 12 controls power to the camera and servos
  digitalWrite(POWER_SWITCH, LOW);

  counter_weight = 1.0f - weight;
  
  delay(2000);


  Serial.println("\n\n\nAdafruit ISM330DHCX test!");

  if (!ism330dhcx.begin_SPI(LSM_CS)) {
    Serial.println("Failed to find ISM330DHCX chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("ISM330DHCX Found!");


  ism330dhcx.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  Serial.print("Accelerometer range set to: ");
  switch (ism330dhcx.getAccelRange()) {
  case LSM6DS_ACCEL_RANGE_2_G:
    Serial.println("+-2G");
    break;
  case LSM6DS_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case LSM6DS_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case LSM6DS_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  }

  ism330dhcx.setGyroRange(LSM6DS_GYRO_RANGE_1000_DPS);
  Serial.print("Gyro range set to: ");
  switch (ism330dhcx.getGyroRange()) {
  case LSM6DS_GYRO_RANGE_125_DPS:
    Serial.println("125 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_250_DPS:
    Serial.println("250 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  case ISM330DHCX_GYRO_RANGE_4000_DPS:
    Serial.println("4000 degrees/s");
    break;
  }

  ism330dhcx.setAccelDataRate(LSM6DS_RATE_104_HZ);
  Serial.print("Accelerometer data rate set to: ");
  switch (ism330dhcx.getAccelDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }

  ism330dhcx.setGyroDataRate(LSM6DS_RATE_104_HZ);
  Serial.print("Gyro data rate set to: ");
  switch (ism330dhcx.getGyroDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial.println("6.66 KHz");
    break;
  }


  Serial.println("\n\n");

  digitalWrite(LSM_CS, HIGH);  // disable
  // digitalWrite(ICM_CS, LOW);  // enable 

  delay(100);   

  
  Serial.println("Adafruit ICM20649 test!");

  // Try to initialize!
  // if (!icm20649.begin_I2C()) {
  if (!icm20649.begin_SPI(ICM_CS)) {
    // if (!icm20649.begin_SPI(ICM_CS, ICM_SCK, ICM_MISO, ICM_MOSI)) {
    Serial.println("Failed to find ICM20649 chip");
    while (1) {
      delay(10);
    }
  }
  Serial.println("ICM20649 Found!");

  
  icm20649.setAccelRange(ICM20649_ACCEL_RANGE_30_G);
  Serial.print("Accelerometer range set to: ");
  switch (icm20649.getAccelRange()) {
  case ICM20649_ACCEL_RANGE_4_G:
    Serial.println("+-4G");
    break;
  case ICM20649_ACCEL_RANGE_8_G:
    Serial.println("+-8G");
    break;
  case ICM20649_ACCEL_RANGE_16_G:
    Serial.println("+-16G");
    break;
  case ICM20649_ACCEL_RANGE_30_G:
    Serial.println("+-30G");
    break;
  }

  icm20649.setGyroRange(ICM20649_GYRO_RANGE_1000_DPS);
  Serial.print("Gyro range set to: ");
  switch (icm20649.getGyroRange()) {
  case ICM20649_GYRO_RANGE_500_DPS:
    Serial.println("500 degrees/s");
    break;
  case ICM20649_GYRO_RANGE_1000_DPS:
    Serial.println("1000 degrees/s");
    break;
  case ICM20649_GYRO_RANGE_2000_DPS:
    Serial.println("2000 degrees/s");
    break;
  case ICM20649_GYRO_RANGE_4000_DPS:
    Serial.println("4000 degrees/s");
    break;
  }

  icm20649.setAccelRateDivisor(10);
  uint16_t accel_divisor = icm20649.getAccelRateDivisor();
  float accel_rate = 1125 / (1.0 + accel_divisor);

  Serial.print("Accelerometer data rate divisor set to: ");
  Serial.println(accel_divisor);
  Serial.print("Accelerometer data rate (Hz) is approximately: ");
  Serial.println(accel_rate);

  icm20649.setGyroRateDivisor(10);
  uint8_t gyro_divisor = icm20649.getGyroRateDivisor();
  float gyro_rate = 1100 / (1.0 + gyro_divisor);

  Serial.print("Gyro data rate divisor set to: ");
  Serial.println(gyro_divisor);
  Serial.print("Gyro data rate (Hz) is approximately: ");
  Serial.println(gyro_rate);
  Serial.println("\n\n");

  delay(2000);
  
}

void loop() {

  // digitalWrite(LSM_CS, LOW);  // enable
  digitalWrite(ICM_CS, HIGH);  // disable  
  
  // Get a new normalized sensor event
  timer1 = micros();
  ism330dhcx.getEvent(&accel, &gyro, &temp);
  timer2 = micros();
  convert_time = timer2 - timer1;

  // Perform offsets and corrections
  raw_accel_x_corrected = (accel.acceleration.x - raw_accel_x_offset) * raw_accel_x_scaling;
  raw_accel_y_corrected = (accel.acceleration.y - raw_accel_y_offset) * raw_accel_y_scaling;
  raw_accel_z_corrected = (accel.acceleration.z - raw_accel_z_offset) * raw_accel_z_scaling;

  accel_1_EWMA_x = (weight * accel_1_EWMA_x) + (counter_weight * raw_accel_x_corrected);
  accel_1_EWMA_y = (weight * accel_1_EWMA_y) + (counter_weight * raw_accel_y_corrected);
  accel_1_EWMA_z = (weight * accel_1_EWMA_z) + (counter_weight * raw_accel_z_corrected);
  
  gyro_1_EWMA_x = (weight * gyro_1_EWMA_x) + (counter_weight * gyro.gyro.x);
  gyro_1_EWMA_y = (weight * gyro_1_EWMA_y) + (counter_weight * gyro.gyro.y);
  gyro_1_EWMA_z = (weight * gyro_1_EWMA_z) + (counter_weight * gyro.gyro.z);


  if (loopcount % 200 == 0) {
    TOGGLE = !TOGGLE;
    if (TOGGLE) {
      digitalWrite(POWER_SWITCH, HIGH);
    }
    else {
      digitalWrite(POWER_SWITCH, LOW);
    }
  }

  if (loopcount % 50 == 0) {

    Serial.print("ism330dhcx conversion time [us]: ");    
    Serial.print(convert_time); Serial.print("\n");

    /* Display the results (acceleration is measured in m/s^2) */
    Serial.print(accel_1_EWMA_x,4);
    Serial.print(", ");
    Serial.print(accel_1_EWMA_y,4);
    Serial.print(", ");
    Serial.println(accel_1_EWMA_z,4);
  
    /* Display the results (rotation is measured in rad/s) */
    Serial.print(gyro_1_EWMA_x,4);
    Serial.print(", ");
    Serial.print(gyro_1_EWMA_y,4);
    Serial.print(", ");
    Serial.print(gyro_1_EWMA_z,4);
    Serial.println("\n");
  
  
    Serial1.print("ism330dhcx conversion time [us]: "); 
    Serial1.print(convert_time); Serial1.print("\n");
  
    /* Display the results (acceleration is measured in m/s^2) */
    Serial1.print(accel_1_EWMA_x,4);
    Serial1.print(", ");
    Serial1.print(accel_1_EWMA_y,4);
    Serial1.print(", ");
    Serial1.println(accel_1_EWMA_z,4);
  
    /* Display the results (rotation is measured in rad/s) */
    Serial1.print(gyro_1_EWMA_x,4);
    Serial1.print(", ");
    Serial1.print(gyro_1_EWMA_y,4);
    Serial1.print(", ");
    Serial1.print(gyro_1_EWMA_z,4);
    Serial1.println("\n");
      
  }


  digitalWrite(LSM_CS, HIGH);  // disable
  // digitalWrite(ICM_CS, LOW);  // enable 

  // Get a new normalized sensor event
  timer1 = micros();
  icm20649.getEvent(&accel, &gyro, &temp);
  timer2 = micros();
  convert_time = timer2 - timer1;

  // Perform offsets and corrections
  raw_accel2_x_corrected = (accel.acceleration.x - raw_accel2_x_offset) * raw_accel2_x_scaling;
  raw_accel2_y_corrected = (accel.acceleration.y - raw_accel2_y_offset) * raw_accel2_y_scaling;
  raw_accel2_z_corrected = (accel.acceleration.z - raw_accel2_z_offset) * raw_accel2_z_scaling;

  accel_2_EWMA_x = (weight * accel_2_EWMA_x) + (counter_weight * raw_accel2_x_corrected);
  accel_2_EWMA_y = (weight * accel_2_EWMA_y) + (counter_weight * raw_accel2_y_corrected);
  accel_2_EWMA_z = (weight * accel_2_EWMA_z) + (counter_weight * raw_accel2_z_corrected);
  
  gyro_2_EWMA_x = (weight * gyro_2_EWMA_x) + (counter_weight * gyro.gyro.x);
  gyro_2_EWMA_y = (weight * gyro_2_EWMA_y) + (counter_weight * gyro.gyro.y);
  gyro_2_EWMA_z = (weight * gyro_2_EWMA_z) + (counter_weight * gyro.gyro.z);


  if (loopcount % 50 == 0) {

    Serial.print("icm20649 conversion time [us]: ");  
    Serial.print(convert_time); Serial.print("\n");
  
    /* Display the results (acceleration is measured in m/s^2) */
    Serial.print(accel_2_EWMA_x,4);
    Serial.print(", ");
    Serial.print(accel_2_EWMA_y,4);
    Serial.print(", ");
    Serial.println(accel_2_EWMA_z,4);
  
    /* Display the results (rotation is measured in rad/s) */
    Serial.print(gyro_2_EWMA_x,4);
    Serial.print(", ");
    Serial.print(gyro_2_EWMA_y,4);
    Serial.print(", ");
    Serial.print(gyro_2_EWMA_z,4);
    Serial.println("\n\n\n\n");
  
  
    Serial1.print("icm20649 conversion time [us]: ");  
    Serial1.print(convert_time); Serial1.print("\n");
  
    /* Display the results (acceleration is measured in m/s^2) */
    Serial1.print(accel_2_EWMA_x,4);
    Serial1.print(", ");
    Serial1.print(accel_2_EWMA_y,4);
    Serial1.print(", ");
    Serial1.println(accel_2_EWMA_z,4);
  
    /* Display the results (rotation is measured in rad/s) */
    Serial1.print(gyro_2_EWMA_x,4);
    Serial1.print(", ");
    Serial1.print(gyro_2_EWMA_y,4);
    Serial1.print(", ");
    Serial1.print(gyro_2_EWMA_z,4);
    Serial1.println("\n\n\n\n");
    
  }

  loopcount++;

  delayMicroseconds(10000);

}





/******************************************/
