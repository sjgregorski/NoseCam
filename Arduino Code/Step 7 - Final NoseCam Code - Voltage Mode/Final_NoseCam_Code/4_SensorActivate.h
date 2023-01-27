#ifndef _SENSOR_H
#define _SENSOR_H


  Serial1.println("Linking to Adafruit ISM330DHCX");

  if (!ism330dhcx.begin_SPI(LSM_CS)) {
    Serial1.println("Failed to find ISM330DHCX chip");
    while (1) {
      delay(10);
    }
  }

  Serial1.println("ISM330DHCX Found!");

  ism330dhcx.setAccelRange(LSM6DS_ACCEL_RANGE_16_G);
  Serial1.print("Accelerometer range set to: ");
  switch (ism330dhcx.getAccelRange()) {
  case LSM6DS_ACCEL_RANGE_2_G:
    Serial1.println("+-2G");
    break;
  case LSM6DS_ACCEL_RANGE_4_G:
    Serial1.println("+-4G");
    break;
  case LSM6DS_ACCEL_RANGE_8_G:
    Serial1.println("+-8G");
    break;
  case LSM6DS_ACCEL_RANGE_16_G:
    Serial1.println("+-16G");
    break;
  }

  ism330dhcx.setGyroRange(LSM6DS_GYRO_RANGE_1000_DPS);
  Serial1.print("Gyro range set to: ");
  switch (ism330dhcx.getGyroRange()) {
  case LSM6DS_GYRO_RANGE_125_DPS:
    Serial1.println("125 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_250_DPS:
    Serial1.println("250 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_500_DPS:
    Serial1.println("500 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_1000_DPS:
    Serial1.println("1000 degrees/s");
    break;
  case LSM6DS_GYRO_RANGE_2000_DPS:
    Serial1.println("2000 degrees/s");
    break;
  case ISM330DHCX_GYRO_RANGE_4000_DPS:
    Serial1.println("4000 degrees/s");
    break;
  }

  ism330dhcx.setAccelDataRate(LSM6DS_RATE_104_HZ);
  Serial1.print("Accelerometer data rate set to: ");
  switch (ism330dhcx.getAccelDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial1.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial1.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial1.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial1.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial1.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial1.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial1.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial1.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial1.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial1.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial1.println("6.66 KHz");
    break;
  }

  ism330dhcx.setGyroDataRate(LSM6DS_RATE_104_HZ);
  Serial1.print("Gyro data rate set to: ");
  switch (ism330dhcx.getGyroDataRate()) {
  case LSM6DS_RATE_SHUTDOWN:
    Serial1.println("0 Hz");
    break;
  case LSM6DS_RATE_12_5_HZ:
    Serial1.println("12.5 Hz");
    break;
  case LSM6DS_RATE_26_HZ:
    Serial1.println("26 Hz");
    break;
  case LSM6DS_RATE_52_HZ:
    Serial1.println("52 Hz");
    break;
  case LSM6DS_RATE_104_HZ:
    Serial1.println("104 Hz");
    break;
  case LSM6DS_RATE_208_HZ:
    Serial1.println("208 Hz");
    break;
  case LSM6DS_RATE_416_HZ:
    Serial1.println("416 Hz");
    break;
  case LSM6DS_RATE_833_HZ:
    Serial1.println("833 Hz");
    break;
  case LSM6DS_RATE_1_66K_HZ:
    Serial1.println("1.66 KHz");
    break;
  case LSM6DS_RATE_3_33K_HZ:
    Serial1.println("3.33 KHz");
    break;
  case LSM6DS_RATE_6_66K_HZ:
    Serial1.println("6.66 KHz");
    break;
  }

  Serial1.println("\n\n");  


  
  digitalWrite(LSM_CS, HIGH);  // disable ISM330DHCX (primary 6DOF chip)

  delay(100);   
  
  Serial1.println("Linking to Adafruit ICM20649");

  // Try to initialize!
  if (!icm20649.begin_SPI(ICM_CS)) {
    Serial1.println("Failed to find ICM20649 chip");
    while (1) {
      delay(10);
    }
  }
  Serial1.println("ICM20649 Found!");

  
  icm20649.setAccelRange(ICM20649_ACCEL_RANGE_30_G);
  Serial1.print("Accelerometer range set to: ");
  switch (icm20649.getAccelRange()) {
  case ICM20649_ACCEL_RANGE_4_G:
    Serial1.println("+-4G");
    break;
  case ICM20649_ACCEL_RANGE_8_G:
    Serial1.println("+-8G");
    break;
  case ICM20649_ACCEL_RANGE_16_G:
    Serial1.println("+-16G");
    break;
  case ICM20649_ACCEL_RANGE_30_G:
    Serial1.println("+-30G");
    break;
  }

  icm20649.setGyroRange(ICM20649_GYRO_RANGE_2000_DPS);
  Serial1.print("Gyro range set to: ");
  switch (icm20649.getGyroRange()) {
  case ICM20649_GYRO_RANGE_500_DPS:
    Serial1.println("500 degrees/s");
    break;
  case ICM20649_GYRO_RANGE_1000_DPS:
    Serial1.println("1000 degrees/s");
    break;
  case ICM20649_GYRO_RANGE_2000_DPS:
    Serial1.println("2000 degrees/s");
    break;
  case ICM20649_GYRO_RANGE_4000_DPS:
    Serial1.println("4000 degrees/s");
    break;
  }

  icm20649.setAccelRateDivisor(10);
  uint16_t accel_divisor = icm20649.getAccelRateDivisor();
  float accel_rate = 1125 / (1.0 + accel_divisor);

  Serial1.print("Accelerometer data rate divisor set to: ");
  Serial1.println(accel_divisor);
  Serial1.print("Accelerometer data rate (Hz) is approximately: ");
  Serial1.println(accel_rate);

  icm20649.setGyroRateDivisor(10);
  uint8_t gyro_divisor = icm20649.getGyroRateDivisor();
  float gyro_rate = 1100 / (1.0 + gyro_divisor);

  Serial1.print("Gyro data rate divisor set to: ");
  Serial1.println(gyro_divisor);
  Serial1.print("Gyro data rate (Hz) is approximately: ");
  Serial1.println(gyro_rate);
  
  Serial1.println("\n\n");

  digitalWrite(ICM_CS, HIGH);  // disable ICM20649(aux. 6DOF chip)

  delay(100);

#endif  // _SENSOR_H


/**************************************************************************************/
