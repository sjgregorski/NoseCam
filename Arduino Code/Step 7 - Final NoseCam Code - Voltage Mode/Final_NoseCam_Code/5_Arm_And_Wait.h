#ifndef _ARMANDWAIT_H
#define _ARMANDWAIT_H


  motor.velocity_limit = 10;
  motor.P_angle.limit = 10; // MUST follow changes to motor.velocity_limit

  motor.PID_velocity.P = 0.20000;
  motor.PID_velocity.I = 2.00000;
  motor.PID_velocity.D = 0.00000;
  motor.LPF_velocity.Tf = 0.010;   // velocity low pass filtering time constant   // smaller means less filtering
  motor.P_angle.P = 10;   // angle loop PID controller parameters   // not used for torque or velocity control
  motor.P_angle.I = 0;
  motor.P_angle.D = 0;  


  loopcount = 1;
  
  while (!ARM) {

    motor.loopFOC();
    motor.move();

    if ( loopcount % 3000 == 0 ) {
  
      loopcount = 0;     
      
      Serial1.println("Type ARM to continue ...........\n");  
  
      while (!ARM && Serial1.available() > 0) {   
        // get the new byte:
        char inChar = Serial1.read();
        // add it to the inputString:
        Incoming += inChar;    
        if (inChar == '\n') {
          Serial1.println(Incoming);
          if (Incoming.indexOf("ARM") >= 0) {
            Serial1.println("System Armed\n\n");
            ARM = true;
          }
  
          else Serial1.println("No Match\n");
          
          Incoming = "";        
          
        }
  
      }  
    
    }
  
    delayMicroseconds(500);
    
    loopcount++;

  }

  while (Serial1.available() > 0) Serial1.read(); // clear buffer 

  
  /*-------------------------------------------------------------------------------------------------*/
  /*-------------------------------------------------------------------------------------------------*/  


  Serial1.println("Powering camera\n\n\n");  
  digitalWrite(POWER_SWITCH, HIGH);  // Pin 12 controls power to the camera
  delay(1000);


  // Initialize the accelerometer and gyro EWMA filters

  /* Get new sensor events */ 
  ism330dhcx.getEvent(&accel, &gyro, &temp);  // About 200us for a single SPI bus conversion

  raw_accel_x_corrected = (accel.acceleration.x - raw_accel_x_offset) * raw_accel_x_scaling;
  raw_accel_y_corrected = (accel.acceleration.y - raw_accel_y_offset) * raw_accel_y_scaling;
  raw_accel_z_corrected = (accel.acceleration.z - raw_accel_z_offset) * raw_accel_z_scaling;

  accel_x_ewma = raw_accel_x_corrected;
  accel_y_ewma = raw_accel_y_corrected;
  accel_z_ewma = raw_accel_z_corrected;
  accel_x_old_ewma = accel_x_ewma;
  accel_y_old_ewma = accel_y_ewma;
  accel_z_old_ewma = accel_z_ewma;

  gyro_x_ewma = gyro.gyro.x;
  gyro_y_ewma = gyro.gyro.y;
  gyro_z_ewma = gyro.gyro.z;
  gyro_x_old_ewma = gyro_x_ewma;
  gyro_y_old_ewma = gyro_y_ewma;
  gyro_z_old_ewma = gyro_z_ewma;


  icm20649.getEvent(&accel, &gyro, &temp);  // About 300us for a single SPI bus conversion


  // Perform offsets and corrections
  raw_accel2_x_corrected = (accel.acceleration.x - raw_accel2_x_offset) * raw_accel2_x_scaling;
  raw_accel2_y_corrected = (accel.acceleration.y - raw_accel2_y_offset) * raw_accel2_y_scaling;
  raw_accel2_z_corrected = (accel.acceleration.z - raw_accel2_z_offset) * raw_accel2_z_scaling;

  accel2_x_ewma = raw_accel2_x_corrected;
  accel2_y_ewma = raw_accel2_y_corrected;
  accel2_z_ewma = raw_accel2_z_corrected;
  accel2_x_old_ewma = accel2_x_ewma;
  accel2_y_old_ewma = accel2_y_ewma;
  accel2_z_old_ewma = accel2_z_ewma;

  gyro2_x_ewma = gyro.gyro.x;
  gyro2_y_ewma = gyro.gyro.y;
  gyro2_z_ewma = gyro.gyro.z;
  gyro2_x_old_ewma = gyro2_x_ewma;
  gyro2_y_old_ewma = gyro2_y_ewma;
  gyro2_z_old_ewma = gyro2_z_ewma;


  Incoming = "";  
  loopcount = 0;
  loop_timer = micros();


  motor.PID_velocity.P = 0.10000;
  motor.PID_velocity.I = 1.0000;
  motor.PID_velocity.D = 0.00000;
  motor.LPF_velocity.Tf = 0.020;   // velocity low pass filtering time constant   // smaller means less filtering
  motor.P_angle.P = 5;   // angle loop PID controller parameters   // not used for torque or velocity control
  motor.P_angle.I = 0;
  motor.P_angle.D = 0;

  /*-------------------------------------------------------------------------------------------------*/
  /*-------------------------------------------------------------------------------------------------*/  


  while (!LAUNCH) {

    while ((micros() - loop_timer) < target_loop_rate_us) {
      motor.loopFOC();
      motor.move();
      delayMicroseconds(500);
    }

    loop_timer = micros();

    loopcount++;
  
    while (!LAUNCH && Serial1.available() > 0) {   
      // get the new byte:
      char inChar = Serial1.read();
      // add it to the inputString:
      Incoming += inChar;    
      if (inChar == '\n') {
        Serial1.println(Incoming);
        if (Incoming.indexOf("LAUNCH") >= 0) {
          Serial1.println("Launch !!!!!\n");
          LAUNCH = true;
        }

        else Serial1.println("No Match\n");
        
        Incoming = "";        
      }

    }


    // Get new sensor events 
    ism330dhcx.getEvent(&accel, &gyro, &temp);  // About 200us for a single SPI bus conversion

    PrevConversionTimer = ConversionTimer;
    ConversionTimer = micros();
    ConversionTimeMicros = ConversionTimer - PrevConversionTimer;

    // Perform offsets and corrections
    raw_accel_x_corrected = (accel.acceleration.x - raw_accel_x_offset) * raw_accel_x_scaling;
    raw_accel_y_corrected = (accel.acceleration.y - raw_accel_y_offset) * raw_accel_y_scaling;
    raw_accel_z_corrected = (accel.acceleration.z - raw_accel_z_offset) * raw_accel_z_scaling;
 
 
    // First check for an actual launch using unfiltered (real time) accel data
    accel_z_vert = raw_accel_x_corrected;   // Adjust to "vertical" orientation of the sensor board
    if ( accel_z_vert > 30 ) LAUNCH = true;  // about 3G's to trigger (actually 2G's when subtracting gravity)


    // Calculate HIGHLY smoothed values using the accel and gyro raw readings
    // These will be used at the moment of launch to determine
    // rocket "tip and tilt" on the pad as well as providing
    // the gyro offsets

    accel_x_old_ewma = accel_x_ewma;
    accel_y_old_ewma = accel_y_ewma;
    accel_z_old_ewma = accel_z_ewma;
  
    gyro_x_old_ewma = gyro_x_ewma;
    gyro_y_old_ewma = gyro_y_ewma;
    gyro_z_old_ewma = gyro_z_ewma;

    if ( abs(accel.acceleration.x) <= 15 ) {  // recall the X axis is the UP direction for ism330dhcx
      accel_x_ewma = 0.998 * accel_x_old_ewma + 0.002 * raw_accel_x_corrected;
      accel_y_ewma = 0.998 * accel_y_old_ewma + 0.002 * raw_accel_y_corrected;
      accel_z_ewma = 0.998 * accel_z_old_ewma + 0.002 * raw_accel_z_corrected;
    }
    else Serial1.println("skip accel");

    if ( abs(gyro.gyro.x - gyro_x_ewma) <= 0.1 && abs(gyro.gyro.y - gyro_y_ewma) <= 0.1 && abs(gyro.gyro.z - gyro_z_ewma) <= 0.1 ) {
      gyro_x_ewma = 0.998 * gyro_x_old_ewma + 0.002 * gyro.gyro.x;
      gyro_y_ewma = 0.998 * gyro_y_old_ewma + 0.002 * gyro.gyro.y;
      gyro_z_ewma = 0.998 * gyro_z_old_ewma + 0.002 * gyro.gyro.z;
    }
    else Serial1.println("skip gyro");


    // Now use the EWMA data adjusted to "vertical" orientation of the sensor board
    // for the remainder of the calculations in this loop
    accel_x_vert = -accel_z_ewma;
    accel_y_vert = accel_y_ewma;
    accel_z_vert = accel_x_ewma;
    gyro_x_vert = -gyro_z_ewma;
    gyro_y_vert = gyro_y_ewma;
    gyro_z_vert = gyro_x_ewma;     
    

    // Calculate yaw and pitch based on the EWMA accelerometer data
    // Yaw is the same as the "left to right" tilt on the pad
    // Pitch is same as the "forward to back" tilt on the pad
    // Since the rocket is still on the pad at this point it is OK
    // to use the accelerometers for these calculations

    // Normalize accelerometer raw values  
    accel_magnitude = sqrt(accel_x_vert * accel_x_vert 
                          + accel_y_vert * accel_y_vert 
                          + accel_z_vert * accel_z_vert);                      
    accel_x_norm = accel_x_vert/accel_magnitude;
    accel_y_norm = accel_y_vert/accel_magnitude;
    accel_z_norm = accel_z_vert/accel_magnitude;
  
    // Old equations - They worked fime but the new equations use all three accelerometer axes
    // pitch_rads_from_accel = -asin(accel_x_norm);
    // yaw_rads_from_accel = asin(accel_y_norm/cos(pitch_rads_from_accel));
    
    pitch_rads_from_accel = -atan(accel_x_norm/sqrt((accel_y_norm*accel_y_norm) + (accel_z_norm*accel_z_norm)));
    yaw_rads_from_accel = atan(accel_y_norm/sqrt((accel_x_norm*accel_x_norm) + (accel_z_norm*accel_z_norm))); 
    pitch_degs_from_accel = 57.295779513 * pitch_rads_from_accel;
    yaw_degs_from_accel = 57.295779513 * yaw_rads_from_accel;

    // Rocket tilt calculation based on pad accel data       
    a = sin(pitch_rads_from_accel);
    b = cos(pitch_rads_from_accel);  
    c = b * sin(yaw_rads_from_accel);
    e = sqrt( (a*a) + (c*c) );
    tilt_rads = asin(e);
    if (isnan(tilt_rads)) tilt_rads = 1.57079632;  // In case asin(e) is NAN because e > 1 due to floating point errors 
    tilt = 57.295779513 * tilt_rads;
    tilt_on_pad = tilt;

    // Calculate the "on the pad" Tilt Bearing using the EWMA accel data
    // This value will be used as the starting seed for the actual gyro-derived tilt bearing
    // which is calculated during the flight of the rocket
    tilt_bearing_deg = 57.295779513 * atan2(yaw_rads_from_accel, pitch_rads_from_accel);


    icm20649.getEvent(&accel, &gyro, &temp);  // About 300us for a single SPI bus conversion

    PrevConversionTimer2 = ConversionTimer2;
    ConversionTimer2 = micros();
    ConversionTime2Micros = ConversionTimer2 - PrevConversionTimer2;

    // Perform offsets and corrections
    raw_accel2_x_corrected = (accel.acceleration.x - raw_accel2_x_offset) * raw_accel2_x_scaling;
    raw_accel2_y_corrected = (accel.acceleration.y - raw_accel2_y_offset) * raw_accel2_y_scaling;
    raw_accel2_z_corrected = (accel.acceleration.z - raw_accel2_z_offset) * raw_accel2_z_scaling;
 
    accel2_x_old_ewma = accel2_x_ewma;
    accel2_y_old_ewma = accel2_y_ewma;
    accel2_z_old_ewma = accel2_z_ewma;
  
    gyro2_x_old_ewma = gyro2_x_ewma;
    gyro2_y_old_ewma = gyro2_y_ewma;
    gyro2_z_old_ewma = gyro2_z_ewma;

    if ( abs(accel.acceleration.y) <= 15 ) {  // recall the Y axis is the UP direction for icm20649
      accel2_x_ewma = 0.998 * accel2_x_old_ewma + 0.002 * raw_accel2_x_corrected;
      accel2_y_ewma = 0.998 * accel2_y_old_ewma + 0.002 * raw_accel2_y_corrected;
      accel2_z_ewma = 0.998 * accel2_z_old_ewma + 0.002 * raw_accel2_z_corrected;
    }

    if ( abs(gyro.gyro.x - gyro2_x_ewma) <= 0.1 && abs(gyro.gyro.y - gyro2_y_ewma) <= 0.1 && abs(gyro.gyro.z - gyro2_z_ewma) <= 0.1 ) {
      gyro2_x_ewma = 0.998 * gyro2_x_old_ewma + 0.002 * gyro.gyro.x;
      gyro2_y_ewma = 0.998 * gyro2_y_old_ewma + 0.002 * gyro.gyro.y;
      gyro2_z_ewma = 0.998 * gyro2_z_old_ewma + 0.002 * gyro.gyro.z;
    }


    //  Now use the EWMA data adjusted to "vertical" orientation of the sensor board
    //  for the remainder of the calculations in this loop
    accel2_x_vert = -accel2_z_ewma;
    accel2_y_vert = -accel2_x_ewma;
    accel2_z_vert = accel2_y_ewma;
    gyro2_x_vert = -gyro2_z_ewma;
    gyro2_y_vert = -gyro2_x_ewma;
    gyro2_z_vert = gyro2_y_ewma;  


    // Calculate yaw and pitch based on the EWMA accelerometer data
    // Yaw is the same as the "left to right" tilt on the pad
    // Pitch is same as the "forward to back" tilt on the pad
    // Since the rocket is still on the pad at this point it is OK
    // to use the accelerometers for these calculations

    // Normalize accelerometer raw values  
    accel2_magnitude = sqrt(accel2_x_vert * accel2_x_vert 
                          + accel2_y_vert * accel2_y_vert 
                          + accel2_z_vert * accel2_z_vert);                      
    accel2_x_norm = accel2_x_vert/accel2_magnitude;
    accel2_y_norm = accel2_y_vert/accel2_magnitude;
    accel2_z_norm = accel2_z_vert/accel2_magnitude;
  
    pitch_rads_from_accel2 = -atan(accel2_x_norm/sqrt((accel2_y_norm*accel2_y_norm) + (accel2_z_norm*accel2_z_norm)));
    yaw_rads_from_accel2 = atan(accel2_y_norm/sqrt((accel2_x_norm*accel2_x_norm) + (accel2_z_norm*accel2_z_norm))); 
    pitch_degs_from_accel2 = 57.295779513 * pitch_rads_from_accel2;
    yaw_degs_from_accel2 = 57.295779513 * yaw_rads_from_accel2;

    // Rocket tilt calculation based on pad accel data       
    a2 = sin(pitch_rads_from_accel2);
    b2 = cos(pitch_rads_from_accel2);  
    c2 = b2 * sin(yaw_rads_from_accel2);
    e2 = sqrt( (a2*a2) + (c2*c2) );
    tilt2_rads = asin(e2);
    if (isnan(tilt2_rads)) tilt2_rads = 1.57079632;  // In case asin(e) is NAN because e > 1 due to floating point errors 
    tilt2 = 57.295779513 * tilt2_rads;
    tilt2_on_pad = tilt2;


    // Calculate the "on the pad" Tilt Bearing using the EWMA accel data
    // This value will be used as the starting seed for the actual gyro-derived tilt bearing
    // which is calculated during the flight of the rocket
    tilt_bearing2_deg = 57.295779513 * atan2(yaw_rads_from_accel2, pitch_rads_from_accel2);


    if (loopcount % 100 == 0 && !LAUNCH) {
  
      loopcount = 0;
      
      Serial1.print("Waiting for launch ..... Or type LAUNCH to continue\n");  

      // Serial1.print(accel_x_vert,2); Serial1.print(", ");
      // Serial1.print(accel_y_vert,2); Serial1.print(", ");
      // Serial1.println(accel_z_vert,2);    

      Serial1.print(gyro_x_vert,4); Serial1.print(", ");
      Serial1.print(gyro_y_vert,4); Serial1.print(", ");
      Serial1.println(gyro_z_vert,4);

      Serial1.print(pitch_degs_from_accel,1); Serial1.print(", ");
      Serial1.print(yaw_degs_from_accel,1); Serial1.print(", ");
      Serial1.println(tilt_on_pad,1);

      Serial1.println(tilt_bearing_deg,1);

      Serial1.println();
      Serial1.println();

/*
      Serial1.print(accel2_x_vert,2); Serial1.print(", ");
      Serial1.print(accel2_y_vert,2); Serial1.print(", ");
      Serial1.println(accel2_z_vert,2);    

      Serial1.print(gyro2_x_vert,4); Serial1.print(", ");  // 2.6"
      Serial1.print(gyro2_y_vert,4); Serial1.print(", ");  // 2.6"
      Serial1.println(gyro2_z_vert,4);  // 2.6"

      Serial1.print(pitch_degs_from_accel2,1); Serial1.print(", ");
      Serial1.print(yaw_degs_from_accel2,1); Serial1.print(", ");
      Serial1.println(tilt2_on_pad,1);

      Serial1.println(tilt_bearing2_deg, 1);

      Serial1.println("\n\n\n\n\n");
*/
    }

  }


#endif  // _ARMANDWAIT_H


/**************************************************************************************/
