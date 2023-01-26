#ifndef _DEMO_H
#define _DEMO_H

/*

DEMO mode is meant to mimic the behavior of the actual code as the nose is tipped and spun
The DEMO code does NOT use the gyros to position the nose, so it is fundamentally 
different from the actual code used during flight

NOTE - The variable "tilt bearing" and its variants are used throughout this DEMO function.
HOWEVER - The value being calculated is not really a tilt bearing, since it does not track
which direction the nose is pointing over time.  But it does work for the purposes of the DEMO.

*/

  if (DEMO) {

    // digitalWrite(POWER_SWITCH, HIGH);  // Pin 12 controls power to the camera
    delay(1000);

    motor.velocity_limit = 40;
    motor.P_angle.limit = 40; // MUST follow changes to motor.velocity_limit

    motor.PID_velocity.P = 0.50000;
    motor.PID_velocity.I = 10.00000;
    motor.PID_velocity.D = 0.00000;
    motor.LPF_velocity.Tf = 0.010;   // velocity low pass filtering time constant   // smaller means less filtering
    motor.P_angle.P = 40;   // angle loop PID controller parameters   // not used for torque or velocity control
    motor.P_angle.I = 0;
    motor.P_angle.D = 0;  
  
    motor.enable();

    // If running the DEMO mode then need to do the following items once
  
    /* Get new sensor events */ 
    ism330dhcx.getEvent(&accel, &gyro, &temp);
  
    raw_accel_x_corrected = (accel.acceleration.x - raw_accel_x_offset) * raw_accel_x_scaling;
    raw_accel_y_corrected = (accel.acceleration.y - raw_accel_y_offset) * raw_accel_y_scaling;
    raw_accel_z_corrected = (accel.acceleration.z - raw_accel_z_offset) * raw_accel_z_scaling;

    //  Adjust to the "vertical" orientation of the sensor board
    accel_x_vert = -raw_accel_z_corrected;
    accel_y_vert = raw_accel_y_corrected;
    accel_z_vert = raw_accel_x_corrected;

    // Normalize accelerometer raw values  
    accel_magnitude = sqrt(accel_x_vert * accel_x_vert 
                          + accel_y_vert * accel_y_vert 
                          + accel_z_vert * accel_z_vert);                      
    accel_x_norm = accel_x_vert/accel_magnitude;
    accel_y_norm = accel_y_vert/accel_magnitude;
    accel_z_norm = accel_z_vert/accel_magnitude;

    pitch_rads_from_accel = -atan(accel_x_norm/sqrt((accel_y_norm*accel_y_norm) + (accel_z_norm*accel_z_norm)));
    yaw_rads_from_accel = atan(accel_y_norm/sqrt((accel_x_norm*accel_x_norm) + (accel_z_norm*accel_z_norm))); 
    pitch_degs_from_accel = 57.295779513 * pitch_rads_from_accel;
    yaw_degs_from_accel = 57.295779513 * yaw_rads_from_accel;

    // Rocket tilt calculation      
    a = sin(pitch_rads_from_accel);
    b = cos(pitch_rads_from_accel);  
    c = b * sin(yaw_rads_from_accel);
    e = sqrt( (a*a) + (c*c) );
    tilt_rads = asin(e);
    if (isnan(tilt_rads)) tilt_rads = 1.57079632;  // In case asin(e) is NAN because e > 1 due to floating point errors 
    tilt = 57.295779513 * tilt_rads;
    tilt_on_pad = tilt;

    // Calculate the "Tilt Bearing" using the accel data
    tilt_bearing_deg = 57.295779513 * atan2(yaw_rads_from_accel, pitch_rads_from_accel);

    prev_tilt_bearing_deg = tilt_bearing_deg;
    tilt_bearing_difference = 0;
    delta_tilt_bearing_deg = 0;  
    windup_tilt_bearing_deg = tilt_bearing_deg;
    EWMA_windup_tilt_bearing_deg = 0;  // This will allow for a smooth transition at start of demo
   
    delta_error_signal = 0;  
    windup_error_signal = -tilt_bearing_deg;

    PID_Setpoint_rads = 0;
    PID_Setpoint_degs = 0;

    motor.target = PID_Setpoint_rads;

    PreviousPositionRads = 0;
    CurrentPositionRads = 0;

    loopcount = 0;

    spin_rate_dps = 0;
    
    loop_timer = micros();
    micros_timer = micros();
    prev_micros_timer = micros();
    Timer1 = micros();
    Timer2 = micros();
    Starttime = micros();

    BLDC_loop_duration_us = 333;

    ISM330DHCX_CONVERSION = false;
  
    delay(100);
     
  }


  while (DEMO) {

    while ((micros() - loop_timer) < BLDC_loop_duration_us) {
      delayMicroseconds(10);
    }
   
    loop_timer = micros();
    prev_micros_timer = micros_timer;
    micros_timer = micros();
    loop_duration_micros = micros_timer - prev_micros_timer;

    // the microsecond timer will rollover about every 1.2 hours    
    if (loop_duration_micros > 1000000) loop_duration_micros = BLDC_loop_duration_us;  // in case of rollover 

    motor.loopFOC();
    motor.move();
    
    PreviousPositionRads = CurrentPositionRads;
    rotary_encoder_rads = encoder.getAngle();
    CurrentPositionRads = rotary_encoder_rads - encoder_offset_rads;       
    position_error_rads = motor.target - PreviousPositionRads;        
    position_error_degs = 57.295779513 * position_error_rads;

    motor.P_angle.P = 10 + (40 * abs(spin_rate_dps/360) );
   
    if ( (micros() - Timer1) >= 5000 && !ISM330DHCX_CONVERSION) {

      ISM330DHCX_CONVERSION = true;
      
      // Get new sensor events 
      ism330dhcx.getEvent(&accel, &gyro, &temp);  // About 200us for a single SPI bus conversion
  
      // Perform corrections
      raw_accel_x_corrected = (accel.acceleration.x - raw_accel_x_offset) * raw_accel_x_scaling;
      raw_accel_y_corrected = (accel.acceleration.y - raw_accel_y_offset) * raw_accel_y_scaling;
      raw_accel_z_corrected = (accel.acceleration.z - raw_accel_z_offset) * raw_accel_z_scaling;
 
      // Calculate smoothed values for the accel and gyro
      // These will be used at the moment of launch to determine
      // rocket "tip and tilt" on the pad as well as providing
      // the gyro offsets
  
      accel_x_old_ewma = accel_x_ewma;
      accel_y_old_ewma = accel_y_ewma;
      accel_z_old_ewma = accel_z_ewma;
    
      gyro_x_old_ewma = gyro_x_ewma;
      gyro_y_old_ewma = gyro_y_ewma;
      gyro_z_old_ewma = gyro_z_ewma;
  
      accel_x_ewma = 0.75 * accel_x_old_ewma + 0.25 * raw_accel_x_corrected;
      accel_y_ewma = 0.75 * accel_y_old_ewma + 0.25 * raw_accel_y_corrected;
      accel_z_ewma = 0.75 * accel_z_old_ewma + 0.25 * raw_accel_z_corrected;

      gyro_x_ewma = 0.75 * gyro_x_old_ewma + 0.25 * gyro.gyro.x;
      gyro_y_ewma = 0.75 * gyro_y_old_ewma + 0.25 * gyro.gyro.y;
      gyro_z_ewma = 0.75 * gyro_z_old_ewma + 0.25 * gyro.gyro.z;


      //  Now use the EWMA data adjusted to "vertical" orientation of the sensor board
      //  for the remainder of the calculations in this loop
      accel_x_vert = -accel_z_ewma;
      accel_y_vert = accel_y_ewma;
      accel_z_vert = accel_x_ewma;
      gyro_x_vert = -gyro_z_ewma;
      gyro_y_vert = gyro_y_ewma;
      gyro_z_vert = gyro_x_ewma;    
      
  
      spin_rate_dps = 57.295779513 * gyro_z_vert;
  
    
      // Calculate yaw and pitch based on the EWMA accelerometer data
      // Since the rocket is still "on the pad" at this point it is OK
      // to use the accelerometers
  
      // Normalize accelerometer raw values  
      accel_magnitude = sqrt(accel_x_vert * accel_x_vert 
                            + accel_y_vert * accel_y_vert 
                            + accel_z_vert * accel_z_vert);                      
      accel_x_norm = accel_x_vert/accel_magnitude;
      accel_y_norm = accel_y_vert/accel_magnitude;
      accel_z_norm = accel_z_vert/accel_magnitude;
    
      pitch_rads_from_accel = -atan(accel_x_norm/sqrt((accel_y_norm*accel_y_norm) + (accel_z_norm*accel_z_norm)));
      yaw_rads_from_accel = atan(accel_y_norm/sqrt((accel_x_norm*accel_x_norm) + (accel_z_norm*accel_z_norm))); 
      pitch_degs_from_accel = 57.295779513 * pitch_rads_from_accel;
      yaw_degs_from_accel = 57.295779513 * yaw_rads_from_accel;
  
      // Rocket tilt calculation      
      a = sin(pitch_rads_from_accel);
      b = cos(pitch_rads_from_accel);  
      c = b * sin(yaw_rads_from_accel);
      e = sqrt( (a*a) + (c*c) );
      tilt_rads = asin(e);
      if (isnan(tilt_rads)) tilt_rads = 1.57079632;  // In case asin(e) is NAN because e > 1 due to floating point errors 
      tilt = 57.295779513 * tilt_rads;
      tilt_on_pad = tilt;
  
      // Calculate the "Tilt Bearing" using the accel data
      prev_tilt_bearing_deg = tilt_bearing_deg;    
      tilt_bearing_deg = 57.295779513 * atan2(yaw_rads_from_accel, pitch_rads_from_accel);
  
      // The "tilt_bearing_difference" variable below has a range of -359.99999 to +359.99999
      tilt_bearing_difference = tilt_bearing_deg - prev_tilt_bearing_deg;
  
      // Calculate the shortest Delta rotation - Between -179.99999 and +180.00000
      if ( tilt_bearing_difference <= -180 ) delta_tilt_bearing_deg = 360 + tilt_bearing_difference;
      else if ( tilt_bearing_difference <= 0 ) delta_tilt_bearing_deg = tilt_bearing_difference;
      else if ( tilt_bearing_difference <= 180 ) delta_tilt_bearing_deg = tilt_bearing_difference;
      else delta_tilt_bearing_deg = tilt_bearing_difference - 360;
  
      windup_tilt_bearing_deg += delta_tilt_bearing_deg; // this is like an integration and so error will build up over time

      if (tilt <= 20) {

        while (windup_tilt_bearing_deg - EWMA_windup_tilt_bearing_deg > 180) windup_tilt_bearing_deg -= 360; 
        
        while (windup_tilt_bearing_deg - EWMA_windup_tilt_bearing_deg < -180) windup_tilt_bearing_deg += 360; 
        
      }

      EWMA_windup_tilt_bearing_deg = (0.05 * windup_tilt_bearing_deg) + ( 0.95 * EWMA_windup_tilt_bearing_deg);

      windup_error_signal = -EWMA_windup_tilt_bearing_deg;  // units are degrees
          
      PID_Setpoint_degs = windup_error_signal;

      PID_Setpoint_rads = windup_error_signal / 57.295779513;

      motor.target = PID_Setpoint_rads;
    
    }


    if ( (micros() - Timer1) >= 10000 && ISM330DHCX_CONVERSION) {
     
      Timer1 = micros();
  
      ISM330DHCX_CONVERSION = false;    

      // Get new sensor events
      icm20649.getEvent(&accel, &gyro, &temp);  // About 300us for a single SPI bus conversion

      gyro2_x_vert = -gyro.gyro.z;
      gyro2_y_vert = -gyro.gyro.x;
      gyro2_z_vert = gyro.gyro.y;

      // Perform offsets and corrections
      raw_accel2_x_corrected = (accel.acceleration.x - raw_accel2_x_offset) * raw_accel2_x_scaling;
      raw_accel2_y_corrected = (accel.acceleration.y - raw_accel2_y_offset) * raw_accel2_y_scaling;
      raw_accel2_z_corrected = (accel.acceleration.z - raw_accel2_z_offset) * raw_accel2_z_scaling;
   
      accel2_x_old_ewma = accel2_x_ewma;
      accel2_y_old_ewma = accel2_y_ewma;
      accel2_z_old_ewma = accel2_z_ewma;

      accel2_x_ewma = 0.75 * accel2_x_old_ewma + 0.25 * raw_accel2_x_corrected;
      accel2_y_ewma = 0.75 * accel2_y_old_ewma + 0.25 * raw_accel2_y_corrected;
      accel2_z_ewma = 0.75 * accel2_z_old_ewma + 0.25 * raw_accel2_z_corrected;

      // Adjust to "vertical" orientation of the sensor board
      accel2_x_vert = -accel2_z_ewma;
      accel2_y_vert = -accel2_x_ewma;
      accel2_z_vert = accel2_y_ewma;

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

    }


    if ( (micros() - Timer2) >= 50000 ) {

      Timer2 = micros();      
/*
      Serial1.print(CurrentPosition); Serial1.print(", ");            
      Serial1.print(position_error_rads); Serial1.print(", ");
      Serial1.print(PID_Output,1); Serial1.print(", ");       
      Serial1.println(loop_duration_micros);  
      Serial1.println();
      Serial1.println();
*/


/*
      Serial1.print(accel_x_vert,2); Serial1.print(", ");
      Serial1.print(accel_y_vert,2); Serial1.print(", ");
      Serial1.println(accel_z_vert,2);    

      Serial1.print(pitch_degs_from_accel,1); Serial1.print(", ");
      Serial1.print(yaw_degs_from_accel,1); Serial1.print(", ");
      Serial1.println(tilt_on_pad,1);

      Serial1.println(tilt_bearing_deg,1);
*/


/*     
      Serial1.print(accel2_x_vert,2); Serial1.print(", ");
      Serial1.print(accel2_y_vert,2); Serial1.print(", ");
      Serial1.println(accel2_z_vert,2);    

      Serial1.print(pitch_degs_from_accel2,1); Serial1.print(", ");
      Serial1.print(yaw_degs_from_accel2,1); Serial1.print(", ");
      Serial1.println(tilt2_on_pad,1);

      Serial1.println(tilt_bearing2_deg,1);
     
      Serial1.println("\n\n\n\n\n");
*/

    }

    loopcount++;
 
  }


#endif  // _DEMO_H


/**************************************************************************************/
