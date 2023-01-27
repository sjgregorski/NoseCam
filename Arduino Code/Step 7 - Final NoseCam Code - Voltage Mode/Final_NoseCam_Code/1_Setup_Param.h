#ifndef _SETUP_PARAM_H
#define _SETUP_PARAM_H

//---------------------------------------------------------------------------------------//

  // The following values are for Adafruit ISM330DHCX accelerometer 
  // bias correction 
  raw_accel_x_offset = 0.000;
  raw_accel_y_offset = 0.000;
  raw_accel_z_offset = 0.000;
  raw_accel_x_scaling = 1.000000;
  raw_accel_y_scaling = 1.000000;
  raw_accel_z_scaling = 1.000000;

  // Adafruit ISM330DHCX gyro correct factor for the rocket vertical (spin) axis
  Z_Gyro_Cal = 1.0000;

  // The following values are for Adafruit ICM20649 accelerometer 
  // bias correction
  raw_accel2_x_offset = 0.000;
  raw_accel2_y_offset = 0.000;
  raw_accel2_z_offset = 0.000;
  raw_accel2_x_scaling = 1.000000;
  raw_accel2_y_scaling = 1.000000;
  raw_accel2_z_scaling = 1.000000;

  // Adafruit ICM20649 gyro correct factor for the rocket vertical (spin) axis
  Z_Gyro2_Cal = 1.0000;

  // These parameters are used to dynamically modify the position (angle)
  // loop Proportional and Derivative gain terms during flight
  // They can also be changed during operation using the Commander Interface
  P_Base = 20;
  P_Gain = 20;
  D_Base = 0;
  D_Gain = 2.00;
  Spin_Rate_Adj = 0.55;


//---------------------------------------------------------------------------------------//

//---------------------------------------------------------------------------------------//


  // SimpleFOC driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 25;
    
  driver.init();
  // link the motor and the driver
  motor.linkDriver(&driver);

  // enable/disable quadrature mode
  encoder.quadrature = Quadrature::ON;

  // check if you need internal pullups
  encoder.pullup = Pullup::USE_EXTERN;
  
  // initialize encoder hardware
  encoder.init();
  // hardware interrupt enable
  encoder.enableInterrupts(doA, doB);
  // link the motor to the sensor
  motor.linkSensor(&encoder);

  Serial.println("Encoder Ready\n");


  // choose FOC modulation
  // FOC torque control requires sinusoidal currents
  // therefore use either Sinusoidal PWM or Space vector PWM
  // FOCModulationType::SinePWM (default)
  // FOCModulationType::SpaceVectorPWM
  // FOCModulationType::Trapezoid_120
  // FOCModulationType::Trapezoid_150
  motor.foc_modulation = FOCModulationType::SinePWM;

  // set torque mode to be used
  // TorqueControlType::voltage  (default)
  // TorqueControlType::dc_current  // requires current sense
  // TorqueControlType::foc_current  // requires current sense
  motor.torque_controller = TorqueControlType::voltage;

  // set motion control loop to be used
  // MotionControlType::torque - torque control 
  // MotionControlType::velocity - velocity motion control
  // MotionControlType::angle - position/angle motion control
  // MotionControlType::velocity_openloop - velocity open-loop control
  // MotionControlType::angle_openloop - position open-loop control
  motor.controller = MotionControlType::angle;


  // controller configuration 
  // default parameters in defaults.h
  // controller configuration based on the control type 

  // aligning voltage [V]
  // A higher value is better in that it results in less measurement noise
  motor.voltage_sensor_align = 6;
  // incremental encoder index search velocity [rad/s]
  motor.velocity_index_search = 1.0; // default 1 rad/s

  motor.voltage_limit = 25;  // Set with ALU

  // angle loop velocity limit // Set using the ALV command
  // initial speed for moving to the correct angular offset
  motor.velocity_limit = 1.0;


  // Other parameters
  motor.PID_velocity.P = 0.50000;
  motor.PID_velocity.I = 5.0000;
  motor.PID_velocity.D = 0.00000;
  motor.LPF_velocity.Tf = 0.010;   // velocity low pass filtering time constant   // smaller means less filtering
  motor.P_angle.P = 40;   // angle loop PID controller parameters   // not used for torque or velocity control
  motor.P_angle.I = 0;
  motor.P_angle.D = 0;     

  
  // jerk control it is in Volts/s or Amps/s
  // for most of applications no change is needed 
  motor.PID_velocity.output_ramp = 1000.0;

  // acceleration limit
  motor.P_angle.output_ramp = 1000.0;
  
  motor.useMonitoring(Serial);

  command.add('A', onMotor, "motor");
  command.add('D', P_Base_Call, "P_Base_Call");
  command.add('E', P_Gain_Call, "P_Gain_Call");
  command.add('F', D_Base_Call, "D_Base_Call");
  command.add('G', D_Gain_Call, "D_Gain_Call");
  command.add('S', Rate_Adj_Call, "Rate_Adj_Call");  // For real-time calibration of the feed forward
  command.add('Z', Z_Gyro_Call, "Z_Gyro_Call");   // For real-time calibration of the Z gyro
  command.add('Y', Z_Gyro2_Call, "Z_Gyro2_Call");   // For real-time calibration of the second Z gyro


#endif  // _SETUP_PARAM_H


/**************************************************************************************/
