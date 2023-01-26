// Closed loop motor control example
// Assumes the power supply to the motor amplifier is at 24 volts
#include <SimpleFOC.h>

// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(11); // 11 pole pairs
BLDCDriver3PWM driver = BLDCDriver3PWM(10, 5, 6, 8);

Encoder encoder = Encoder(2, 3, 2560);
// interrupt routine intialization
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}

// commander communication instance
Commander command = Commander(Serial);
void onMotor(char* cmd){ command.motor(&motor, cmd); }


float time_delay_micros;

long loopcount = 0;


void setup() {

  Serial.begin(115200);
  delay(1500);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 24;
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
  Serial.println("Encoder ready\n");


  motor.foc_modulation = FOCModulationType::SinePWM;

  motor.torque_controller = TorqueControlType::voltage;

  motor.controller = MotionControlType::velocity;


  // Aligning voltage [V]
  // A higher value is better in that it results in less measurement noise
  motor.voltage_sensor_align = 6;
  // incremental encoder index search velocity [rad/s]
  motor.velocity_index_search = 1.0; // default 1 rad/s

  // set overall motor limits
  motor.voltage_limit = 24; // Set with ALU when running

  // velocity loop PID controller parameters
  // not used for torque control    
  // default P=0.5 I=10 D=0
  motor.PID_velocity.P = 0.200;
  motor.PID_velocity.I = 5.000;
  motor.PID_velocity.D = 0.000;

  // jerk control it is in Volts/s or Amps/s
  // for most of applications no change is needed 
  motor.PID_velocity.output_ramp = 1000.0;

  // velocity low pass filtering time constant
  // smaller means less filtering
  motor.LPF_velocity.Tf = 0.010;

  motor.useMonitoring(Serial);

  // set the inital target value
  motor.target = 0.0;

  // initialize motor
  motor.init();

  // align encoder and start FOC
  Serial.println("\n\nAligning the encoder");

  motor.initFOC();

  Serial.println("\n\nEncoder Aligned\n\n");
  
  // subscribe motor to the commander
  command.add('A', onMotor, "motor");  // All commands must start with "A"

  Serial.println("\n\nMotor ready\n\n");

  time_delay_micros = 333;
  
}


void loop() {

  // iterative setting FOC phase voltage
  motor.loopFOC();
  
  // iterative function setting the outer loop target
  motor.move();

  // user communication
  command.run();

  if (loopcount % 1000 == 0) {
    encoder.update();
    Serial.println(encoder.getAngle(),4);
  }

  delayMicroseconds(time_delay_micros);

  loopcount++;
  
}

  /*

  Typical commands for velocity control include the following:

  char* my_string = "ALV10";   
  command.run(my_string);
  
  ? - list all commands
  #5 - set decimal point number to 5
  
  AMG or AMG0 - get motor "A" target

  AMG5 - get motor current velocity
  
  AMG6 - get motor current angular position
  
  A5.0 - set motor "A" target to 5.0
    
  AVP - get motor "A" velocity loop PID proportional gain
  AVP0.15 - set motor "A" velocity loop PID proportional gain to 0.15
  AVI
  AVD
  AVR - get motor "A" velocity loop voltage or current ramp limit
  AVL - get motor "A" velocity loop voltage or current limit
  AVF - get motor "A" velocity loop filter time constant  

  AAP - get motor "A" angle (position) loop PID proportional gain  

  AQP - get motor "A" Q current loop PID proportional gain      
  
  ADP - get motor "A" D current loop PID proportional gain    


  ALC - get motor "A" current limit
  ALU - get motor "A" voltage limit
  ALV - get motor "A" velocity limit

  AE - get motor "A" enable (1) / disable (0) status

  AR - get motor "A" phase resistance

  ASM - sensor offset

  ASE - sensor electrical offset


  PhaseCurrent_s currents = current_sense.getPhaseCurrents();
  float current_magnitude = current_sense.getDCCurrent();

  Serial.print(currents.a*1000); // milli Amps
  Serial.print("\t");
  Serial.print(currents.b*1000); // milli Amps
  Serial.print("\t");
  Serial.print(currents.c*1000); // milli Amps
  Serial.print("\t");
  Serial.println(current_magnitude*1000); // milli Amps

  Serial.println(motor.Ua);
  Serial.println(motor.Ub);
  Serial.println(motor.Uc);   

  Serial.println(motor.voltage.q);  
  Serial.println(motor.voltage.d); 
   
  Serial.println(motor.current.q);  
  Serial.println(motor.current.d); 

  Serial.println(motor.shaft_velocity);   
  Serial.println(motor.shaft_velocity_sp);  


  */
  
  /*
  
    // Some useful commands - substitute "encoder" for "sensor" as needed 
    sensor.getMechanicalAngle();
    sensor.getAngle();    
    sensor.getPreciseAngle();
    sensor.getVelocity();
    sensor.getFullRotations(); 
    sensor.update(); 
  
    sensor offset [rad]
    motor.sensor_offset = 0; // default 0 rad
  
    motor.enable(); // just a test
    motor.disable(); // just a test  
  


    // You can skip the alignment and add this to your code
    // (should work only for SimpleFOCshield v2):
    // invert phase b gain
    current_sense.gain_b *=-1;
    // skip alignment
    current_sense.skip_align = true;
    ...
    // align all sensors
    motor.initFOC();

    

    Commander command = Commander(Serial);
    void doTarget(char* cmd) { command.scalar(&target, cmd); }
    void doPidAngle(char* cmd) { command.pid(&motor.P_angle, cmd); }
    void doPidVelo(char* cmd) { command.pid(&motor.PID_velocity, cmd); }
    void doLpf(char* cmd) { command.lpf(&motor.LPF_velocity, cmd); }
    void doMotor(char* cmd) { command.motor(&motor, cmd); }

    // current sense init hardware
    current_sense.init();
    // link the current sense to the motor
    motor.linkCurrentSense(&current_sense);
    current_sense.gain_b *= -1;
    current_sense.skip_align = true;
    // set torque mode:
    motor.torque_controller = TorqueControlType::foc_current; 
    // set motion control loop to be used
    motor.controller = MotionControlType::torque;

    
    motor.zero_electric_angle = 0.00;


*/
  
/*****************************************************************************/
