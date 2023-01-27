// Open loop motor control example
// Assumes the power supply to the motor amplifier is at 16 volts
#include <SimpleFOC.h>

long loopcount = 0;

// BLDC motor & driver instance
// BLDCMotor motor = BLDCMotor(pole pair number);
BLDCMotor motor = BLDCMotor(11);
// BLDCDriver3PWM driver = BLDCDriver3PWM(pwmA, pwmB, pwmC, Enable(optional));
BLDCDriver3PWM driver = BLDCDriver3PWM(10, 5, 6, 8);

Encoder encoder = Encoder(2, 3, 2560);
// interrupt routine intialization
void doA(){encoder.handleA();}
void doB(){encoder.handleB();}

// instantiate the commander
Commander command = Commander(Serial);
void onMotor(char* cmd){ command.motor(&motor, cmd); }

void setup() {

  Serial.begin(115200);
  delay(1500);

  // driver config
  // power supply voltage [V]
  driver.voltage_power_supply = 16;
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

  // limiting motor movements
  motor.voltage_limit = 5;   // ALU
  motor.velocity_limit = 20; // ALV
 
  // open loop control config
  motor.controller = MotionControlType::angle_openloop;

  // init motor hardware
  motor.init();

  // All other commands to change config parameters must start with an "A"
  command.add('A', onMotor, "motor");

  // Also really helpful ..........
  // Command "AE0" to disable the motor amplifier
  // Command "AE1" to enable the motor amplifier
  
  // subscribe motor to the commander
  Serial.println("Motor ready!");
  Serial.println("Set target angle [rad]\n\n");
  delay(1000);
  
}

void loop() {

  motor.move();  

  encoder.update();
  
  if (loopcount % 10000 == 0 ) {
    Serial.println(encoder.getAngle(),4);
  }
  
  // user communication
  command.run();

  loopcount++;
  
}




/*******************************/
