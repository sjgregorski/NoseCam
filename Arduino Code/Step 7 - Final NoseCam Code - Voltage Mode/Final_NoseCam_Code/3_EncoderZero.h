#ifndef _ENCODER_H
#define _ENCODER_H


  // set the inital target value
  motor.target = 0.0;

  encoder_offset_rads = 0.0;
  motor.sensor_offset = encoder_offset_rads;

  Serial.println("Aligning the Encoder");
  Serial1.println("Aligning the Encoder");
  motor.init();
  motor.initFOC();

  Serial.print("\n\nSet Zero Position Manually Now\n\n");
  Serial1.print("\n\nSet Zero Position Manually Now\n\n");
  motor.disable();

  ZERO = false;

  while (!ZERO) {
  
    Serial1.println("Type ZERO to align encoder\n");
    delay(1000);
  
    while (!ZERO && Serial1.available() > 0) {   
      // get the new byte:
      char inChar = Serial1.read();
      // add it to the inputString:
      Incoming += inChar;    
      if (inChar == '\n') {
        Serial1.println(Incoming);
        
        if (Incoming.indexOf("ZERO") >= 0) {
          ZERO = true;
        }

        else Serial1.println("No Match\n");
          
        Incoming = "";   
          
      }
           
    }

  }

  while (Serial1.available() > 0) Serial1.read(); // clear buffer

  delay(500);

  encoder.update();

  encoder_offset_rads = encoder.getAngle();
  
  motor.sensor_offset = encoder_offset_rads;  
  
  motor.enable();

  Serial.println("\nEncoder Aligned\n\n");
  Serial.println("Motor ready\n\n");


#endif  // _ENCODER_H


/**************************************************************************************/
