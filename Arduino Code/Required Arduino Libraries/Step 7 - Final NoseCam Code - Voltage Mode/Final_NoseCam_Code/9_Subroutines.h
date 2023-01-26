#ifndef _SUB_H
#define _SUB_H


/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/  


void Write_Data_To_Flash() {
    
  // create a new data log file
  char filename[] = "LOGGER00.CSV";
  for (uint8_t i = 0; i < 100; i++) {
    filename[6] = i/10 + '0';
    filename[7] = i%10 + '0';
    if (! fatfs.exists(filename)) {
      // only open a new file if it doesn't exist
      DataFile = fatfs.open(filename, FILE_WRITE); 
      break;  // leave the loop!
    }
  }

  DataFile.print(F("Time[s],Spin Rate DPS,Lat Accel Mag,Accel Z,Accel2 Z,Vel m/s,Alt m,Z Gyro Degs,Z Gyro2 Degs,"));   
  DataFile.print(F("Tilt Degs,Tilt2 Degs,EWMA Euler Tilt Bearing,EWMA Euler Tilt Bearing2,Motor Pos Setpoint Degs,"));  
  DataFile.println(F("Motor Pos Error Degs"));    
  
  DataFile.print(F("Pre-Launch Roll Pitch and Tilt Degs From Accel:"));
  DataFile.print(",,,,,"); 
  DataFile.print(yaw_degs_from_accel,2); 
  DataFile.print(",");     
  DataFile.print(pitch_degs_from_accel,2);   
  DataFile.print(",");     
  DataFile.println(tilt_on_pad,2);    
  DataFile.flush();

  DataFile.print("Gyro Zero Rate Offset:");
  DataFile.print(",,,,,");   
  DataFile.print(gyro_x_offset,6);  DataFile.print(", ");
  DataFile.print(gyro_y_offset,6);  DataFile.print(", ");
  DataFile.print(gyro_z_offset,6);  DataFile.print(", ");
  DataFile.println();

  DataFile.print("Gyro2 Zero Rate Offset:");
  DataFile.print(",,,,,");   
  DataFile.print(gyro2_x_offset,6);  DataFile.print(", ");
  DataFile.print(gyro2_y_offset,6);  DataFile.print(", ");
  DataFile.print(gyro2_z_offset,6);  DataFile.print(", ");
  DataFile.println();

  DataFile.println();  
  DataFile.flush();


  for (memorycount = 0; memorycount <= 2000; memorycount++) {

    DataFile.print(float(time_array[memorycount])/1000000,6);
    DataFile.print(",");         
    DataFile.print(float(spin_rate_dps_array[memorycount])/20,2);   
    DataFile.print(",");   
    DataFile.print(float(lat_accel_mag[memorycount])/1000,3);   
    DataFile.print(",");        
    DataFile.print(float(accel_z_vert_array[memorycount])/1000,3);   
    DataFile.print(","); 
    DataFile.print(float(accel2_z_vert_array[memorycount])/1000,3);   
    DataFile.print(",");   
    DataFile.print(float(vel_mps_array[memorycount])/10,2);   
    DataFile.print(",");   
    DataFile.print(float(alt_m_array[memorycount])/10,1);     
    DataFile.print(",");        
    DataFile.print(bearing_deg_from_intg_z_rotation_array[memorycount]);   
    DataFile.print(",");  
    DataFile.print(bearing2_deg_from_intg_z_rotation_array[memorycount]);   
    DataFile.print(",");        
    DataFile.print(float(tilt_degs_array[memorycount])/100,1);     
    DataFile.print(",");
    DataFile.print(float(tilt2_degs_array[memorycount])/100,1);     
    DataFile.print(",");     
    DataFile.print(float(tilt_bearing_deg_array[memorycount])/10,2);      
    DataFile.print(",");
    DataFile.print(float(tilt_bearing2_deg_array[memorycount])/10,2);      
    DataFile.print(",");    
    DataFile.print(motor_pos_setpoint_array[memorycount],3); 
    DataFile.print(",");     
    DataFile.println(motor_pos_error_array[memorycount],3); 

    if (memorycount % 25 == 0) {
      DataFile.flush(); 
    }

    if (memorycount % 50 == 0) {
      Serial1.print("Transfering ...");
      Serial1.print(float(memorycount)/20,1);
      Serial1.println("%");
    }
 
  }   

  DataFile.flush();    
  DataFile.close();  

}


/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/  


void List_Files() {

  File root;
  File file;

  int count = 0;

  if (!root.open("/")) {
    Serial1.println("open root failed");
  }
  
  root.rewindDirectory();

  while (file.openNext(&root, O_RDONLY)) {
    count++;
    Serial1.print(count);
    Serial1.print("    ");  
    
    file.printName(&Serial1);
    if (file.isDir()) {
      // Indicate a directory.
      Serial1.write('/');
    }

    Serial1.print("\t");    
    file.printFileSize(&Serial1);

    Serial1.println();
    file.close();
  }
  
  if (root.getError()) {
    Serial1.println("\nopenNext failed\n");
  } 

  number_of_files = count;

  Serial1.print("\nNumber of files and directories is ");
  Serial1.println(number_of_files);
  Serial1.println("\n");

  root.close(); 
 
}


/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/  


void Print_File() {

  String Text_Input;

  File root;
  File file;
 
  int file_number = 0;

  Serial1.println("Which File Number ???\n");

  while (Serial1.available() == 0) delay(10); 
  file_number = Serial1.parseInt();
  while (Serial1.available() > 0) Serial1.read();  // clear buffer after reading one integer

  Serial1.print("You want to list the contents of file number: "); 
  Serial1.println(file_number);   
  Serial1.print("\n\n"); 

  if ( file_number > number_of_files ) {
    Serial1.println("Not a Valid File Number !!!\n\n");
    return;
  }

  root.open("/");
  
  root.rewindDirectory();

  for (int x = 1; x < file_number; x++) {
    file.openNext(&root, O_RDONLY);
    file.close();
  }

  file.openNext(&root, O_RDONLY);

  char filename[64];
  file.getName(filename, sizeof(filename));
  Serial1.print("Printing file "); Serial1.println(filename);
  Serial1.print("\n\n"); 

  while (file.available()) {
    char c = file.read();
    Serial.print(c);
    delayMicroseconds(250);
    Serial1.print(c);
    delayMicroseconds(250);
  }

  Serial.print("\n\n"); 
  Serial1.print("\n\n"); 

  file.close();
     
  root.close(); 
    
}


/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/  


void Del_File() {

  String Text_Input;

  File root;
  File file;
 
  int file_number = 0;

  Serial1.println("Which File Number ???\n");

  while (Serial1.available() == 0) delay(10); 
  file_number = Serial1.parseInt();
  while (Serial1.available() > 0) Serial1.read();  // clear buffer after reading one integer

  Serial1.print("You want to delete file number: "); 
  Serial1.println(file_number);   
  Serial1.print("\n"); 

  if ( file_number > number_of_files ) {
    Serial1.println("Not a Valid File Number !!!\n\n");
    return;
  }


  root.open("/");
  
  root.rewindDirectory();

  for (int x = 1; x < file_number; x++) {
    file.openNext(&root, O_RDONLY);
    file.close();
  }

  file.openNext(&root, O_RDONLY);
  char filename[64];
  file.getName(filename, sizeof(filename));
  Serial1.print("Remove file "); Serial1.println(filename);

  file.close();

  fatfs.remove(filename);

  Serial1.print("\n\n"); 
 
  root.close(); 
     
}


/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/  


void File_Operations() {

  String Text_Input;

  Serial1.println("\n\nBeginning File Operations\n"); 

  Serial.begin(115200);  // USB connection
  
  while (1) {

    Serial1.println("Directory Contents:\n");
    
    List_Files();
    List_Files();  // Double list to prevent incorrect listing

    Serial1.println("Type PRINT to export a file");
    Serial1.println("Type DEL to delete a file\n\n");    

    while (Serial1.available() == 0) delay(10);
  
    while (Serial1.available() > 0) {   
      // get the new byte:
      char inChar = Serial1.read();
      // add it to the inputString:
      Text_Input += inChar;    
      if (inChar == '\n') {
        
        Serial1.println(Text_Input);

        if (Text_Input.indexOf("PRINT") >= 0) {
          Print_File();   
        }

        else if (Text_Input.indexOf("DEL") >= 0) {
          Del_File();
        }
        
        else Serial1.println("No Match\n\n");
        
        Text_Input = "";    
            
      }

    }

  }
  
}


/*-------------------------------------------------------------------------------------------------*/
/*-------------------------------------------------------------------------------------------------*/  


#endif  // _SUB_H


/**************************************************************************************/
