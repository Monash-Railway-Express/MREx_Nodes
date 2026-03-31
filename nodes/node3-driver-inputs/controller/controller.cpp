#include <controller.h>
#include <stdint.h>
#include <CAN_MREx.h>

//Leaving Blank here as section 1.1.5 states function prototypes must be in header but does not state functions need to be in .cpp file
/**
//function definitions
// function to go to stopped / pre-op / operational mode
void sendToNewOpMode(int opMode) {
  //Checks if EMCY 
  //if(!checkMajorEMCY() || !checkMinorEMCY()){

      if (opMode == 1) {
      sendAllNMT(0x02);
      nodeOperatingMode = 0x02;
      Serial.println("Stopped Mode");
    }
    if (opMode == 3) {
      sendAllNMT(0x01);
      nodeOperatingMode = 0x01;
      Serial.println("Normal Mode");
    }
    if (opMode == 2) {
      sendAllNMT(0x80);
      nodeOperatingMode = 0x80;
      Serial.println("Preop Mode");

    }
  //}
  
}

// function that is called to send NMT to all nodes
void sendAllNMT(uint8_t operatingMode) {
  sendNMT(operatingMode, 0x01); // motor
  sendNMT(operatingMode, 0x02); // brakes
  sendNMT(operatingMode, 0x04); // lights
  sendNMT(operatingMode, 0x05); // audio sys
  sendNMT(operatingMode, 0x09); // LCD screen
}

// function where all inputs are read
void HandleInputs() {
  // ===== Potentiometer Inputs =====
  regenBrake   = 1023 - analogRead(BRAKE_PIN);
  desiredSpeed = 1023 - analogRead(SPEED_PIN);
  Serial.print("Brake: ");
  Serial.print(regenBrake);
  Serial.print("   ||   Throttle: ");
  Serial.println(desiredSpeed);


  // ===== Button Inputs =====
  button1 = digitalRead(BUTTON_1_PIN);
  button2 = digitalRead(BUTTON_2_PIN);

  // ===== Switch Inputs =====
  switch1 = digitalRead(SWITCH_1_PIN);
  switch2 = digitalRead(SWITCH_2_PIN);

  // Store raw 5-position states too if you want to use them elsewhere later
  
  //conditionMode = check5Switch(analogRead(CONDITION_MODE_PIN));
}

// Used for debugging. Prints all inputs and their values
void print_status() {
  // Check readings of brake and speed
   Serial.print("Speed: ");
   Serial.print(desiredSpeed);
   Serial.print(" | Brake: ");
   Serial.println(regenBrake);

  // Check buttons
  // Serial.print(" || Button 1: ");
  // Serial.print(button1);
  // Serial.print(" | Button 2: ");
  // Serial.println(button2);

  // Check switches
  Serial.print(" || Switch 1: ");
  Serial.print(switch1);
  Serial.print(" | Switch 2: ");
  Serial.println(switch2);


  // Serial.print("Op_mode raw: ");
  // Serial.print(analogRead(OP_MODE_PIN));
  // Serial.print(" | Op_mode pos3: ");
  // Serial.print(check3Switch(analogRead(OP_MODE_PIN)));

  // Serial.print(" || Direction raw: ");
  // Serial.print(analogRead(DIRECTION_MODE_PIN));
  // Serial.print(" | Condition pos3: ");
  // Serial.print(check3Switch(analogRead(DIRECTION_MODE_PIN)));

  Serial.print("Challenge raw: ");
  Serial.print(analogRead(CHALLENGE_MODE_PIN));
  Serial.print(" | Challenge pos5: ");
  Serial.print(map5PosTo3State(analogRead(CHALLENGE_MODE_PIN)));

  // Serial.print(" || Condition raw: ");
  // Serial.print(analogRead(CONDITION_MODE_PIN));
  // Serial.print(" | Condition pos5: ");
  // Serial.print(check5Switch(analogRead(CONDITION_MODE_PIN)));
  
  // Mapping 3 to 5 - Back up test Code
  // Serial.print(" | OpMode mapped: ");
  // Serial.println(map5PosTo3State(conditionModeRaw));
}


// function that does edge detection on horn button and calls SDO write to horn node
// NOTE: Horn is currently assigned to Button 1
void HandleHorn() {
  if (button1 != b1prev) {
    directionMode = map5PosTo3State(analogRead(CHALLENGE_MODE_PIN));
    b1prev = button1;
    uint8_t invertedBtn1 = (uint8_t)!button1;
    executeSDOWrite(nodeID, 5, 0x6065, 0x00, sizeof(button1), &invertedBtn1);
  }
}

void HandleParking() {
  if (switch1 != s1prev) {
    //1 is brake on - 0 is off
    s1prev = switch1;
    executeSDOWrite(nodeID, 2, 0x3012, 0x01, sizeof(switch1), &switch1);
    Serial.print("Sending Parking");
    Serial.println(switch1);
  }
}

void HandleDirection() {
  directionMode = map5PosTo3State(analogRead(CHALLENGE_MODE_PIN));
  // Test values 
  // Serial.println(directionMode);
  // Serial.println(dirprev);
  if ((directionMode != dirprev) && directionMode != 101) {
    // 1 is forawrd, 2 is neutral, 3 is back 
    dirprev = directionMode;
    executeSDOWrite(nodeID, 1, 0x6060, 0x00, sizeof(directionMode), &directionMode);
    Serial.print("Sending direction");
    Serial.println(directionMode);
  }
}

// old 3-position switch checker
int check3Switch(int read) {
  Serial.println(read);
  if (read < 200) {
    // neutral and pre op
    return 1;
  }
  else if (read > 500) {
    // op and forward
    return 3;
  }
  else {
    // stopped and backward 
    return 2;
  }
}


// 5-position switch checker
int check5Switch(int read) {
  //Serial.println(read);
  if (read >= 0 && read < 50) {
    return 1;
  }
  else if (read > 100 && read < 250) {
    return 2;
  }
  else if (read > 300 && read < 450) {
    return 3;
  }
  else if (read > 500 && read < 650) {
    return 4;
  }
  else if (read > 700 && read < 850) {
    return 5;
  }
  else {
    return 101;
  }
}


// map 5-position switch into old 3-state behaviour
// positions 1,2 -> state 1
// position 3   -> state 2
// positions 4,5 -> state 3
int map5PosTo3State(int read) {
  int pos = check5Switch(read);

  if (pos <= 2) {
    return 1;
  }
  else if (pos == 3) {
    return 2;
  }
  else if (pos == 4 || pos == 5) {
    return 3;
  }
  else {
    return 101;
  }
}
*/