/*
  Arduino callback functions
  created on 19.11.2016
*/

// HYDRAULICS STATES ===================================================
enum
{
  H_IDLE = 0,
  H_CLOSED = 1,
  H_OPENED = 2
} hydraulics_state;

// DIO setup ===========================================================
const int relais_nippers_move = 4;   // pin 4, nipper actuation
const int relais_nippers_cut = 7;   // pin 7, nipper cut
const int relais_3 = 8;   // pin 8
const int relais_weld = 12;   // pin 12, weld signal

bool are_nippers_in_front = false;
bool are_electrodes_closed = false;


// const int pin_5 = 5;   // pin 5
// const int pin_6 = 6;   // pin 6
// const int pin_7 = 7;   // pin 7
// const int pin_8 = 8;   // pin 8


// motors setup ========================================================
const int M_FRONT = 1;
const int M_BACK = 2;
const int M_FEEDER = 3;

const int m_front_dir_pin = 40; // DIRECTION PIN bend
const int m_front_step_pin = 41; // STEP PIN

const int m_back_dir_pin = 34; // DIRECTION PIN rotate
const int m_back_step_pin = 35; // STEP PIN

//int m_feeder_dir_pin = 46; // DIRECTION PIN feed
//int m_feeder_step_pin = 47; // STEP PIN

const int m_feeder_dir_pin_1 = 46; // DIRECTION PIN
const int m_feeder_step_pin_1 = 47; // STEP PIN
const int m_feeder_dir_pin_2 = 48; // DIRECTION PIN FOR SECOND FEEDER MOTOR
const int m_feeder_step_pin_2 = 49; // STEP PIN FOR SECOND FEEDER MOTOR

AccelStepper m_front(1, m_front_step_pin, m_front_dir_pin);
AccelStepper m_back(1, m_back_step_pin, m_back_dir_pin);

//AccelStepper m_feeder(1, m_feeder_step_pin, m_feeder_dir_pin);
AccelStepper m_feeder_1(1, m_feeder_step_pin_1, m_feeder_dir_pin_1);
AccelStepper m_feeder_2(1, m_feeder_step_pin_2, m_feeder_dir_pin_2);

int m_front_speed = 500; //steps per second
int m_front_accel = 100; //steps per second
int m_front_max_speed = 550; //steps per second

int m_back_speed = 500; //steps per second
int m_back_accel = 100; //steps per second
int m_back_max_speed = 550; //steps per second

int m_feeder_speed = 400; //steps per second
int m_feeder_accel = 100; //steps per second
int m_feeder_max_speed = 450; //steps per second

// RELATIVE ENCODER ERN1020 PARAMS =========================================
//ENCODER SIGNAL A 
#define ArduinoDigitalPinSigA 20  //the digital pin number on the upper layer (arduino layer)

//ENCODER SIGNAL B
#define ArduinoDigitalPinSigB 21

//read in interrupts (and loop) ==> volatile
volatile long encoder_pos = 0;
volatile boolean A_high = false;
volatile boolean B_high = false;


// setup ==================================================================
void pin_setup() {

  // digital outputs
  pinMode(relais_nippers_move, OUTPUT); // pin 4 // relais 1
  pinMode(relais_nippers_cut, OUTPUT); // pin 7  // relais 2
  pinMode(relais_3, OUTPUT); // pin 8           // relais 3
  pinMode(relais_weld, OUTPUT); // pin 12       // relais 4
  // pinMode(pin_10, OUTPUT); // pin 10 --> cannot be used as an input/output
  // pin 10 --> pin 11-13 cannot be used with the ethernet board --> for due has to be checked

}

void motors_setup() {

  m_front.setMaxSpeed(m_front_max_speed);
  m_front.setSpeed(m_front_speed);
  m_front.setAcceleration(m_front_accel);

  m_back.setMaxSpeed(m_back_max_speed);
  m_back.setSpeed(m_back_speed);
  m_back.setAcceleration(m_back_accel);

  /*
    m_feeder.setMaxSpeed(m_feeder_max_speed);
    m_feeder.setSpeed(m_feeder_speed);
    m_feeder.setAcceleration(m_feeder_accel);
  */

  m_feeder_1.setMaxSpeed(m_feeder_max_speed);
  m_feeder_1.setSpeed(m_feeder_speed);
  m_feeder_1.setAcceleration(m_feeder_accel);

  m_feeder_2.setMaxSpeed(m_feeder_max_speed);
  m_feeder_2.setSpeed(m_feeder_speed);
  m_feeder_2.setAcceleration(m_feeder_accel);

}

void encoder_setup() {
  pinMode(ArduinoDigitalPinSigA, INPUT);      //set as input
  pinMode(ArduinoDigitalPinSigB, INPUT); 
  digitalWrite(ArduinoDigitalPinSigA, HIGH);  // turn on pull-up resistor
  digitalWrite(ArduinoDigitalPinSigB, HIGH);  

  // encoder pin A
  attachInterrupt(digitalPinToInterrupt(ArduinoDigitalPinSigA), encoderSigA, CHANGE);
  // encoder pin B
  attachInterrupt(digitalPinToInterrupt(ArduinoDigitalPinSigB), encoderSigB, CHANGE);  
}

// ========================================================================
// callback functions
// ========================================================================
void cb_set_do(long do_state[2]) { // set digital output: do_state[0] = do_num, do_state[1] = do_state
  Serial.print("set_do: ");
  Serial.print(do_state[0]);
  Serial.print(" -- ");
  Serial.println(do_state[1]);

  digitalWrite(do_state[0], do_state[1]);
}

// ========================================================================
void cb_rotate_motor_abs(long int_vals[2]) {
  int motor_position = (int)int_vals[0]; // int1 = absolute motor value for bending
  int motor_type = (int)int_vals[1]; // int2: motor_type = 1 = "front" / motor_type = 2 = "back" / motor_type = 3 = "feeder"

  Serial.print("exec cmd rotate motor abs with int vals: ");
  Serial.println(motor_position);
  Serial.println(motor_type);

  switch (motor_type) {
    case M_FRONT:
      move_abs(motor_position, m_front); /// rotate the bender motor to abs motor pos
      break;

    case M_BACK:
      move_abs(motor_position, m_back); /// rotate the rotate motor to abs motor pos
      //move_abs_with_enc(motor_position, m_back); /// rotate the motor to abs motor pos
      break;

    case M_FEEDER:
      //move_abs(motor_position, m_feeder); /// rotate the feeder motor to abs motor pos
      move_abs_2_motors_minv(motor_position, m_feeder_1, m_feeder_2); /// rotate the motor to abs motor pos
      break;
  }
}

// ========================================================================
void cb_rotate_motor_rel(long int_vals[2]) {
  int motor_steps = (int)int_vals[0]; // int1 = relative motor step value
  int motor_type = (int)int_vals[1]; // int2: motor_type = 1 = "front" / motor_type = 2 = "back" / motor_type = 3 = "feeder"

  Serial.print("exec cmd rotate motor rel with int vals: ");
  Serial.println(motor_steps);
  Serial.println(motor_type);

  switch (motor_type) {
    case M_FRONT:
      move_rel(motor_steps, m_front); /// rotate the bender motor rel motor steps
      break;

    case M_BACK:
      move_rel(motor_steps, m_back); /// rotate the rotate motor rel motor steps
      //move_rel_with_enc(motor_steps, m_back);
      break;

    case M_FEEDER:
      //move_rel(motor_steps, m_feeder); /// rotate the feeder motor rel motor steps
      move_rel_2_motors_minv(motor_steps, m_feeder_1, m_feeder_2); /// rotate the motor rel motor steps
      break;
  }
}

// ========================================================================
void cb_bend(long int_vals[2]) {
  int motor_pos_m_front = (int)int_vals[0]; // int1 = motor pos value for front motor
  int motor_pos_m_back = (int)int_vals[1]; // int2 = motor pos value for back motor

  Serial.print("exec cmd bend with int vals: ");
  Serial.println(motor_pos_m_front);
  Serial.println(motor_pos_m_back);

  move_abs_2_motors_for_bending(motor_pos_m_front, motor_pos_m_back, m_front, m_back); /// rotate the 2 motors to abs motor pos
}

// ========================================================================
void cb_feed_wire(long int_val) {
  int motor_steps = (int)int_val; // int1 = relative motor step values

  Serial.print("exec cmd feed wire with int vals: ");
  Serial.println(motor_steps);

  feed_wire_with_motorsteps(motor_steps);

}

// ========================================================================
void cb_weld() {
  Serial.println("exec cmd weld");
  //weld();
}

// ========================================================================
void cb_reset_encoder() {
  Serial.println("exec cmd reset encoder");
  //accessing a long variable is NOT atomic! disable interrupts during the assignment
  //when an interrupt occurs between noInterrupt and interrupt it is NOT lost but executed after interrupt()!
  noInterrupts();
  encoder_pos = 0;
  interrupts();
  
}

// ========================================================================
void cb_get_encoder_val(long* encoder_pos_copy) {
  Serial.println("exec cmd get encoder val");
  //accessing a long variable is NOT atomic! disable interrupts during the assignment
  //when an interrupt occurs between noInterrupt and interrupt it is NOT lost but executed after interrupt()!
  noInterrupts();
  *encoder_pos_copy = encoder_pos;
  interrupts();
  
}

// ========================================================================
void cb_actuate_electrodes(long int_val) {
  int electrode_state = (int)int_val;
  bool ok = false;

  Serial.print("exec cmd actuate electrodes with int val: ");
  Serial.println(electrode_state);

  /*
  if (electrode_state == 0)
  {
    bool ok = electrodes_open();
  }
  else
  {
    bool ok = electrodes_close();
  }*/

  switch (electrode_state) {
    case 0:
      ok = electrodes_open();
      break;
    case 1:
      ok = electrodes_close();
      break;
    case 2:
      ok = electrodes_stop();
      break;
  }
}

// ========================================================================
void cb_actuate_nippers(long int_val) {
  int nippers_state = (int)int_val; // int1 = relative motor step values

  Serial.print("exec cmd actuate nippers with int val: ");
  Serial.println(nippers_state);

  switch (nippers_state) {
    case 0:
      nippers_move_to_back();
      break;
    case 1:
      nippers_move_to_front();
      break;
    case 2:
      nippers_cut();
      break;
    case 3:
      nippers_release();
      break;
  }
}


// ===================================================
bool cb_routine_feed_cut_weld(long int_val) {
  int motor_steps = (int)int_val; // int1 = relative motor step values for feeding the wire
  bool ok = false;

  Serial.print("exec cmd cb_routine_feed_cut_weld with int val: ");
  Serial.println(motor_steps);

  //1 feed wire length int_val
  //  change to one control; direction will be switched in drive board
  ok = feed_wire_with_motorsteps(motor_steps);
  delay(500);
  if (ok != true) {
    return false;
  }

  //2 close electrodes ..
  ok = electrodes_close();
  delay(500);
  if (ok != true) {
    return false;
  }

  //3 move scissor to front
  ok = nippers_move_to_front();
  delay(500);
  if (ok != true) {
    return false;
  }

  //4 cut
  ok = nippers_cut();
  delay(500);
  if (ok != true) {
    return false;
  }

  //5 move scissor back
  ok = nippers_move_to_back();
  delay(500);
  if (ok != true) {
    return false;
  }

  //6 weld
  /*
  ok = weld();
  delay(500);
  if (ok != true) {
    return false;
  }
  */

  //7 open electrodes
  ok = electrodes_open();
  delay(500);
  if (ok != true) {
    return false;
  }

  return true;

}


// ===================================================
// low level functions
// ===================================================

// ========================================================================
bool feed_wire_with_motorsteps(int motor_steps) {
  //move_rel(motor_steps, m_feeder); /// rotate the feeder motor rel motor steps
  move_rel_2_motors_minv(motor_steps, m_feeder_1, m_feeder_2); /// rotate the motor rel motor steps
  return true;
}

// ========================================================================
bool weld() {
  // bool ok = wait_for_hydraulic_status(H_CLOSED, 100);     ////!! check for appropriate pressure & position according to wires used -> replace H_CLOSED !!
  // *** Closed if wait_for_hydraulic_status returned true ***
  
  bool ok = are_electrodes_closed; //hack
  
  // *** Closed if wait_for_hydraulic_status returned true ***
  // (choose timeout appropriate to closing time of clamps)
  if (ok == true)
  {
    Serial.println("weld triggered");
    digitalWrite(relais_weld, HIGH);
    delay (100);
    digitalWrite(relais_weld, LOW);
    return true;
  } else
  {
    Serial.println("weld NOT triggered");
    return false;
  }
}

bool nippers_move_to_back() {
  Serial.println("nippers_move_to_back");

  digitalWrite(relais_nippers_move, LOW);

  delay(1000);
  
  are_nippers_in_front = false;
  return true;
}

// ===================================================
bool nippers_move_to_front() {
  Serial.println("nippers_move_to_front");

  //bool ok = wait_for_hydraulic_status(H_CLOSED, 100);
  // *** Closed if wait_for_hydraulic_status returned true ***
  
  bool ok = are_electrodes_closed; //hack

  if (ok == true)
  {
    Serial.println("nippers_move_to_front");
    digitalWrite(relais_nippers_move, HIGH);
    delay(4000);
    are_nippers_in_front = true;
    return true;
  } else
  {
    Serial.println("cannot move nippers, clamps not closed");
    return false;
  }
}

// ===================================================
bool nippers_cut() {
  Serial.println("nippers_cut");
  //bool ok = wait_for_hydraulic_status(H_CLOSED, 100);
  // *** Closed if wait_for_hydraulic_status returned true ***
  
  bool ok = are_electrodes_closed; //hack

  if (ok == true && are_nippers_in_front == true)
  {
    digitalWrite(relais_nippers_cut, HIGH);
    delay (5000);
    digitalWrite(relais_nippers_cut, LOW);
    delay (300);
    return true;
  } else
  { 
    Serial.println("nippers cannot cut, bool ok: ");
    Serial.println(ok);
    Serial.println("nippers cannot cut, bool are_nippers_in_front: ");
    Serial.println(are_nippers_in_front);
    return false;
  }
}

// ===================================================
bool nippers_release() {
  Serial.println("nippers_release");
  digitalWrite(relais_nippers_cut, LOW);
  delay (100);
  return true;
}

// ===================================================
bool electrodes_open() {
  Serial.println("open electrodes");

  if (are_nippers_in_front == false)
  {
    // *** Start opening ***
    Serial1.println("Open");
    bool ok = wait_for_hydraulic_status(H_OPENED, 5000);
    // *** Opened if wait_for_hydraulic_status returned true ***
    // (choose timeout appropriate to opening time of clamps)
  
    if (ok == true)
    {
      Serial.println("electrodes opened.");
      are_electrodes_closed = false; //hack
      return true;
    } else
    {
      Serial.println("electrodes NOT opened!");
      are_electrodes_closed = true; //hack
      return false;
    }
  }else
  {
    Serial.println("electrodes NOT opened, nippers are still in front!");
    are_electrodes_closed = true; //hack
    return false;
   }

}

// ===================================================
bool electrodes_close() {
  Serial.println("close electrodes");

  // *** Start closing ***
  Serial1.println("Close");
  bool ok = wait_for_hydraulic_status(H_CLOSED, 5000);
  // *** Closed if wait_for_hydraulic_status returned true ***
  // (choose timeout appropriate to closing time of clamps)

  if (ok == true)
  {
    Serial.println("electrodes closed.");
    are_electrodes_closed = true; //hack
    return true;
  } else
  {
    Serial.println("electrodes NOT closed!");
    are_electrodes_closed = false; //hack
    return false;
  }
}

// ===================================================
bool electrodes_stop() {
  Serial.println("stop electrodes");

  // *** Start closing ***
  Serial1.println("Stop");
  bool ok = wait_for_hydraulic_status(H_IDLE, 500);
  // *** Stopped if WaitForStatus returned true ***
  // (choose timeout appropriate to stopping time of clamps)

  if (ok == true)
  {
    Serial.println("electrodes stopped.");
    return true;
  } else
  {
    Serial.println("electrodes NOT stopped!");
    return false;
  }
}

// ===================================================
void move_rel(int motor_steps, AccelStepper & motor) {
  motor.move(motor_steps);
  while (abs(motor.distanceToGo()) > 0) {
    motor.run();
    //Serial.println(motor.distanceToGo());
  }
}

// ===================================================
void move_abs(int motor_pos, AccelStepper & motor) {

  Serial.print("current position: (before command) - ");
  Serial.println(motor.currentPosition());

  motor.moveTo(motor_pos);

  Serial.print("steps to go: (before command) - ");
  Serial.println(motor.distanceToGo());

  while (motor.distanceToGo() != 0) {
    //Serial.print("steps to go: ");
    //Serial.println(motor.distanceToGo());
    //Serial.print("current position: ");
    //Serial.println(motor.currentPosition());
    motor.run();
  }

  //Serial.print("steps to go: (after command) - ");
  //Serial.println(motor.distanceToGo());

  Serial.print("current position: (after command) - ");
  Serial.println(motor.currentPosition());
}

// ===================================================
void move_rel_2_motors_minv(int motor_steps, AccelStepper & motor_1, AccelStepper & motor_2) {

  motor_1.move(-motor_steps);
  motor_2.move(motor_steps);

  Serial.print("motor_1.distanceToGo(): ");
  Serial.println(motor_1.distanceToGo());
  Serial.print("motor_2.distanceToGo(): ");
  Serial.println(motor_2.distanceToGo());
  
  while (motor_1.distanceToGo() != 0 || motor_2.distanceToGo() != 0)  {
    motor_1.run();
    motor_2.run();

    //Serial.print("current position: ");
    //Serial.println(motor_1.currentPosition());
    //Serial.println(motor_2.currentPosition());
  }
}

// ===================================================
void move_abs_2_motors_minv(int motor_pos, AccelStepper & motor_1, AccelStepper & motor_2) {

  motor_1.moveTo(-motor_pos);
  motor_2.moveTo(motor_pos);

  Serial.print("motor_1.distanceToGo(): ");
  Serial.println(motor_1.distanceToGo());
  Serial.print("motor_2.distanceToGo(): ");
  Serial.println(motor_2.distanceToGo());
  
  while (motor_1.distanceToGo() != 0 || motor_2.distanceToGo() != 0) {
    motor_1.run();
    motor_2.run();
  }
}

// ===================================================
void move_abs_2_motors_for_bending(int motor_pos_1, int motor_pos_2, AccelStepper & motor_1, AccelStepper & motor_2) {

  motor_1.moveTo(motor_pos_1);
  motor_2.moveTo(motor_pos_2);

  Serial.print("motor_1.distanceToGo(): ");
  Serial.println(motor_1.distanceToGo());
  Serial.print("motor_2.distanceToGo(): ");
  Serial.println(motor_2.distanceToGo());
  
  while (motor_1.distanceToGo() != 0 || motor_2.distanceToGo() != 0) {
    motor_1.run();
    motor_2.run();
  }
}

// ===================================================
bool wait_for_hydraulic_status(byte status_des, int timeout_ms) // wait for the desired hydraulics status with a timeout
{
  unsigned long time_start = millis();
  byte h_status = -1;

  // Wait until desired status
  while (h_status != status_des)
  {
    // Get latest status
    while (Serial1.available())
    {
      h_status = Serial1.read();
      Serial.println(h_status);
    }

    if (millis() - time_start > timeout_ms)
    {
      return false;
    }
    delay(20);
  }
  return true;
}

//========== ENCODER ERN1020 INTERRUPTS ============
//when the motor is rotates with very roughly 1 revolution/ second (4000 * 4 Ticks) these two interrupts use around 10% of the CPU load.
void encoderSigA(){
  A_high = digitalRead(ArduinoDigitalPinSigA);
  encoder_pos += (A_high != B_high) ? +1 : -1;
}

void encoderSigB(){
  B_high = digitalRead(ArduinoDigitalPinSigB);
  encoder_pos += (A_high == B_high) ? +1 : -1;
}

