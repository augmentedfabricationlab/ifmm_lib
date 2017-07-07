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
const int relais_1 = 4;   // pin 4
const int relais_2 = 7;   // pin 7
const int relais_3 = 8;   // pin 8
const int relais_4 = 12;   // pin 12


// const int pin_5 = 5;   // pin 5
// const int pin_6 = 6;   // pin 6
// const int pin_7 = 7;   // pin 7
// const int pin_8 = 8;   // pin 8
// const int pin_9 = 9;   // pin 9
// const int pin_10 = 10; // pin 10


// motors setup ========================================================
const int M_FRONT = 1;
const int M_BACK = 2;
const int M_FEEDER = 3;

int m_front_dir_pin = 2; // DIRECTION PIN
int m_front_step_pin = 3; // STEP PIN

int m_back_dir_pin = 33; // DIRECTION PIN
int m_back_step_pin = 34; // STEP PIN

int m_feeder_dir_pin_1 = 5; // DIRECTION PIN
int m_feeder_step_pin_1 = 6; // STEP PIN

int m_feeder_dir_pin_2 = 9; // DIRECTION PIN FOR SECOND FEEDER MOTOR
int m_feeder_step_pin_2 = 10; // STEP PIN FOR SECOND FEEDER MOTOR

AccelStepper m_front(1, m_front_step_pin, m_front_dir_pin);
AccelStepper m_back(1, m_back_step_pin, m_back_dir_pin);
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

// encoders setup =========================================================
Encoder enc_m_back(29, 30);
long old_pos_enc_m_back  = -999;


// setup ==================================================================
void pin_setup() {

  // digital outputs
  pinMode(relais_1, OUTPUT); // pin 4
  pinMode(relais_2, OUTPUT); // pin 7
  pinMode(relais_3, OUTPUT); // pin 8
  pinMode(relais_4, OUTPUT); // pin 12
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

  m_feeder_1.setMaxSpeed(m_feeder_max_speed);
  m_feeder_1.setSpeed(m_feeder_speed);
  m_feeder_1.setAcceleration(m_feeder_accel);

  m_feeder_2.setMaxSpeed(m_feeder_max_speed);
  m_feeder_2.setSpeed(m_feeder_speed);
  m_feeder_2.setAcceleration(m_feeder_accel);
  
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
      move_abs(motor_position, m_front); /// rotate the motor to abs motor pos  
      break;

    case M_BACK:
      move_abs(motor_position, m_back); /// rotate the motor to abs motor pos 
      //move_abs_with_enc(motor_position, m_back); /// rotate the motor to abs motor pos  
      break;

    case M_FEEDER:
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
      move_rel(motor_steps, m_front); /// rotate the motor rel motor steps
      break;

    case M_BACK:
      move_rel(motor_steps, m_back); /// rotate the motor rel motor steps
      //move_rel_with_enc(motor_steps, m_back);
      break;

    case M_FEEDER:
      move_rel_2_motors_minv(motor_steps, m_feeder_1, m_feeder_2); /// rotate the motor rel motor steps 
      break;
  }
}

// ========================================================================
void cb_feed_wire(long int_val) {
  int motor_steps = (int)int_val; // int1 = relative motor step values
  
  Serial.print("exec cmd feed wire with int vals: ");
  Serial.println(motor_steps);
  
  move_rel_2_motors_minv(motor_steps, m_feeder_1, m_feeder_2); /// rotate the motor rel motor steps 
}

// ========================================================================
void cb_weld(){
  Serial.println("exec cmd weld");
  }

// ========================================================================
void cb_actuate_electrodes(long int_val) {
  int electrode_state = (int)int_val; // int1 = relative motor step values
  
  Serial.print("exec cmd actuate electrodes with int val: ");
  Serial.println(electrode_state);

  if (electrode_state == 0) 
  {
    electrodes_open();
  }
  else
  {
    electrodes_close(); 
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
  }
}


// ===================================================
void cb_routine_feed_cut_weld(long int_val) {
  int motor_steps = (int)int_val; // int1 = relative motor step values for feeding the wire
  
  Serial.print("exec cmd cb_routine_feed_cut_weld with int val: ");
  Serial.println(motor_steps);
  
}


// ===================================================
// low level functions
// ===================================================
void nippers_move_to_back(){
  Serial.println("nippers_move_to_back");
}

// ===================================================
void nippers_move_to_front(){
  Serial.println("nippers_move_to_front");
}

// ===================================================
void nippers_cut(){
  Serial.println("nippers_cut");
}

// ===================================================
void electrodes_open(){
  Serial.println("open electrodes");

  // *** Start opening ***
  Serial1.println("Open");
  bool ok = wait_for_hydraulic_status(H_OPENED, 5000); 
  // *** Opened if wait_for_hydraulic_status returned true ***
  // (choose timeout appropriate to opening time of clamps)

  if (ok==true)
  {
    Serial.println("electrodes opened.");
  }else
  {
    Serial.println("something went wrong.");
  }

}

// ===================================================
void electrodes_close(){
  Serial.println("close electrodes");

  // *** Start closing ***
  Serial1.println("Close");
  bool ok = wait_for_hydraulic_status(H_CLOSED, 5000);
  // *** Closed if wait_for_hydraulic_status returned true ***
  // (choose timeout appropriate to closing time of clamps)

  if (ok==true)
  {
    Serial.println("electrodes closed.");
  }else
  {
    Serial.println("something went wrong.");
  }

}

// ===================================================
void electrodes_stop(){
  Serial.println("stop electrodes");

  // *** Start closing ***
  Serial1.println("Stop");
  bool ok = wait_for_hydraulic_status(H_IDLE, 500);
  // *** Stopped if WaitForStatus returned true ***
  // (choose timeout appropriate to stopping time of clamps)

  if (ok==true)
  {
    Serial.println("electrodes stopped.");
  }else
  {
    Serial.println("something went wrong.");
  }
}

// ===================================================
void read_enc_val(long &old_pos, Encoder &enc){
  long new_pos = enc.read();
  Serial.println(new_pos);
  if (new_pos != old_pos) {
    old_pos = new_pos;
    Serial.println(new_pos);
  } 
}

// ===================================================
void move_rel(int motor_steps, AccelStepper &motor) {
  motor.move(motor_steps);
  while (abs(motor.distanceToGo()) > 0) {    
    motor.run();
    //Serial.println(motor.distanceToGo());
  }
}

// ===================================================
void move_rel_with_enc(int motor_steps, AccelStepper &motor, long &old_pos, Encoder &enc) {
  motor.move(motor_steps);
  while (abs(motor.distanceToGo()) > 0) {    
    motor.run();
    read_enc_val(old_pos, enc);
    //Serial.println(motor.distanceToGo());
  }
}

// ===================================================
void move_abs(int motor_pos, AccelStepper &motor) {

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
void move_abs_with_enc(int motor_pos, AccelStepper &motor, long &old_pos, Encoder &enc) {
  Serial.print("current position: (before command) - ");
  Serial.println(motor.currentPosition());

  motor.moveTo(motor_pos);

  Serial.print("steps to go: (before command) - ");
  Serial.println(motor.distanceToGo());

  while (motor.distanceToGo() != 0) {
    motor.run();
    read_enc_val(old_pos, enc);
  }
  Serial.print("current position: (after command) - ");
  Serial.println(motor.currentPosition());
}

// ===================================================
void move_rel_2_motors_minv(int motor_steps, AccelStepper &motor_1, AccelStepper &motor_2) {
  motor_1.move(motor_steps);
  motor_2.move(-motor_steps);
  while (abs(motor_1.distanceToGo()) > 0) {    
    motor_1.run();
    motor_2.run();
  }
}

// ===================================================
void move_abs_2_motors_minv(int motor_pos, AccelStepper &motor_1, AccelStepper &motor_2) {
  motor_1.moveTo(motor_pos);
  motor_2.moveTo(-motor_pos);
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
  while(h_status != status_des)
  {
    // Get latest status
    while (Serial1.available())
    {
      h_status = Serial1.read();
    } 

    if (millis() - time_start > timeout_ms)
    {
      return false;
    }
    delay(20);
  }
  return true;
}

