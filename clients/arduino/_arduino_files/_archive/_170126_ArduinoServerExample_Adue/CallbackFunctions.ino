/*
  Arduino callback functions
  created on 19.11.2016
*/

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


// motors setup ===========================================================
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

// callback functions =====================================================
void cb_set_do(long do_state[2]) { // set digital output: do_state[0] = do_num, do_state[1] = do_state
  Serial.print("set_do: ");
  Serial.print(do_state[0]);
  Serial.print(" -- ");
  Serial.println(do_state[1]);

  digitalWrite(do_state[0], do_state[1]);
}

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
      Serial.println("move abs back motor");
      //move_abs(motor_position, m_back); /// rotate the motor to abs motor pos 
      move_abs_with_enc(motor_position, m_back); /// rotate the motor to abs motor pos  
      break;

    case M_FEEDER:
      move_abs_2_motors(motor_position, m_feeder_1, m_feeder_2); /// rotate the motor to abs motor pos  
      break;
  }
}

void read_enc_val(){
  long new_pos = enc_m_back.read();
  Serial.println(new_pos);
  
  if (new_pos != old_pos_enc_m_back) {
    old_pos_enc_m_back = new_pos;
    Serial.println(new_pos);
  } 
}

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
      //move_rel(motor_steps, m_back); /// rotate the motor rel motor steps
      Serial.println("move rel back motor");
      move_rel_with_enc(motor_steps, m_back);
      break;

    case M_FEEDER:
      move_rel_2_motors(motor_steps, m_feeder_1, m_feeder_2); /// rotate the motor rel motor steps 
      break;
  }
  
}

void move_rel(int motor_steps, AccelStepper &motor) {
  motor.move(motor_steps);
  while (abs(motor.distanceToGo()) > 0) {    
    motor.run();
    //Serial.println(motor.distanceToGo());
  }
}

void move_rel_with_enc(int motor_steps, AccelStepper &motor) {
  motor.move(motor_steps);
  while (abs(motor.distanceToGo()) > 0) {    
    motor.run();
    //Serial.println(motor.distanceToGo());
    long new_pos = enc_m_back.read();

    if (new_pos != old_pos_enc_m_back) {
      old_pos_enc_m_back = new_pos;
      Serial.println(new_pos);
    }
  }
}

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

void move_abs_with_enc(int motor_pos, AccelStepper &motor) {

  Serial.print("current position: (before command) - ");
  Serial.println(motor.currentPosition());

  motor.moveTo(motor_pos);

  Serial.print("steps to go: (before command) - ");
  Serial.println(motor.distanceToGo());

  while (motor.distanceToGo() != 0) {
    motor.run();
    
    long new_pos = enc_m_back.read();

    if (new_pos != old_pos_enc_m_back) {
      old_pos_enc_m_back = new_pos;
      Serial.println(new_pos);
    }
    
  }

  Serial.print("current position: (after command) - ");
  Serial.println(motor.currentPosition());
}

void move_rel_2_motors(int motor_steps, AccelStepper &motor_1, AccelStepper &motor_2) {
  motor_1.move(motor_steps);
  motor_2.move(-motor_steps);
  while (abs(motor_1.distanceToGo()) > 0) {    
    motor_1.run();
    motor_2.run();
  }
}

void move_abs_2_motors(int motor_pos, AccelStepper &motor_1, AccelStepper &motor_2) {
  motor_1.moveTo(motor_pos);
  motor_2.moveTo(-motor_pos);
  while (motor_1.distanceToGo() != 0 || motor_2.distanceToGo() != 0) {    
    motor_1.run();
    motor_2.run();
  }
}



