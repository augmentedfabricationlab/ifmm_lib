/*
  Arduino callback functions
  created on 19.11.2016
*/

// DIO setup ===========================================================
const int relais_1 = 4;    // pin 4
const int relais_2 = 7;    // pin 7
const int relais_3 = 8;    // pin 8
const int relais_4 = 12;   // pin 12

// const int pin_9 = 9;   // pin 9
// const int pin_10 = 10; // pin 10


// motor setup ===========================================================
/*
motor1 = bender
motor2 = rotation backpart
motor34 = wirefeeders - same signal
*/

int m1_dir_pin = 33; // DIRECTION PIN
int m1_step_pin = 34; // STEP PIN
/*
int m2_dir_pin = ; // DIRECTION PIN
int m2_step_pin = ; // STEP PIN
*/
int m34_dir_pin = 5; // DIRECTION PIN
int m34_step_pin = 6; // STEP PIN

//Stepper(steps, pin1, pin2):
AccelStepper motor1(1, m1_step_pin, m1_dir_pin);
//AccelStepper motor2(1, m2_step_pin, m2_dir_pin);
AccelStepper motor34(1, m34_step_pin, m34_dir_pin);


int motor1_speed = 500; //steps per second
int motor1_accel = 100; //steps per second
int motor1_max_speed = 550; //steps per second
/*
int motor2_speed = 500; //steps per second
int motor2_accel = 100; //steps per second
int motor2_max_speed = 550; //steps per second
*/
int motor34_speed = 100; //steps per second
int motor34_accel = 50; //steps per second
int motor34_max_speed = 150; //steps per second


// setup ==================================================================
void pin_setup() {

  // digital outputs
  pinMode(relais_1, OUTPUT); // pin 4
  pinMode(relais_2, OUTPUT); // pin 7
  pinMode(relais_3, OUTPUT); // pin 8
  pinMode(relais_4, OUTPUT); // pin 12

  // pinMode(pin_10, OUTPUT); // pin 10 --> cannot be used as an input/output
  // pin 10 --> pin 11-13 cannot be used with the ethernet board --> for due has to be checked
  // pin 0,1 and 9 not used; the rest 2-13 is used or cant be used

}
void motors_setup() {
  motor1.setMaxSpeed(motor1_max_speed);
  motor1.setSpeed(motor1_speed);
  motor1.setAcceleration(motor1_accel);
/*
  motor2.setMaxSpeed(motor2_max_speed);
  motor2.setSpeed(motor2_speed);
  motor2.setAcceleration(motor2_accel);
*/
  motor34.setMaxSpeed(motor34_max_speed);
  motor34.setSpeed(motor34_speed);
  motor34.setAcceleration(motor34_accel);
  
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
  
  int motor1_position = (int)int_vals[0]; // int1 = absolute motor value for bending
  int motor_type = (int)int_vals[1,2,34]; // int2: motor_type = 1 = "front" / motor_type = 2 = "back"

  int motor2_position = (int)int_vals[0]; // int1 = absolute motor value for bending

  int motor34_position = (int)int_vals[0]; // int1 = absolute motor value for bending

  Serial.print("exec cmd rotate motor abs with int vals: ");
  Serial.println(motor1_position);
  Serial.println(motor2_position);
  Serial.println(motor34_position);
  Serial.println(motor_type);
  
  if (motor_type == 1) { //motor_type = 1
    Serial.println("motor 1");
    motor1.setSpeed(motor1_speed);
    move_abs(motor1_position, motor1); /// rotate the motor to abs motor pos  
  }
/* 
  if (motor_type == 2) { //motor_type = 2
    //Serial.println("motor 1");
    //motor2.setSpeed(motor2_speed);
    move_abs(motor2_position, motor2); /// rotate the motor to abs motor pos  
  }
*/
  if (motor_type == 34) { //motor_type = 34
    Serial.println("motor 34");
    motor34.setSpeed(motor34_speed);
    move_abs(motor34_position, motor34); /// rotate the motor to abs motor pos  
  }  
}

void cb_rotate_motor_rel(long int_vals[2]) {
  int motor1_steps = (int)int_vals[0]; // int1 = relative motor step value
  int motor_type = (int)int_vals[1]; // int2: motor_type = 1 = "front" / motor_type = 2 = "back"
  int motor2_steps = (int)int_vals[0]; // int1 = relative motor step value
  //int motor_type = (int)int_vals[2]; // int2: motor_type = 1 = "front" / motor_type = 2 = "back"
  int motor34_steps = (int)int_vals[0]; // int1 = relative motor step value
  //int motor_type = (int)int_vals[34]; // int2: motor_type = 1 = "front" / motor_type = 2 = "back"

  Serial.print("exec cmd rotate motor rel with int vals: ");
  Serial.println(motor1_steps);
  Serial.println(motor2_steps);
  Serial.println(motor34_steps);
  //Serial.println(motor_type);
  
  if (motor_type == 1) { //motor_type = 1
    //Serial.println("motor 1");
    //motor1.setSpeed(motor_speed);
    move_rel(motor1_steps, motor1); /// rotate the motor rel motor steps
  }
  /*
    if (motor_type == 2) { //motor_type = 2
    Serial.println("motor 2");
    motor2.setSpeed(motor2_speed);
    move_rel(motor2_steps, motor2); /// rotate the motor rel motor steps
  }
  */
  
  if (motor_type == 34) { //motor_type = 34
    Serial.println("motor 34");
    motor34.setSpeed(motor34_speed);
    move_rel(motor34_steps, motor34); /// rotate the motor rel motor steps
  }
}

void move_rel(int motor_steps, AccelStepper &motor) {
  motor.move(motor_steps);
  while (abs(motor.distanceToGo()) > 0) {    
    motor.run();
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



