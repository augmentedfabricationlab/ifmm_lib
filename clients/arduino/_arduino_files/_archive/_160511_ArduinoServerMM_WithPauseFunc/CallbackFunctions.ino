/*
  Arduino callback functions for mesh mould wire welding
  created on 02.10.2015
*/

// relais setup ===========================================================
const int relay_pneu_feeder = 8;   // relay for pneu feeder
const int relay_pneu_welder = 9;   // relay for pneu welder
const int relay_pneu_cutter = 12;  // relay for pneu cutter
const int relay_trigger_weld = 11; // relay for triggering the welding mechanism

// motor setup ===========================================================
int m1_dir_pin = 4; // DIRECTION PIN // motor for turning the bender
int m1_step_pin = 5; // STEP PIN
int m2_dir_pin = 2; // DIRECTION PIN // motor for turning the back part of the mm tool (after bending)
int m2_step_pin = 3; // STEP PIN
int m3_dir_pin = 6; // DIRECTION PIN // motor for moving tool part down and up, for welding, holding the wire and retraction
int m3_step_pin = 7; // STEP PIN

AccelStepper motor_1_front(1, m1_step_pin, m1_dir_pin);
AccelStepper motor_2_back(1, m2_step_pin, m2_dir_pin);
AccelStepper motor_3_vertical(1, m3_step_pin, m3_dir_pin);


int motor_speed = 500; //steps per second
int motor_accel = 200; //steps per second
int motor_max_speed = 550; //steps per second
// ========================================================================
int motor_3_speed = 6000; //steps per second
int motor_3_accel = 2000; //steps per second
int motor_3_max_speed = 7000; //steps per second
// ========================================================================

/*
  // elevator heights --> for 20mm layers
  int depth_for_weld_upper = 1600; // 1mm = 100 steps
  int depth_for_weld_lower_long = 3600; // 1mm = 100 steps
  int depth_for_weld_lower_short = 2600; // 1mm = 100 steps
  int depth_cutting_offset_long = 200;
  int depth_cutting_offset_short = 300;
  int overbending_val = 20;*/

// elevator heights --> for 22mm layers
//int depth_for_weld_upper = 1800; //2200; //1600; // 1mm = 100 steps
//int depth_for_weld_lower_long = 3800; //3800; // 1mm = 100 steps
int depth_for_weld_double_long = 3700; // was 3500
//int depth_for_cutting_short = 3400; // 1mm = 100 steps ---> should be 4200 as well
int depth_for_weld_double_short = 3000; //2700; // 1mm = 100 steps was 3000
int depth_cutting_offset_long = 200;
int depth_cutting_offset_short = 200;
int overbending_val = 10;
//int depth_for_cutting_long = 3400;
int depth_for_bend = 1200; // was 2600
int depth_for_cutting = 3800;

// setup ==================================================================
void pin_setup() {
  //pinMode(relay_led, OUTPUT); // relay for LED
  pinMode(di_interrupt, INPUT); // for reading the state of the pause switch

  // relais for welding
  pinMode(relay_pneu_feeder, OUTPUT); // relay for pneu feeder
  pinMode(relay_pneu_welder, OUTPUT); // relay for pneu welder
  pinMode(relay_pneu_cutter, OUTPUT); // relay for pneu cutter
  pinMode(relay_trigger_weld , OUTPUT);  // relay for triggering the welding mechanism

  //digitalWrite(relay_led, LOW); //turn the LED off
}
void motors_setup() {
  motor_1_front.setMaxSpeed(motor_max_speed);
  motor_1_front.setSpeed(motor_speed);
  motor_1_front.setAcceleration(motor_accel);

  motor_2_back.setMaxSpeed(motor_max_speed);
  motor_2_back.setSpeed(motor_speed);
  motor_2_back.setAcceleration(motor_accel);

  motor_3_vertical.setMaxSpeed(motor_3_max_speed);
  motor_3_vertical.setSpeed(motor_3_speed);
  motor_3_vertical.setAcceleration(motor_3_accel);
}
// ========================================================================

// callback functions =====================================================
void cb_set_do(long do_state[2]) { // set digital output: do_state[0] = do_num, do_state[1] = do_state
  Serial.print("set_do: ");
  Serial.print(do_state[0]);
  Serial.print(" -- ");
  Serial.println(do_state[1]);

  /* //hack for testing the welding
    if (do_state[0] == relay_trigger_weld){
    weld();
    }else{
    digitalWrite(do_state[0], do_state[1]);
    }*/
  digitalWrite(do_state[0], do_state[1]);
}

void cb_rotate_motor_abs(long int_vals[2]) {
  int motor_position = (int)int_vals[0]; // int1 = absolute motor value for bending
  int motor_type = (int)int_vals[1]; // int2: motor_type = 1 = "front" / motor_type = 2 = "back"

  Serial.print("exec cmd rotate motor abs with int vals: ");
  Serial.println(motor_position);
  Serial.println(motor_type);
  
  if (motor_type == 1) { //motor_type = 1 = "front"
    Serial.println("motor type: front");
    move_abs(motor_position, motor_1_front); /// rotate the motor to abs motor pos
  }else{ // motor_type = 2 = "back"
    Serial.println("motor type: back");
    move_abs(-motor_position, motor_2_back); /// rotate the motor to abs motor pos
  }
  
}

void cb_insert_and_bend(long int_vals[2]) {
  int vertical_type = (int)int_vals[0]; // int1 = long or short vertical boolean
  int motor_position = (int)int_vals[1]; // int2 = absolute motor value for bending

  Serial.print("exec cmd insert and bend with int vals: ");
  Serial.println(vertical_type);
  Serial.println(motor_position);

  boolean retract = false;
  boolean approach = false;

  if (vertical_type > 0) { // long vertical: two welding spots
    //insert_vertical(retract, depth_for_weld_upper, depth_for_weld_lower_long, depth_cutting_offset_long);
    insert_vertical(retract, depth_for_weld_double_long, depth_cutting_offset_long);
  } else { // short vertical: one welding spot
    //insert_vertical(retract, depth_for_weld_upper, depth_for_weld_lower_short, depth_cutting_offset_short);
    insert_vertical(retract, depth_for_weld_double_short, depth_cutting_offset_short);
  }

  bend(approach, motor_position); // execute the bending without moving elevator down to bending pos
}

// ========================================================================
void cb_exec_cmd_bend(float f[1]) { // execute the bending command
  Serial.print("exec cmd bend with float val: ");
  Serial.println(f[0]);
  int motor_position = (int)f[0]; // absolute motor position
  boolean approach = true;

  bend(approach, motor_position); // execute the bending with moving elevator down to bending pos
}
// ========================================================================

// ========================================================================
void cb_exec_cmd_insert_vertical(float f[1]) { // execute the insert vertical command, which consists of a series of tasks
  Serial.print("exec insert vertical cmd with float val: ");
  Serial.println(f[0]);

  int vertical_type = (int)f[0]; // float value indicates, if it is a long or a short vertical, 0 = short, 1 = long
  boolean retract = true;

  if (vertical_type > 0) { // long vertical: two welding spots
    //insert_vertical(retract, depth_for_weld_upper, depth_for_weld_lower_long, depth_cutting_offset_long);
    insert_vertical(retract, depth_for_weld_double_long, depth_cutting_offset_long);
  } else { // short vertical: one welding spot
    //insert_vertical(retract, depth_for_weld_upper, depth_for_weld_lower_short, depth_cutting_offset_short);
    insert_vertical(retract, depth_for_weld_double_short, depth_cutting_offset_short);
  }
}
// ========================================================================

void cb_exec_cmd_insert_vert_bef_tilt(float f[1]){
  Serial.print("exec cb_exec_cmd_insert_vert_bef_tilt with float val: ");
  Serial.println(f[0]);

  int vertical_type = (int)f[0]; // float value indicates, if it is a long or a short vertical, 0 = short, 1 = long

  if (vertical_type > 0) { // long vertical: two welding spots
    insert_vertical_bef_tilt(depth_for_weld_double_long, depth_cutting_offset_long, false);
  } else { // short vertical: one welding spot
    insert_vertical_bef_tilt(depth_for_weld_double_short, depth_cutting_offset_short, false);
  }
}

void cb_exec_cmd_fval_bend_after_tilt(float f[1]) {
  Serial.print("exec cb_exec_cmd_fval_bend_after_tilt with float val: ");
  Serial.println(f[0]);

  boolean approach = false;
  int motor_position = (int)f[0]; // absolute motor position

  bend(approach, motor_position); // execute the bending without moving elevator down to bending pos
  
}

// ========================================================================
void cb_exec_cmd_retract(float f[1]) { //execute the retracting command
  Serial.print("exec cmd retract with float val: ");
  Serial.println(f[0]);
  int motor_position = (int)f[0];
  move_rel(motor_position, motor_3_vertical); //move_abs(motor_position, motor_2_back);
}
// ========================================================================
boolean valid_relative_motor_steps(int next_motor_pos) {

  int relative_motor_steps;

  /*
  relative_motor_steps = abs(motor_1_front.currentPosition() - next_motor_pos);
  Serial.print("relative motor: ");
  Serial.println(relative_motor_steps);

  if (relative_motor_steps > 253) { // 240 steps = 90 degrees
  //if (relative_motor_steps > 120) {
    Serial.println("relative motor steps too large");
    return false;
  } else {
    return true;
  }*/

  relative_motor_steps = abs(motor_1_front.currentPosition() - next_motor_pos);
  Serial.print("relative motor: ");
  Serial.println(relative_motor_steps);

  //if (relative_motor_steps > 253 || motor_1_front.currentPosition() < -160) { // 240 steps = 90 degrees
  if (relative_motor_steps > 248 || motor_1_front.currentPosition() < -248) { // 240 steps = 90 degrees
    Serial.println("relative motor steps too large");
    return false;
  } else {
    return true;
  }
}


// low level functions ====================================================
void bend(boolean approach, int motor_position) {

  int motor_position_with_overbending;

  /*
    if(motor_position >= 0) {                             /// overbending  val
    motor_position_with_overbending = motor_position + overbending_val;
    } else {
    motor_position_with_overbending = motor_position - overbending_val;
    }*/

  if (motor_1_front.currentPosition() != motor_position) {
  
    if (motor_position >= motor_1_front.currentPosition()) {                            /// overbending  val
      motor_position_with_overbending = motor_position + overbending_val;
    } else {
      motor_position_with_overbending = motor_position - overbending_val;
    }
  
    if (valid_relative_motor_steps(motor_position_with_overbending)) { /// relative movement is below 90 degrees
  
      if (approach) {
        move_rel(depth_for_bend, motor_3_vertical); /// move elevetor to bending position
      }
  
      digitalWrite(relay_pneu_welder, HIGH);              /// close welding electrodes
      delay(100);
  
      move_abs(motor_position_with_overbending, motor_1_front); /// overbending
      delay(100);
  
      move_abs(motor_position, motor_1_front);              /// go to actual bending position
      digitalWrite(relay_pneu_welder, LOW);                 /// open welding electrodes
      delay(100);
  
      move_rel(-depth_for_bend, motor_3_vertical);    /// move elevator out of the way
      delay(100);
  
      move_abs(-motor_position, motor_2_back);              /// realign rear
  
    } else {
  
      if (approach) {
        Serial.println("elevator is already out of the way");
      } else {
        //move_rel(-depth_for_weld_upper, motor_3_vertical);    /// move elevator out of the way
        move_rel(-depth_for_bend, motor_3_vertical);    /// move elevator out of the way
      }
  
      move_abs_2_motors(motor_position, motor_1_front, motor_2_back);  /// move both motors together
  
      move_abs(motor_position_with_overbending, motor_1_front); /// overbending
      delay(100);
      move_abs(motor_position, motor_1_front);                  /// go to actual bending position
      delay(100);
    }
  }else{

    move_rel(-depth_for_bend, motor_3_vertical);    /// move elevator out of the way
    delay(100);
    
    }
}

/*void insert_vertical_oldweld(boolean retract, int depth_for_weld_upper, int depth_for_weld_lower, int depth_cutting_offset){
  Serial.println("inserting vertical");

  digitalWrite(relay_pneu_feeder, HIGH);                      /// close feeder
  move_rel(depth_for_weld_lower, motor_3_vertical);           /// move elevator down all the way
  delay (100);

  weld();                                                     /// weld the first knot

  if (depth_for_weld_lower > depth_for_weld_lower_short){     /// --> long vertical


    int steps_to_go_back_up = depth_for_weld_lower - depth_for_cutting_long;

    digitalWrite(relay_pneu_feeder, LOW);                     /// open feeder
    move_rel(-steps_to_go_back_up, motor_3_vertical);         /// move elevator up to cutting pos
    digitalWrite(relay_pneu_feeder, HIGH);                    /// close feeder
    delay (100);

    cut();                                                 ///cut

    depth_for_weld_lower = depth_for_weld_lower - steps_to_go_back_up; // must be 3400 after calc

  }else{                                                   /// --> short vertical

      int remaining_steps = depth_for_cutting_short - depth_for_weld_lower_short;

      digitalWrite(relay_pneu_feeder, LOW);                 /// open feeder
      move_rel(remaining_steps, motor_3_vertical);          /// move elevator down to cutting pos
      digitalWrite(relay_pneu_feeder, HIGH);                /// close feeder
      delay (100);

      cut();                                                ///cut

      depth_for_weld_lower = depth_for_cutting_short;       // must be 3400
  }


  move_rel(-depth_cutting_offset, motor_3_vertical);        /// take the wire some mm up
  delay(100);
  digitalWrite(relay_pneu_feeder, LOW);                     /// open feeder
  delay(100);

  move_rel(-(depth_for_weld_lower - depth_cutting_offset - depth_for_weld_upper), motor_3_vertical); /// move elevator up to the upper welding position
  delay (100);


  // just temporary ----->>>>
  //digitalWrite(relay_pneu_feeder, LOW);                     /// open feeder
  //delay(100);
  // move_rel(-(depth_for_weld_lower - depth_for_weld_upper), motor_3_vertical); /// move elevator up to the upper welding position
  // just temporary ----->>>>

  weld();                                                   /// weld the second knot

  if (retract){
      move_rel(-depth_for_weld_upper, motor_3_vertical);     /// move elevator up to the top
      delay (100);

      // just temporary ----->>>>
      //delay (2000);
      // just temporary ----->>>>
  }

  }
*/

/*
void insert_vertical(boolean retract, int depth_for_weld, int depth_cutting_offset) {
  Serial.println("inserting vertical");

  digitalWrite(relay_pneu_feeder, HIGH);                      /// close feeder
  move_rel(depth_for_weld, motor_3_vertical);                 /// move elevator down all the way
  delay (100);

  weld();                                                     /// weld both knots
  delay(100);


  // cut deeper then weld:
  digitalWrite(relay_pneu_feeder, LOW);                       /// open feeder
  move_rel((depth_for_cutting-depth_for_weld), motor_3_vertical);
  cut();    
  delay(50); 
  move_rel(-(depth_for_cutting-depth_for_weld), motor_3_vertical);
  digitalWrite(relay_pneu_feeder, HIGH);                      /// close feeder
  delay(50); 

  move_rel(-depth_cutting_offset, motor_3_vertical);          /// take the wire some mm up
  delay(100);
  digitalWrite(relay_pneu_feeder, LOW);                       /// open feeder
  delay(100);
  
  if (retract) {
    move_rel(-(depth_for_weld - depth_cutting_offset), motor_3_vertical);        /// move elevator up to the top
    delay (100);
  }
}*/

void insert_vertical(boolean retract, int depth_for_weld, int depth_cutting_offset) {
  Serial.println("inserting vertical");

  digitalWrite(relay_pneu_feeder, HIGH);                      /// close feeder
  move_rel(depth_for_weld, motor_3_vertical);                 /// move elevator down all the way
  delay (100);

  cut_and_weld();                                             /// weld both knots nad cut before
  delay(100);

  /*
  // cut deeper then weld:
  digitalWrite(relay_pneu_feeder, LOW);                       /// open feeder
  move_rel((depth_for_cutting-depth_for_weld), motor_3_vertical);
  cut();    
  delay(50); 
  move_rel(-(depth_for_cutting-depth_for_weld), motor_3_vertical); 
  digitalWrite(relay_pneu_feeder, HIGH);                      /// close feeder
  delay(50); */

  move_rel(-depth_cutting_offset, motor_3_vertical);          /// take the wire some mm up
  delay(100);
  digitalWrite(relay_pneu_feeder, LOW);                       /// open feeder
  delay(100);
  
  if (retract) {
    move_rel(-(depth_for_weld - depth_cutting_offset), motor_3_vertical);        /// move elevator up to the top
    delay (100);
  }
}

void insert_vertical_bef_tilt(int depth_for_weld, int depth_cutting_offset, bool cut_deeper) {
  Serial.println("inserting vertical");

  digitalWrite(relay_pneu_feeder, HIGH);                      /// close feeder
  move_rel(depth_for_weld, motor_3_vertical);                 /// move elevator down all the way
  delay (100);

  cut_and_weld();                                             /// weld both knots nad cut before
  delay(100);

  if (cut_deeper) {
    // cut deeper then weld:
    digitalWrite(relay_pneu_feeder, LOW);                       /// open feeder
    move_rel((depth_for_cutting-depth_for_weld), motor_3_vertical);
    cut();    
    delay(50); 
    move_rel(-(depth_for_cutting-depth_for_weld), motor_3_vertical); 
    digitalWrite(relay_pneu_feeder, HIGH);                      /// close feeder
    delay(50);
  }

  move_rel(-depth_cutting_offset, motor_3_vertical);          /// take the wire some mm up
  delay(100);
  digitalWrite(relay_pneu_feeder, LOW);                       /// open feeder
  delay(100);
  
  move_rel(-(depth_for_weld - depth_cutting_offset - depth_for_bend), motor_3_vertical);        /// move elevator up to the bending pos
  delay (100);

}

void weld() { // welding a knot
  // 1. close the pneumatic cylinder
  digitalWrite(relay_pneu_welder, HIGH);
  delay (300);

  // 2. trigger the welding mechanism
  digitalWrite(relay_trigger_weld, HIGH);
  Serial.println("WELDING SIGNAL ON");
  delay (200);
  digitalWrite(relay_trigger_weld, LOW);
  Serial.println("WELDING SIGNAL OFF");
  delay (100);

  // 3. open the pneumatic cylinder
  digitalWrite(relay_pneu_welder, LOW);
  delay(50);
}

void cut_and_weld() { // welding a knot
  // 1. close the pneumatic cylinder
  digitalWrite(relay_pneu_welder, HIGH);
  delay (50);


  cut_with_open_feeder();    
  delay(100);
  cut();

  //sleepy time for stopping before welding
  for (int i=0; i <= 50; i++){ //100 for one second
      test_interrupt_state(true);
      delay(10);
  } 

  // 2. trigger the welding mechanism
  digitalWrite(relay_trigger_weld, HIGH);
  Serial.println("WELDING SIGNAL ON");
  delay (200);
  digitalWrite(relay_trigger_weld, LOW);
  Serial.println("WELDING SIGNAL OFF");
  delay (100);

  // 3. open the pneumatic cylinder
  digitalWrite(relay_pneu_welder, LOW);
  delay(50);
  
}

void move_rel(int motor_steps, AccelStepper &motor) {
  motor.move(motor_steps);
  while (abs(motor.distanceToGo()) > 0) {

    // check for pause
    test_interrupt_state(true);
    
    motor.run();
  }
}

void move_abs_2_motors(int motor_pos, AccelStepper &motor_f, AccelStepper &motor_b) {
  motor_f.moveTo(motor_pos);
  motor_b.moveTo(-motor_pos);
  while (motor_f.distanceToGo() != 0 || motor_b.distanceToGo() != 0) {

    // check for pause
    test_interrupt_state(true);
    
    motor_f.run();
    motor_b.run();
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

    // check for pause
    test_interrupt_state(true);
    
    motor.run();
  }

  //Serial.print("steps to go: (after command) - ");
  //Serial.println(motor.distanceToGo());

  Serial.print("current position: (after command) - ");
  Serial.println(motor.currentPosition());
}

void cut() {

  digitalWrite(relay_pneu_cutter, HIGH);                     /// cut
  delay (200);
  digitalWrite(relay_pneu_cutter, LOW);
  delay (100);

  /*
    for (int i = 0; i < 2; i++) {
    digitalWrite(relay_pneu_cutter, HIGH);                   /// cut
    delay (300);
    digitalWrite(relay_pneu_cutter, LOW);
    delay (300);
    }   */

  /*
    digitalWrite(relay_pneu_feeder, LOW);                      /// open feeder
    delay (2000);
    digitalWrite(relay_pneu_feeder, HIGH);                     /// close feeder
    delay (50);*/

}

void cut_with_open_feeder() {

  digitalWrite(relay_pneu_feeder, LOW);                      /// open feeder
  delay(50);
  digitalWrite(relay_pneu_cutter, HIGH);                     /// cut
  delay (200);
  digitalWrite(relay_pneu_feeder, HIGH);                      /// close feeder
  delay(50);
  digitalWrite(relay_pneu_cutter, LOW);
  delay (100);

}




// interrupt functions ====================================================
// test if the button is on interrupt high
void test_interrupt_state(boolean pausing) {
  di_interrupt_state = digitalRead(di_interrupt);
  //Serial.println(di_interrupt_state);
  if (di_interrupt_state == LOW) {
    delay(50);
    Serial.print("CHECK PAUSE -- di_interrupt_state: ");
    Serial.println(di_interrupt_state);
    if (pausing == true) {
      Serial.println("GO INTO PAUSE"); 
      pause();
    }
  }
}
// get into the while loop for pausing
void pause() {
  di_interrupt_state = digitalRead(di_interrupt);
  while (di_interrupt_state == LOW) {
    di_interrupt_state = digitalRead(di_interrupt);
    Serial.print("pausing: ");
    Serial.println(di_interrupt_state);
    delay(100);
  }
}

