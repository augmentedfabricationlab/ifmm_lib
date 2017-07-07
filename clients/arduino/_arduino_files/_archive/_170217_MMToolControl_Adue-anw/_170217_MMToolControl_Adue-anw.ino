/*

  . . . . . . . . . . . . . . . . . . . . . .
  .                                         .
  .   <<  <<><><>  <<      ><  <<      ><   .
  .   <<  <<       < ><   ><<  < ><   ><<   .
  .   <<  <<><><>  << >< > ><  << >< > ><   .
  .   <<  <<       <<  ><  ><  <<  ><  ><   .
  .   <<  <<       <<      ><  <<      ><   .
  .   <<  <<       <<      ><  <<      ><   .
  .                                         .
  .               GKR 2017                  .
  . . . . . . . . . . . . . . . . . . . . . .

@walzdorf

  Arduino Server
  This server can receive messages from a client, and execute callback functions.
*/

#include <SPI.h>
#include <AccelStepper.h>
#include <Ethernet.h> //#include <Ethernet2.h>
#include <Encoder.h>

long int_value;
bool cb_success;


// server setup =========================================================
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED };
IPAddress ip(192, 168, 125, 10);
IPAddress gateway(192, 168, 10, 1);
IPAddress subnet(255, 255, 255, 0);
int SERVER_PORT = 30001;

EthernetServer server(SERVER_PORT);
boolean CONNECTED = false; // whether or not the client is connected

// global parameter setup ===============================================
long msg_len = 0;
long msg_id = 0;
long msg_counter = 0;
long msg_wait_for_response = 0;

// message types ========================================================
const int MSG_CMD_RECEIVED = 1;
const int MSG_CMD_EXECUTED = 2;
const int MSG_DIGITAL_OUT = 3;
const int MSG_FLOAT_LIST = 4;
const int MSG_INFO = 5;
const int MSG_STRING = 6;
const int MSG_CMD_ROTATE_MOTOR_ABS = 7; //[int1, int2] # int1 = absolute motor value for rotation, int2 = motor num
const int MSG_CMD_ROTATE_MOTOR_REL = 8; //[int1, int2] # int1 = absolute motor value for rotation, int2 = motor num
const int MSG_CMD_FEED_WIRE = 9; //[int1] # int1 = feed len in mm
const int MSG_CMD_ELECTRODES = 10; //[int1] # int1 = 0: OPEN # int1 = 1: CLOSE # int1 = 1: STOP
const int MSG_CMD_NIPPERS = 11; // # int1 = 0: MOVE_TO_BACK, int1 = 1: MOVE_TO_FRONT, int1 = 2: CUT
const int MSG_CMD_WELD = 12; //[] empty message
const int MSG_CMD_ROUTINE_FEED_CUT_WELD = 13; //[int1] # int1 = feed len in mm


// ====================================================================

// conversions ========================================================
long bytesToInteger(byte b[4]) {
  long val = 0;
  val = ((long )b[3]) << 24;
  val |= ((long )b[2]) << 16;
  val |= ((long )b[1]) << 8;
  val |= b[0];
  return val;
}
//
void integerToBytes(long val, byte b[4]) {
  b[3] = (byte )((val >> 24) & 0xff);
  b[2] = (byte )((val >> 16) & 0xff);
  b[1] = (byte )((val >> 8) & 0xff);
  b[0] = (byte )(val & 0xff);
}
// ====================================================================

// sending ============================================================
void send_string(EthernetClient c, char msg[]) {
  c.write(msg);
}
//
void send_int(EthernetClient c, long integer) {
  byte bbuffer[4];
  integerToBytes(integer, bbuffer);
  for (int i = 0; i < 4; i++) {
    c.write(bbuffer[i]);
  }
}
//
void send_msg_header(EthernetClient c, long msg_len, long msg_id, long msg_counter) {
  send_int(c, msg_len); // 1. send msg length // = 4 bytes
  send_int(c, msg_id);  // 2. send msg type // = 4 bytes
  send_int(c, msg_counter); // 3. send msg counter // = 4 bytes
}
//
void send_info_msg(EthernetClient c, long msg_counter) {
  Serial.println("SENDING MSG_INFO");
  //char info_msg[] = "Surrender!";
  char *info_msgs[] = {"Surrender!", "Gleep!", "Boing!", "Clash!", "Splat!"};
  long info_msgs_len[] = {10, 6, 6, 6, 6};
  long info_msg_num;
  info_msg_num = random(sizeof(info_msgs) / sizeof(char*));
  char *info_msg = (info_msgs[info_msg_num]);
  long msg_length = 4 + (info_msgs_len[info_msg_num]);
  //long msg_length = 4 + (sizeof(info_msg)-1);
  //Serial.print("info msg len"); //Serial.println(sizeof(info_msg));
  send_msg_header(c, msg_length, MSG_INFO, msg_counter);
  send_string(c, info_msg);
}
//
void send_msg_cmd_exec(EthernetClient c, long msg_counter) {
  Serial.println("SENDING MSG_CMD_EXECUTED");
  long msg_length = 4;
  send_msg_header(c, msg_length, MSG_CMD_EXECUTED, msg_counter);
}
//
void send_msg_cmd_received(EthernetClient c, long msg_counter) {
  Serial.println("SENDING MSG_CMD_RECEIVED");
  long msg_length = 4;
  send_msg_header(c, msg_length, MSG_CMD_RECEIVED, msg_counter);
}
// ====================================================================

// reading ============================================================
void read_msg_raw_from_socket(EthernetClient c, long msg_length) {
  byte msg_buffer[msg_length];
  for (int i = 0; i < msg_length; i++) {
    byte b = c.read();
    msg_buffer[i] = b;
  }
}
//
long read_int(EthernetClient c) {
  byte buf[4];
  for (int i = 0; i < 4; i++) {
    byte b = c.read();
    buf[i] = b;
  }
  return bytesToInteger(buf);
}
//
float read_float(EthernetClient c) {
  union {
    float float_variable;
    byte b[4];
  } my_union;
  for (int i = 0; i < 4; i++) {
    byte b = c.read();
    my_union.b[i] = b;
  }
  return my_union.float_variable;
}
//
void read_msg_header_from_socket(EthernetClient c) {
  // 1. read msg length from socket // = 4 bytes
  msg_len = read_int(c) - 4;
  // 2. read msg type from socket // = 4 bytes
  msg_id = read_int(c);
  //  3. read msg counter from socket // = 4 bytes
  msg_counter = read_int(c);
  //  4. read if client is waiting for response // = 4 bytes
  msg_wait_for_response = read_int(c);
}
//
void read_msg_do_from_socket(EthernetClient c, long do_state[2]) {
  // 1. read do int // = 4 bytes
  do_state[0] = read_int(c);
  // 2. read do state // = 4 bytes
  do_state[1] = read_int(c);
}

//
void read_msg_cmd_rotabs_from_socket(EthernetClient c, long int_vals_for_rot[2]) {
  // 1. read int vals // = 2 * 4 bytes
  for (int i = 0; i < 2; i++) {
    int_vals_for_rot[i] = read_int(c);
  }
}

//
void read_msg_cmd_rotrel_from_socket(EthernetClient c, long int_vals_for_rot_rel[2]) {
  // 1. read 2 int vals // = 2 * 4 bytes
  for (int i = 0; i < 2; i++) {
    int_vals_for_rot_rel[i] = read_int(c);
  }
}

//
void read_msg_cmd_rotbend_from_socket(EthernetClient c, long int_vals_for_rot_bend[2]) {
  // 1. read 2 int vals // = 2 * 4 bytes
  for (int i = 0; i < 2; i++) {
    int_vals_for_rot_bend[i] = read_int(c);
  }
}

//
void read_msg_cmd_from_socket(EthernetClient c, float f[1]) {
  // 1. read float // = 4 bytes
  f[0] = read_float(c);
}
// ====================================================================

// ==================================================================
// setup ============================================================
void setup() {
  // set up Ethernet server
  Ethernet.begin(mac, ip, gateway, subnet); // initialize the ethernet device
  server.begin();

  // start serial
  Serial.begin(9600);

  //while (!Serial) {
  //  ; // wait for serial port to connect. // Needed for Leonardo only
  //}

  Serial.print("server address: ");
  Serial.println(Ethernet.localIP());

  // set up the pin outputs
  pin_setup();
  // set up the motor
  motors_setup();

  // communication with hydraulics control arduino
  Serial1.begin(9600);
}
// ==================================================================

// ==================================================================
// ==================================================================
// loop =============================================================
void loop() {

  EthernetClient client = server.available(); // wait for a new client
  //Serial.print("server.available: ");//Serial.print(server.available());//Serial.print("- connected: ");//Serial.println(client.connected());

  if (client) {
    CONNECTED = true;
    Serial.print("connected: ");
    Serial.println(client.connected());

    while (client.connected()) { //while (CONNECTED) {

      if (client.available() > 0) { //if bytes are available on the socket
        Serial.println("reading header."); // read the bytes incoming from the client for the message header
        read_msg_header_from_socket(client);
        /*
          Serial.print("msg len: ");
          Serial.println(msg_len);
          Serial.print("msg_id: ");
          Serial.println(msg_id);*/
        Serial.print("msg_counter: ");
        Serial.println(msg_counter);
        Serial.print("wait for response: ");
        Serial.println(msg_wait_for_response);

        // callback routines, depending on the message type
        switch (msg_id) {
          case MSG_INFO:
            Serial.println("MSG TYPE: MSG_INFO");
            //send back msg received
            send_msg_cmd_received(client, msg_counter);
            delay(1);
            send_info_msg(client, msg_counter); // send back the info message as string.
            break;

          case MSG_DIGITAL_OUT:
            Serial.println("MSG TYPE: MSG_DIGITAL_OUT");
            long do_state[2];
            read_msg_do_from_socket(client, do_state);
            //send back msg received
            send_msg_cmd_received(client, msg_counter);
            //exec callback function
            cb_set_do(do_state);
            //send back msg cmd executed
            if (msg_wait_for_response == 1) {
              send_msg_cmd_exec(client, msg_counter);
            }
            break;

          case MSG_CMD_ROTATE_MOTOR_ABS:
            Serial.println("MSG TYPE: MSG_CMD_ROTATE_MOTOR_ABS");
            long int_vals_for_rot[2];
            read_msg_cmd_rotabs_from_socket(client, int_vals_for_rot);
            //send back msg received
            send_msg_cmd_received(client, msg_counter);
            //exec callback function
            cb_rotate_motor_abs(int_vals_for_rot);
            //send back msg cmd executed
            if (msg_wait_for_response == 1) {
              send_msg_cmd_exec(client, msg_counter);
            }
            break;

          case MSG_CMD_ROTATE_MOTOR_REL:
            Serial.println("MSG TYPE: MSG_CMD_ROTATE_MOTOR_REL");
            long int_vals_for_rot_rel[2];
            read_msg_cmd_rotrel_from_socket(client, int_vals_for_rot_rel);
            //send back msg received
            send_msg_cmd_received(client, msg_counter);
            //exec callback function
            cb_rotate_motor_rel(int_vals_for_rot_rel);
            //send back msg cmd executed
            if (msg_wait_for_response == 1) {
              send_msg_cmd_exec(client, msg_counter);
            }
            break;

          case MSG_CMD_BEND:
            Serial.println("MSG TYPE: MSG_CMD_BEND");
            long int_vals_for_rot_bend[2];
            read_msg_cmd_rotbend_from_socket(client, int_vals_for_rot_bend);
            //send back msg received
            send_msg_cmd_received(client, msg_counter);
            //exec callback function
            cb_bend(int_vals_for_rot_bend);
            //send back msg cmd executed
            if (msg_wait_for_response == 1) {
              send_msg_cmd_exec(client, msg_counter);
            }
            break;

          case MSG_CMD_WELD:
            Serial.println("MSG TYPE: MSG_CMD_WELD");
            //send back msg received
            send_msg_cmd_received(client, msg_counter);
            //exec callback function
            cb_weld();
            //send back msg cmd executed
            if (msg_wait_for_response == 1) {
              send_msg_cmd_exec(client, msg_counter);
            }
            break;

          case MSG_CMD_RESET_ENCODER:
            Serial.println("MSG TYPE: MSG_CMD_RESET_ENCODER");
            //send back msg received
            send_msg_cmd_received(client, msg_counter);
            //exec callback function
            cb_reset_encoder();
            //send back msg cmd executed
            if (msg_wait_for_response == 1) {
              send_msg_cmd_exec(client, msg_counter);
            }
            break;

          case MSG_CMD_FEED_WIRE:
            Serial.println("MSG TYPE: MSG_CMD_FEED_WIRE");
            int_value = read_int(client);
            //send back msg received
            send_msg_cmd_received(client, msg_counter);
            //exec callback function
            cb_feed_wire(int_value);
            //send back msg cmd executed
            if (msg_wait_for_response == 1) {
              send_msg_cmd_exec(client, msg_counter);
            }
            break;

          case MSG_CMD_ELECTRODES:
            Serial.println("MSG TYPE: MSG_CMD_ELECTRODES");
            int_value = read_int(client);
            //send back msg received
            send_msg_cmd_received(client, msg_counter);
            //exec callback function
            cb_actuate_electrodes(int_value);
            //send back msg cmd executed
            if (msg_wait_for_response == 1) {
              send_msg_cmd_exec(client, msg_counter);
            }
            break;

          case MSG_CMD_NIPPERS:
            Serial.println("MSG TYPE: MSG_CMD_NIPPERS");
            int_value = read_int(client);
            //send back msg received
            send_msg_cmd_received(client, msg_counter);
            //exec callback function
            cb_actuate_nippers(int_value);
            //send back msg cmd executed
            if (msg_wait_for_response == 1) {
              send_msg_cmd_exec(client, msg_counter);
            }
            break;

          case MSG_CMD_ROUTINE_FEED_CUT_WELD:
            Serial.println("MSG TYPE: MSG_CMD_ROUTINE_FEED_CUT_WELD");
            int_value = read_int(client);
            //send back msg received
            send_msg_cmd_received(client, msg_counter);
            //exec callback function
            cb_success = cb_routine_feed_cut_weld(int_value);
            //cb_routine_feed_cut_weld(int_value);
            //send back msg cmd executed
            if (msg_wait_for_response == 1 && cb_success  == true) {
              send_msg_cmd_exec(client, msg_counter);
              // TO DO if feed_cut_weld_ok is NOT ok, no MSG_CMD will be sent back to Rhino,
              // TO DO could be changed eg. with msg_counter = -1
            }
            break;

          default:
            // if nothing else matches, do the default
            // default is optional
            Serial.println("MSG TYPE UNKOWN.");
            break;
        } //endswitch
      } //endif(client.available()
    }// end while(client.connected())

    //delay(1);
    client.stop(); // close the connection
    CONNECTED = false;
    Serial.println("disconnected.");
  }
  //delay(1);
}

// ==================================================================
// ==================================================================
// ==================================================================

