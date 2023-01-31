#include <Servo.h> 

///////////////// motors config////////////


Servo right_thruster;
int right_thruster_pinA = 9;
int right_thruster_code = 1;   //used to map the dof over the encrypted message sent via serial from the python code
int right_thruster_min_pwm = 1100;
int right_thruster_max_pwm = 1900;
int right_thruster_last_command = 0;


Servo left_thruster;
int left_thruster_pin = 10;
int left_thruster_code = 2;
int left_thruster_min_pwm = 1100;
int left_thruster_max_pwm = 1900;
int left_thruster_last_command = 0;


Servo bow_thruster;
int bow_thruster_pin = 11;
int bow_thruster_code = 3;
int bow_thruster_min_pwm = 1100;
int bow_thruster_max_pwm = 1900;
int bow_thruster_last_command = 0;


Servo right_servo;
int right_servo_pin = 5;
int right_servo_code = 4;
int right_servo_min_pwm = 1100;
int right_servo_max_pwm = 1900;
int right_servo_last_command = 0;


Servo left_servo;
int left_servo_pin = 6;
int left_servo_code = 5;
int left_servo_min_pwm = 1100;
int left_servo_max_pwm = 1900;
int left_servo_last_command = 0;


float thrusters_command = 0;
//////////////////////////////////////////



/////////// serial communication config ///////////////////////

int message = 0;
int code = 0;
int value = 0;
int translated_message[2];  //  [the wanted motor, the wanted command]

////////////////////


void setup() {
    /////////// serial communication config ///////////////////////
  Serial.begin(9600);
  Serial.setTimeout(4);
  Serial. flush();
 ////////////////////
 
 
 ///////////////// motors config////////////
 
  right_thruster.attach(right_thruster_pin, right_thruster_min_pwm, right_thruster_max_pwm);
  left_thruster.attach(left_thruster_pin, left_thruster_min_pwm, left_thruster_max_pwm); 
  bow_thruster.attach(bow_thruster_pin, bow_thruster_min_pwm, bow_thruster_max_pwm); 
  right_servo.attach(right_servo_pin, right_servo_min_pwm, right_servo_max_pwm); 
  left_servo.attach(left_servo_pin, left_servo_min_pwm, left_servo_max_pwm);
  
  ////////////////////////////////////////


  /////////////// motor initialise ///////////////
  right_thruster.writeMicroseconds(1500);
  delay(7000);
  left_thruster.writeMicroseconds(1500);
  delay(7000);
  bow_thruster.writeMicroseconds(1500);
  delay(7000);
  /////////////////////////////////////

  void loop() {
    while (!Serial.available());
    message = Serial.readString().toInt();
    // message = 1000;
    
    code = message / 1000;
    Serial.println(code);
    value = message - code * 1000;
    Serial.println(value);
    
    if (code == right_thruster_code) {
        if (value != right_thruster_last_command) {
            right_thruster_last_command == value;
            thrusters_command = map(value, 0, 180, 1100, 1900);
            right_thruster.writeMicroseconds(thrusters_command);
            Serial.println("right thruster");
        }
    }
    
    if (code == left_thruster_code) {
        if (value != left_thruster_last_command) {
            left_thruster_last_command == value;
            thrusters_command = map(value, 0, 180, 1100, 1900);
            left_thruster.writeMicroseconds(thrusters_command);
            Serial.println("left thruster");
        }
    }
    
    if (code == bow_thruster_code) {
        if (value != bow_thruster_last_command) {
            bow_thruster_last_command == value;
            thrusters_command = map(value, 0, 180, 1100, 1900);
            bow_thruster.writeMicroseconds(thrusters_command);
            Serial.println("bow thruster");
        }
    }
    
    if (code == right_servo_code) {
        if (value != right_servo_last_command) {
            right_servo_last_command == value;
            thrusters_command = map(value, 0, 180, 1100, 1900);
            right_servo.writeMicroseconds(thrusters_command);
            Serial.println("right servo");
        }
    }
    
    if (code == left_servo_code) {
        if (value != left_servo_last_command) {
            left_servo_last_command == value;
            thrusters_command = map(value, 0, 180, 1100, 1900);
            left_servo.writeMicroseconds(thrusters_command);
            Serial.println("left servo");
        } ;
    }
}
