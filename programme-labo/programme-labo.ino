#include <Servo.h>
#include <pwmWrite.h>

// Program ro control the cleaning robot
// ESP32 MAC adress: cc:db:a7:49:58:0a

#include <PS4Controller.h>
#include <ps4.h>
#include <ps4_int.h>

#include "esp_bt_main.h"
#include "esp_bt_device.h"
#include "esp_gap_bt_api.h"
#include "esp_err.h"

const int BAUDRATE = 115200;

#define in1 18
#define in2 19
#define in3 33
#define in4 32

#define in5 16
#define in6 17

#define ENA 26
#define ENB 25

#define MAG_SERVO 21
#define CLAW_SERVO 15

Servo servo = Servo();

const float MAX_SPD = 256;
const float SPD_FRAC = 16;

int speed = 2*MAX_SPD/3;

bool claw_open = true;
int claw_angle = 75;
int claw_value = 0;

bool estop = false;
bool lock_claw = true;


void setup() {
  Serial.begin(BAUDRATE);

  PS4.attach(notify);
  PS4.attachOnConnect(onConnect);
  PS4.attachOnDisconnect(onDisConnect);

  PS4.begin();
  removePairedDevices(); // This helps to solve connection issues

  servo.write(MAG_SERVO, 0);
  servo.write(CLAW_SERVO, 75);
  claw_open = true;

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(in5, OUTPUT);
  pinMode(in6, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);

  Serial.print("This device MAC is: "); // ESP32 MAC adress: cc:db:a7:49:58:0a
  printDeviceAddress();

  Serial.println("Robot Started!");
}

void notify(){
  if (PS4.isConnected()){

    // Movement control
    if (PS4.Up()){
      moveForward();
    } else if (PS4.Down()) {
      moveBackward();
    } else if (PS4.Right()) {
      turnRight();
    } else if (PS4.Left()) {
      turnLeft();
    } else {
      stop();
    }

    
    if (PS4.R2()) {
      digitalWrite(in5, LOW);
	    digitalWrite(in6, HIGH);
    } else if (PS4.L2()){
      digitalWrite(in5, HIGH);
	    digitalWrite(in6, LOW);
    } else if (lock_claw) {
      digitalWrite(in5, HIGH);
	    digitalWrite(in6, LOW);
    } else {
      digitalWrite(in5, LOW);
	    digitalWrite(in6, LOW);
    }

    if (PS4.event.button_down.options) {
      lock_claw = !lock_claw;
    }
    

    if (PS4.event.button_down.r1) {
      speed = constrain(speed + (MAX_SPD/SPD_FRAC), 0, MAX_SPD);
      Serial.printf("Increasing the speed: %d\n", speed);
    }

    if (PS4.event.button_down.l1) {
      speed = constrain(speed - (MAX_SPD/SPD_FRAC), 0, MAX_SPD);
      Serial.printf("Decreasing the speed: %d\n",speed);
    }

    if (PS4.event.button_down.triangle) {
      for (int pos = 0; pos <= 180; pos++) {  // go from 0-180 degrees
        servo.write(MAG_SERVO, pos);        // set the servo position (degrees)
      }
    } else if (PS4.event.button_down.square) {
      for (int pos = 180; pos >= 0; pos--) {  // go from 180-0 degrees
        servo.write(MAG_SERVO, pos);        // set the servo position (degrees)
      }
    }

    if (PS4.event.button_down.circle) {

      // claw_value = constrain(claw_value++,0,100);
      // claw_angle = map(claw_value, 0, 100, 75, 120);
      claw_angle = 170;

    } else if (PS4.event.button_down.cross) {
      // claw_value = constrain(claw_value--,0,100);
      // claw_angle = map(claw_value, 0, 100, 75, 120);
      claw_angle = 75;
      
    }

    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
  } else {
    stop();
  }
}

void loop() {
  servo.write(CLAW_SERVO, claw_angle,300.0,0.0);
}

void testSequence(){
  moveForward();

  for (int i = 0; i < MAX_SPD; i+=(MAX_SPD/SPD_FRAC)) {

    speed = i;
    Serial.printf("Increasing the speed: %d\n", speed);
    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
    delay(1500);
  }

  speed = 0;
  stop();
}

void moveForward(){
  Serial.println("MOVING FORWARD");
	digitalWrite(in1, HIGH);
	digitalWrite(in2, LOW);
	digitalWrite(in3, LOW);
	digitalWrite(in4, HIGH);
}

void moveBackward(){
  Serial.println("MOVING BACKWARD");
	digitalWrite(in1, LOW);
	digitalWrite(in2, HIGH);
	digitalWrite(in3, HIGH);
	digitalWrite(in4, LOW);
}

void turnLeft(){
  Serial.println("TURNING LEFT");
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
	digitalWrite(in3, LOW);
	digitalWrite(in4, HIGH);
}

void turnRight(){
  Serial.println("TURNING RIGHT");
	digitalWrite(in1, HIGH);
	digitalWrite(in2, LOW);
	digitalWrite(in3, LOW);
	digitalWrite(in4, LOW);
}

void stop(){
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
	digitalWrite(in3, LOW);
	digitalWrite(in4, LOW);
}

void printDeviceAddress() {
  const uint8_t* point = esp_bt_dev_get_address();
  for (int i = 0; i < 6; i++) {
    char str[3];
    sprintf(str, "%02x", (int)point[i]);
    Serial.print(str);
    if (i < 5) {
      Serial.print(":");
    }
  }
  Serial.println();
}

void onConnect() {
  Serial.println("Connected!");
  servo.write(MAG_SERVO, 0);
  servo.write(CLAW_SERVO, 75);
}

void onDisConnect() {
  Serial.println("Disconnected!");
  servo.write(MAG_SERVO, 0);
  servo.write(CLAW_SERVO, 180);
  stop();
}


void removePairedDevices() {
  uint8_t pairedDeviceBtAddr[20][6];
  int count = esp_bt_gap_get_bond_device_num();
  esp_bt_gap_get_bond_device_list(&count, pairedDeviceBtAddr);
  for (int i = 0; i < count; i++) {
    esp_bt_gap_remove_bond_device(pairedDeviceBtAddr[i]);
  }
}


