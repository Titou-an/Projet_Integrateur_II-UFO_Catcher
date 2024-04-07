#include <PS4Controller.h>
#include <ps4.h>
#include <ps4_int.h>


const int BAUDRATE = 115200;

const int in1 = 18;
const int in2 = 19;
const int in3 = 33;
const int in4 = 32;

const int ENA = 26;
const int ENB = 25;

const float MAX_SPD = 256;
const float SPD_FRAC = 16;

int speed = 2*MAX_SPD/3;

bool estop = false;

void setup() {
  Serial.begin(BAUDRATE);

  PS4.attach(notify);

  PS4.begin();

  pinMode(in1, OUTPUT);
  pinMode(in2, OUTPUT);
  pinMode(in3, OUTPUT);
  pinMode(in4, OUTPUT);
  pinMode(ENA, OUTPUT);
  pinMode(ENB, OUTPUT);

  digitalWrite(ENA, LOW);
  digitalWrite(ENB, LOW);

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

    if (PS4.event.button_down.r1) {
      speed = constrain(speed + (MAX_SPD/SPD_FRAC), 0, MAX_SPD);
      Serial.printf("Increasing the speed: %d\n", speed);
    }

    if (PS4.event.button_down.l1) {
      speed = constrain(speed - (MAX_SPD/SPD_FRAC), 0, MAX_SPD);
      Serial.printf("Decreasing the speed: %d\n",speed);
    }

    if (PS4.event.button_down.triangle) {
      testSequence();
    }

    analogWrite(ENA, speed);
    analogWrite(ENB, speed);
  } else {
    stop();
  }
}

void loop() {
  
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
	digitalWrite(in3, HIGH);
	digitalWrite(in4, LOW);
}

void moveBackward(){
  Serial.println("MOVING BACKWARD");
	digitalWrite(in1, LOW);
	digitalWrite(in2, HIGH);
	digitalWrite(in3, LOW);
	digitalWrite(in4, HIGH);
}

void turnLeft(){
  Serial.println("TURNING LEFT");
	digitalWrite(in1, HIGH);
	digitalWrite(in2, LOW);
	digitalWrite(in3, LOW);
	digitalWrite(in4, LOW);
}

void turnRight(){
  Serial.println("TURNING RIGHT");
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
	digitalWrite(in3, HIGH);
	digitalWrite(in4, LOW);
}

void stop(){
	digitalWrite(in1, LOW);
	digitalWrite(in2, LOW);
	digitalWrite(in3, LOW);
	digitalWrite(in4, LOW);
}
