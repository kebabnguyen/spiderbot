#include <Wire.h>
#include <Adafruit_PWMServoDriver.h>
#include <PS4BT.h>
#include <usbhub.h>
#ifdef dobogusinclude
#include <spi4teensy3.h>
#endif
#include <SPI.h>
USB Usb;
BTD Btd(&Usb);
PS4BT PS4(&Btd);
bool active = false, idle = false;
Adafruit_PWMServoDriver pwm = Adafruit_PWMServoDriver(0x40,Wire);
uint8_t servonum = 0; int buttonState = 0; int buttonStop = 0;
int val1; int val2; int val3;
int p1; int p2; int p3;
String str1, str2, str3;
int d = 5;
int start[] = {295,194,240,318,447,215,212,205,305,201,407,305};
void moveLeg1(int a, int b, int c) {
  pwm.setPWM(0,0,a);
  delay(d);
  pwm.setPWM(1,0,b);
  delay(d);
  pwm.setPWM(2,0,c);
  delay(d);
}
void moveLeg2(int a, int b, int c) {
  pwm.setPWM(3,0,a);
  delay(d);
  pwm.setPWM(4,0,b);
  delay(d);
  pwm.setPWM(5,0,c);
  delay(d);
}
void moveLeg3(int a, int b, int c) {
  pwm.setPWM(8,0,c);
  delay(d);
  pwm.setPWM(7,0,b);
  delay(d);
  pwm.setPWM(6,0,a);
  delay(d);
}
void moveLeg4(int a, int b, int c) {
  pwm.setPWM(11,0,c);
  delay(d);
  pwm.setPWM(10,0,b);
  delay(d);
  pwm.setPWM(9,0,a);
  delay(d);
}
void legInit() {
  moveLeg1(start[0],start[1],start[2]);
  moveLeg2(start[3],start[4],start[5]);
  moveLeg3(start[6],start[7],start[8]);
  moveLeg4(start[9],start[10],start[11]);
}
void setup() {
   Serial.begin(115200);
   Wire.begin();
   pwm.begin();
   pinMode(2, INPUT); //button input
   pinMode(LED_BUILTIN,OUTPUT);
  if (Usb.Init() == -1) {
    Serial.print(F("\r\nOSC did not start"));
    while (1); // Halt
  }
   pwm.setOscillatorFrequency(27000000);  // The int.osc. is closer to 27MHz  
   pwm.setPWMFreq(50);
  Serial.println("Initializing...");
  for(int pin = 0; pin < 11; pin++)
  {
     pwm.setPWM(pin, 4096, 0);
     delay(50);
  }
  legInit();
  Serial.println("good to go!");
}
void leg1Test() {//function to manuever the three servos in leg1 using potentiometers
  servonum = 0;
    val1 = analogRead(0);
  val2 = analogRead(1);
  val3 = analogRead(2);
  val1 = map(val1, 0, 1023, 530, 280);
  val2 = map(val2, 0, 1023, 75, 530);
  val3 = map(val3, 0, 1023, 360, 90);
    moveLeg1(val1,val2,val3);
  buttonState = digitalRead(2);
  str1 = String("servo ");
  str2 = String(": ");
  if((buttonState != 0)) {
    Serial.println(str1 + servonum + str2 + val1);
    Serial.println(str1 + (servonum+1) + str2 + val2);
    Serial.println(str1 + (servonum+2) + str2 + val3);
    delay(200);
  }
}
void leg2Test() {//function to manuever the three servos in leg2 using potentiometers
  servonum = 3;
    val1 = analogRead(0);
  val2 = analogRead(1);
  val3 = analogRead(2);
  val1 = map(val1, 0, 1023, 90, 330);
  val2 = map(val2, 0, 1023, 530, 110);
  val3 = map(val3, 0, 1023,75, 320);
   moveLeg2(val1,val2,val3);
  buttonState = digitalRead(2);
  str1 = String("servo ");
  str2 = String(": ");
  if((buttonState != 0)) {
    Serial.println(str1 + servonum + str2 + val1);
    Serial.println(str1 + (servonum+1) + str2 + val2);
    Serial.println(str1 + (servonum+2) + str2 + val3);
    delay(200);
  }
}
void leg3Test() {//move the three servos in leg3 using potentiometers
  servonum = 6;
    val1 = analogRead(0);
  val2 = analogRead(1);
  val3 = analogRead(2);
  val1 = map(val1, 0, 1023, 360, 100);
  val2 = map(val2, 0, 1023, 75, 530);
  val3 = map(val3, 0, 1023, 510, 260);
    moveLeg3(val1,val2,val3);
  buttonState = digitalRead(2);
  str1 = String("servo ");
  str2 = String(": ");
  if((buttonState != 0)) {
    Serial.println(str1 + servonum + str2 + val1);
    Serial.println(str1 + (servonum+1) + str2 + val2);
    Serial.println(str1 + (servonum+2) + str2 + val3);
    delay(200);
  }
}
void leg4Test() { //move the three servos in leg4 using pots
  servonum = 9;
    val1 = analogRead(0);
  val2 = analogRead(1);
  val3 = analogRead(2);
  val1 = map(val1, 0, 1023, 75, 320);
  val2 = map(val2, 0, 1023, 530, 90);
  val3 = map(val3, 0, 1023, 75, 530);
    moveLeg4(val1,val2,val3);
  buttonState = digitalRead(2);
  str1 = String("servo ");
  str2 = String(": ");
  if((buttonState != 0)) {
    Serial.println(str1 + servonum + str2 + val1);
    Serial.println(str1 + (servonum+1) + str2 + val2);
    Serial.println(str1 + (servonum+2) + str2 + val3);
    delay(200);
  }
}

void leg1Forward() { //leg1 takes a step forward
  moveLeg1(start[0],start[1],start[2]);
  delay(50);
  pwm.setPWM(1,0,134);
  delay(50);
  pwm.setPWM(2,0,(300+40));
  delay(100);
  pwm.setPWM(1,0,start[1]);
  delay(50);
  pwm.setPWM(2,0,start[2]);
}

void leg2Forward() { //leg2 takes a step forward
  moveLeg2(start[3],start[4],start[5]);
  delay(50);
  pwm.setPWM(4,0,507);
  delay(50);
  pwm.setPWM(5,0,(275+40));
  delay(100);
  pwm.setPWM(4,0,start[4]);
  delay(50);
  pwm.setPWM(5,0,start[5]);
}
void leg3Forward() { //leg3 takes a step forward
  moveLeg3(start[6],start[7],start[8]);
  delay(50);
  pwm.setPWM(7,0,158);
  delay(50);
  pwm.setPWM(6,0,(152-40));
  delay(100);
  pwm.setPWM(7,0,start[7]);
  delay(50);
  pwm.setPWM(6,0,start[6]);
}
void leg4Forward() { //leg4 takes a step forward
  moveLeg4(start[9],start[10],start[11]);
  delay(50);
  pwm.setPWM(10,0,473);
  delay(50);
  pwm.setPWM(9,0,(141-40));
  delay(100);
  pwm.setPWM(10,0,start[10]);
  delay(50);
  pwm.setPWM(9,0,start[9]);
}
void moveForward() { //combine all the legs taking a step to take one step forward
  leg1Forward();
  leg2Forward();
  leg3Forward();
  leg4Forward();
}
void leg1Right() { //leg1 takes a step to turn right
  moveLeg1(start[0],start[1],start[2]);
  delay(50);
  pwm.setPWM(1,0,134);
  delay(50);
  pwm.setPWM(2,0,180);
  delay(50);
  pwm.setPWM(1,0,start[1]);
  delay(50);
  pwm.setPWM(2,0,start[2]);
}
void leg2Right() { //leg2 step to turn right
  moveLeg2(start[3],start[4],start[5]);
  delay(50);
  pwm.setPWM(4,0,507);
  delay(50);
  pwm.setPWM(5,0,155);
  delay(50);
  pwm.setPWM(4,0,start[4]);
  delay(50);
  pwm.setPWM(5,0,start[5]);
}
void leg3Right() { //leg3 step to turn right
  moveLeg3(start[6],start[7],start[8]);
  delay(50);
  pwm.setPWM(7,0,145);
  delay(50);
  pwm.setPWM(6,0,152);
  delay(50);
  pwm.setPWM(7,0,start[7]);
  delay(50);
  pwm.setPWM(6,0,start[6]);
}
void leg4Right() { //leg4 step to turn right
  moveLeg4(start[9], start[10], start[11]);
  delay(50);
  pwm.setPWM(10,0,467);
  delay(50);
  pwm.setPWM(9,0,141);
  delay(50);
  pwm.setPWM(10,0,start[10]);
  delay(50);
  pwm.setPWM(9,0,start[9]);
}
void turnRight() { //rotate right one iteration
  leg1Right();
  leg2Right();
  leg3Right();
  leg4Right();
}
void leg1Left() { //leg1 step to rotate left
  moveLeg1(start[0],start[1],start[2]);
  delay(50);
  pwm.setPWM(1,0,134);
  delay(50);
  pwm.setPWM(2,0,300);
  delay(50);
  pwm.setPWM(1,0,start[1]);
  delay(50);
  pwm.setPWM(2,0,start[2]);
}
void leg2Left() { //leg2 step to rotate left
  moveLeg2(start[3],start[4],start[5]);
  delay(50);
  pwm.setPWM(4,0,507);
  delay(50);
  pwm.setPWM(5,0,275);
  delay(50);
  pwm.setPWM(4,0,start[4]);
  delay(50);
  pwm.setPWM(5,0,start[5]);
}
void leg3Left() { //leg3 step to rotate left
  moveLeg3(start[6],start[7],start[8]);
  delay(50);
  pwm.setPWM(7,0,145);
  delay(50);
  pwm.setPWM(6,0,272);
  delay(50);
  pwm.setPWM(7,0,start[7]);
  delay(50);
  pwm.setPWM(6,0,start[6]);
}
void leg4Left() { //leg4 step to rotate left
  moveLeg4(start[9],start[10],start[11]);
  delay(50);
  pwm.setPWM(10,0,467);
  delay(50);
  pwm.setPWM(9,0,261);
  delay(50);
  pwm.setPWM(10,0,start[10]);
  delay(50);
  pwm.setPWM(9,0,start[9]);
}
void turnLeft() { //one iteration to rotating left
  leg4Left();
  leg3Left();
  leg2Left();
  leg1Left();
}
void safechoice() {
  if(PS4.getButtonClick(PS)) {
    PS4.setLed(Red);
  }
  else {
    PS4.setLed(Green);
    if(PS4.getButtonClick(UP)) {
      moveForward();
    }
    if(PS4.getButtonClick(RIGHT)) {
      turnRight();
    }
    if(PS4.getButtonClick(LEFT)) {
      turnLeft();
    }
  }
}
void endgame() { //controller functions and making bot move
  int active;
  Usb.Task();
  if (PS4.connected()) {
    if(active == 1) {      
        if(PS4.getButtonClick(TRIANGLE)) {
          Serial.println("enter idle");
          PS4.setLed(Yellow);
          active = 0;
        }
        else {
          PS4.setLed(Green);
          digitalWrite(LED_BUILTIN,HIGH);
          Serial.println(active);
          if (PS4.getButtonClick(UP)) {
            Serial.println("forward!");
            moveForward();
          } 
          if (PS4.getButtonClick(RIGHT)) {
            Serial.println("right!");
            turnRight();
          } 
          if (PS4.getButtonClick(DOWN)) {
          } 
          if (PS4.getButtonClick(LEFT)) {
            Serial.println("left!");
            turnLeft();    
          }
        }
    }
    else{
      PS4.setLed(Yellow);
      if(PS4.getButtonClick(PS)) {
        PS4.disconnect();
      }
      else if (PS4.getButtonClick(CIRCLE)) {
        PS4.setLed(Green);
        active = 1;
      }
    }
  }
}
void loop() {
  // put your main codere, to run repeatedly
  Usb.Task();
  if(PS4.connected()) {
    digitalWrite(LED_BUILTIN, HIGH);
    safechoice();
  }
  else {
    digitalWrite(LED_BUILTIN, LOW);
  }
  //safechoice();
   
}
