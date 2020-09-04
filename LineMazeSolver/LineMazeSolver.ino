#include <AFMotor.h>
#include <Ultrasonic.h>
#include <Wire.h>
#include <LiquidCrystal_I2C.h>
//simple maxze solver
//version 1.0.0
//hi

#define lineSensor0 A0//most right,active low(0:white,1:black/nothing)
#define lineSensor1 A1
#define lineSensor2 A2//middle
#define lineSensor3 A3
#define lineSensor4 2//most left
//#define colorSensor A5
#define allDarkCountStop 55
#define pauseMoveDelay 480
#define longRotate 600
#define shortRotate 400
#define StepSpeed 20

//// Full battery 7.0V
//// Empty battery 6.3V

#define RightSpeed 140//135
#define LeftSpeed 120//115
#define drivePastDelay 45//40-65

#define drivePastDelayRatio 1.2
#define RightMediumSpeed RightSpeed-StepSpeed
#define LeftMediumSpeed LeftSpeed-StepSpeed
#define RightRotateSpeed RightSpeed+18
#define LeftRotateSpeed LeftSpeed+20

AF_DCMotor RightMotor(1);//MOTOR12_64KHZ,MOTOR12_8KHZ,MOTOR12_1KHZ(default)
AF_DCMotor LeftMotor(2);

Ultrasonic ultrasonic(10, 9); // (Trig PIN,Echo PIN,Timeouts)
LiquidCrystal_I2C lcd(0x27, 16, 2);

int lineSensor0_val;
int lineSensor1_val;
int lineSensor2_val;
int lineSensor3_val;
int lineSensor4_val;
int lineSensor_val;
int lineSensor_val_previous;
int lineSensor_val_beforeWhite;
int lineSensor_val_beforeMiddle;
//int colorSensor_val;
//int colorSensor_val_previous;
//int colorSensor_val_filt;

String path;
String path_reverse;
int path_index = 0;
bool isOptimum = false;

bool breakFlag = false;
int blackLineCnt = 0;
int checkResult;

int disp_state = 0;

//prototype declaration
void pauseMove(void);
void moveForward(void);
void moveBackward(void);
void rotateRight(void);
void rotateLeft(void);
void slightRight(void);
void slightLeft(void);
int straightCheck(void);
void waitHandShake(void) ;

void setup() {
  Serial.begin(9600);// set up Serial library at 9600 bps
  Serial.println("Initial");
  path.reserve(100);
  path_reverse.reserve(100);

  pinMode(lineSensor0, INPUT);
  pinMode(lineSensor1, INPUT);
  pinMode(lineSensor2, INPUT);
  pinMode(lineSensor3, INPUT);
  pinMode(lineSensor4, INPUT);

  lcd.init();
  lcd.backlight();
  //lcd.noBacklight();

  // turn on motor
  RightMotor.setSpeed(RightSpeed);
  LeftMotor.setSpeed(LeftSpeed);


  pauseMove();
  //== For Test Simplify Path ==
  //path="SLBLBLBLLXBLB";
  //path = "LBLBLLBLL";//1 ->RSL
  //path = "LBLLBLL";//2 ->SSL
  //path = "LLBLL";//3 ->LSL
  //path = "LL";//4 ->LL
  //path = "SSLBLBLBLLBLL";//5 ->R
  //SimplifyPath();


  Serial.println(" wait");
  while (ultrasonic.read() > 12) {//wait hand in
    lineSensor_val_previous = lineSensor_val;
    lineSensor_val = sensorReading();

    if (lineSensor_val_previous != lineSensor_val) {
      lcd.clear();
      lcd.setCursor(0, 0);
      lcd.print(lineSensor_val, DEC);

      lcd.setCursor(3, 0);
      lcd.print(lineSensor_val % 2);
      lcd.print((lineSensor_val >> 1) % 2);
      lcd.print((lineSensor_val >> 2) % 2);
      lcd.print((lineSensor_val >> 3) % 2);
      lcd.print((lineSensor_val >> 4) % 2);

      if (lineSensor_val == 16) {
        lcd.print(" test FWD");
      } else  if (lineSensor_val == 1) {
        lcd.print(" test ROT");
      } else {
        lcd.print("         ");
      }
      lcd.setCursor(0, 1);
      lcd.print("R ");
      lcd.print(RightSpeed);
      lcd.print("  L ");
      lcd.print(LeftSpeed);
    }
  }
  Serial.println(F(" ready"));
  while (ultrasonic.read() < 10) {//wait hand out
    delay(1);
  }
  if (lineSensor_val == 1) {
    RightMotor.setSpeed(RightSpeed);
    LeftMotor.setSpeed(LeftSpeed);
    moveForward();
    while (true) {}
  } else if (lineSensor_val == 3) {
    getPassSectionTime();
  } else if (lineSensor_val == 16) {
    RightMotor.setSpeed(RightSpeed);
    LeftMotor.setSpeed(LeftSpeed);
    rotateRight();
    while (true) {}
  }
  //lcd.backlight();
  lcd.noBacklight();
  lcd.clear();
  lcd.setCursor(0, 0);
  Serial.println(F(" go"));
  delay(1000);
}
void getPassSectionTime() {
  moveForward();
  unsigned long t0, t1;

  while (true) {
    lineSensor_val_previous = lineSensor_val;
    lineSensor_val = sensorReading();
    if ((lineSensor_val != 0) && (lineSensor_val != 3)) {
      t0 = millis();
      break;
    }
  }
  while (true) {
    lineSensor_val_previous = lineSensor_val;
    lineSensor_val = sensorReading();
    if (lineSensor_val == 0) {
      t1 = millis();
      break;
    }
  }
  Serial.println(t1 - t0);
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("t= ");
  lcd.print(t1 - t0, DEC);
  while (true) {
    pauseMove();
  }
}
void loop() {
  MazeSolving();//0
  //path = "LBLBLLBLL";//1 ->RSL
  //path = "LBLLBLL";//2 ->SSL
  //path = "LLBLL";//3 ->LSL
  //path = "LL";//4 ->LL
  //path = "SSLBLBLBLLBLL";//5 ->R
  SimplifyPath();// Simplify the learned path.
  RunningBack();
  //FastestRouteRunning();
  waitHandShake();
  path = "";
}
void waitHandShake(void) {
  lcd.backlight();
  while (ultrasonic.read() > 12) {//wait hand in
    delay(1);
  }
  while (ultrasonic.read() < 10) {//wait hand out
    delay(1);
  }
  lcd.noBacklight();
}
void MazeSolving()
{
  Serial.println(F("MazeSolving"));
  while (true) {
    movement_task();
    //Sensor_task();
    //communication_task();
    if (breakFlag) {
      breakFlag = false;
      break;
    }
  }
}
void RunningBack()
{
  Serial.println(F("RunningBack"));
  while (true) {
    runBack_task();
    //Sensor_task();
    //communication_task();
    if (breakFlag) {
      breakFlag = false;
      break;
    }
  }
}
//  Wall Following -- Left Hand Rule
//    Always turn left if you can
//  If you cannot turn left, go straight
//  If you cannot turn left, or go straight, turn right
//  If you cannot turn left, go straight, or turn right, turn around because you must be at a dead end

// The path variable will store the path that the robot has taken.  It
// is stored as an array of characters, each of which represents the
// turn that should be made at one intersection in the sequence:
//  'L' for left
//  'R' for right
//  'S' for straight (going straight through an intersection)
//  'B' for back (U-turn)
// You should check to make sure that the path_length of your
// maze design does not exceed the bounds of the array.

//    Reduction
//    LBR = B
//    LBS = R
//    LBL = S
//    SBL = R
//    SBS = B
//    RBL = B

void SimplifyPath()
{
  int Xindex = path.indexOf('X');
  path = path.substring(0, Xindex);
  Serial.println(F("SimplifyPath"));
  Serial.print(F(" Search path: "));//LBLBLLBSLLSRBLLSSLBRLBLL
  Serial.println(path);
  do {
    path.replace("LBR", "B");
    path.replace("LBS", "R");
    path.replace("RBL", "B");
    path.replace("SBL", "R");
    path.replace("SBS", "B");
    path.replace("LBL", "S");
    isOptimum = !((path.indexOf("LBR") >= 0) || (path.indexOf("LBS") >= 0) || (path.indexOf("RBL") >= 0) || (path.indexOf("SBL") >= 0) || (path.indexOf("SBS") >= 0) || (path.indexOf("LBL") >= 0));
    Serial.print(" ");
    Serial.println(path);
  } while (!isOptimum);
  Serial.print(F(" Optimal path: "));//RRLLRR
  Serial.println(path);

  path_reverse = path;
  path_reverse.replace("B", "b");
  path_reverse.replace("L", "r");
  path_reverse.replace("R", "l");
  path_reverse.replace("S", "s");
  Serial.print(F(" Optimal path_reverse: "));//llrrll
  Serial.println(path_reverse);
  path_index = path_reverse.length() - 1;
  Serial.println(path_index);

  //Serial.println(path_reverse.charAt(path_index-1));
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print(path);
  lcd.setCursor(0, 1);
  lcd.print(path_reverse);
  lcd.setCursor(9, 1);
  lcd.print(" :");
  lcd.print(path_index);
} // end SimplifyPath

void FastestRouteRunning()
{
  Serial.println(F("FastestRouteRunning"));
  while (true) {
    //movement_task();
    //Sensor_task();
    //communication_task();
    if (breakFlag) {
      breakFlag = false;
      break;
    }
  }
}

void movement_task() {
  lineSensor_val_previous = lineSensor_val;
  lineSensor_val = sensorReading();
  decisionMaking(lineSensor_val);
  carAction(lineSensor_val);
}

void Sensor_task() {

}
void runBack_task() {
  lineSensor_val_previous = lineSensor_val;
  lineSensor_val = sensorReading();
  decisionMaking(lineSensor_val);
  runBack();
}
int sensorReading() {
  //(0:white,1:black/nothing)
  lineSensor0_val = !digitalRead(lineSensor0);//right
  lineSensor1_val = !digitalRead(lineSensor1);
  lineSensor2_val = !digitalRead(lineSensor2);//middle
  lineSensor3_val = !digitalRead(lineSensor3);
  lineSensor4_val = !digitalRead(lineSensor4);//left

  //  colorSensor_val = analogRead(colorSensor);
  //  colorSensor_val_filt = (0.2 * colorSensor_val) + (0.8 * colorSensor_val_previous);
  //  if (colorSensor_val_filt > 500) {//(1024->0:white,0->1:black/nothing)
  //    lineSensor5_val = 0;
  //  } else {
  //    lineSensor5_val = 1;
  //  }
  //  colorSensor_val_previous = colorSensor_val_filt;

  int val =  (lineSensor4_val * 16) + (lineSensor3_val * 8) + (lineSensor2_val * 4) + (lineSensor1_val * 2) + lineSensor0_val;

  return val;
}

//     16 8421
// 0b0000 0000 -  0 all white
// 0b0000 0001 -  1 most right
// 0b0000 0011 -  3
// 0b0000 0010 -  2
// 0b0000 0110 -  6
// 0b0000 0100 -  4 middle
// 0b0000 1100 - 12
// 0b0000 1000 -  8
// 0b0001 1000 - 24
// 0b0001 0000 - 16 most left


// 7,28,31 found intersection

// 0b0001 1100 - 28 turn left detect
// 0b0001 1110 - 30

// 0b0000 0111 -  7 turn right detect
// 0b0000 1111 - 15

// 0b0001 1111 - 31 (all black) T,cross,stop detect

//unknown
// 0b0000 0101 - 5
// 0b0000 1001 - 9
// 0b0000 1010 - 10
// 0b0000 1011 - 11
// 0b0000 1101 - 13
// 0b0000 1110 - 14

// 0b0001 0001 - 17
// 0b0001 0010 - 18
// 0b0001 0011 - 19
// 0b0001 0100 - 20
// 0b0001 0101 - 21
// 0b0001 0110 - 22
// 0b0001 0111 - 23
// 0b0001 1001 - 25
// 0b0001 1010 - 26
// 0b0001 1011 - 27
// 0b0001 1101 - 29
// 0b0001 1110 - 30

void decisionMaking(int sensorValue) {
  if (lineSensor_val_previous != sensorValue) {
    disp_state = 1;
  }
  switch (disp_state) {
    case 1:
      Serial.print(F(" "));
      Serial.println(sensorValue);
      disp_state++;
      break;
    case 2:
      Serial.print(F(" "));
      Serial.println(path);
      disp_state++;
      break;
    case 3:
      lcd.clear();
      disp_state++;
      break;
    case 4:
      lcd.setCursor(0, 0);
      lcd.print(path);
      disp_state++;
      break;
    case 5:
      lcd.setCursor(9, 0);
      lcd.print(" ");
      lcd.print(sensorValue);
      disp_state++;
      break;
    case 6:
      lcd.setCursor(0, 1);
      lcd.print(path_reverse);
      disp_state++;
      break;
    case 7:
      lcd.setCursor(9, 1);
      lcd.print(" ");
      lcd.print(path_index);
      disp_state++;
      break;
    default:
      disp_state = 0;
      break;
  }
  if (sensorValue != 0) {
    lineSensor_val_beforeWhite = sensorValue;
  }
  if (sensorValue != 4) {
    lineSensor_val_beforeMiddle = sensorValue;
  }
  switch (sensorValue) {//1,3,2,6,4,12,8,24,16 line following
    case 0: //line detected by none = all white
      break;
    case 1://line detected by right sensor
      break;
    case 3:
      break;
    case 2:
      break;
    case 6:
      break;
    case 4://line detected by middle sensor
      break;
    case 12:
      break;
    case 8:
      break;
    case 24:
      break;
    case 16: //line detected by left sensor
      break;
    case 30: //turn left
    case 28: //turn left
      break;
    case 15: //turn right
    case 7: //turn right
      break;
    case 31://line detected all black
      if (lineSensor_val_previous != sensorValue) {
        blackLineCnt++;
      }
      break;
    default:
      Serial.print(F(" unknown "));
      Serial.println(sensorValue);
      break;
  }

  //return something?????
}


void carAction(int sensorValue) {
  switch (sensorValue) {
    case 0: //line detected by none = all white
      if ((lineSensor_val_beforeWhite == 2) || (lineSensor_val_beforeWhite == 6) || (lineSensor_val_beforeWhite == 4) || (lineSensor_val_beforeWhite == 12) || (lineSensor_val_beforeWhite == 8)) {
        checkResult = straightCheck();
        if (checkResult == 0) {//can't go straight
          //U-turn
          Serial.println(F(" U-turn"));

          moveForward();
          RightMotor.setSpeed(RightSpeed);
          LeftMotor.setSpeed(LeftSpeed);
          delay(4 * drivePastDelay);

          turnAround();
        }
      } else if (lineSensor_val_beforeWhite == 1) {
        //left out
        Serial.println(F(" left out"));
      } else if (lineSensor_val_beforeWhite == 16) {
        //right out
        Serial.println(F(" right out"));
      }
      break;
    case 1://line detected by right sensor
    case 3:
    case 2:
      rotateRight();
      RightMotor.setSpeed(RightSpeed);
      LeftMotor.setSpeed(LeftSpeed);
      break;
    case 6:
      slightRight();
      RightMotor.setSpeed(RightSpeed);
      LeftMotor.setSpeed(LeftSpeed);
      break;
    case 4:
      if ((lineSensor_val_beforeMiddle == 6) || ( lineSensor_val_beforeMiddle == 2)) {
        moveForward();
        RightMotor.setSpeed(RightSpeed);
        LeftMotor.setSpeed(LeftMediumSpeed);
      } else if ((lineSensor_val_beforeMiddle == 12) || ( lineSensor_val_beforeMiddle == 8)) {
        moveForward();
        RightMotor.setSpeed(RightMediumSpeed);
        LeftMotor.setSpeed(LeftSpeed);
      } else {
        moveForward();
        RightMotor.setSpeed(RightSpeed);
        LeftMotor.setSpeed(LeftSpeed);
      }
      break;
    case 12:
      slightLeft();
      RightMotor.setSpeed(RightSpeed);
      LeftMotor.setSpeed(LeftSpeed);
      break;
    case 8:
    case 24:
    case 16: //line detected by left sensor
      rotateLeft();
      RightMotor.setSpeed(RightSpeed);
      LeftMotor.setSpeed(LeftSpeed);
      break;
    case 30:
    case 28: //turn left
      moveForward();
      RightMotor.setSpeed(RightSpeed);
      LeftMotor.setSpeed(LeftSpeed);
      delay(50);//repeat read sensor <130 ms
      lineSensor_val_previous = lineSensor_val;
      lineSensor_val = sensorReading();
      if (lineSensor_val_previous != lineSensor_val) {
        delay(5);
        lineSensor_val_previous = lineSensor_val;
        lineSensor_val = sensorReading();
      }
      Serial.print(F(" ^"));
      Serial.println(lineSensor_val);
      if ((lineSensor_val == 28) || (lineSensor_val == 30)) {
        checkResult = straightCheck();
        if (checkResult > 0) {//can go straight
          path += 'L';
          turnLeft();
        } else {
          turnLeft();
        }
      }
      break;
    case 15:
    case 7: //turn right
      moveForward();
      RightMotor.setSpeed(RightSpeed);
      LeftMotor.setSpeed(LeftSpeed);
      delay(50);//repeat read sensor <130 ms
      lineSensor_val_previous = lineSensor_val;
      lineSensor_val = sensorReading();
      if (lineSensor_val_previous != lineSensor_val) {
        delay(5);
        lineSensor_val_previous = lineSensor_val;
        lineSensor_val = sensorReading();
      }
      Serial.print(F(" ^"));
      Serial.println(lineSensor_val);
      if ((lineSensor_val == 7) || (lineSensor_val == 15)) {
        checkResult = straightCheck();
        if (checkResult > 0) {//can go straight
          path += 'S';
          moveForward();
        } else {
          turnRight();
        }
      }
      break;
    case 31://line detected all black
      Serial.println(F(" =31="));
      //pauseMove();
      checkResult = straightCheck();
      if (checkResult > 0) {//can go straight ,4 junction
        turnLeft();
        path += 'L';
      } else if (checkResult == 0) {//can't go straight ,3 junction
        turnLeft();
        path += 'L';
      } else {
        path += 'X';
        pauseMove();
        moveBackward();
        delay(50);
        pauseMove();
        turnAround();
        pauseMove();
        breakFlag = true;
        lcd.backlight();
        lcd.clear();
        lcd.setCursor(0, 0);
        lcd.print(path);
        lcd.setCursor(0, 1);
        lcd.print(F("I'm thinking!!"));
        delay(10000);
        lcd.noBacklight();
      }
      break;
    case 9:// 01001
    case 13://01101
    case 18://10010
    case 22://10110
    case 29://11101
    case 23://10111
    case 19://10011
    case 25://11001
    case 20://10100
    case 5://00101
      moveForward();
      RightMotor.setSpeed(RightSpeed / 2);
      LeftMotor.setSpeed(LeftSpeed / 2);
      break;
    case 17:
      rotateRight();
      RightMotor.setSpeed(RightSpeed);
      LeftMotor.setSpeed(LeftSpeed);
      break;
    default:
      pauseMove();
      break;
  }
}
void runBack(void) {
  switch (lineSensor_val) {
    case 0: //line detected by none = all white
      if ((lineSensor_val_beforeWhite == 2) || (lineSensor_val_beforeWhite == 6) || (lineSensor_val_beforeWhite == 4) || (lineSensor_val_beforeWhite == 12) || (lineSensor_val_beforeWhite == 8)) {
        checkResult = straightCheck();
        if (checkResult == 0) {//can't go straight
          // Finish
          Serial.println(F(" Finish"));

          moveForward();
          RightMotor.setSpeed(RightSpeed);
          LeftMotor.setSpeed(LeftSpeed);
          delay(4 * drivePastDelay);

          turnAround();

        }
      } else if (lineSensor_val_beforeWhite == 1) {
        //left out
        Serial.println(F(" left out"));
      } else if (lineSensor_val_beforeWhite == 16) {
        //right out
        Serial.println(F(" right out"));
      }
      pauseMove();
      path_index--;
      breakFlag = true;
      break;
    case 1://line detected by right sensor
    case 3:
    case 2:
      rotateRight();
      RightMotor.setSpeed(RightSpeed);
      LeftMotor.setSpeed(LeftSpeed);
      break;
    case 6:
      slightRight();
      RightMotor.setSpeed(RightSpeed);
      LeftMotor.setSpeed(LeftSpeed);
      break;
    case 4:
      if ((lineSensor_val_beforeMiddle == 6) || ( lineSensor_val_beforeMiddle == 2)) {
        moveForward();
        RightMotor.setSpeed(RightSpeed);
        LeftMotor.setSpeed(LeftMediumSpeed);
      } else if ((lineSensor_val_beforeMiddle == 12) || ( lineSensor_val_beforeMiddle == 8)) {
        moveForward();
        RightMotor.setSpeed(RightMediumSpeed);
        LeftMotor.setSpeed(LeftSpeed);
      } else {
        moveForward();
        RightMotor.setSpeed(RightSpeed);
        LeftMotor.setSpeed(LeftSpeed);
      }
      break;
    case 12:
      slightLeft();
      RightMotor.setSpeed(RightSpeed);
      LeftMotor.setSpeed(LeftSpeed);
      break;
    case 8:
    case 24:
    case 16: //line detected by left sensor
      rotateLeft();
      RightMotor.setSpeed(RightSpeed);
      LeftMotor.setSpeed(LeftSpeed);
      break;
    case 30:
    case 28: //turn left
      moveForward();
      RightMotor.setSpeed(RightSpeed);
      LeftMotor.setSpeed(LeftSpeed);
      delay(50);//repeat read sensor <130 ms
      lineSensor_val_previous = lineSensor_val;
      lineSensor_val = sensorReading();
      if (lineSensor_val_previous != lineSensor_val) {
        delay(5);
        lineSensor_val_previous = lineSensor_val;
      }
      lineSensor_val = sensorReading();
      Serial.print(F(" ^"));
      Serial.println(lineSensor_val);

      if ((lineSensor_val == 28) || (lineSensor_val == 30)) {
        Serial.print(F(" 28/30L"));
        checkResult = straightCheck();
        if (checkResult > 0) {//can go straight
          Serial.println(path_reverse.charAt(path_index));
          lcd.setCursor(10, 0);
          lcd.print(path_reverse.charAt(path_index));
          lcd.print(">");
          lcd.print(path_reverse.charAt(path_index - 1));
          if (path_reverse.charAt(path_index) == 's') {
            moveForward();
          } else if (path_reverse.charAt(path_index) == 'r') {
            turnRight();
          } else if (path_reverse.charAt(path_index) == 'l') {
            turnLeft();
          }
          path_index--;
        } else {
          //lcd.setCursor(10, 0);
          //lcd.print(" ");
          turnLeft();
        }
      }
      break;
    case 15:
    case 7: //turn right
      moveForward();
      RightMotor.setSpeed(RightSpeed);
      LeftMotor.setSpeed(LeftSpeed);
      delay(50);//repeat read sensor <130 ms
      lineSensor_val_previous = lineSensor_val;
      lineSensor_val = sensorReading();
      if (lineSensor_val_previous != lineSensor_val) {
        delay(5);
        lineSensor_val_previous = lineSensor_val;
        lineSensor_val = sensorReading();
      }
      Serial.print(F(" ^"));
      Serial.println(lineSensor_val);
      if ((lineSensor_val == 7) || (lineSensor_val == 15)) {
        Serial.print(F(" 7/15R"));
        checkResult = straightCheck();
        if (checkResult > 0) {//can go straight
          Serial.println(path_reverse.charAt(path_index));
          lcd.setCursor(10, 0);
          lcd.print(path_reverse.charAt(path_index));
          lcd.print(">");
          lcd.print(path_reverse.charAt(path_index - 1));
          if (path_reverse.charAt(path_index) == 's') {
            moveForward();
          } else if (path_reverse.charAt(path_index) == 'r') {
            turnRight();
          } else if (path_reverse.charAt(path_index) == 'l') {
            turnLeft();
          }
          path_index--;
        } else {
          //lcd.setCursor(10, 0);
          //lcd.print(" ");
          turnRight();
        }
      }
      break;
    case 31://line detected all black
      //pauseMove();
      Serial.print(F(" 31"));
      checkResult = straightCheck();
      Serial.println(path_reverse.charAt(path_index));
      lcd.setCursor(10, 0);
      lcd.print(path_reverse.charAt(path_index));
      lcd.print(">");
      lcd.print(path_reverse.charAt(path_index - 1));
      if (checkResult > 0) {//can go straight ,4 junction
        if (path_reverse.charAt(path_index) == 's') {
          moveForward();
        } else if (path_reverse.charAt(path_index) == 'r') {
          turnRight();
        } else if (path_reverse.charAt(path_index) == 'l') {
          turnLeft();
        }
        path_index--;
      } else if (checkResult == 0) {//can't go straight ,3 junction
        if (path_reverse.charAt(path_index) == 's') {
          moveForward();
        } else if (path_reverse.charAt(path_index) == 'r') {
          turnRight();
        } else if (path_reverse.charAt(path_index) == 'l') {
          turnLeft();
        }
        path_index--;
      } else {
        //nothing may be U-turn
      }
      break;
    case 9:// 01001
    case 13://01101
    case 18://10010
    case 22://10110
    case 29://11101
    case 23://10111
    case 19://10011
    case 25://11001
    case 20://10100
    case 5://00101
      moveForward();
      RightMotor.setSpeed(RightSpeed / 2);
      LeftMotor.setSpeed(LeftSpeed / 2);
      break;
    case 17:
      rotateRight();
      RightMotor.setSpeed(RightSpeed);
      LeftMotor.setSpeed(LeftSpeed);
      break;
    default:
      pauseMove();
      break;
  }
}
void pauseMove() {
  RightMotor.run(RELEASE);
  LeftMotor.run(RELEASE);
}
void moveForward() {
  RightMotor.run(FORWARD);
  LeftMotor.run(FORWARD);
}
void moveBackward() {
  RightMotor.run(BACKWARD);
  LeftMotor.run(BACKWARD);
}
void rotateRight() {
  RightMotor.run(BACKWARD);
  LeftMotor.run(FORWARD);
}
void rotateLeft() {
  RightMotor.run(FORWARD);
  LeftMotor.run(BACKWARD);
}
void slightRight() {
  RightMotor.run(RELEASE);
  LeftMotor.run(FORWARD);
}
void slightLeft() {
  RightMotor.run(FORWARD);
  LeftMotor.run(RELEASE);
}
int straightCheck() {
  unsigned long passT0, passT1;
  int allDarkCnt = 0;
  Serial.print(F(" straightCheck "));
  moveForward();
  RightMotor.setSpeed(RightSpeed);
  LeftMotor.setSpeed(LeftSpeed);

  passT0 = millis();
  while (true) {
    lineSensor_val_previous = lineSensor_val;
    lineSensor_val = sensorReading();
    if ((lineSensor_val == 3) || (lineSensor_val == 2) || (lineSensor_val == 6) || (lineSensor_val == 4) || (lineSensor_val == 12) || (lineSensor_val == 8) || (lineSensor_val == 24) || (lineSensor_val == 0)) {
      RightMotor.setSpeed(RightSpeed);
      LeftMotor.setSpeed(LeftSpeed);
      break;
    } else if (lineSensor_val == 28) {//000 11100
      RightMotor.setSpeed(RightSpeed);
      LeftMotor.setSpeed(LeftSpeed);
    } else if (lineSensor_val == 30) {//000 11110
      RightMotor.setSpeed(RightSpeed);
      LeftMotor.setSpeed(LeftSpeed - 20);
    } else if (lineSensor_val == 7) { //000 00111
      RightMotor.setSpeed(RightSpeed);
      LeftMotor.setSpeed(LeftSpeed);
    } else if (lineSensor_val == 15) {//000 01111
      RightMotor.setSpeed(RightSpeed - 20);
      LeftMotor.setSpeed(LeftSpeed);
    } else if (lineSensor_val == 31) {//000 11111
      RightMotor.setSpeed(RightSpeed);
      LeftMotor.setSpeed(LeftSpeed);
      if (allDarkCnt++ > allDarkCountStop) {
        //finish maze
        Serial.println(F(" finish maze"));
        return -1;
      }
      Serial.println(allDarkCnt);
    } else {
      RightMotor.setSpeed(RightSpeed);
      LeftMotor.setSpeed(LeftSpeed);
    }
  }
  passT1 = millis();

  delay(drivePastDelayRatio * (passT1 - passT0)); //compensate drivePastDelay
  delay(drivePastDelay);
  pauseMove();
  delay(pauseMoveDelay);
  lineSensor_val_previous = lineSensor_val;
  lineSensor_val = sensorReading();
  Serial.println(lineSensor_val);
  if ((lineSensor_val == 3) || (lineSensor_val == 2) || (lineSensor_val == 6) || (lineSensor_val == 4) || (lineSensor_val == 12) || (lineSensor_val == 8) || (lineSensor_val == 24) ) { //line detected by middle sensor
    //can go stress
    Serial.println(F(" can go stress"));
    return 1;
  } else if ((lineSensor_val == 31) || (lineSensor_val == 15) || (lineSensor_val == 30)) { //line detected all black
    //finish maze
    Serial.println(F(" finish maze"));
    return -1;
  }

  return 0;
}

void turnRight() {
  pauseMove();
  delay(pauseMoveDelay);
  RightMotor.setSpeed(RightRotateSpeed);
  LeftMotor.setSpeed(LeftRotateSpeed);
  rotateRight();
  if ((lineSensor_val == 1) || (lineSensor_val == 3) || (lineSensor_val == 2)) {
    delay(longRotate);
  } else {
    delay(shortRotate);
  }
  pauseMove();
  delay(pauseMoveDelay);
  while (true) { //line detected by middle sensor
    rotateRight();
    lineSensor_val_previous = lineSensor_val;
    lineSensor_val = sensorReading();
    if ((lineSensor_val == 6) || (lineSensor_val == 4) || (lineSensor_val == 12)) {
      break;
    }
  }
  RightMotor.setSpeed(RightSpeed);
  LeftMotor.setSpeed(LeftSpeed);
  while (true) {
    lineSensor_val_previous = lineSensor_val;
    lineSensor_val = sensorReading();
    if (lineSensor_val == 4) {
      break;
    } else if (lineSensor_val == 6) {
      rotateRight();
    } else if (lineSensor_val == 12) {
      rotateLeft();
    }
  }

}
void turnLeft() {
  pauseMove();
  delay(pauseMoveDelay);
  rotateLeft();
  RightMotor.setSpeed(RightRotateSpeed);
  LeftMotor.setSpeed(LeftRotateSpeed);
  if ((lineSensor_val == 16) || (lineSensor_val == 24) || (lineSensor_val == 8)) {
    delay(longRotate);
  } else {
    delay(shortRotate);
  }
  pauseMove();
  delay(pauseMoveDelay);
  while (true) { //line detected by middle sensor
    rotateLeft();
    lineSensor_val_previous = lineSensor_val;
    lineSensor_val = sensorReading();
    if ((lineSensor_val == 6) || (lineSensor_val == 4) || (lineSensor_val == 12)) {
      break;
    }
  }
  RightMotor.setSpeed(RightSpeed);
  LeftMotor.setSpeed(LeftSpeed);
  while (true) {
    lineSensor_val_previous = lineSensor_val;
    lineSensor_val = sensorReading();
    if (lineSensor_val == 4) {
      break;
    } else if (lineSensor_val == 6) {
      rotateRight();
    } else if (lineSensor_val == 12) {
      rotateLeft();
    }
  }
}
void turnAround()
{
  turnLeft();
  path += 'B';
}
