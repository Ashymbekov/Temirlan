#include <Fuzzy.h>
#include <FuzzyComposition.h>
#include <FuzzyInput.h>
#include <FuzzyIO.h>
#include <FuzzyOutput.h>
#include <FuzzyRule.h>
#include <FuzzyRuleAntecedent.h>
#include <FuzzyRuleConsequent.h>
#include <FuzzySet.h>
#include <Ultrasonic.h>
#include "timer.h"
#include <Ultrasonic.h>
Ultrasonic
  ultrasonic_rear(40, 41),
  ultrasonic_left_rear(38, 39),
  ultrasonic_left_front(36, 37),
  ultrasonic_front(34, 35);

#include <AFMotor.h>
AF_DCMotor Left_Front_Engine(4);
AF_DCMotor Right_Front_Engine(3);
AF_DCMotor Left_Rear_Engine(1);
AF_DCMotor Right_Rear_Engine(2);


// definitions of ultrasonic sensors

#define left 0             //left direction command
#define Right 1            //right direction command
#define forward 2          //forward command
#define rearward 3         //reverse command
#define minimum_limit 15   //width of the car (cm)
#define minimum_limit1 30  //the length of the car (cm)

Fuzzy *fuzzy = new Fuzzy();
Timer timer;

byte parking_status = 0;
int signalpin = 21;
volatile int val;
const int LM393 = 21;
int output = 0;
int t = 0;
int pulser = 0;
int counter = 0;
int current_status = 0;
int previous_status = 0;

int speed(int percent) {
  return map(percent, 0, 100, 0, 255);
}

void count1(int count) {
  for (int i = 0; i <= count; i + 1) {
    val = digitalRead(signalpin);
    if (val == LOW) {

      current_status = 0;
    } else {

      current_status = 1;
    }

    if (current_status != previous_status) {
      if (current_status == 1) {
        counter = counter + 1;
        Serial.println(counter);
        i = i + 1;
      } else {
        i = i;
      }

      previous_status = current_status;
    }
    if (i == count) {

      Left_Front_Engine.run(RELEASE);
      Right_Front_Engine.run(RELEASE);
      Left_Rear_Engine.run(RELEASE);
      Right_Rear_Engine.run(RELEASE);
    }
  }
}

void motor_pinSetup() {

  Left_Front_Engine.run(RELEASE);
  Right_Front_Engine.run(RELEASE);
  Left_Rear_Engine.run(RELEASE);
  Right_Rear_Engine.run(RELEASE);
}

// Motion functions
void Robot_Movement(byte engine, byte spd) {
  if (engine == forward) {
    Left_Front_Engine.setSpeed(spd);
    Right_Front_Engine.setSpeed(spd);
    Left_Rear_Engine.setSpeed(spd);
    Right_Rear_Engine.setSpeed(spd);
    Left_Front_Engine.run(FORWARD);
    Right_Front_Engine.run(FORWARD);
    Left_Rear_Engine.run(FORWARD);
    Right_Rear_Engine.run(FORWARD);
  }
  if (engine == rearward) {
    Left_Front_Engine.setSpeed(spd);
    Right_Front_Engine.setSpeed(spd);
    Left_Rear_Engine.setSpeed(spd);
    Right_Rear_Engine.setSpeed(spd);
    Left_Front_Engine.run(BACKWARD);
    Right_Front_Engine.run(BACKWARD);
    Left_Rear_Engine.run(BACKWARD);
    Right_Rear_Engine.run(BACKWARD);
  }
  if (engine == left) {
    Left_Front_Engine.setSpeed(spd);
    Right_Front_Engine.setSpeed(spd);
    Left_Rear_Engine.setSpeed(spd);
    Right_Rear_Engine.setSpeed(spd);
    Left_Front_Engine.run(BACKWARD);
    Right_Front_Engine.run(FORWARD);
    Left_Rear_Engine.run(BACKWARD);
    Right_Rear_Engine.run(FORWARD);
  }

  if (engine == Right) {
    Left_Front_Engine.setSpeed(spd);
    Right_Front_Engine.setSpeed(spd);
    Left_Rear_Engine.setSpeed(spd);
    Right_Rear_Engine.setSpeed(spd);
    Left_Front_Engine.run(FORWARD);
    Right_Front_Engine.run(BACKWARD);
    Left_Rear_Engine.run(FORWARD);
    Right_Rear_Engine.run(BACKWARD);
  }
}

void Robot_Stop() {
  Left_Front_Engine.run(RELEASE);
  Right_Front_Engine.run(RELEASE);
  Left_Rear_Engine.run(RELEASE);
  Right_Rear_Engine.run(RELEASE);
}

// Parking spot search
bool Park_Location_Control() {

  long front_Sensor = ultrasonic_front.Ranging(CM);
  long right_Sensor = ultrasonic_left_front.Ranging(CM);
  long right_rear_Sensor = ultrasonic_left_rear.Ranging(CM);

  if ((right_Sensor <= minimum_limit) && (right_rear_Sensor <= minimum_limit) && (parking_status == 0)) {
    Robot_Movement(forward, 100);
    parking_status = 1;
    Serial.println(parking_status);
  }


  if ((right_Sensor > minimum_limit) && (right_Sensor < minimum_limit1) && (right_rear_Sensor > minimum_limit) && (right_rear_Sensor < minimum_limit1) && (parking_status == 1)) {
    Robot_Movement(forward, 100);
    parking_status = 2;
    Serial.println(parking_status);
  }


  if ((right_Sensor >= minimum_limit1) && (right_rear_Sensor >= minimum_limit1) && (parking_status == 1)) {
    /* Upright Parking Decision */
    Robot_Stop();
    delay(500);
    parking_status = 10;
    Serial.println(parking_status);
  }


  if ((right_Sensor <= minimum_limit) && (right_rear_Sensor <= minimum_limit) && (parking_status == 2)) {
    /* Parallel Parking Decision */
    parking_status = 3;
    Serial.println(parking_status);
  }

  return parking_status;
}


void Park_find() {
  Park_Location_Control();
  if (parking_status == 3) {
    Robot_Stop();
    Serial.println(parking_status);
    delay(400);
    parking_status = 4;
  }


  if (parking_status == 4) {

    Robot_Movement(rearward, 120);
    count1(18);
    Robot_Stop();
    Serial.println(parking_status);
    delay(500);
    Robot_Movement(Right, 150);
    count1(9);
    Robot_Stop();
    delay(500);
    parking_status = 5;
  }


  if (parking_status == 5) {

    Robot_Movement(rearward, 120);
    long rear_Sensor = ultrasonic_rear.Ranging(CM);
    Serial.println(rear_Sensor);

    if (rear_Sensor > 0 && rear_Sensor <= 13) {
      Robot_Stop();
      delay(400);
      parking_status = 6;
    }
    return rear_Sensor;
  }

  if (parking_status == 6) {
    Robot_Movement(left, 150);
    long right_Sensor = ultrasonic_left_front.Ranging(CM);
    Serial.println(right_Sensor);
    long right_rear_Sensor = ultrasonic_left_rear.Ranging(CM);
    Serial.println(right_rear_Sensor);

    if (right_Sensor == right_rear_Sensor) {
      Robot_Stop();
      parking_status = 7;
    }

    return right_Sensor, right_rear_Sensor;
  }


  if (parking_status == 7) {
    long front_Sensor = ultrasonic_front.Ranging(CM);

    if (front_Sensor <= 6) {
      Robot_Stop();
      parking_status = 8;
    } else {
      Robot_Movement(forward, 100);
    }
    return front_Sensor;
  }


  if (parking_status == 10) {

    Robot_Movement(left, 180);
    count1(14);
    Robot_Stop();
    delay(500);
    parking_status = 7;
  }
}

void setup() {
  Serial.begin(9600);
  attachInterrupt(5, count1, CHANGE);
  pinMode(signalpin, INPUT);

  motor_pinSetup();

  attachInterrupt(digitalPinToInterrupt(LM393), count, RISING);
  timer.setInterval(1000);
  timer.setCallback(speed1);
  timer.start();
 
  // fuzzy sets
  // distance
  FuzzySet *small = new FuzzySet(0, 0, 0, 2);
  FuzzySet *mid = new FuzzySet(2, 2, 2, 15);
  FuzzySet *big = new FuzzySet(2, 2, 16, 32);
  FuzzySet *verybig = new FuzzySet(16, 35, 520, 520);


  FuzzySet *lm = new FuzzySet(100, 130, 130, 150);



  // speed1
  FuzzySet *off = new FuzzySet(0, 20, 20, 40);
  FuzzySet *lowb = new FuzzySet(30, 60, 60, 90);
  FuzzySet *midb = new FuzzySet(50, 80, 80, 110);
  FuzzySet *highb = new FuzzySet(100, 120, 120, 140);


  // variables
  // variable distance with universe 0-60 as input
  FuzzyInput *distance = new FuzzyInput(1);
  distance->addFuzzySet(small);
  distance->addFuzzySet(mid);
  distance->addFuzzySet(big);
  distance->addFuzzySet(verybig);
  fuzzy->addFuzzyInput(distance);

  // variable speed with universe 0-255 as output
  FuzzyOutput *speed1 = new FuzzyOutput(1);
  speed1->addFuzzySet(off);
  speed1->addFuzzySet(lowb);
  speed1->addFuzzySet(midb);
  speed1->addFuzzySet(highb);
  fuzzy->addFuzzyOutput(speed1);

  FuzzyInput *lm393 = new FuzzyInput(2);
  lm393->addFuzzySet(lm);
  fuzzy->addFuzzyInput(lm393);

  // rules

  // if distance is small then speed is off
  FuzzyRuleAntecedent *ifDistanceSmall = new FuzzyRuleAntecedent();
  ifDistanceSmall->joinSingle(small);
  FuzzyRuleConsequent *thenSpeed1Off = new FuzzyRuleConsequent();
  thenSpeed1Off->addOutput(off);
  FuzzyRule *fuzzyRule1 = new FuzzyRule(1, ifDistanceSmall, thenSpeed1Off);
  fuzzy->addFuzzyRule(fuzzyRule1);

  // if distance is mid then speed is low
  FuzzyRuleAntecedent *ifDistanceMid = new FuzzyRuleAntecedent();
  ifDistanceMid->joinSingle(mid);
  FuzzyRuleConsequent *thenSpeed1Low = new FuzzyRuleConsequent();
  thenSpeed1Low->addOutput(lowb);
  FuzzyRule *fuzzyRule2 = new FuzzyRule(2, ifDistanceMid, thenSpeed1Low);
  fuzzy->addFuzzyRule(fuzzyRule2);

  // if distance is big then speed is mid
  FuzzyRuleAntecedent *ifDistanceBig = new FuzzyRuleAntecedent();
  ifDistanceBig->joinSingle(big);
  FuzzyRuleConsequent *thenSpeed1Midb = new FuzzyRuleConsequent();
  thenSpeed1Midb->addOutput(midb);
  FuzzyRule *fuzzyRule3 = new FuzzyRule(3, ifDistanceBig, thenSpeed1Midb);
  fuzzy->addFuzzyRule(fuzzyRule3);


  FuzzyRuleAntecedent *ifDistanceVeryBig = new FuzzyRuleAntecedent();
  ifDistanceVeryBig->joinSingle(verybig);
  FuzzyRuleConsequent *thenSpeed1High = new FuzzyRuleConsequent();
  thenSpeed1High->addOutput(highb);
  FuzzyRule *fuzzyRule4 = new FuzzyRule(4, ifDistanceVeryBig, thenSpeed1High);
  fuzzy->addFuzzyRule(fuzzyRule4);
}

void count() {
  counter++;
}

int distance() {
  long pulse =  ultrasonic_front.Ranging(CM);
  return pulse;
}


int speed1() {
  int t = (60 * counter / 200);
  counter = 0;
  return t;
}

void loop() {
  Park_find();

  int dist = distance();
  int speed = speed1();
  timer.update();
  Serial.print("distance: ");
  Serial.print(dist);
  Serial.print(" speed: ");
  Serial.print(speed);
  Serial.print(" => output: ");
  Serial.print(output);
  Serial.println();


  // fuzzyfication
  fuzzy->setInput(1, dist);  // dist as fuzzy input 1 (distance)
                             // speed as fuzzy input 2 (lm393)
  fuzzy->fuzzify();

  // defuzzyfication
  output = fuzzy->defuzzify(1);  // defuzzify fuzzy output 1 (speed)




  if (parking_status == 10) {
    while (1) {
      // Infinite loop to stop robot movement
    }
  }
}
