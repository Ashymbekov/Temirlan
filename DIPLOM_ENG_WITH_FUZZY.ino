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
#include <AFMotor.h>
AF_DCMotor Sol_On_Motor(4);
AF_DCMotor Sag_On_Motor(3);
AF_DCMotor Sol_Arka_Motor(1);
AF_DCMotor Sag_Arka_Motor(2);
#include <Ultrasonic.h>

Ultrasonic
  ultrasonic_arka(32, 33),
  ultrasonic_sol_arka(38, 39),
  ultrasonic_sol_on(36, 37),
  ultrasonic_on(2, 3);
// ultrasomik sensörlerin tanımlamamalrı

#define Sol 0              //sol yön komutu
#define Sag 1              //sağ yön komutu
#define Ileri 2            //ileri yön komutu
#define Geri 3             //geri yön komutu
#define minimum_limit 20   //Arabanın genişliği (cm)
#define minimum_limit1 32  //arabanın uzunluğu (cm)
#define LED 6

Fuzzy *fuzzy = new Fuzzy();
Timer timer;

byte park_durumu = 0;
int sinyalpin = 21;
volatile int val;
int output = 0;
int t = 0;
int pulser = 0;
int counter = 0;
const int LM393 = 21;
int sayac = 0;
int guncel_durum = 0;
int onceki_durum = 0;

int speed(int percent) {
  return map(percent, 0, 100, 0, 255);
}


void say(int saydir) {
  for (int i = 0; i <= saydir; i + 1) {
    val = digitalRead(sinyalpin);
    if (val == LOW) {

      guncel_durum = 0;
    } else {

      guncel_durum = 1;
    }

    if (guncel_durum != onceki_durum) {
      if (guncel_durum == 1) {
        sayac = sayac + 1;
        Serial.println(sayac);
        i = i + 1;
      } else {
        i = i;
      }

      onceki_durum = guncel_durum;
    }
    if (i == saydir) {

      Sol_On_Motor.run(RELEASE);
      Sag_On_Motor.run(RELEASE);
      Sol_Arka_Motor.run(RELEASE);
      Sag_Arka_Motor.run(RELEASE);
    }
  }
}

void motor_pinSetup() {

  Sol_On_Motor.run(RELEASE);
  Sag_On_Motor.run(RELEASE);
  Sol_Arka_Motor.run(RELEASE);
  Sag_Arka_Motor.run(RELEASE);
}

// Hareket fonksiyonları
void Robot_Hareket(byte motor, byte spd) {
  if (motor == Ileri) {
    Sol_On_Motor.setSpeed(spd);
    Sag_On_Motor.setSpeed(spd);
    Sol_Arka_Motor.setSpeed(spd);
    Sag_Arka_Motor.setSpeed(spd);
    Sol_On_Motor.run(FORWARD);
    Sag_On_Motor.run(FORWARD);
    Sol_Arka_Motor.run(FORWARD);
    Sag_Arka_Motor.run(FORWARD);
    Serial.println("ileri");
  }
  if (motor == Geri) {
    Sol_On_Motor.setSpeed(spd);
    Sag_On_Motor.setSpeed(spd);
    Sol_Arka_Motor.setSpeed(spd);
    Sag_Arka_Motor.setSpeed(spd);
    Sol_On_Motor.run(BACKWARD);
    Sag_On_Motor.run(BACKWARD);
    Sol_Arka_Motor.run(BACKWARD);
    Sag_Arka_Motor.run(BACKWARD);
    Serial.println("geri");
  }
  if (motor == Sol) {
    Sol_On_Motor.setSpeed(spd);
    Sag_On_Motor.setSpeed(spd);
    Sol_Arka_Motor.setSpeed(spd);
    Sag_Arka_Motor.setSpeed(spd);
    Sol_On_Motor.run(BACKWARD);
    Sag_On_Motor.run(FORWARD);
    Sol_Arka_Motor.run(BACKWARD);
    Sag_Arka_Motor.run(FORWARD);
    Serial.println("sol");
  }

  if (motor == Sag) {
    Sol_On_Motor.setSpeed(spd);
    Sag_On_Motor.setSpeed(spd);
    Sol_Arka_Motor.setSpeed(spd);
    Sag_Arka_Motor.setSpeed(spd);
    Sol_On_Motor.run(FORWARD);
    Sag_On_Motor.run(BACKWARD);
    Sol_Arka_Motor.run(FORWARD);
    Sag_Arka_Motor.run(BACKWARD);
    Serial.println("sag");
  }
}

void Robot_Dur() {
  Sol_On_Motor.run(RELEASE);
  Sag_On_Motor.run(RELEASE);
  Sol_Arka_Motor.run(RELEASE);
  Sag_Arka_Motor.run(RELEASE);
  Serial.println("stop_motor");
}

// Park yeri araması
bool Park_Yeri_Kontrol() {

  long on_Sensor = ultrasonic_on.Ranging(CM);
  Serial.print("on_Sensor=");
  Serial.println(on_Sensor);
  long sag_Sensor = ultrasonic_sol_on.Ranging(CM);
  Serial.print("sag_Sensor=");
  Serial.println(sag_Sensor);
  long sag_arka_Sensor = ultrasonic_sol_arka.Ranging(CM);
  Serial.print("sag_arka_Sensor=");
  Serial.println(sag_arka_Sensor);
  long arka_Sensor = ultrasonic_arka.Ranging(CM);
  Serial.print("arka_Sensor=");
  Serial.println(arka_Sensor);
  if ((sag_Sensor <= minimum_limit) && (sag_arka_Sensor <= minimum_limit) && (park_durumu == 0)) {
    Robot_Hareket(Ileri, output);
    park_durumu = 1;
    Serial.println(park_durumu);
  }

  if ((sag_Sensor > minimum_limit) && (sag_Sensor < minimum_limit1) && (sag_arka_Sensor > minimum_limit) && (sag_arka_Sensor < minimum_limit1) && (park_durumu == 1)) {
    Robot_Hareket(Ileri, output);
    park_durumu = 2;
    Serial.println(park_durumu);
  }

  if ((sag_Sensor >= minimum_limit1) && (sag_arka_Sensor >= minimum_limit1) && (park_durumu == 1)) {
    /* Dik Park Etme Kararı */
    Robot_Dur();
    delay(500);
    park_durumu = 10;
    Serial.println(park_durumu);
    Serial.println("dik");
  }

  if ((sag_Sensor <= minimum_limit) && (sag_arka_Sensor < minimum_limit) && (park_durumu == 2)) {
    /* Paralel Park Etme Kararı */
    Robot_Dur();
    delay(500);
    park_durumu = 3;
    Serial.println(park_durumu);
    Serial.println("paralel");
  }

  return park_durumu;
}

void Park_bul() {
  Park_Yeri_Kontrol();
  if (park_durumu == 3) {
    Robot_Dur();
    Serial.println(park_durumu);
    delay(400);
    park_durumu = 4;
  }
  if (park_durumu == 4) {

    Robot_Hareket(Geri, output);
    say(18);
    Robot_Dur();
    Serial.println(park_durumu);
    delay(500);
    Robot_Hareket(Sag, 150);
    say(9);
    Robot_Dur();
    delay(500);
    park_durumu = 5;
  }
  if (park_durumu == 5) {

    Robot_Hareket(Geri, output);
    long arka_Sensor = ultrasonic_arka.Ranging(CM);
    Serial.print("arka_Sensor");
    Serial.println(arka_Sensor);

    if (arka_Sensor > 0 && arka_Sensor <= 13) {
      Robot_Dur();
      delay(400);
      park_durumu = 6;
    }
    return arka_Sensor;
  }

  if (park_durumu == 6) {
    Robot_Hareket(Sol, 150);
    long sag_Sensor = ultrasonic_sol_on.Ranging(CM);
    Serial.println(sag_Sensor);
    long sag_arka_Sensor = ultrasonic_sol_arka.Ranging(CM);
    Serial.println(sag_arka_Sensor);

    if (sag_Sensor == sag_arka_Sensor) {
      Robot_Dur();
      park_durumu = 7;
    }

    return sag_Sensor, sag_arka_Sensor;
  }
  if (park_durumu == 7) {
    long on_Sensor = ultrasonic_on.Ranging(CM);

    if (on_Sensor <= 6) {
      Robot_Dur();
      park_durumu = 8;
    } else {
      Robot_Hareket(Ileri, output);
    }
    return on_Sensor;
  }
  if (park_durumu == 10) {

    Robot_Hareket(Sol, 180);
    say(14);
    Robot_Dur();
    delay(500);
    park_durumu = 7;
  }
}

void setup() {
  Serial.begin(9600);
  attachInterrupt(5, say, CHANGE);
  pinMode(sinyalpin, INPUT);

  motor_pinSetup();

  attachInterrupt(digitalPinToInterrupt(LM393), count, RISING);
  timer.setInterval(1000);
  timer.setCallback(speed1);
  timer.start();
  pinMode(LED, OUTPUT);

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
  long pulse = ultrasonic_on.Ranging(CM);
  return pulse;
}


int speed1() {
  int t = (60 * counter / 200);
  counter = 0;
  return t;
}

void loop() {
  Park_bul();

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

  analogWrite(LED, output);
}