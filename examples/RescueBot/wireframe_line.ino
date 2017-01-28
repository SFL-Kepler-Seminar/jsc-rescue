// -*- c++ -*-

// See https://www.arduino.cc/en/Reference/PortManipulation
// Wo ist unser Liniensensor, der NICHT am SCL/SDA-Pin haengt?
#define SDA_PORT PORTD
#define SDA_PIN 6
#define SCL_PORT PORTD
#define SCL_PIN 7

#define I2C_TIMEOUT 100
#define I2C_SLOWMODE 1

//assuming tactile switch
#define OBSTACLE_SENSOR_PIN 12
#define VICTIM_SENSOR_PIN 8

//Motor pins
//PWM PINS!
#define ML_B_PIN 5
#define ML_F_PIN 9
#define MR_B_PIN 10
#define MR_F_PIN 11

//Welchen Wert nehmen wir als Linie wahr?
#define LINE_THRESHHOLD 100

#include <SoftWire.h>
#include <Wire.h>
#include <LineSensor.h>

//#define btdebug // Wollen wir ueber Bluetooth informationen senden?

#ifdef btdebug
#define BT_RX_PIN 2
#define BT_TX_PIN 3
#include<SoftwareSerial.h>
SoftwareSerial mySerial(BT_RX_PIN, BT_TX_PIN);
#define Serial mySerial
#endif


LineSensor<SoftWire> line_l;
LineSensor<TwoWire> line_r;


typedef enum {FOLLOWING, EVACUATION_VICTIM_SEARCH, EVACUATION_EVACUATE, OBSTACLE} State;

class Motor {
  public:
    Motor(int f_pin, int b_pin): f_pin(f_pin), b_pin(b_pin) {
      pinMode(f_pin, OUTPUT);
      pinMode(b_pin, OUTPUT);
    }
    void drive(int speed) {
      bool forward = speed > 0;
      speed = abs(speed);
      speed = min(speed, 120);
      if (forward) {
        analogWrite(b_pin, LOW);
        analogWrite(f_pin, speed);
      } else {
        analogWrite(f_pin, LOW);
        analogWrite(b_pin, speed);
      }
    }
  private:
    int f_pin, b_pin;
};

State state = FOLLOWING;
Motor left(ML_F_PIN, ML_B_PIN);
Motor right(MR_F_PIN, MR_B_PIN);
unsigned long last_time = 0;
unsigned long last_state_switch_time = 0;

void setup() {
  Serial.begin(9600);
  Serial.println("Bootup...");
  // Liniensensoren initialisieren
  line_l.setup();
  line_r.setup();
  // Pins initialisieren
  pinMode(OBSTACLE_SENSOR_PIN, INPUT);
  // Pulldown
  digitalWrite(OBSTACLE_SENSOR_PIN, LOW);
  pinMode(VICTIM_SENSOR_PIN, INPUT);
  // Pulldown
  digitalWrite(VICTIM_SENSOR_PIN, LOW);

  // Motorpins initialisieren
  pinMode(ML_F_PIN, OUTPUT);
  pinMode(ML_B_PIN, OUTPUT);
  pinMode(MR_F_PIN, OUTPUT);
  pinMode(MR_B_PIN, OUTPUT);

  Serial.println("Pins inited.");
  last_time = last_state_switch_time = micros();
}


void changeState(State state_) {
  Serial.print("Time:");
  Serial.print(micros() / (1000l * 1000));
  Serial.print("s - Switch into state: ");
  Serial.println(state);
  state = state_;
  last_state_switch_time = micros();
  right.drive(0);
  left.drive(0);
}

void loop() {
  unsigned long current_time = micros();
  unsigned long delta_t = current_time - last_time;
  last_time = current_time;

  // Sensorwerte für die "Runde"
  uint16_t g_l = line_l.getG(); // Gruen links
  uint16_t g_r = line_r.getG(); // Gruen rechts
  uint16_t r_l = line_l.getR(); // Rot links
  uint16_t r_r = line_r.getR(); // Rot rechts
  uint16_t b_l = line_l.getB(); // Blau links
  uint16_t b_r = line_r.getB(); // Blau rechts
  uint16_t white_l = line_l.getC(); // Weiss links
  uint16_t white_r = line_r.getC(); // Weiss Rechts
  uint16_t prox_l = line_l.getProximity(); // Linie links
  uint16_t prox_r = line_r.getProximity(); // Linie rechts

  // Welcher Modus ist gerade aktiv?
  switch (state) {
    // Modus: Linie folgen
    case FOLLOWING:
      if (prox_l < LINE_THRESHHOLD && prox_r < LINE_THRESHHOLD) {
        //changeState(EVACUATION_VICTIM_SEARCH); // Moeglicher Indikator für die Evacuation-Zone?
        break;
      }
      if (digitalRead(OBSTACLE_SENSOR_PIN) == HIGH) { // Kollision mit Gegenstand
        changeState(OBSTACLE); // Wechsel in den Hindernismodus
        break;
      }
      if (prox_l < LINE_THRESHHOLD) {
        right.drive(50);
        left.drive(-50);
      } else if (prox_r < LINE_THRESHHOLD) {
        left.drive(50);
        right.drive(-50);
      } else {
        right.drive(70);
        left.drive(70);
      }
      break;

    // Modus: Opfer finden
    case EVACUATION_VICTIM_SEARCH:
      // Fahre irgendwie rum.
      if (digitalRead(VICTIM_SENSOR_PIN) == HIGH) {
        // Opfer gefunden!!!11!!
        changeState(EVACUATION_EVACUATE);
      }
      break;

    case EVACUATION_EVACUATE:
      // Suche Ablage und lege Opfer ab.
      // Wenn fertig:
      //changeState(EVACUATION_VICTIM_SEARCH);
      break;

    // Modus: Hindernis ausweichen
    case OBSTACLE:
      // Hier musst du etwas bauen, was dem Hindernis ausweicht.
      // Finde die Linie wieder
      // Wenn wieder auf der Linie:
      // changeState(FOLLOWING);
      break;
  }
}

