/*
 * Lijnvolgende robot met obstakelvermijding
 * Origineel door ASHIQ SHIBU (MAKER Q)
 * Nederlandse versie
 */

#include "mqtt_comm.h"

// ===== CONFIGURATIE VARIABELEN =====
// Snelheidsinstellingen voor de motoren
int vSpeed = 80;         // Normale rijsnelheid
int turn_speed = 200;    // Snelheid tijdens draaien (0-255)
int t_p_speed = 100;      // Snelheid tijdens obstakelvermijding
int stop_distance = 12;  // Afstand in cm waarop robot stopt voor obstakel

// IR Sensor thresholds
int IR_THRESHOLD_LEFT = 500;   // Threshold waarde voor linker IR sensor (0-4095)
int IR_THRESHOLD_RIGHT = 500;  // Threshold waarde voor rechter IR sensor (0-4095)

// ===== PIN CONFIGURATIE =====
// Ultrasone sensor (HC-SR04) aansluitingen
const int trigPin = 25;  // Trigger pin
const int echoPin = 26;  // Echo pin

// Motor driver (L293) aansluitingen
const int motorR1 = 18;      // Rechter motor richting pin 1
const int motorR2 = 19;      // Rechter motor richting pin 2
const int motorRspeed = 15;  // Rechter motor snelheid pin
const int motorL1 = 21;       // Linker motor richting pin 1
const int motorL2 = 5;      // Linker motor richting pin 2
const int motorLspeed = 4;  // Linker motor snelheid pin

// Lijnsensor aansluitingen
const int left_sensor_pin = 33;   // Linker IR sensor (analog)
const int right_sensor_pin = 35;  // Rechter IR sensor (analog)

// ===== GLOBALE VARIABELEN =====
int turnspeed;            // Variabele voor draaisnelheid
int left_sensor_value;    // Analoge waarde linker IR sensor
int right_sensor_value;   // Analoge waarde rechter IR sensor
bool left_sensor_state;   // Status linker IR sensor (true = lijn gedetecteerd)
bool right_sensor_state;  // Status rechter IR sensor (true = lijn gedetecteerd)
long duration;            // Tijdsduur voor ultrasone sensor
int distance;            // Gemeten afstand in cm

// MQTT throttle
unsigned long lastMqttSend = 0;
const unsigned long mqttInterval = 500; // 500 ms

// ===== MOTOR FUNCTIES =====
void stopMotors() {
  analogWrite(motorRspeed, 0);
  analogWrite(motorLspeed, 0);

  publishMotorSpeeds(0, 0);
}

void driveForward(int speed) {
  digitalWrite(motorR1, HIGH);
  digitalWrite(motorR2, LOW);                       
  digitalWrite(motorL1, HIGH);
  digitalWrite(motorL2, LOW);
  analogWrite(motorRspeed, speed);
  analogWrite(motorLspeed, speed);

  publishMotorSpeeds(speed, vSpeed);
  publishMovement("forward");
}

void turnRight(int speed) {
  digitalWrite(motorR1, LOW);
  digitalWrite(motorR2, HIGH);                       
  digitalWrite(motorL1, HIGH);
  digitalWrite(motorL2, LOW);
  analogWrite(motorRspeed, speed);
  analogWrite(motorLspeed, turn_speed);

  publishMotorSpeeds(speed, turn_speed);
  publishMovement("turnRight");
}

void turnLeft(int speed) {
  digitalWrite(motorR1, HIGH);
  digitalWrite(motorR2, LOW);                       
  digitalWrite(motorL1, LOW);
  digitalWrite(motorL2, HIGH);
  analogWrite(motorRspeed, turn_speed);
  analogWrite(motorLspeed, speed);

  publishMotorSpeeds(speed, turn_speed);
  publishMovement("turnLeft");
}

// ===== OBSTAKELVERMIJDING FUNCTIE =====
void avoidObstacle(int distance) {
  Serial.println("Obstakel gedetecteerd! Start ontwijkmanoeuvre");
  Serial.print("Afstand tot obstakel: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Ontwijkmanoeuvre naar links
  turnLeft(t_p_speed);
  delay(250);
  
  // Stop
  publishMovement("stop");
  stopMotors();
  delay(500);
  
  // Draai
  turnLeft(t_p_speed);
  delay(900);

  // Rij vooruit
  driveForward(t_p_speed);
  delay(800);

  // Draai terug
  turnRight(t_p_speed);
  delay(900);

  // Rij vooruit
  driveForward(t_p_speed);
  delay(700);

  // Draai
  turnRight(t_p_speed);
  delay(650);

  // Rij vooruit
  driveForward(t_p_speed);

  // Zoek naar lijn
  left_sensor_state = HIGH;
  while(left_sensor_state == LOW) {
    left_sensor_state = digitalRead(left_sensor_pin);
    right_sensor_state = digitalRead(right_sensor_pin);
    Serial.println("Zoeken naar lijn...");
    delay(100);
  }

  // Finale positionering
  turnLeft(t_p_speed);
  delay(100);
  
  driveForward(t_p_speed);
}

void setup() {
  // Start seriële communicatie
  Serial.begin(115200);
  Serial.println("\n=== Lijnvolgende Robot met Obstakelvermijding ===");
  Serial.println("Initialiseren van pinnen...");
  
  // Configureer motor pinnen als output
  pinMode(motorR1, OUTPUT);
  pinMode(motorR2, OUTPUT);
  pinMode(motorL1, OUTPUT);
  pinMode(motorL2, OUTPUT);
  Serial.println("Motor pinnen geïnitialiseerd");

  // Configureer ultrasone sensor pinnen
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT); 
  Serial.println("Ultrasone sensor pinnen geïnitialiseerd");
  
  // Configureer lijnsensor pinnen
  pinMode(left_sensor_pin, INPUT);
  pinMode(right_sensor_pin, INPUT);
  Serial.println("Lijnsensor pinnen geïnitialiseerd");

  // Initialize WiFi and MQTT
  setupWiFi();
  setupMqtt();
  if (connectMqtt()) {
    publishStatus("Robot initialized and connected to MQTT");
  }

  Serial.println("Setup voltooid. Starten over 3 seconden...");
  delay(3000);                              
  Serial.println("Robot is nu actief!");
}

void loop() {
  // Update MQTT connection
  updateMqtt();

  // Lees ultrasone sensor
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  duration = pulseIn(echoPin, HIGH);
  if (duration == 0) {
    distance = 999; // Timeout waarde
  } else {
    distance = duration * 0.034 / 2;
    if (distance < 1) {
      distance = 999; // Ignore values below 1 cm
    }
  }

  // Lees IR sensoren (analog)
  left_sensor_value = analogRead(left_sensor_pin);
  right_sensor_value = analogRead(right_sensor_pin);
  
  // Bepaal sensor status op basis van threshold
  left_sensor_state = (left_sensor_value > IR_THRESHOLD_LEFT);
  right_sensor_state = (right_sensor_value > IR_THRESHOLD_RIGHT);

  Serial.print("IR Sensoren - Links: ");
  Serial.print(left_sensor_value);
  Serial.print(" (");
  Serial.print(left_sensor_state ? "LIJN" : "GEEN LIJN");
  Serial.print(") Rechts: ");
  Serial.print(right_sensor_value);
  Serial.print(" (");
  Serial.print(right_sensor_state ? "LIJN" : "GEEN LIJN");
  Serial.println(")");

  publishDistances(distance);
  publishLineSensors(left_sensor_state, right_sensor_state);

  // Besturing op basis van IR sensor input
  if(right_sensor_state == false && left_sensor_state == true) {
    Serial.println("Actie: Draait naar rechts");
    turnRight(vSpeed);
  }
  else if(right_sensor_state == true && left_sensor_state == false) {
    Serial.println("Actie: Draait naar links");
    turnLeft(vSpeed);
  }
  else if(right_sensor_state == true && left_sensor_state == true) {
    Serial.println("Actie: Rijdt vooruit");
    driveForward(vSpeed);
  }
  else {
    stopMotors();
  }

  if(distance < stop_distance) {
    publishStatus("Obstacle detected - starting avoidance maneuver");
    avoidObstacle(distance);
    publishStatus("Obstacle avoidance completed");
  }
}










 
