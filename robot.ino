/*
 * Lijnvolgende robot met obstakelvermijding
 * Origineel door ASHIQ SHIBU (MAKER Q)
 * Nederlandse versie
 */

#include "mqtt_comm.h"

// ===== CONFIGURATIE VARIABELEN =====
// Snelheidsinstellingen voor de motoren
int vSpeed = 100;         // Normale rijsnelheid
int turn_speed = 200;    // Snelheid tijdens draaien (0-255)
int t_p_speed = 100;      // Snelheid tijdens obstakelvermijding
int stop_distance = 12;  // Afstand in cm waarop robot stopt voor obstakel

// ===== PIN CONFIGURATIE =====
// Ultrasone sensor (HC-SR04) aansluitingen
const int trigPin = 25;  // Trigger pin
const int echoPin = 26;  // Echo pin

// Motor driver (L293) aansluitingen
const int motorR1 = 22;      // Rechter motor richting pin 1
const int motorR2 = 23;      // Rechter motor richting pin 2
const int motorRspeed = 15;  // Rechter motor snelheid pin
const int motorL1 = 2;       // Linker motor richting pin 1
const int motorL2 = 14;      // Linker motor richting pin 2
const int motorLspeed = 16;  // Linker motor snelheid pin

// Lijnsensor aansluitingen
const int left_sensor_pin = 33;   // Linker lijnsensor
const int right_sensor_pin = 35;  // Rechter lijnsensor

// ===== GLOBALE VARIABELEN =====
int turnspeed;            // Variabele voor draaisnelheid
int left_sensor_state;    // Status linker lijnsensor
int right_sensor_state;   // Status rechter lijnsensor
long duration;            // Tijdsduur voor ultrasone sensor
int distance;            // Gemeten afstand in cm

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
}

void turnRight(int speed) {
  digitalWrite(motorR1, LOW);
  digitalWrite(motorR2, HIGH);                       
  digitalWrite(motorL1, HIGH);
  digitalWrite(motorL2, LOW);
  analogWrite(motorRspeed, speed);
  analogWrite(motorLspeed, turn_speed);

  publishMotorSpeeds(speed, vSpeed);
}

void turnLeft(int speed) {
  digitalWrite(motorR1, HIGH);
  digitalWrite(motorR2, LOW);                       
  digitalWrite(motorL1, LOW);
  digitalWrite(motorL2, HIGH);
  analogWrite(motorRspeed, turn_speed);
  analogWrite(motorLspeed, speed);

  publishMotorSpeeds(speed, vSpeed);
}

void turnRightSharp(int speed) {
  digitalWrite(motorR1, LOW);
  digitalWrite(motorR2, HIGH);                       
  digitalWrite(motorL1, LOW);
  digitalWrite(motorL2, HIGH);
  analogWrite(motorRspeed, speed);
  analogWrite(motorLspeed, speed);

  publishMotorSpeeds(speed, vSpeed);
}

void turnLeftSharp(int speed) {
  digitalWrite(motorR1, HIGH);
  digitalWrite(motorR2, LOW);                       
  digitalWrite(motorL1, HIGH);
  digitalWrite(motorL2, LOW);
  analogWrite(motorRspeed, speed);
  analogWrite(motorLspeed, speed);

  publishMotorSpeeds(speed, vSpeed);
}

// ===== OBSTAKELVERMIJDING FUNCTIE =====
void avoidObstacle() {
  Serial.println("Obstakel gedetecteerd! Start ontwijkmanoeuvre");
  Serial.print("Afstand tot obstakel: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Ontwijkmanoeuvre naar links
  Serial.println("Ontwijkmanoeuvre naar links");
  turnLeftSharp(t_p_speed);
  delay(250);
  
  // Stop
  stopMotors();
  delay(500);
  
  // Draai
  Serial.println("Draai naar links");
  turnLeftSharp(t_p_speed);
  delay(900);

  // Rij vooruit
  Serial.println("Rij vooruit");
  driveForward(t_p_speed);
  delay(800);

  // Draai terug
  Serial.println("Draai terug rechts");
  turnRightSharp(t_p_speed);
  delay(900);

  // Rij vooruit
  Serial.println("Rij vooruit");
  driveForward(t_p_speed);
  delay(700);

  // Draai
  Serial.println("Draai rechts");
  turnRightSharp(t_p_speed);
  delay(650);

  // Rij vooruit
  Serial.println("Rij vooruit");
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
  Serial.println("Finale positionering");
  turnLeftSharp(t_p_speed);
  delay(100);
  
  Serial.println("Rij vooruit");
  driveForward(t_p_speed);
  delay(500);
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
  }
  Serial.print("Afstand: ");
  Serial.print(distance);
  Serial.println(" cm");

  // Publish distance data
  publishDistances(distance);  // Using same distance for left and right sensors

  // Lees lijnsensoren
  left_sensor_state = digitalRead(left_sensor_pin);
  right_sensor_state = digitalRead(right_sensor_pin);
  Serial.print("Lijnsensoren - Links: ");
  Serial.print(left_sensor_state);
  Serial.print(" Rechts: ");
  Serial.println(right_sensor_state);

  // Publish line sensor data
  publishLineSensors(left_sensor_state, right_sensor_state);

  // Besturing op basis van lijnsensor input
  if(right_sensor_state == LOW && left_sensor_state == HIGH) {
    Serial.println("Actie: Draait naar rechts");
    turnRight(vSpeed);
  }
  else if(right_sensor_state == HIGH && left_sensor_state == LOW) {
    Serial.println("Actie: Draait naar links");
    turnLeft(vSpeed);
  }
  else if(right_sensor_state == HIGH && left_sensor_state == HIGH) {
    Serial.println("Actie: Rijdt vooruit");
    driveForward(vSpeed);
    delay(100);
  }
  else {
    stopMotors();
  }

  if(distance < stop_distance) {
    publishStatus("Obstacle detected - starting avoidance maneuver");
    avoidObstacle();
    publishStatus("Obstacle avoidance completed");
  }
}










 
