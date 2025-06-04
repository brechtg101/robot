/*
 * Lijnvolgende robot met obstakelvermijding
 * Origineel door ASHIQ SHIBU (MAKER Q)
 * Nederlandse versie
 */

// ===== CONFIGURATIE VARIABELEN =====
// Snelheidsinstellingen voor de motoren
int vSpeed = 80;         // Normale rijsnelheid
int turn_speed = 160;    // Snelheid tijdens draaien (0-255)
int t_p_speed = 100;      // Snelheid tijdens obstakelvermijding
int stop_distance = 12;  // Afstand in cm waarop robot stopt voor obstakel

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
}

void driveForward(int speed) {
  digitalWrite(motorR1, HIGH);
  digitalWrite(motorR2, LOW);                       
  digitalWrite(motorL1, HIGH);
  digitalWrite(motorL2, LOW);
  analogWrite(motorRspeed, speed);
  analogWrite(motorLspeed, speed);
}

void setup() {
  /
}

void loop() {
  // Besuring op basis van lijnsensor input
    driveForward(vSpeed);
    delay(100);
}










 
