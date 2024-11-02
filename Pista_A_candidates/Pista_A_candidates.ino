#include <AFMotor.h>

// Inicializa los motores
AF_DCMotor motor1(1);  // Motor en el puerto M1
AF_DCMotor motor2(2);  // Motor en el puerto M2
AF_DCMotor motor3(3);  // Motor en el puerto M3
AF_DCMotor motor4(4);  // Motor en el puerto M4

// Pines de los sensores ultrasónicos
const int trigPinFront = 22, echoPinFront = 23;
const int trigPinBack = 24, echoPinBack = 25;
const int trigPinLeft = 26, echoPinLeft = 27;
const int trigPinRight = 28, echoPinRight = 29;

// Distancia mínima para considerar un obstáculo (en cm)
const int obstacleDistance = 18;

// Variables que almaenan los valores Derecha/Izquierda
int distDcha = 0;
int distIzda = 0;

void setup() {
  Serial.begin(9600);

  // Configuración incial de los motores
  motor1.setSpeed(180);
  motor1.run(RELEASE);
  motor2.setSpeed(180);
  motor2.run(RELEASE);
  motor3.setSpeed(180);
  motor3.run(RELEASE);
  motor4.setSpeed(180);
  motor4.run(RELEASE);  

  // Configura los pines de los sensores
  pinMode(trigPinFront, OUTPUT);
  pinMode(echoPinFront, INPUT);
  pinMode(trigPinBack, OUTPUT);
  pinMode(echoPinBack, INPUT);
  pinMode(trigPinLeft, OUTPUT);
  pinMode(echoPinLeft, INPUT);
  pinMode(trigPinRight, OUTPUT);
  pinMode(echoPinRight, INPUT);
}

void loop() {
  int ddistanceFront = 0;
  int ddistanceLeft = 0;
  int ddistanceRight = 0;
  moveForward();
  long distanceFront = readUltrasonicDistance(trigPinFront, echoPinFront);

  // Imprime las distancias en el Monitor Serial (opcional para pruebas)
  Serial.print("Front: ");
  Serial.print(ddistanceFront);
  Serial.print(" cm, Left: ");
  Serial.print(ddistanceLeft);
  Serial.print(" cm, Right: ");
  Serial.println(ddistanceRight);

  // Decide la dirección basada en las lecturas
  if (distanceFront <= obstacleDistance) {
    stopMotors();
    moveBackward();
    delay(500);
    stopMotors();
    long distanceRight = readUltrasonicDistance(trigPinLeft, echoPinLeft);
    delay(1000);
    long distanceLeft = readUltrasonicDistance(trigPinRight, echoPinRight);
    delay(1000);
    if (distanceRight <= obstacleDistance){
      turnRight();
      stopMotors();
    }
    else if (distanceLeft <= obstacleDistance){
      turnLeft();
      stopMotors();
    }
  }
else{
  moveForward();
  }
}

// Función para obtener la distancia con un sensor
long readUltrasonicDistance(int trigPin, int echoPin) {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);

  long duration = pulseIn(echoPin, HIGH);
  long distance = duration / 59;
  return distance;
}

// Funciones de movimiento
void moveForward() {
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);
}

void stopMotors() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}

void turnLeft() {
  motor1.setSpeed(250); 
  motor2.setSpeed(250); 
  motor3.setSpeed(250);
  motor4.setSpeed(250);
  motor1.run(FORWARD);
  motor2.run(BACKWARD);
  motor3.run(FORWARD);
  motor4.run(BACKWARD);
  delay(850);
}

void turnRight() {
  motor1.setSpeed(200); 
  motor2.setSpeed(200); 
  motor3.setSpeed(200);
  motor4.setSpeed(200);
  motor1.run(BACKWARD);
  motor2.run(FORWARD);
  motor3.run(BACKWARD);
  motor4.run(FORWARD);
  delay(850);
}

void moveBackward() {
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);
}
