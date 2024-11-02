#include <AFMotor.h>

// Configuración de los motores
AF_DCMotor motor1(1);  // Motor en el puerto M1
AF_DCMotor motor2(2);  // Motor en el puerto M2
AF_DCMotor motor3(3);  // Motor en el puerto M3
AF_DCMotor motor4(4);  // Motor en el puerto M4

void setup() {
  Serial.begin(9600);  // Para depuración
  Serial.println("Iniciando...");

  // Configurar la velocidad de los motores (0-255)
  motor1.setSpeed(180);
  motor2.setSpeed(180);
  motor3.setSpeed(180);
  motor4.setSpeed(180);
}

void loop() {
  // Mover los motores hacia adelante
  motor1.run(FORWARD);
  motor2.run(FORWARD);
  motor3.run(FORWARD);
  motor4.run(FORWARD);

  Serial.println("Motores moviéndose hacia adelante...");
  delay(2000);  // Mover hacia adelante por 2 segundos

  // Detener los motores
  motor1.run(RELEASE);
  motor2.run(RELEASE);
  motor3.run(RELEASE);
  motor4.run(RELEASE);

  Serial.println("Motores detenidos.");
  delay(2000);  // Esperar 2 segundos antes de repetir
}
