#include <AFMotor.h>
#include <QTRSensors.h>

QTRSensors qtr;

#define Kp 0.065 
#define Kd 0.0001
#define MaxSpeed 230
#define BaseSpeed 150
#define SpeedTurn 180
#define CheckPoint 800

const uint8_t SensorCount = 8;
// IR Sensors Count 
uint16_t sensorValues[SensorCount];

// Configuración de los motores
AF_DCMotor motor1(1);  // Motor en el puerto M1
AF_DCMotor motor2(2);  // Motor en el puerto M2
AF_DCMotor motor3(3);  // Motor en el puerto M3
AF_DCMotor motor4(4);  // Motor en el puerto M4

int lastError = 0;

void setup()
{   
    qtr.setTypeAnalog();
    qtr.setSensorPins((const uint8_t[]){A8, A9, A10, A11, A12, A13, A14, A15}, SensorCount);

    Serial.begin(9600);
    delay(500);
    
    pinMode(LED_BUILTIN, OUTPUT);

    // LED turns ON to indicate the start of the calibration
    digitalWrite(LED_BUILTIN, HIGH);
    delay(3000);
    
    for (int i = 0; i < 400; i++)
    {   
        // Los motores giran para permitir la calibración en movimiento
        setMotorSpeeds(SpeedTurn, -SpeedTurn); // Uno hacia adelante y otro hacia atrás
        qtr.calibrate();
        delay(20);
    }

    digitalWrite(LED_BUILTIN, LOW);
    delay(3000); 
}  

void loop()
{  
    followLine();
}

void followLine(){
    uint16_t position = qtr.readLineBlack(sensorValues);  
    for (uint8_t i = 0; i < SensorCount; i++)
    {
        Serial.print(sensorValues[i]);
        Serial.print('\t');
    }
    Serial.println(position);

    // Demasiado a la derecha, el robot gira a la izquierda
    if(position > 3800)  
    {
        setMotorSpeeds(BaseSpeed, -BaseSpeed); // Un par de motores gira en direcciones opuestas
        return;    
    }

    // Demasiado a la izquierda, el robot gira a la derecha
    if(position < 2900)
    {  
        setMotorSpeeds(-BaseSpeed, BaseSpeed); // El otro par de motores gira en direcciones opuestas
        return;
    }

    int error = position - 3500;
    int motorSpeed = Kp * error + Kd * (error - lastError);
    lastError = error;

    Serial.print("Error: ");
    Serial.println(error);

    int rightMotorSpeed = BaseSpeed + motorSpeed;
    int leftMotorSpeed = BaseSpeed - motorSpeed;
  
    // Limitar las velocidades dentro del máximo permitido
    rightMotorSpeed = constrain(rightMotorSpeed, 0, MaxSpeed); 
    leftMotorSpeed = constrain(leftMotorSpeed, 0, MaxSpeed);
    
    // Ajustar las velocidades de cada par de motores
    setMotorSpeeds(leftMotorSpeed, rightMotorSpeed);
}

void setMotorSpeeds(int leftSpeed, int rightSpeed) {
    // Controla la velocidad y dirección de cada motor
    motor1.setSpeed(abs(leftSpeed));
    motor2.setSpeed(abs(leftSpeed));
    motor3.setSpeed(abs(rightSpeed));
    motor4.setSpeed(abs(rightSpeed));
  
    // Establece la dirección según la velocidad
    if (leftSpeed > 0) {
        motor1.run(FORWARD);
        motor2.run(FORWARD);
    } else {
        motor1.run(BACKWARD);
        motor2.run(BACKWARD);
    }

    if (rightSpeed > 0) {
        motor3.run(FORWARD);
        motor4.run(FORWARD);
    } else {
        motor3.run(BACKWARD);
        motor4.run(BACKWARD);
    }
}
