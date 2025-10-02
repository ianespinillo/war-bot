// Incluir la biblioteca
#include <L298N.h>

// Definición de pines de los motores
const unsigned int IN3 = 32;
const unsigned int IN4 = 25;
const unsigned int ENB = 33;
const unsigned int ENA = 19;
const unsigned int IN1 = 18;
const unsigned int IN2 = 5;

// Definición del pin del sensor Hall 44E
const int HALL_PIN = 35;
const int HALL_2_PIN = 34;

// instancia de imanes y vueltas
int hall_magnet_count = 0;
int hall_2_magnet_count = 0;
int hall_laps_count = 0;
int hall_2_laps_count = 0;

// Variables para el control de estabilidad
int last_hall_count = 0;
int last_hall_2_count = 0;
unsigned long last_correction_time = 0;
const unsigned long CORRECTION_INTERVAL = 70; // ms entre correcciones

// Parámetros de control
const int MAX_SPEED_DIFFERENCE = 10; // Diferencia máxima de velocidad permitida
const int BASE_SPEED = 70; // Velocidad base
const int MAGNET_DIFFERENCE_THRESHOLD = 1; // Umbral de diferencia de imanes para corrección

// Crear una instancia de los motores
L298N motor(ENA, IN1, IN2);
L298N motor2(ENB, IN3, IN4);

void IRAM_ATTR hallInterrupt(){
   hall_magnet_count += 1;
   if(hall_magnet_count >= 8){
      hall_laps_count += 1;
      hall_magnet_count = 0;
   }
}

void IRAM_ATTR hall2Interrupt(){
   hall_2_magnet_count += 1;
   if(hall_2_magnet_count >= 8){
      hall_2_laps_count += 1;
      hall_2_magnet_count = 0;
   }
}

void setup()
{
  // Usado para mostrar información
  Serial.begin(115200);

  // Configurar el pin del sensor hall como entrada
  pinMode(HALL_PIN, INPUT);
  pinMode(HALL_2_PIN, INPUT);

  // manejo de deteccion de iman
  attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(HALL_2_PIN), hall2Interrupt, FALLING);
  
  // Establecer la velocidad inicial
  motor.setSpeed(BASE_SPEED);
  motor2.setSpeed(BASE_SPEED);
}

void loop()
{ 
  // Control de estabilidad del vehículo
  stabilizeVehicle();
  
  // Leer el estado del sensor hall
  // Indicar al motor que avance (depende del cableado)
  motor.forward();
  motor2.forward();

  // Imprimir el estado del motor en el monitor serial
  printSomeInfo();

  delay(50); // Reducir delay para mejor respuesta
}

/*
Algoritmo de estabilización para evitar que el auto se gire
*/
void stabilizeVehicle() {
  unsigned long current_time = millis();
  
  // Ejecutar corrección solo cada cierto intervalo
  if (current_time - last_correction_time >= CORRECTION_INTERVAL) {
    last_correction_time = current_time;
    
    // Calcular diferencia entre los conteos de imanes
    int magnet_difference = hall_magnet_count - hall_2_magnet_count;
    int total_difference = (hall_laps_count * 8 + hall_magnet_count) - 
                          (hall_2_laps_count * 8 + hall_2_magnet_count);
    
    // Aplicar corrección solo si la diferencia supera el umbral
    if (abs(magnet_difference) > MAGNET_DIFFERENCE_THRESHOLD) {
      adjustMotorSpeeds(magnet_difference);
    } else {
      // Mantener velocidades base si la diferencia es mínima
      motor.setSpeed(BASE_SPEED);
      motor2.setSpeed(BASE_SPEED);
    }
    
    // Debug información
    Serial.print("Diferencia: ");
    Serial.print(magnet_difference);
    Serial.print(" | Total: ");
    Serial.print(total_difference);
    Serial.print(" | M1: ");
    Serial.print(hall_magnet_count);
    Serial.print(" | M2: ");
    Serial.println(hall_2_magnet_count);
  }
}

/*
Ajusta las velocidades de los motores basado en la diferencia de imanes
*/
void adjustMotorSpeeds(int difference) {
  int speed_motor1 = BASE_SPEED;
  int speed_motor2 = BASE_SPEED;
  
  if (difference > 0) {
    // Motor 1 está girando más rápido (más imanes detectados)
    // Reducir velocidad del motor 1 o aumentar motor 2
    speed_motor1 = constrain(BASE_SPEED - abs(difference) * 2, BASE_SPEED - MAX_SPEED_DIFFERENCE, BASE_SPEED);
    speed_motor2 = constrain(BASE_SPEED + abs(difference), BASE_SPEED, BASE_SPEED + MAX_SPEED_DIFFERENCE);
  } else if (difference < 0) {
    // Motor 2 está girando más rápido (más imanes detectados)
    // Reducir velocidad del motor 2 o aumentar motor 1
    speed_motor1 = constrain(BASE_SPEED + abs(difference), BASE_SPEED, BASE_SPEED + MAX_SPEED_DIFFERENCE);
    speed_motor2 = constrain(BASE_SPEED - abs(difference) * 2, BASE_SPEED - MAX_SPEED_DIFFERENCE, BASE_SPEED);
  }
  
  // Aplicar las nuevas velocidades
  motor.setSpeed(speed_motor1);
  motor2.setSpeed(speed_motor2);
  
  Serial.print("Ajustando - M1: ");
  Serial.print(speed_motor1);
  Serial.print(" | M2: ");
  Serial.println(speed_motor2);
}

/*
Imprimir algunas informaciones en el Monitor Serial
*/
void printSomeInfo()
{
  Serial.print("Motor 1 - Movimiento: ");
  Serial.print(motor.isMoving());
  Serial.print(" | Velocidad: ");
  Serial.print(motor.getSpeed());
  Serial.print(" | Imanes: ");
  Serial.print(hall_magnet_count);
  Serial.print(" | Vueltas: ");
  Serial.println(hall_laps_count);

  Serial.print("Motor 2 - Movimiento: ");
  Serial.print(motor2.isMoving());
  Serial.print(" | Velocidad: ");
  Serial.print(motor2.getSpeed());
  Serial.print(" | Imanes: ");
  Serial.print(hall_2_magnet_count);
  Serial.print(" | Vueltas: ");
  Serial.println(hall_2_laps_count);
  Serial.println("------------------------");
}
