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

// Instancia de imanes y vueltas
int hall_magnet_count = 0;
int hall_2_magnet_count = 0;
int hall_laps_count = 0;
int hall_2_laps_count = 0;

// Variables para el control de estabilidad
unsigned long last_correction_time = 0;
const unsigned long CORRECTION_INTERVAL = 20; // ms entre correcciones (reducido para mejor respuesta)

// Parámetros de control mejorados
const int BASE_SPEED = 70;
const int MOTOR1_OFFSET = 0;    // Ajusta este valor si motor 1 es más débil (ej: 5, 10, 15)
const int MOTOR2_OFFSET = 0;    // Ajusta este valor si motor 2 es más débil (ej: 5, 10, 15)

// Control PID simplificado
float Kp = 3.0;  // Ganancia proporcional - aumenta para corrección más agresiva
float Ki = 0.1;  // Ganancia integral - acumula error a lo largo del tiempo
float Kd = 0.5;  // Ganancia derivativa - suaviza cambios bruscos

float error_integral = 0;
float last_error = 0;

// Límites
const int MAX_SPEED_ADJUSTMENT = 20;
const int MIN_SPEED = 50;
const int MAX_SPEED = 100;

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
  Serial.begin(115200);

  pinMode(HALL_PIN, INPUT);
  pinMode(HALL_2_PIN, INPUT);

  attachInterrupt(digitalPinToInterrupt(HALL_PIN), hallInterrupt, FALLING);
  attachInterrupt(digitalPinToInterrupt(HALL_2_PIN), hall2Interrupt, FALLING);
  
  // Establecer velocidad inicial con offset de calibración
  motor.setSpeed(BASE_SPEED + MOTOR1_OFFSET);
  motor2.setSpeed(BASE_SPEED + MOTOR2_OFFSET);
  
  Serial.println("Sistema iniciado con calibración");
  Serial.print("Motor 1 offset: "); Serial.println(MOTOR1_OFFSET);
  Serial.print("Motor 2 offset: "); Serial.println(MOTOR2_OFFSET);
}

void loop()
{ 
  // Control de estabilidad del vehículo
  stabilizeVehiclePID();
  
  // Motores en movimiento
  motor.backward();
  motor2.backward();

  // Imprimir información de depuración
  printDebugInfo();

  delay(30);
}

/*
Algoritmo de estabilización con control PID
*/
void stabilizeVehiclePID() {
  unsigned long current_time = millis();
  
  if (current_time - last_correction_time >= CORRECTION_INTERVAL) {
    float delta_time = (current_time - last_correction_time) / 1000.0; // en segundos
    last_correction_time = current_time;
    
    // Calcular conteo total de imanes de cada motor
    int total_count_motor1 = hall_laps_count * 8 + hall_magnet_count;
    int total_count_motor2 = hall_2_laps_count * 8 + hall_2_magnet_count;
    
    // Error: diferencia entre motores (positivo si motor1 va más rápido)
    float error = total_count_motor1 - total_count_motor2;
    
    // Control PID
    error_integral += error * delta_time;
    // Anti-windup: limitar la integral
    error_integral = constrain(error_integral, -50, 50);
    
    float error_derivative = (error - last_error) / delta_time;
    
    // Calcular ajuste PID
    float pid_output = (Kp * error) + (Ki * error_integral) + (Kd * error_derivative);
    
    // Limitar ajuste
    pid_output = constrain(pid_output, -MAX_SPEED_ADJUSTMENT, MAX_SPEED_ADJUSTMENT);
    
    // Calcular nuevas velocidades con offset de calibración
    int speed_motor1 = BASE_SPEED + MOTOR1_OFFSET - pid_output;
    int speed_motor2 = BASE_SPEED + MOTOR2_OFFSET + pid_output;
    
    // Aplicar límites
    speed_motor1 = constrain(speed_motor1, MIN_SPEED, MAX_SPEED);
    speed_motor2 = constrain(speed_motor2, MIN_SPEED, MAX_SPEED);
    
    // Aplicar velocidades
    motor.setSpeed(speed_motor1);
    motor2.setSpeed(speed_motor2);
    
    // Guardar error para próxima iteración
    last_error = error;
    
    // Debug
    Serial.print("Error: "); Serial.print(error);
    Serial.print(" | PID: "); Serial.print(pid_output);
    Serial.print(" | M1 Speed: "); Serial.print(speed_motor1);
    Serial.print(" | M2 Speed: "); Serial.println(speed_motor2);
  }
}

/*
Imprimir información de depuración
*/
void printDebugInfo()
{
  int total_m1 = hall_laps_count * 8 + hall_magnet_count;
  int total_m2 = hall_2_laps_count * 8 + hall_2_magnet_count;
  
  Serial.println("=== Estado de Motores ===");
  Serial.print("Motor 1 - Total imanes: "); Serial.print(total_m1);
  Serial.print(" | Velocidad: "); Serial.println(motor.getSpeed());
  
  Serial.print("Motor 2 - Total imanes: "); Serial.print(total_m2);
  Serial.print(" | Velocidad: "); Serial.println(motor2.getSpeed());
  
  Serial.print("Diferencia total: "); Serial.println(total_m1 - total_m2);
  Serial.print("Integral acumulada: "); Serial.println(error_integral);
  Serial.println("------------------------");
}
