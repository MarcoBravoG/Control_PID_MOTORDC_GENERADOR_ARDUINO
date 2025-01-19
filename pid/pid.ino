// Pines
const int pinSetpoint = A0;  // Entrada de referencia
const int pinFeedback = A1;  // Entrada de retroalimentación
const int pinPWM = 9;        // Salida PWM

// Variables PID
double setpoint = 0;     // Valor de referencia (0-100)
double feedback = 0;     // Valor real medido (0-100)
double output = 0;       // Salida PID (0-100)

// Parámetros PID (ajustables)
double Kp = 2.0;  // Ganancia proporcional
double Ki = 0.5;  // Ganancia integral
double Kd = 1.0;  // Ganancia derivativa

// Variables para el cálculo PID
double errorAnterior = 0;
double integral = 0;

void setup() {
  pinMode(pinPWM, OUTPUT);      // Configurar pin PWM como salida
  Serial.begin(9600);           // Comunicación serial para monitoreo
}

void loop() {
  // Leer entradas analógicas y normalizar (0-1023 → 0-100)
  setpoint = map(analogRead(pinSetpoint), 0, 1023, 0, 100);
  feedback = map(analogRead(pinFeedback), 0, 1023, 0, 100);

  // Calcular error
  double error = setpoint - feedback;

  // Integración del error
  integral += error;

  // Derivada del error
  double derivada = error - errorAnterior;

  // Cálculo del PID
  output = (Kp * error) + (Ki * integral) + (Kd * derivada);

  // Limitar la salida entre 0% y 100%
  output = constrain(output, 0, 100);

  // Convertir la salida (0-100) al rango PWM (0-255)
  int pwmOutput = map(output, 0, 100, 0, 255);
  analogWrite(pinPWM, pwmOutput);

  // Guardar error para el siguiente ciclo
  errorAnterior = error;

  // **Formato para el Plotter Serial**
 Serial.print(setpoint);
  Serial.print("\t");
  Serial.print(feedback);
  Serial.print("\t");
  Serial.println(output);

  delay(100);  // Pequeña pausa para estabilidad
}
