#include <Servo.h>

const int PIN_TRIG = 9;
const int PIN_ECHO = 10;
const int PIN_SERVO = 6;
Servo meuServo;

const int ANGULO_SERVO_RETO = 90;
const int MAX_INCLINACAO = 30;

// PID otimizado para movimento contínuo
double Kp = 2.0;   
double Ki = 0.005; 
double Kd = 9.0;   // Aumentamos o Kd para o sistema "frear" antes de chegar no limite

double setpointDinamico = 17.875;
double posicaoAtual = 17.875;
double erro, erroAnterior = 0, integral = 0;

unsigned long tempoAnterior = 0;
const unsigned long DT = 40;

void setup() {
  Serial.begin(115200);
  meuServo.attach(PIN_SERVO);
  pinMode(PIN_TRIG, OUTPUT);
  pinMode(PIN_ECHO, INPUT);
}

void loop() {
  unsigned long tempoAtual = millis();
  if (tempoAtual - tempoAnterior >= DT) {
    double dt = (tempoAtual - tempoAnterior) / 1000.0;
    tempoAnterior = tempoAtual;

    // 1. Leitura
    digitalWrite(PIN_TRIG, LOW); delayMicroseconds(2);
    digitalWrite(PIN_TRIG, HIGH); delayMicroseconds(10); // Ajustado para 10us padrão
    digitalWrite(PIN_TRIG, LOW);
    double dist = pulseIn(PIN_ECHO, HIGH, 15000) * 0.0343 / 2.0;
    
    if (dist < 1.0 || dist > 40.0) dist = posicaoAtual;
    posicaoAtual = (posicaoAtual * 0.6) + (dist * 0.4); 

    // 2. Trajetória Senoidal Contínua
    setpointDinamico = 16.5 + 11.5 * sin((2.0 * PI * tempoAtual) / 15000.0);

    // 3. PID
    erro = setpointDinamico - posicaoAtual;
    integral = constrain(integral + (erro * dt), -2, 2);
    double derivativo = (erro - erroAnterior) / dt;
    double outputPID = (Kp * erro) + (Ki * integral) + (Kd * derivativo);
    erroAnterior = erro;

    // 4. APLICAÇÃO DE FORÇA (Sem "Kicks" bruscos)
    // Se o lado direito é lerdo, aplicamos um ganho proporcional ao erro, 
    // não uma força fixa que trava o sistema.
    if (outputPID < 0) {
      outputPID *= 1.3; // Aumenta 30% da força apenas quando vai para a direita
    }

    outputPID = constrain(outputPID, -MAX_INCLINACAO, MAX_INCLINACAO);
    meuServo.write(ANGULO_SERVO_RETO - (int)outputPID);
  }
}