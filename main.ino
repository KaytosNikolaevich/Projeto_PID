#include <Servo.h>
#include <Wire.h> // Biblioteca nativa do Arduino para o I2C (A4 e A5)

const int trigPin = 10;
const int echoPin = 11;
const int servoPin = 9;

Servo meuServo;

// --- CONFIGURAÇÕES DO MPU-6050 ---
const int MPU_ADDR = 0x68; 
int16_t AcX, AcY, AcZ;

// --- CONFIGURAÇÕES FÍSICAS ---
int centroServo = 50;    
double setpoint = 19.0;  
double distanciaFiltrada = 19.0;
const float alpha = 0.4; 

// --- VARIÁVEIS DO CONTROLE PD ---
double Kp = 1.8, Ki = 0.0, Kd = 2.5; 
double erro, erroAnterior = 0;
unsigned long tempoAnterior = 0;

int anguloAtual = 50; 
int modoResgate = 0; 

// --- VARIÁVEIS DE DESATOLAMENTO ---
unsigned long tempoInicioResgate = 0;
unsigned long tempoPresoDireita = 0;
bool presoDireita = false;

void setup() {
  Serial.begin(115200); 
  pinMode(trigPin, OUTPUT);
  pinMode(echoPin, INPUT);
  meuServo.attach(servoPin);
  
  meuServo.write(centroServo); 

  // --- INICIALIZAÇÃO DO MPU-6050 ---
  Wire.begin();
  Wire.beginTransmission(MPU_ADDR);
  Wire.write(0x6B); // Registro de gerenciamento de energia
  Wire.write(0);    // Escreve 0 para acordar o sensor
  Wire.endTransmission(true);
  
  delay(1000);
}

void loop() {
  unsigned long tempoAtual = millis();
  double dt = (double)(tempoAtual - tempoAnterior) / 1000.0;

  if (dt >= 0.02) { 
    // --- 1. LEITURA DO MPU-6050 ---
    Wire.beginTransmission(MPU_ADDR);
    Wire.write(0x3B); 
    Wire.endTransmission(false);
    Wire.requestFrom(MPU_ADDR, 6, true); 
    
    AcX = Wire.read() << 8 | Wire.read(); 
    AcY = Wire.read() << 8 | Wire.read(); 
    AcZ = Wire.read() << 8 | Wire.read(); 
    
    // Calcula o ângulo real da bandeja (de -90 a 90 graus)
    float anguloRealBandeja = atan2(AcY, sqrt(AcX * AcX + AcZ * AcZ)) * 180.0 / PI;

    // --- 2. LEITURA DA DISTÂNCIA ---
    double rawDist = lerDistancia();
    
    // --- 3. MODO CEGO / PAREDE DA ESQUERDA (> 17.5cm) ---
    if (rawDist == 0 || rawDist > 17.5) { 
      
      if (modoResgate == 0) {
        tempoInicioResgate = tempoAtual; 
        if (distanciaFiltrada >= 14.0) {
          modoResgate = 1; 
        } else {
          modoResgate = 2; 
        }
      }

      if (tempoAtual - tempoInicioResgate > 3000) {
        modoResgate = (modoResgate == 1) ? 2 : 1; 
        tempoInicioResgate = tempoAtual; 
      }

      if (modoResgate == 1) {
        anguloAtual = 37; 
      } else {
        anguloAtual = 63; 
      }
      
      meuServo.write(anguloAtual);
      erroAnterior = 0; 
      tempoAnterior = tempoAtual;
      
      // Imprime o log de emergência com o MPU
      imprimirGrafico(distanciaFiltrada, erro, anguloAtual, anguloRealBandeja);
      return; 
    }

    modoResgate = 0; 
    distanciaFiltrada = (alpha * rawDist) + ((1.0 - alpha) * distanciaFiltrada);

    // --- 4. PAREDE DA DIREITA ANTECIPADA (<= 11 cm) ---
    if (distanciaFiltrada <= 11.0) {
      
      if (!presoDireita) {
        presoDireita = true;
        tempoPresoDireita = tempoAtual;
      }

      if (tempoAtual - tempoPresoDireita > 2000) {
        anguloAtual = 37; 
      } else {
        anguloAtual = 63; 
      }
      
      meuServo.write(anguloAtual); 
      erroAnterior = setpoint - distanciaFiltrada;
      tempoAnterior = tempoAtual;
      
      imprimirGrafico(distanciaFiltrada, erro, anguloAtual, anguloRealBandeja);
      return; 
    } else {
      presoDireita = false; 
    }

    // --- 5. CÁLCULO DO PID --- 
    erro = setpoint - distanciaFiltrada;
    double derivativo = (erro - erroAnterior) / dt;
    double output = (Kp * erro) + (Kd * derivativo);
    
    int anguloAlvo = centroServo + (int)output;
    anguloAtual = constrain(anguloAlvo, 20, 80); 
    
    meuServo.write(anguloAtual); 
    
    // Imprime o gráfico em funcionamento normal
    imprimirGrafico(distanciaFiltrada, erro, anguloAtual, anguloRealBandeja);
    
    erroAnterior = erro;
    tempoAnterior = tempoAtual;
  }
}

// --- FUNÇÃO PARA MANTER A PLOTADORA LIMPA ---
void imprimirGrafico(double dist, double err, int angServo, float angReal) {
    Serial.print("Dist:"); Serial.print(dist); Serial.print(",");
    // Multiplicado por 10 para ficar mais visível junto com os ângulos no gráfico
    Serial.print("ErroX10:"); Serial.print(err * 10); Serial.print(","); 
    Serial.print("AnguloServo:"); Serial.print(angServo); Serial.print(",");
    Serial.print("AnguloReal:"); Serial.println(angReal);
}

double lerDistancia() {
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2);
  digitalWrite(trigPin, HIGH);
  delayMicroseconds(10);
  digitalWrite(trigPin, LOW);
  
  long duracao = pulseIn(echoPin, HIGH, 3000); 
  if (duracao == 0) return 999.0; 
  return (duracao * 0.034 / 2.0);
}