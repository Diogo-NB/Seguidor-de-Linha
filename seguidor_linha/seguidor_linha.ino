// Motores
#define M1 11    // Pino Motor Esquerda/Direita Porta IN2 ponte H
#define M2 9     // Pino Motor Esquerda/Direita Porta IN4 ponte H
#define dir1 5   // Pino Motor Esquerda/Direita Porta IN1 ponte H
#define dir2 10  // Pino Motor Esquerda/Direita Porta IN3 ponte H

// Infravermelho
#define pin_S1 13
#define pin_S2 12
#define pin_S3 8
#define pin_S4 7

bool Sensor1 = 0;
bool Sensor2 = 0;
bool Sensor3 = 0;
bool Sensor4 = 0;

bool SensorEsq = 0;
bool SensorDir = 0;

// Ultrassonico
int PinTrigger = 3;  // Pino usado para disparar os pulsos do sensor
int PinEcho = 2;     // pino usado para ler a saida do sensor
float TempoEcho = 0;
const float VelocidadeSom_mpors = 340;        // em metros por segundo
const float VelocidadeSom_mporus = 0.000340;  // em metros por microsegundo
float DistanciaUltra = 0.0;

// Velocidade padrão dos motores
const int v_Padrao = 100;

// Velocidade padrão para curvas
const int v_CurvaF = 100;  // Frente
const int v_CurvaT = 160;  // Tras

void setup() {
  // Pinos sensores infravermelhos
  pinMode(pin_S1, INPUT);
  pinMode(pin_S2, INPUT);
  pinMode(pin_S3, INPUT);
  pinMode(pin_S4, INPUT);

  // Pinos sensor Ultrassônico
  // Configura pino de Trigger como saída e inicializa com nível baixo
  pinMode(PinTrigger, OUTPUT);
  digitalWrite(PinTrigger, LOW);
  pinMode(PinEcho, INPUT);  // configura pino ECHO como entrada

  // Setamos os pinos de controle dos motores como saída
  pinMode(M1, OUTPUT);
  pinMode(M2, OUTPUT);
  pinMode(dir1, OUTPUT);
  pinMode(dir2, OUTPUT);
  LerUltra();
}

void loop() {

  LerUltra();

  if (DistanciaUltra <= 15.0) {
    // Ultrassônico "detectou" um objeto a 15cm de distancia
    Desvia();
  }

  LerInfras();

  if (SensorEsq) {
    Esquerda();
  } else if (SensorDir) {
    Direita();
  } else {
    Frente1(v_Padrao);
    Frente2(v_Padrao);
  }

  delay(100);
}

/* --- Funções dos sensores Infravermelho ---*/
void LerInfras() {
  Sensor1 = digitalRead(pin_S1);
  Sensor2 = digitalRead(pin_S2);
  Sensor3 = digitalRead(pin_S3);
  Sensor4 = digitalRead(pin_S4);

  if (Sensor3 || Sensor4)  // Identificou na esquerda
    SensorEsq = true;
  else
    SensorEsq = false;

  if (Sensor2 || Sensor1)  // Identificou na direita
    SensorDir = true;
  else
    SensorDir = false;
}

void PrintInfras() {
  Serial.print("Sensor 1: ");
  Serial.println(Sensor1);
  Serial.print("Sensor 2: ");
  Serial.println(Sensor2);
  Serial.print("Sensor 3: ");
  Serial.println(Sensor3);
  Serial.print("Sensor 4: ");
  Serial.println(Sensor4);
  Serial.println("\n");
}

/* --------------- Funções do sensor Ultrassônico ---------------*/
void DisparaPulsoUltrassonico() {
  // Para fazer o HC-SR04 enviar um pulso Ultrassônico, nos temos
  // que enviar para o pino de trigger um sinal de nivel alto
  // com pelo menos 10us de duraçao
  digitalWrite(PinTrigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(PinTrigger, LOW);
}

float CalculaDistancia(float tempo_us) {  // Calcular a distancia em centimetros
  return (((tempo_us * VelocidadeSom_mporus) / 2) * 100);
}

void LerUltra() {
  DisparaPulsoUltrassonico();
  TempoEcho = pulseIn(PinEcho, HIGH);
  DistanciaUltra = (CalculaDistancia(TempoEcho));
}

void PrintUltra() {
  Serial.println("Distancia em centimetros: ");
  Serial.println(DistanciaUltra);
  Serial.println("\n");
}

/* ------ Funções dos Motores ------*/
void Frente1(int velocidade) {
  analogWrite(M1, velocidade);
  digitalWrite(dir1, LOW);
}

void Tras1(int velocidade) {
  digitalWrite(M1, LOW);
  analogWrite(dir1, velocidade);
}

void Frente2(int velocidade) {
  digitalWrite(M2, LOW);
  analogWrite(dir2, velocidade);
}

void Tras2(int velocidade) {
  analogWrite(M2, velocidade);
  digitalWrite(dir2, LOW);
}

void Parar() {  // Parar ambos os motores
  digitalWrite(M1, LOW);
  digitalWrite(dir1, LOW);
  digitalWrite(M2, LOW);
  digitalWrite(dir2, LOW);
}

void Esquerda() {
  Frente2(v_CurvaF);
  Tras1(v_CurvaT);
}

void Direita() {
  Frente1(v_CurvaF);
  Tras2(v_CurvaT);
}

void Desvia() {
  Tras1(v_Padrao);
  Tras2(v_Padrao);
  delay(1000);

  Direita();
  delay(1000);

  Frente1(v_Padrao);
  Frente2(v_Padrao);
  delay(2000);

  Esquerda();
  delay(1000);

  Frente1(v_Padrao);
  Frente2(v_Padrao);
  delay(2000);

  Esquerda();
  delay(1000);

  Frente1(v_Padrao);
  Frente2(v_Padrao);
  delay(500);
}
