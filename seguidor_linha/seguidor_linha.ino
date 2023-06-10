// Motores
#define M1 11    // Pino Motor Esquerda/Direita Porta IN2 ponte H
#define M2 9     // Pino Motor Esquerda/Direita Porta IN4 ponte H
#define dir1 5   // Pino Motor Esquerda/Direita Porta IN1 ponte H
#define dir2 10  // Pino Motor Esquerda/Direita Porta IN3 ponte H

// Infravermelho
#define pin_S1 13  // Mid
#define pin_S2 12  // Dir
#define pin_S3 8   // Esq

bool SensorEsq = 0;
bool SensorDir = 0;
bool SensorMid = 0;

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

int status = 0;  // 0 - Parado; 1 - Frente; 2 - Esquerda; 3 - Direita

void setup() {
  // Pinos sensores infravermelhos
  pinMode(pin_S1, INPUT);
  pinMode(pin_S2, INPUT);
  pinMode(pin_S3, INPUT);

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

  if (SensorDir)
    Direita();
  else if (SensorEsq)
    Esquerda();
  else if (SensorMid)
    Frente();
  else {
    Frente();
  }

  delay(100);
}

/* --- Funções dos sensores Infravermelho ---*/
void LerInfras() {
  SensorEsq = digitalRead(pin_S3);
  SensorDir = digitalRead(pin_S2);
  SensorMid = digitalRead(pin_S1);
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

  if (status != 0) {
    digitalWrite(M1, LOW);
    digitalWrite(dir1, LOW);
    digitalWrite(M2, LOW);
    digitalWrite(dir2, LOW);
    status = 0;
  }
}

void Esquerda() {
  if (status != 2) {
    Frente2(v_CurvaF);
    Tras1(v_CurvaT);
    status = 2;
  }
}

void Direita() {
  if (status != 3) {
    Frente1(v_CurvaF);
    Tras2(v_CurvaT);
    status = 3;
  }
}

void Frente() {
  if (status != 1)  // Se não já estava indo pra frente
  {
    Frente1(v_Padrao);
    Frente2(v_Padrao);
    status = 1;
  }
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

void procurar() {
  int count = 0;
  bool aux = true;

  while (!SensorDir && !SensorDir && !SensorMid) {
    if (count >= 3) {
      if (aux) {
        Direita();
        aux = false;
      } else {
        Esquerda();
        aux = true;
      }
      count = 0;
    }

    LerInfras();
    delay(100);
  }

  if (SensorMid) {
    Frente();
  } else if (SensorDir) {
    Direita();
    delay(100);
  } else {
    Esquerda();
    delay(100);
  }
}
