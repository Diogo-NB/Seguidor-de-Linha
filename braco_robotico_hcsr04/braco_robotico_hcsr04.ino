#include <Servo.h>

// Servos motores
#define pin_eixo_vertical 9
#define pin_eixo_horizontal 10
#define pin_eixo_base 11
#define pin_eixo_garra 12

#define defaultDelay 500

// Ultrassonico
#define pin_trigger A0  // Pino usado para disparar os pulsos do sensor
#define pin_echo A1     // Pino usado para ler a saida do sensor

float TempoEcho = 0;
const float VelocidadeSom_mpors = 340;        // em metros por segundo
const float VelocidadeSom_mporus = 0.000340;  // em metros por microsegundo
float DistanciaUltra = 0.0;

// Um eixo do braço robótico
class Eixo {
public:
  int lim_inf;  // limite_inferior em graus
  int lim_sup;  // limite_superior em graus
  int pos;      // Posição do servo
  Servo s;      // Servo do eixo

  Eixo() {
    lim_inf = 0;
    lim_sup = 180;
  }

  Eixo(int limite_inferior, int limite_superior) {
    lim_inf = limite_inferior;
    lim_sup = limite_superior;
  }

  void attachPin(int pin) {
    s.attach(pin);
    s.write(90);
    pos = s.read();
  }

  void movef(int to) {
    pos = s.read();
    int from = pos;

    if (from == to)
      return;

    if (to > lim_sup)
      to = lim_sup;
    else if (to < lim_inf)
      to = lim_inf;

    if (from < to) {
      for (pos = from; pos <= to; pos += 1) {
        s.write(pos);
        delay(15);
      }
    } else {
      for (pos = from; pos >= to; pos -= 1) {
        s.write(pos);
        delay(15);
      }
    }

    pos = s.read();
  }
};

/* Declaração dos servos (usado antes de implementar a classe)
Servo s1;  // (esquerda), varia bem entre 120 e 175 ,abaixa, sobe
Servo s2;  // (direita) varia bem entre 70 e 140, extende, contrai
Servo s3;  // (base) varia bem entre 30 e 150, anti-horario, horario
Servo s4;  // (garra) varia bem entre 0 e 90, abre, fecha
*/

// Criação dos eixos
Eixo vertical = Eixo(85, 100);    // (esquerda), varia bem entre 85 e 100 ,abaixa, sobe
Eixo horizontal = Eixo(90, 180);  // (direita) varia bem entre 90 e 180, extende, contrai
Eixo base = Eixo(30, 150);        // (base) varia bem entre 30 e 150, anti-horario, horario
Eixo garra = Eixo(0, 100);        // (garra) varia bem entre 0 e 100, abre, fecha

void setup() {
  // Setando os pinos para os eixos
  vertical.attachPin(pin_eixo_vertical);
  horizontal.attachPin(pin_eixo_horizontal);
  base.attachPin(pin_eixo_base);
  garra.attachPin(pin_eixo_garra);

  // Inicializando os eixos
  abrirGarra();
  base.movef(100);  // Centraliza

  // Pinos sensor Ultrassônico
  // Configura pino de Trigger como saída e inicializa com nível baixo
  pinMode(pin_trigger, OUTPUT);
  digitalWrite(pin_trigger, LOW);
  pinMode(pin_echo, INPUT);  // configura pino ECHO como entrada
  
  Serial.begin(9600);
}

void loop() {
  LerUltra();
  PrintUltra();
  delay(500);

  /*
  // Desce
  horizontal.movef(150);
  delay(defaultDelay);

  // Vertical
  vertical.movef(vertical.lim_sup);
  delay(defaultDelay);

  fecharGarra();

  // Vertical
  vertical.movef(vertical.lim_inf);
  delay(defaultDelay);

  // Sobe
  horizontal.movef(horizontal.lim_inf);
  delay(defaultDelay);

  // Base - Direita
  base.movef(base.lim_inf);
  delay(defaultDelay);

  // Desce
  horizontal.movef(150);
  delay(defaultDelay);

  abrirGarra();

  // Sobe
  horizontal.movef(horizontal.lim_inf);
  delay(defaultDelay);

  // Centraliza
  base.movef(100);
  delay(defaultDelay);
  */
}

/* --------------- Funções dos eixos/servos ---------------*/

// Testando os limites de um eixo
void teste(Eixo e) {
  e.movef(e.lim_inf);
  delay(750);
  e.movef(e.lim_sup);
  delay(750);
  e.movef(e.lim_inf);
  delay(750);
}

// Eixo garra
void abrirGarra() {
  garra.movef(garra.lim_inf);
  delay(defaultDelay);
}

void fecharGarra() {
  garra.movef(garra.lim_sup);
  delay(defaultDelay);
}

void move(Servo s, int from, int to) {
  int pos;

  if (from < 0)
    from = 0;
  else if (from > 180)
    from = 180;

  if (to > 180)
    to = 180;
  else if (to < 0)
    to = 0;

  if (from == to)
    return;

  if (from < to) {
    for (pos = from; pos <= to; pos += 1) {
      s.write(pos);
      //Serial.println(s.read());
      delay(15);
    }
  } else {
    for (pos = from; pos >= to; pos -= 1) {
      s.write(pos);
      //Serial.println(s.read());
      delay(15);
    }
  }
}

/* --------------- Funções do sensor Ultrassônico ---------------*/
void DisparaPulsoUltrassonico() {
  // Para fazer o HC-SR04 enviar um pulso Ultrassônico, nos temos
  // que enviar para o pino de trigger um sinal de nivel alto
  // com pelo menos 10us de duraçao
  digitalWrite(pin_trigger, HIGH);
  delayMicroseconds(10);
  digitalWrite(pin_trigger, LOW);
}

float CalculaDistancia(float tempo_us) {  // Calcular a distancia em centimetros
  return (((tempo_us * VelocidadeSom_mporus) / 2) * 100);
}

void LerUltra() {
  DisparaPulsoUltrassonico();
  TempoEcho = pulseIn(pin_echo, HIGH);
  DistanciaUltra = (CalculaDistancia(TempoEcho));
}

void PrintUltra() {
  Serial.println("Distancia em centimetros: ");
  Serial.println(DistanciaUltra);
  Serial.println("\n");
}