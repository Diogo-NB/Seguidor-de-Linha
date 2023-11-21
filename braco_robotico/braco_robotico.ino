#include <Servo.h>

#define pin_eixo_vertical 9
#define pin_eixo_horizontal 10
#define pin_eixo_base 11
#define pin_eixo_garra 12

// Eixo do braço robótico
class Eixo {
public:
  int lim_inf;  // limite_inferior em graus
  int lim_sup;  // limite_superior em graus
  int pos;      // Posição do servo
  Servo s;      // Servo do eixo

  Eixo(int limite_inferior, int limite_superior) {  // Constructor with parameters
    lim_inf = limite_inferior;
    lim_sup = limite_superior;
  }

  void attachPin(int pin){
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

Servo s1;  // (esquerda), varia bem entre 120 e 175 ,abaixa, sobe
Servo s2;  // (direita) varia bem entre 70 e 140, extende, contrai
Servo s3;  // (base) varia bem entre 30 e 150, anti-horario, horario
Servo s4;  // (garra) varia bem entre 0 e 90, abre, fecha

Eixo vertical = Eixo(120, 175);
Eixo horizontal = Eixo(90, 150);
Eixo base = Eixo(30, 150);
Eixo garra = Eixo(0, 90);

void setup() {
  vertical.attachPin(pin_eixo_vertical);
  horizontal.attachPin(pin_eixo_horizontal);
  base.attachPin(pin_eixo_base);
  garra.attachPin(pin_eixo_garra);

  Serial.begin(9600);

  delay(500);
}

void teste(Eixo e) {
  e.movef(e.lim_inf);
  delay(750);
  e.movef(e.lim_sup);
  delay(750);
  e.movef(e.lim_inf);
  delay(750);
}

void loop() {

  Serial.print("Testando a base, posição inicial: ");
  Serial.println(base.pos);
  Serial.println(base.s.read());

  teste(base);
  delay(500);

  abrirGarra();
  delay(500);
  fecharGarra();
  delay(500);

  teste(horizontal);
  delay(500);

  teste(vertical);
  delay(500);

}

// Garra - s4
void abrirGarra() {
  garra.movef(garra.lim_inf);
  delay(500);
}

// Garra - s4
void fecharGarra() {
  garra.movef(garra.lim_sup);
  delay(500);
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