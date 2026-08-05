const int RPWM = 5;
const int LPWM = 6;
const int REN  = 7;
const int LEN  = 8;

void setup() {
  pinMode(RPWM, OUTPUT);
  pinMode(LPWM, OUTPUT);
  pinMode(REN, OUTPUT);
  pinMode(LEN, OUTPUT);

  // Habilita a BTS7960
  digitalWrite(REN, HIGH);
  digitalWrite(LEN, HIGH);

  // Motor parado
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);
}

void loop() {

  // =========================
  // GIRA PARA UM LADO
  // =========================
  analogWrite(LPWM, 0);
  analogWrite(RPWM, 150);

  delay(3000);

  // =========================
  // PARA
  // =========================
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);

  delay(2000);

  // =========================
  // GIRA PARA O OUTRO LADO
  // =========================
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 150);

  delay(3000);

  // =========================
  // PARA
  // =========================
  analogWrite(RPWM, 0);
  analogWrite(LPWM, 0);

  delay(2000);
}
