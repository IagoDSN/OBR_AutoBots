#include <Adafruit_NeoPixel.h>

#define PIN_NEOPIXEL 6
#define NUM_PIXELS   24

Adafruit_NeoPixel pixels(NUM_PIXELS, PIN_NEOPIXEL, NEO_GRB + NEO_KHZ800);

void setup() {
  Serial.begin(9600);
  pixels.begin();
  pixels.show();  // começa apagado
}

void loop() {
  if (Serial.available() > 0) {
    char cmd = Serial.read();
    if (cmd == '1') {
      pixels.fill(pixels.Color(255, 255, 255));
      pixels.show();
    } else if (cmd == '0') {
      pixels.clear();
      pixels.show();
    }
  }
}