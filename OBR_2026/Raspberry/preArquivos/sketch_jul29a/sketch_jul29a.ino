#include <Adafruit_NeoPixel.h>

#define LED_PIN     7
#define LED_COUNT   24

Adafruit_NeoPixel strip(LED_COUNT, LED_PIN, NEO_GRB + NEO_KHZ800);

void setup() {
  strip.begin();
  strip.setBrightness(255);   // Brilho máximo correto (0 a 255)

  // Acende todos os LEDs em branco puro (R:255, G:255, B:255)
  for (int i = 0; i < LED_COUNT; i++) {
    strip.setPixelColor(i, strip.Color(255, 255, 255));
  }

  strip.show();
}

void loop() {
  // Nada a fazer
}
