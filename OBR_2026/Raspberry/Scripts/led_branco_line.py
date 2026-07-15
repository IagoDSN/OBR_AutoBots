import time
import sys
from rpi_ws281x import PixelStrip, Color

# ====== CONFIGURAÇÃO DO NEOPIXEL ======
LED_COUNT = 24        # Número de LEDs no seu anel
# ATENÇÃO: No Raspberry Pi, o pino mais estável para NeoPixel é o GPIO 18 (PWM)
LED_PIN = 18          
LED_FREQ_HZ = 800000  # Frequência do sinal (800khz)
LED_DMA = 10          # Canal DMA para gerar o sinal
LED_BRIGHTNESS = 150  # Brilho (0 a 255)
LED_INVERT = False    # Inverter sinal (necessário se usar transistor NPN)
LED_CHANNEL = 0       # Canal de PWM

# Inicializa a fita de LED
strip = PixelStrip(LED_COUNT, LED_PIN, LED_FREQ_HZ, LED_DMA, LED_INVERT, LED_BRIGHTNESS, LED_CHANNEL)
strip.begin()

def set_all_white():
    """Acende todos os LEDs em Branco Puro"""
    # Color(R, G, B) -> Branco é (255, 255, 255)
    white_color = Color(255, 255, 255)
    for i in range(strip.numPixels()):
        strip.setPixelColor(i, white_color)
    strip.show()

# ====== LOOP PRINCIPAL ======
def main():
    print("Ligando o anel de NeoPixel em BRANCO infinitamente...")
    set_all_white()
    
    try:
        # Mantém o programa rodando infinitamente com o LED aceso
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        # Desliga os LEDs se você fechar o programa com Ctrl+C
        print("\nDesligando NeoPixels...")
        for i in range(strip.numPixels()):
            strip.setPixelColor(i, Color(0, 0, 0))
        strip.show()

if __name__ == "__main__":
    main()