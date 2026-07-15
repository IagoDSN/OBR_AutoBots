import time
import board
import neopixel_spi as neopixel
from mp_manager import terminate  # Importa a variável compartilhada para saber quando parar

# Configurações do NeoPixel
NUM_PIXELS = 10                  # Altere para a quantidade real de LEDs que você tem
PIXEL_ORDER = neopixel.GRB       # Ordem de cores do seu LED (geralmente GRB)

# Inicializa o SPI no pino GPIO 10 (MOSI)
spi = board.SPI()
pixels = neopixel.NeoPixel_SPI(spi, NUM_PIXELS, pixel_order=PIXEL_ORDER, auto_write=False)

def led_branco_loop():
    print("[LED] Processo de LED iniciado. LEDs acesos em BRANCO.")
    try:
        # Em vez de True, usamos a variável de controle compartilhada do robô
        while not terminate.value:
            pixels.fill((255, 255, 255))
            pixels.show()
            time.sleep(0.5)  # Intervalo para não sobrecarregar o processador
            
    except Exception as e:
        print(f"[LED] Ocorreu um erro no loop do LED: {e}")
        
    finally:
        # Este bloco SEMPRE rodará quando o loop acima terminar (ao mudar 'terminate' para True)
        print("[LED] Desligando os LEDs com segurança...")
        pixels.fill((0, 0, 0))
        pixels.show()
        print("[LED] LEDs desligados.")

# IMPORTANTE: Este bloco garante que o loop só roda sozinho se executares
# diretamente "python3 led_branco.py". Quando importado pelo main.py, ele NÃO roda.
if __name__ == "__main__":
    try:
        led_branco_loop()
    except KeyboardInterrupt:
        pass