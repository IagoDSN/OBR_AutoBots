import time
from multiprocessing import Process

from control import control_loop
from line_cam import line_cam_loop
from mp_manager import *
from led_branco import led_branco_loop  # Já importado

def main():
    print("==========================================")
    print("Iniciando Sistema: Line Cam + Control + LED")
    print("==========================================")

    # Configurações iniciais do gerenciador de variáveis compartilhadas
    program_start_time.value = time.perf_counter()
    run.value = True # Inicia permitindo que o robô ande
    terminate.value = False

    # Adicionado o led_branco_loop na lista de processos ativos
    processes = [
        Process(target=line_cam_loop, args=(), name="Camera_Loop"),
        Process(target=control_loop, args=(), name="Motor_Control_Loop"),
        Process(target=led_branco_loop, args=(), name="LED_Branco_Loop")  # <--- Novo processo adicionado aqui
    ]

    # Inicia os processos paralelamente
    for process in processes:
        process.start()
        print(f"--> Processo {process.name} iniciado.")
        time.sleep(0.5)

    print("\nSistema rodando em background.")
    print("Pressione [CTRL + C] no terminal para parar o robô e encerrar o código.\n")

    # Mantém o arquivo main aberto rodando e escutando por CTRL+C
    try:
        while True:
            time.sleep(1)
            
    except KeyboardInterrupt:
        print("\n[MAIN] Comando de parada recebido (CTRL+C). Encerrando segurança...")
        
        # Sinaliza para os loops pararem
        terminate.value = True
        run.value = False
        time.sleep(0.5)
        
        # Derruba todos os processos (incluindo o do LED, desligando-o se o led_branco_loop tratar o KeyboardInterrupt/terminate)
        for process in processes:
            process.terminate()
            process.join()
            
        print("[MAIN] Todos os processos foram encerrados. Fim.")

if __name__ == "__main__":
    main()