import time
from multiprocessing import Process

from control import control_loop
from line_cam import line_cam_loop
from mp_manager import *

def main():
    print("==========================================")
    print("Iniciando Sistema: Line Cam + Control")
    print("==========================================")

    # Configurações iniciais do gerenciador de variáveis compartilhadas
    program_start_time.value = time.perf_counter()
    run.value = True # Inicia permitindo que o robô ande
    terminate.value = False

    # Define apenas os processos essenciais
    processes = [
        Process(target=line_cam_loop, args=(), name="Camera_Loop"),
        Process(target=control_loop, args=(), name="Motor_Control_Loop")
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
        
        # Derruba os processos
        for process in processes:
            process.terminate()
            process.join()
            
        print("[MAIN] Todos os processos foram encerrados. Fim.")

if __name__ == "__main__":
    main()