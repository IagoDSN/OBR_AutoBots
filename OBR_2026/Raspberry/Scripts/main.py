import multiprocessing as mp
import time

from shared_state import EstadoCompartilhado
import processo_camera
import processo_motor


def main():
    estado = EstadoCompartilhado()
    
    evento_parar = mp.Event()

    p_camera = mp.Process(
        target=processo_camera.rodar,
        args=(estado, evento_parar),
        name="processo-camera",
    )
    p_motor = mp.Process(
        target=processo_motor.rodar,
        args=(estado, evento_parar),
        name="processo-motor",
    )

    p_camera.start()
    p_motor.start()

    try:
        # processo principal so fica de olho se os filhos continuam vivos
        while p_camera.is_alive() and p_motor.is_alive():
            time.sleep(0.2)
    except KeyboardInterrupt:
        print("\n[main] Ctrl+C recebido, encerrando...")
    finally:
        evento_parar.set()
        p_camera.join(timeout=2)
        p_motor.join(timeout=2)

        # se algum processo nao encerrou sozinho a tempo, forca o fim
        if p_camera.is_alive():
            p_camera.terminate()
        if p_motor.is_alive():
            p_motor.terminate()

        print("[main] encerrado.")


if __name__ == "__main__":
    mp.set_start_method("fork")
    main()
