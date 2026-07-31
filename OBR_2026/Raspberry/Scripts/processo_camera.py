
import time
import line_cam


def rodar(estado, evento_parar):

    line_cam.iniciar_camera()
    print("[camera] câmera iniciada")

    try:
        while not evento_parar.is_set():
            rgb_img = line_cam.capturar_frame()

            if rgb_img is None:
                print("[camera] câmera não encontrada, tentando de novo...")
                time.sleep(0.1)
                continue

            direcao, erro, potencia = line_cam.deteccao_erro(rgb_img)

            if direcao is None:
                estado.atualizar(None, 0, 0.0)
                continue

            estado.atualizar(direcao, erro, potencia)
    finally:
        line_cam.parar_camera()
        print("[camera] processo encerrado")
