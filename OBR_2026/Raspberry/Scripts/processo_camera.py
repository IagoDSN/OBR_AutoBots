"""
Corpo do processo da camera. So se importa com visao computacional:
captura frame -> calcula erro/direcao/potencia -> escreve no estado
compartilhado. NAO sabe nada sobre motor.
"""

import time
import cv2
import line_cam

# Liga/desliga a janela de debug (cv2.imshow) mostrando as ROIs, os pontos
# detectados e o texto com direcao/erro/potencia por cima do frame ao vivo.
#
# IMPORTANTE: cv2.imshow precisa de um display de verdade (monitor ligado
# na Raspberry Pi, ou X11 forwarding via SSH com `ssh -X`). Rodando 100%
# headless (sem display nenhum), deixe False, senao o processo da camera
# vai travar/dar erro tentando abrir a janela.
MOSTRAR_DEBUG = True


def rodar(estado, evento_parar):
    """
    estado: instancia de EstadoCompartilhado (shared_state.py)
    evento_parar: multiprocessing.Event -- quando setado, o processo encerra
    """
    line_cam.iniciar_camera()
    print("[camera] camera iniciada")

    try:
        while not evento_parar.is_set():
            rgb_img = line_cam.capturar_frame()

            if rgb_img is None:
                print("[camera] camera nao encontrada, tentando de novo...")
                time.sleep(0.1)
                continue

            direcao, erro, potencia = line_cam.deteccao_erro(rgb_img)

            if direcao is None:
                # Linha nao encontrada -> avisa o motor pra parar, escrevendo
                # direcao=None explicitamente em vez de deixar o ultimo
                # comando "grudado" pra sempre.
                estado.atualizar(None, 0, 0.0)
            else:
                estado.atualizar(direcao, erro, potencia)

            if MOSTRAR_DEBUG:
                img_debug = line_cam.desenhar_debug(rgb_img, direcao, erro, potencia)
                cv2.imshow("debug - line_cam", img_debug)

                tecla = cv2.waitKey(1) & 0xFF
                if tecla == ord('q'):
                    # 'q' na janela de debug tambem derruba os dois processos
                    evento_parar.set()
                    break
    finally:
        if MOSTRAR_DEBUG:
            cv2.destroyAllWindows()
        line_cam.parar_camera()
        print("[camera] processo encerrado")
