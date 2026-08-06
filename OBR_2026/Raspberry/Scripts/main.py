import time
import cv2 as cv
import numpy as np
from multiprocessing import Process, Value, Lock
from multiprocessing.shared_memory import SharedMemory

from line_cam import (
    capturar_e_processar, FRAME_SHAPE, FRAME_NBYTES, LINE_LOST, DEBUG
)
from control import controlar_motores


def main():
    shm = SharedMemory(create=True, size=FRAME_NBYTES)
    frame_lock = Lock()
    novo_frame_flag = Value('i', 0)
    camera_ok = Value('i', 0)

    line_status = Value('i', LINE_LOST)
    line_angle = Value('d', 0.0)
    cx_alvo_v = Value('i', -1)
    stop_flag = Value('i', 0)

    p_cam = Process(
        target=capturar_e_processar,
        args=(shm.name, frame_lock, novo_frame_flag, camera_ok,
              line_status, line_angle, cx_alvo_v),
        daemon=True
    )
    p_cam.start()

    for _ in range(50):
        if camera_ok.value != 0:
            break
        time.sleep(0.05)

    if camera_ok.value == 0:
        print("ERRO: nao consegui iniciar a Picamera2.")
        print("Verifique a fita flex, se 'picamera2' esta instalado e se")
        print("nenhum outro processo esta usando a camera.")
        p_cam.terminate()
        p_cam.join()
        shm.close()
        shm.unlink()
        raise SystemExit(1)

    p_ctrl = Process(
        target=controlar_motores,
        args=(camera_ok, line_status, line_angle, stop_flag),
        daemon=True
    )
    p_ctrl.start()

    frame_view = np.ndarray(FRAME_SHAPE, dtype=np.uint8, buffer=shm.buf)

    try:
        while True:
            if camera_ok.value == 0:
                print("ERRO: a camera desconectou / parou de responder")
                break

            if novo_frame_flag.value:
                with frame_lock:
                    frame_local = frame_view.copy()
                    novo_frame_flag.value = 0
                if DEBUG:
                    cv.imshow("Linha", cv.cvtColor(frame_local, cv.COLOR_RGB2BGR))
                    if cv.waitKey(1) & 0xFF == ord('q'):
                        break

            time.sleep(0.05)
    except KeyboardInterrupt:
        pass
    finally:
        stop_flag.value = 1
        p_ctrl.join(timeout=2)
        if p_ctrl.is_alive():
            p_ctrl.terminate()
            p_ctrl.join()

        p_cam.terminate()
        p_cam.join()
        shm.close()
        shm.unlink()
        if DEBUG:
            cv.destroyAllWindows()


if __name__ == "__main__":
    main()