from multiprocessing import Value, Lock, Event
from multiprocessing.shared_memory import SharedMemory

from constants import FRAME_NBYTES, LINE_LOST

# Sinal global de parada — todos os processos leem, só o main.py escreve
terminate = Event()

# Memória compartilhada do frame de câmera
shm = SharedMemory(create=True, size=FRAME_NBYTES)
frame_lock = Lock()
novo_frame_flag = Value('i', 0)

# Estado da linha, publicado pelo line_cam e lido pelo control
camera_ok = Value('i', 0)
line_status = Value('i', LINE_LOST)
line_angle = Value('d', 0.0)
cx_alvo_v = Value('i', 0)