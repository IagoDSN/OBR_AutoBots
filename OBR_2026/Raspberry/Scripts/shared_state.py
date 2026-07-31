

import multiprocessing as mp
import time


class EstadoCompartilhado:
    DIRECAO_PARADO = -1
    DIRECAO_RETO = 0
    DIRECAO_DIREITA = 1
    DIRECAO_ESQUERDA = 2

    _CODIGO_PARA_NOME = {
        DIRECAO_PARADO: None,
        DIRECAO_RETO: "reto",
        DIRECAO_DIREITA: "direita",
        DIRECAO_ESQUERDA: "esquerda",
    }
    _NOME_PARA_CODIGO = {v: k for k, v in _CODIGO_PARA_NOME.items()}

    def __init__(self):
        
        self._direcao = mp.Value('i', self.DIRECAO_PARADO)
        self._erro = mp.Value('d', 0.0)
        self._potencia = mp.Value('d', 0.0)
        self._timestamp = mp.Value('d', 0.0)
        self._lock = mp.Lock()

    def atualizar(self, direcao_str, erro, potencia):
        codigo = self._NOME_PARA_CODIGO.get(direcao_str, self.DIRECAO_PARADO)
        with self._lock:
            self._direcao.value = codigo
            self._erro.value = float(erro)
            self._potencia.value = float(potencia)
            self._timestamp.value = time.time()

    def ler(self):
       
        with self._lock:
            direcao_str = self._CODIGO_PARA_NOME[self._direcao.value]
            erro = self._erro.value
            potencia = self._potencia.value
            timestamp = self._timestamp.value
        return direcao_str, erro, potencia, timestamp
