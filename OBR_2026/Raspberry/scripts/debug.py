"""
Debug do pipeline de visao (deteccao de preto + ROI_CIMA/ESQUERDA_CIMA/DIREITA_CIMA)
usando webcam USB. NAO usa multiprocessing/shared_memory -- e so pra calibrar
threshold e posicao das ROIs num PC antes de rodar no Raspberry Pi com Picamera2.
"""
import cv2 as cv
import numpy as np

CAMERA_INDEX = 0  # troque se tiver mais de uma camera USB conectada

FRAME_WIDTH = 320
FRAME_HEIGHT = 200
MIN_CONTOUR_AREA = 80

# (x1, x2, y1, y2) -- as 3 tiles ocupam a faixa y=0..50 lado a lado
ROI_ESQUERDA_CIMA = (0,   60,  0, 50)
ROI_CIMA          = (60, 260,  0, 50)
ROI_DIREITA_CIMA  = (260, 320, 0, 50)

LINE_LOST = 0
LINE_FOUND = 1
STATUS_NOME = {LINE_LOST: "PERDIDA", LINE_FOUND: "OK"}


def mascara_preto(frame_rgb, thresh):
    """255 onde os 3 canais (R,G,B) estao abaixo do threshold -> pixel preto."""
    return np.all(frame_rgb < thresh, axis=2).astype(np.uint8) * 255


def detectar_centro(frame, roi, thresh):
    x1, x2, y1, y2 = roi
    recorte = frame[y1:y2, x1:x2]
    mask = mascara_preto(recorte, thresh)

    contours, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if not contours:
        return None, mask

    maior = max(contours, key=cv.contourArea)
    if cv.contourArea(maior) < MIN_CONTOUR_AREA:
        return None, mask

    M = cv.moments(maior)
    if M["m00"] == 0:
        return None, mask

    cx_local = int(M["m10"] / M["m00"])
    cy_local = int(M["m01"] / M["m00"])
    if mask[0, cx_local] == 255:
        cy_local = 0

    return (cx_local + x1, cy_local + y1), mask


def escolher_alvo(centro_cima, centro_esq, centro_dir, center_x):
    """Decide onde mirar com base nos 3 pontos de cima:
    - preto so na esquerda (+ centro) -> ramal esquerda, mira nela
    - preto so na direita  (+ centro) -> ramal direita, mira nela
    - preto nos dois lados ao mesmo tempo -> cruzamento, segue reto (usa o centro)
    - nenhum dos lados -> comportamento normal (centro)
    """
    cx_cima = centro_cima[0] if centro_cima is not None else None
    cx_esq = centro_esq[0] if centro_esq is not None else None
    cx_dir = centro_dir[0] if centro_dir is not None else None

    tem_cima = cx_cima is not None
    tem_esq = cx_esq is not None
    tem_dir = cx_dir is not None

    if tem_esq and tem_dir:
        alvo_x = cx_cima if tem_cima else center_x
        modo = "RETO (cruzamento)"
    elif tem_cima and tem_esq:
        alvo_x = cx_esq
        modo = "RAMAL ESQUERDA"
    elif tem_cima and tem_dir:
        alvo_x = cx_dir
        modo = "RAMAL DIREITA"
    elif tem_cima:
        alvo_x = cx_cima
        modo = "NORMAL"
    else:
        alvo_x = None
        modo = "PERDIDA"

    return alvo_x, modo


def nothing(_):
    pass


def main():
    cap = cv.VideoCapture(CAMERA_INDEX)
    if not cap.isOpened():
        print(f"ERRO: nao consegui abrir a camera USB no indice {CAMERA_INDEX}")
        print("Tente CAMERA_INDEX = 1, 2... se tiver mais de uma camera.")
        return

    cap.set(cv.CAP_PROP_FRAME_WIDTH, FRAME_WIDTH)
    cap.set(cv.CAP_PROP_FRAME_HEIGHT, FRAME_HEIGHT)

    cv.namedWindow("Debug")
    cv.createTrackbar("BLACK_THRESH", "Debug", 60, 255, nothing)

    center_x = FRAME_WIDTH // 2
    y1c, y2c = ROI_CIMA[2], ROI_CIMA[3]

    while True:
        ok, frame_bgr = cap.read()
        if not ok:
            print("ERRO: falha ao ler frame da camera")
            break

        # webcam entrega BGR; pipeline assume RGB (mesma convencao do Picamera2/RGB888)
        frame = cv.resize(frame_bgr, (FRAME_WIDTH, FRAME_HEIGHT))
        frame_rgb = cv.cvtColor(frame, cv.COLOR_BGR2RGB)

        thresh = cv.getTrackbarPos("BLACK_THRESH", "Debug")

        centro_cima, mask_cima = detectar_centro(frame_rgb, ROI_CIMA, thresh)
        centro_esq, mask_esq = detectar_centro(frame_rgb, ROI_ESQUERDA_CIMA, thresh)
        centro_dir, mask_dir = detectar_centro(frame_rgb, ROI_DIREITA_CIMA, thresh)

        alvo_x, modo = escolher_alvo(centro_cima, centro_esq, centro_dir, center_x)

        if alvo_x is not None:
            status = LINE_FOUND
            erro = alvo_x - center_x
        else:
            status = LINE_LOST
            erro = 0

        # --- desenho de debug (em BGR, pra cv.imshow ficar com cor certa) ---
        overlay = frame.copy()

        rois = [
            (ROI_ESQUERDA_CIMA, (0, 255, 255), "esq-cima"),
            (ROI_CIMA, (255, 0, 0), "cima"),
            (ROI_DIREITA_CIMA, (255, 0, 255), "dir-cima"),
        ]
        for (x1, x2, y1, y2), cor, nome in rois:
            cv.rectangle(overlay, (x1, y1), (x2, y2), cor, 1)
            cv.putText(overlay, nome, (x1 + 2, y1 + 12),
                       cv.FONT_HERSHEY_SIMPLEX, 0.35, cor, 1)

        # linha vertical no meio da tela (referencia do "erro zero")
        cv.line(overlay, (center_x, 0), (center_x, FRAME_HEIGHT), (200, 200, 200), 1)

        if centro_cima is not None:
            cv.circle(overlay, centro_cima, 3, (255, 0, 0), -1)
        if centro_esq is not None:
            cv.circle(overlay, centro_esq, 3, (0, 255, 255), -1)
        if centro_dir is not None:
            cv.circle(overlay, centro_dir, 3, (255, 0, 255), -1)

        if alvo_x is not None:
            alvo_pt = (alvo_x, y1c + (y2c - y1c) // 2)
            cv.circle(overlay, alvo_pt, 6, (0, 0, 255), 2)
            cv.line(overlay, (center_x, alvo_pt[1]), alvo_pt, (0, 200, 255), 1)

        cv.putText(overlay, f"status: {STATUS_NOME[status]}", (5, FRAME_HEIGHT - 34),
                   cv.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv.putText(overlay, f"modo: {modo}", (5, FRAME_HEIGHT - 20),
                   cv.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)
        cv.putText(overlay, f"erro: {erro} px", (5, FRAME_HEIGHT - 6),
                   cv.FONT_HERSHEY_SIMPLEX, 0.4, (255, 255, 255), 1)

        cv.imshow("Debug", overlay)

        # janela extra com as mascaras de preto das 3 ROIs (util pra calibrar o threshold)
        mask_view = np.zeros((FRAME_HEIGHT, FRAME_WIDTH), dtype=np.uint8)
        mask_view[y1c:y2c, ROI_ESQUERDA_CIMA[0]:ROI_ESQUERDA_CIMA[1]] = mask_esq
        mask_view[y1c:y2c, ROI_CIMA[0]:ROI_CIMA[1]] = mask_cima
        mask_view[y1c:y2c, ROI_DIREITA_CIMA[0]:ROI_DIREITA_CIMA[1]] = mask_dir
        cv.imshow("Mascara preto (esq-cima / cima / dir-cima)", mask_view)

        key = cv.waitKey(1) & 0xFF
        if key == ord('q'):
            break

    cap.release()
    cv.destroyAllWindows()


if __name__ == "__main__":
    main()
