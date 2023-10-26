import cv2
import numpy as np


def draw_cross(img, center, color, d):
    cv2.line(img, (center[0] - d, center[1] - d), (center[0] + d, center[1] + d), color, 2, cv2.LINE_AA, 0)
    cv2.line(img, (center[0] + d, center[1] - d), (center[0] - d, center[1] + d), color, 2, cv2.LINE_AA, 0)


def on_mouse_event(event, x, y, flags, param):
    global drawing

    if event == cv2.EVENT_MOUSEMOVE:
        drawing.insert(0, (x, y))


KalmanFilter = cv2.KalmanFilter
mousePos = np.zeros((2, 1), dtype=np.float32)
img = np.zeros((600, 800, 3), dtype=np.uint8)
mousev, kalmanv = [], []
dtm, dts = 10, 10e-3
n = 0
val = None

# Inicialización del filtro de Kalman
KF = KalmanFilter(4, 2, 0)
KF.transitionMatrix = np.array([[1, 0, dts, 0],
                                [0, 1, 0, dts],
                                [0, 0, 1, 0],
                                [0, 0, 0, 1]], dtype=np.float32)
measurement = np.zeros((2, 1), dtype=np.float32)
KF.statePre[:2] = mousePos
KF.processNoiseCov = np.identity(4, dtype=np.float32) * 1e-4
KF.measurementMatrix = np.identity(2, dtype=np.float32)
KF.measurementMatrix *= 1
KF.measurementNoiseCov = np.identity(2, dtype=np.float32) * 10
KF.errorCovPost = np.identity(4, dtype=np.float32) * 0.1

cv2.namedWindow("mouse points_kalman", cv2.WINDOW_AUTOSIZE)

while True:
    if len(mousev) > 1:
        prediction = KF.predict()
        predictPt = (int(prediction[0]), int(prediction[1]))
        measurement[0], measurement[1] = mousev[0][0], mousev[0][1]

    cv2.imshow("mouse points_kalman", img)

    if val == 27:
        break

    n += 1

cv2.destroyAllWindows()
