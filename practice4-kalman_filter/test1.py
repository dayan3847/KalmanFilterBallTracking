import numpy as np
import cv2

# Inicializar el filtro de Kalman
kalman = cv2.KalmanFilter(4, 2)  # 4 estados (x, y, velocidad en x, velocidad en y), 2 observaciones (x, y)

# Definir las matrices de transición y medición (aquí, un ejemplo simple)
kalman.transitionMatrix = np.array([
    [1, 0, 1, 0],
    [0, 1, 0, 1],
    [0, 0, 1, 0],
    [0, 0, 0, 1]
], dtype=np.float32)

kalman.measurementMatrix = np.array([
    [1, 0, 0, 0],
    [0, 1, 0, 0]
], dtype=np.float32)

# Inicializar el estado y la covarianza
kalman.statePre = np.array([0, 0, 0, 0], dtype=np.float32)
kalman.errorCovPre = np.eye(4, dtype=np.float32)

# Simular mediciones (en un bucle real, esto vendría de sensores)
measurements = np.array([[1, 0], [0, 1], [2, 2]], dtype=np.float32)

for measurement in measurements:
    # Predicción
    prediction = kalman.predict()

    # Actualización basada en la medición actual
    kalman.correct(measurement)

    # Obtener la estimación actual
    estimated_position = kalman.statePost[:2]

    print("Predicción:", prediction[:2])
    print("Medición:", measurement)
    print("Estimación:", estimated_position)
    print("-" * 30)
