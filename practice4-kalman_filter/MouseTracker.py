import numpy as np
import cv2


class MouseTracker:

    def __init__(self):
        self.window_name: str = 'Mouse Tracker'
        self.img = np.zeros((600, 800, 3), dtype=np.uint8)
        self.running: bool = False
        # Measurement
        self.measurement: np.array = np.array([0, 0], dtype=np.float32)
        # Vector of points Measurements
        self.points: np.ndarray = np.array([self.measurement.astype(np.int32)], dtype=np.int32)
        # Vector of points Kalman
        self.points_kalman: np.ndarray = np.array([self.measurement.astype(np.int32)], dtype=np.int32)
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

    def mouse_callback(self, event: int, x: int, y: int, flags: int, param) -> None:
        if event == cv2.EVENT_MOUSEMOVE:
            self.measurement = np.array([x, y], dtype=np.float32)

    def run(self) -> None:
        # KalmanFilter
        kalman = cv2.KalmanFilter(4, 2)
        # delta time in milliseconds and seconds.
        dtm: float = 10
        dts: float = dtm * 10e-3
        # transitionMatrix
        kalman.transitionMatrix = np.array(
            [
                [1, 0, dts, 0],
                [0, 1, 0, dts],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ],
            dtype=np.float32)
        # measurementMatrix
        kalman.measurementMatrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=np.float32)
        # statePre
        kalman.statePre = np.array(
            [
                [0],  # x
                [0],  # y
                [0],  # dx
                [0],  # dy
            ],
            dtype=np.float32)
        # errorCovPre
        kalman.errorCovPre = np.eye(4, dtype=np.float32)
        # processNoiseCov
        kalman.processNoiseCov = np.identity(4, dtype=np.float32) * 1e-4
        # measurementNoiseCov
        kalman.measurementNoiseCov = np.identity(2, dtype=np.float32) * 10
        # errorCovPost
        kalman.errorCovPost = np.identity(4, dtype=np.float32) * .1

        self.running = True
        while self.running:
            # Predict
            prediction: np.ndarray = kalman.predict()
            # Correct
            kalman.correct(self.measurement)
            # Get the estimated position
            estimated_position: np.ndarray = kalman.statePost[:2]

            self.points = np.append(self.points, [self.measurement.astype(np.int32)], axis=0)
            self.points_kalman = np.append(self.points_kalman, [prediction.T[0][:2].astype(np.int32)], axis=0)

            # Draw
            if not np.array_equal(self.points[-2], self.points[-1]):
                cv2.line(self.img, self.points[-2], self.points[-1], (0, 0, 255), 2, cv2.LINE_AA, 0)
            if not np.array_equal(self.points_kalman[-2], self.points_kalman[-1]):
                cv2.line(self.img, self.points_kalman[-2], self.points_kalman[-1], (255, 0, 0), 1, cv2.LINE_AA, 0)

            # Show image
            cv2.imshow(self.window_name, self.img)
            # q to exit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False
        cv2.destroyAllWindows()


if __name__ == '__main__':
    mt: MouseTracker = MouseTracker()
    mt.run()
