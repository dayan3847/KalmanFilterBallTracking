import numpy as np
import cv2
import time


class MouseTracker:

    def __init__(self):
        self.window_name: str = 'Mouse Tracker'
        self.img = np.zeros((600, 800, 3), dtype=np.uint8)
        self.running: bool = False
        # Measurement
        self.z: np.array = np.array([0, 0], dtype=np.float32)
        # Vector of points Measurements
        self.points: np.ndarray = np.array([self.z.astype(np.int32)], dtype=np.int32)
        # Vector of points Kalman
        self.points_kalman: np.ndarray = np.array([self.z.astype(np.int32)], dtype=np.int32)
        cv2.namedWindow(self.window_name)
        cv2.setMouseCallback(self.window_name, self.mouse_callback)

        # delta time in milliseconds and seconds.
        # dtm: float = 10
        # dts: float = dtm * 10e-3
        self.dt: float = 1 / 60

        # KalmanFilter
        self.kalman = cv2.KalmanFilter(4, 2)

        # Transition Matrix
        self.kalman.transitionMatrix = np.array(
            [
                [1, 0, self.dt, 0],
                [0, 1, 0, self.dt],
                [0, 0, 1, 0],
                [0, 0, 0, 1],
            ],
            dtype=np.float32)
        # measurementMatrix
        self.kalman.measurementMatrix = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=np.float32)
        # statePre
        self.kalman.statePre = np.array(
            [
                [0],  # x
                [0],  # y
                [0],  # dx
                [0],  # dy
            ],
            dtype=np.float32)
        # errorCovPre
        self.kalman.errorCovPre = np.eye(4, dtype=np.float32)
        # processNoiseCov
        self.kalman.processNoiseCov = np.identity(4, dtype=np.float32) * 1e-4
        # measurementNoiseCov
        self.kalman.measurementNoiseCov = np.identity(2, dtype=np.float32) * 10
        # errorCovPost
        self.kalman.errorCovPost = np.identity(4, dtype=np.float32) * .1

    def mouse_callback(self, event: int, x: int, y: int, flags: int, param) -> None:
        if event == cv2.EVENT_MOUSEMOVE:
            self.z = np.array([x, y], dtype=np.float32)

    def run(self) -> None:
        self.running = True
        while self.running:
            # Predict
            prediction: np.ndarray = self.kalman.predict()
            # Correct
            self.kalman.correct(self.z)
            # Get the estimated position
            estimated_position: np.ndarray = prediction.T[0][:2].astype(np.int32)
            estimated_position_: np.ndarray = self.kalman.statePost.T[0][:2].astype(np.int32)

            self.points = np.append(self.points, [self.z.astype(np.int32)], axis=0)
            self.points_kalman = np.append(self.points_kalman, [estimated_position], axis=0)

            # Draw
            if not np.array_equal(self.points[-2], self.points[-1]):
                cv2.line(self.img, self.points[-2], self.points[-1], (0, 0, 255), 1, cv2.LINE_AA, 0)
            if not np.array_equal(self.points_kalman[-2], self.points_kalman[-1]):
                cv2.line(self.img, self.points_kalman[-2], self.points_kalman[-1], (255, 0, 0), 1, cv2.LINE_AA, 0)

            # Show image
            cv2.imshow(self.window_name, self.img)
            # sleep
            time.sleep(self.dt)
            # q to exit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False
        cv2.destroyAllWindows()


if __name__ == '__main__':
    mt: MouseTracker = MouseTracker()
    mt.run()
