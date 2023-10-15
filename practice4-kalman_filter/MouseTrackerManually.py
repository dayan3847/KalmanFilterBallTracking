import numpy as np
import cv2
import time


class MouseTrackerManually:

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

        # delta time
        self.dt: float = 1 / 60
        # Transition Matrix (A)
        self.A = np.array([
            [1, 0, self.dt, 0],
            [0, 1, 0, self.dt],
            [0, 0, 1, 0],
            [0, 0, 0, 1],
        ], dtype=np.float32)
        # measurementMatrix (H)
        self.H = np.array([
            [1, 0, 0, 0],
            [0, 1, 0, 0]
        ], dtype=np.float32)
        # state vector (x)
        self.x = np.array([
            [0],  # x
            [0],  # y
            [0],  # dx
            [0],  # dy
        ], dtype=np.float32)
        # errorCovPre (P)
        self.P = np.eye(4, dtype=np.float32)
        # processNoiseCov (Q)
        self.Q = np.identity(4, dtype=np.float32) * 1e-4
        # measurementNoiseCov (R)
        self.R = np.identity(2, dtype=np.float32) * 10

    def mouse_callback(self, event: int, x: int, y: int, flags: int, param) -> None:
        if event == cv2.EVENT_MOUSEMOVE:
            self.z = np.array([x, y], dtype=np.float32)

    def run(self) -> None:
        self.running = True
        while self.running:
            # Predict
            self.x = np.dot(self.A, self.x)
            self.P = np.dot(np.dot(self.A, self.P), self.A.T) + self.Q
            # Correct
            # Get the z (z)
            # Get the z error
            y = self.z - np.dot(self.H, self.x)
            # Get the z error covariance
            S = np.dot(np.dot(self.H, self.P), self.H.T) + self.R
            # Get the Kalman gain
            K = np.dot(np.dot(self.P, self.H.T), np.linalg.inv(S))
            # Update the state estimate
            self.x = self.x + np.dot(K, y)
            # Update the covariance
            self.P = np.dot((np.eye(4, dtype=np.float32) - np.dot(K, self.H)), self.P)

            # Get the estimated position
            estimated_position: np.ndarray = self.x.T[0][:2].astype(np.int32)

            self.points = np.append(self.points, [self.z.astype(np.int32)], axis=0)
            self.points_kalman = np.append(self.points_kalman, [estimated_position], axis=0)

            # Draw
            if not np.array_equal(self.points[-2], self.points[-1]):
                cv2.line(self.img, tuple(self.points[-2]), tuple(self.points[-1]), (0, 0, 255), 1, cv2.LINE_AA, 0)
            if not np.array_equal(self.points_kalman[-2], self.points_kalman[-1]):
                cv2.line(self.img, tuple(self.points_kalman[-2]), tuple(self.points_kalman[-1]), (255, 0, 0), 1,
                         cv2.LINE_AA, 0)

            # Show image
            cv2.imshow(self.window_name, self.img)
            # sleep
            time.sleep(self.dt)
            # q to exit
            if cv2.waitKey(1) & 0xFF == ord('q'):
                self.running = False
        cv2.destroyAllWindows()


if __name__ == '__main__':
    mt: MouseTrackerManually = MouseTrackerManually()
    mt.run()
