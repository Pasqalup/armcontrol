"""Detection and multi-object tracking primitives."""

import numpy as np


class KalmanFilter:
    """Discrete constant-velocity Kalman filter for (x, y, vx, vy).

    State vector        : x_k = [x, y, vx, vy]
    Transition          : x_{k+1} = F @ x_k
    Measurement         : z_k = H @ x_k  (we observe x and y only)
    Prediction step     : x' = F x,  P' = F P F^T + Q
    Update step         : K = P' H^T (H P' H^T + R)^-1,
                          x = x' + K (z - H x'),
                          P = (I - K H) P'
    """

    def __init__(self, measurement_noise=1.0, process_noise=0.1):
        self._dt = 1.0  # timestep placeholder; real dt applied in updates
        self.F = None  # rebuilt per-dt by _build_matrices
        self.H = np.array([[1, 0, 0, 0],
                           [0, 1, 0, 0]], dtype=float)
        self.Q = np.eye(4) * process_noise
        self.R = np.eye(2) * measurement_noise
        self.x = np.zeros(4)
        self.P = np.eye(4) * 10.0  # optimistic initial covariance
        self._initialized = False

    def _build_matrices(self, dt):
        """Constant velocity state transition for a given dt."""
        F = np.array([[1, 0, dt, 0],
                      [0, 1, 0, dt],
                      [0, 0, 1, 0],
                      [0, 0, 0, 1]], dtype=float)
        return F

    def reset(self, x, y, dt):
        """Start the filter from a first measurement."""
        self.x = np.array([x, y, 0.0, 0.0], dtype=float)
        self.P = np.eye(4) * 10.0
        self._initialized = True
        self.last_update_dt = dt

    def predict(self, dt):
        """Advance the state estimate by dt (used before measurement)."""
        if not self._initialized:
            return self.x
        F = self._build_matrices(dt)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q
        return self.x

    def update(self, x, y, dt):
        """Correct the state using a measured (x, y) position."""
        self.last_update_dt = dt
        F = self._build_matrices(dt)
        self.x = F @ self.x
        self.P = F @ self.P @ F.T + self.Q
        return self._correct(np.array([x, y]))

    def _correct(self, z):
        """Apply the measurement update for a 2-vector measurement."""
        innovation = z - self.H @ self.x
        S = self.H @ self.P @ self.H.T + self.R
        # K = P H^T S^-1
        K = self.P @ self.H.T @ np.linalg.inv(S)
        self.x = self.x + K @ innovation
        self.P = (np.eye(4) - K @ self.H) @ self.P
        return self.x

    def estimate_position(self):
        """Return the filtered (x, y)."""
        return float(self.x[0]), float(self.x[1])

    def estimate_velocity(self):
        """Return the filtered (vx, vy)."""
        return float(self.x[2]), float(self.x[3])


class DetectedObject:
    """A single object detected in one video frame."""

    def __init__(self, x, y, radius):
        self.x = float(x)
        self.y = float(y)
        self.radius = float(radius)


def parse_centers(centers):
    """Convert a flat ``[x, y, radius, ...]`` list into detections."""
    if len(centers) % 3 != 0:
        raise ValueError("centers must contain x, y, radius triples")

    return [
        DetectedObject(centers[i], centers[i + 1], centers[i + 2])
        for i in range(0, len(centers), 3)
    ]


class TrackedObject:
    """An object track with a smoothed constant-velocity estimate."""

    def __init__(self, object_id, detection, timestamp):
        self.id = object_id
        self.x = detection.x
        self.y = detection.y
        self.radius = detection.radius
        self.vx = 0.0
        self.vy = 0.0
        self.last_seen_time = timestamp
        self.missed_frames = 0
        self.hits = 1
        self.confirmed = False

        self.predicted_x = self.x
        self.predicted_y = self.y

    def predict(self, time):
        """Return this object's ``(x, y)`` position after ``time`` seconds.

        The prediction uses the most recently estimated constant velocity.
        ``time`` is relative to the object's latest observed position and must
        be non-negative.
        """
        if time < 0:
            raise ValueError("time must be non-negative")

        return self.x + self.vx * time, self.y + self.vy * time

    def _predict_at(self, timestamp):
        """Update the internal prediction used to match a new frame."""
        dt = max(0.0, timestamp - self.last_seen_time)
        self.predicted_x, self.predicted_y = self.predict(dt)

    def update(self, detection, timestamp, velocity_alpha):
        dt = timestamp - self.last_seen_time

        if dt > 0.0:
            measured_vx = (detection.x - self.x) / dt
            measured_vy = (detection.y - self.y) / dt
            self.vx = (
                velocity_alpha * measured_vx
                + (1.0 - velocity_alpha) * self.vx
            )
            self.vy = (
                velocity_alpha * measured_vy
                + (1.0 - velocity_alpha) * self.vy
            )

        self.x = detection.x
        self.y = detection.y
        self.radius = 0.7 * self.radius + 0.3 * detection.radius
        self.last_seen_time = timestamp
        self.missed_frames = 0
        self.hits += 1


class KalmanTrackedObject:
    """A track whose (x, y, vx, vy) are produced by a Kalman filter."""

    def __init__(self, object_id, detection, timestamp, kf=None):
        self.id = object_id
        self.radius = detection.radius

        self.kf = kf if kf is not None else KalmanFilter()
        self.kf.reset(detection.x, detection.y, 0.0)

        self.x, self.y = self.kf.estimate_position()
        self.vx, self.vy = self.kf.estimate_velocity()

        self.last_seen_time = timestamp
        self.missed_frames = 0
        self.hits = 1
        self.confirmed = False

        self.predicted_x = self.x
        self.predicted_y = self.y

    def predict(self, time):
        """Return filtered ``(x, y)`` extrapolated ``time`` seconds ahead."""
        if time < 0:
            raise ValueError("time must be non-negative")
        return (
            self.x + self.vx * time,
            self.y + self.vy * time,
        )

    def _predict_at(self, timestamp):
        """Internal: where the track expects the target next frame.

        Pure extrapolation from the current filter state; does not commit
        a prediction to the Kalman filter (only update() does predict+correct).
        """
        dt = max(0.0, timestamp - self.last_seen_time)
        self.predicted_x, self.predicted_y = self.predict(dt)

    def update(self, detection, timestamp, velocity_alpha=None):
        """Update the filter with a new measurement."""
        dt = max(1e-9, timestamp - self.last_seen_time)
        self.kf.update(detection.x, detection.y, dt)
        self.x, self.y = self.kf.estimate_position()
        self.vx, self.vy = self.kf.estimate_velocity()
        self.radius = 0.7 * self.radius + 0.3 * detection.radius
        self.last_seen_time = timestamp
        self.missed_frames = 0
        self.hits += 1


class ObjectTracker:
    """Greedy one-to-one tracker for detections across video frames."""

    def __init__(
        self,
        max_distance=40.0,
        max_missed_frames=5,
        confirmation_hits=2,
        velocity_alpha=0.4,
        radius_weight=0.25,
        max_speed=1000.0,
        use_kalman=False,
    ):
        self.tracks = []
        self.next_id = 1
        self.max_distance = max_distance
        self.max_missed_frames = max_missed_frames
        self.confirmation_hits = confirmation_hits
        self.velocity_alpha = velocity_alpha
        self.radius_weight = radius_weight
        self.max_speed = max_speed
        self.use_kalman = use_kalman
        self.last_timestamp = None

    def update(self, detections, timestamp):
        """Update tracks from one frame and return the current tracks."""
        if self.last_timestamp is not None and timestamp < self.last_timestamp:
            raise ValueError("timestamp must be monotonic")

        self.last_timestamp = timestamp

        for track in self.tracks:
            track._predict_at(timestamp)

        candidates = []
        for ti, track in enumerate(self.tracks):
            for di, detection in enumerate(detections):
                dx = detection.x - track.predicted_x
                dy = detection.y - track.predicted_y
                distance_sq = dx * dx + dy * dy

                if distance_sq > self.max_distance ** 2:
                    continue

                radius_scale = max(track.radius, 1.0)
                radius_difference = (
                    (detection.radius - track.radius) / radius_scale
                )
                cost = (
                    distance_sq
                    + self.radius_weight
                    * self.max_distance ** 2
                    * radius_difference ** 2
                )
                candidates.append((cost, ti, di))

        candidates.sort(key=lambda candidate: candidate[0])
        matched_tracks = set()
        matched_detections = set()

        for _, ti, di in candidates:
            if ti in matched_tracks or di in matched_detections:
                continue

            self.tracks[ti].update(
                detections[di], timestamp, self.velocity_alpha
            )
            matched_tracks.add(ti)
            matched_detections.add(di)

        for ti, track in enumerate(self.tracks):
            if ti not in matched_tracks:
                track.missed_frames += 1

        self.tracks = [
            track
            for track in self.tracks
            if track.missed_frames <= self.max_missed_frames
        ]

        for di, detection in enumerate(detections):
            if di in matched_detections:
                continue

            track_cls = KalmanTrackedObject if self.use_kalman else TrackedObject
            track = track_cls(self.next_id, detection, timestamp)
            self.next_id += 1
            if self.confirmation_hits <= 1:
                track.confirmed = True
            self.tracks.append(track)

        for track in self.tracks:
            if track.hits >= self.confirmation_hits:
                track.confirmed = True

        return list(self.tracks)

    def get_confirmed_tracks(self):
        """Return confirmed tracks, including temporarily missing tracks."""
        return [track for track in self.tracks if track.confirmed]
