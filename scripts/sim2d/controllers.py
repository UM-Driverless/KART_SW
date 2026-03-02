"""Controllers: geometric, neural net v1, v2, and v3."""

import math
import numpy as np

HALF_TRACK_WIDTH = 1.5  # fixed physical constant (m)


# ── Geometric controller ──────────────────────────────────────────────────

class GeometricController:
    """Same algorithm as ``cone_follower_node.py``, parameterised by 6 genes.

    Genes
    -----
    0  steering_gain      [0.1, 5.0]
    1  max_steer          [0.1, 1.0]   rad
    2  max_speed          [1.0, 5.0]   m/s
    3  min_speed          [0.1, 2.0]   m/s
    4  lookahead_max      [3.0, 20.0]  m
    5  speed_curve_factor [0.0, 3.0]
    """

    NUM_GENES = 6

    DEFAULTS = np.array([1.0, 0.5, 2.0, 0.5, 15.0, 1.0])
    RANGES = np.array([
        [0.1, 5.0],
        [0.1, 1.0],
        [1.0, 5.0],
        [0.1, 2.0],
        [3.0, 20.0],
        [0.0, 3.0],
    ])

    def __init__(self, genes):
        g = np.asarray(genes, dtype=np.float64)
        self.genes = g
        self.steering_gain = float(g[0])
        self.max_steer = float(g[1])
        self.max_speed = float(g[2])
        self.min_speed = float(g[3])
        self.lookahead_max = float(g[4])
        self.speed_curve_factor = float(g[5])
        self._last_steer = 0.0

    def reset(self):
        self._last_steer = 0.0

    def control(self, visible_cones):
        """Return ``(steer, speed)`` given cones in the optical frame."""
        nearest_blue = None
        nearest_yellow = None
        min_bd = float("inf")
        min_yd = float("inf")

        for cls, x, y, _z in visible_cones:
            fwd = x
            left = y

            if fwd < 0.5:
                continue
            dist = math.hypot(fwd, left)
            if dist > self.lookahead_max:
                continue

            if cls == "blue_cone" and dist < min_bd:
                min_bd = dist
                nearest_blue = (fwd, left)
            elif cls == "yellow_cone" and dist < min_yd:
                min_yd = dist
                nearest_yellow = (fwd, left)

        # Midpoint (same logic as cone_follower_node.py)
        if nearest_blue and nearest_yellow:
            mid_f = (nearest_blue[0] + nearest_yellow[0]) / 2.0
            mid_l = (nearest_blue[1] + nearest_yellow[1]) / 2.0
        elif nearest_blue:
            mid_f = nearest_blue[0]
            mid_l = nearest_blue[1] - HALF_TRACK_WIDTH
        elif nearest_yellow:
            mid_f = nearest_yellow[0]
            mid_l = nearest_yellow[1] + HALF_TRACK_WIDTH
        else:
            return self._last_steer, self.min_speed

        angle = math.atan2(mid_l, mid_f)
        steer = max(-self.max_steer,
                     min(self.max_steer, self.steering_gain * angle))
        self._last_steer = steer

        speed = self.max_speed * (1.0 - self.speed_curve_factor * abs(steer))
        speed = max(self.min_speed, min(self.max_speed, speed))
        return steer, speed


# ── Neural-net controller ─────────────────────────────────────────────────

class NeuralNetController:
    """Small feed-forward net evolved by GA.

    Architecture
    ------------
    Input  (8) : 2 nearest blue + 2 nearest yellow  × (dist, angle)
    Hidden (8) : tanh activation
    Output (2) : tanh → steer,  sigmoid → speed

    Total genes = 8×8 + 8 + 8×2 + 2 = 90
    """

    INPUT_SIZE = 8
    HIDDEN_SIZE = 8
    OUTPUT_SIZE = 2
    NUM_GENES = (INPUT_SIZE * HIDDEN_SIZE + HIDDEN_SIZE
                 + HIDDEN_SIZE * OUTPUT_SIZE + OUTPUT_SIZE)  # 90

    MAX_STEER = 0.5
    MAX_SPEED = 10.0

    def __init__(self, genes):
        g = np.asarray(genes, dtype=np.float64)
        self.genes = g
        i = 0
        n = self.INPUT_SIZE * self.HIDDEN_SIZE
        self.W1 = g[i:i + n].reshape(self.INPUT_SIZE, self.HIDDEN_SIZE)
        i += n
        self.b1 = g[i:i + self.HIDDEN_SIZE]
        i += self.HIDDEN_SIZE
        n = self.HIDDEN_SIZE * self.OUTPUT_SIZE
        self.W2 = g[i:i + n].reshape(self.HIDDEN_SIZE, self.OUTPUT_SIZE)
        i += n
        self.b2 = g[i:i + self.OUTPUT_SIZE]

    def reset(self):
        pass

    def control(self, visible_cones):
        """Return ``(steer, speed)`` given cones in kart frame."""
        blues = []
        yellows = []

        for cls, x, y, _z in visible_cones:
            dist = math.hypot(x, y)
            angle = math.atan2(y, x)
            if cls == "blue_cone":
                blues.append((dist, angle))
            elif cls == "yellow_cone":
                yellows.append((dist, angle))

        blues.sort()
        yellows.sort()

        inp = np.zeros(self.INPUT_SIZE)
        for j, (d, a) in enumerate(blues[:2]):
            inp[j * 2] = d / 15.0
            inp[j * 2 + 1] = a / np.pi
        for j, (d, a) in enumerate(yellows[:2]):
            inp[4 + j * 2] = d / 15.0
            inp[4 + j * 2 + 1] = a / np.pi

        hidden = np.tanh(inp @ self.W1 + self.b1)
        out = hidden @ self.W2 + self.b2

        steer = float(np.tanh(out[0])) * self.MAX_STEER
        speed = float(1.0 / (1.0 + np.exp(-out[1]))) * self.MAX_SPEED
        return steer, speed


# ── Neural-net v2 controller ──────────────────────────────────────────────

class NeuralNetV2Controller:
    """Larger net with more cone context and speed feedback.

    Architecture
    ------------
    Input  (17): 4 nearest blue × (dist, angle)
               + 4 nearest yellow × (dist, angle)
               + current speed (normalized)
    Hidden (16): tanh activation
    Output  (2): tanh → steer,  sigmoid → speed

    Total genes = 17×16 + 16 + 16×2 + 2 = 322
    """

    INPUT_SIZE = 17
    HIDDEN_SIZE = 16
    OUTPUT_SIZE = 2
    NUM_GENES = (INPUT_SIZE * HIDDEN_SIZE + HIDDEN_SIZE
                 + HIDDEN_SIZE * OUTPUT_SIZE + OUTPUT_SIZE)  # 322

    MAX_STEER = 0.5
    MAX_SPEED = 10.0

    def __init__(self, genes):
        g = np.asarray(genes, dtype=np.float64)
        self.genes = g
        i = 0
        n = self.INPUT_SIZE * self.HIDDEN_SIZE
        self.W1 = g[i:i + n].reshape(self.INPUT_SIZE, self.HIDDEN_SIZE)
        i += n
        self.b1 = g[i:i + self.HIDDEN_SIZE]
        i += self.HIDDEN_SIZE
        n = self.HIDDEN_SIZE * self.OUTPUT_SIZE
        self.W2 = g[i:i + n].reshape(self.HIDDEN_SIZE, self.OUTPUT_SIZE)
        i += n
        self.b2 = g[i:i + self.OUTPUT_SIZE]
        self._last_speed = 0.0

    def reset(self):
        self._last_speed = 0.0

    def control(self, visible_cones, current_speed=None):
        """Return ``(steer, speed)`` given cones in kart frame."""
        if current_speed is not None:
            self._last_speed = current_speed

        blues = []
        yellows = []

        for cls, x, y, _z in visible_cones:
            dist = math.hypot(x, y)
            angle = math.atan2(y, x)
            if cls == "blue_cone":
                blues.append((dist, angle))
            elif cls == "yellow_cone":
                yellows.append((dist, angle))

        blues.sort()
        yellows.sort()

        inp = np.zeros(self.INPUT_SIZE)
        # 4 nearest blue
        for j, (d, a) in enumerate(blues[:4]):
            inp[j * 2] = d / 15.0
            inp[j * 2 + 1] = a / np.pi
        # 4 nearest yellow
        for j, (d, a) in enumerate(yellows[:4]):
            inp[8 + j * 2] = d / 15.0
            inp[8 + j * 2 + 1] = a / np.pi
        # current speed
        inp[16] = self._last_speed / self.MAX_SPEED

        hidden = np.tanh(inp @ self.W1 + self.b1)
        out = hidden @ self.W2 + self.b2

        steer = float(np.tanh(out[0])) * self.MAX_STEER
        speed = float(1.0 / (1.0 + np.exp(-out[1]))) * self.MAX_SPEED
        return steer, speed


# ── Neural-net v3 controller ──────────────────────────────────────────────

class NeuralNetV3Controller:
    """Two-hidden-layer net with more inputs for richer decision-making.

    Architecture
    ------------
    Input  (19): 4 nearest blue × (dist, angle)
               + 4 nearest yellow × (dist, angle)
               + current speed (normalized)
               + current steer (normalized by MAX_STEER)
               + steer rate  (delta steer / MAX_STEER, measures recent change)
    Hidden1 (24): tanh activation
    Hidden2 (12): tanh activation
    Output   (2): tanh → steer,  sigmoid → speed

    Total genes = 19×24 + 24 + 24×12 + 12 + 12×2 + 2 = 806
    """

    INPUT_SIZE = 19
    HIDDEN1_SIZE = 24
    HIDDEN2_SIZE = 12
    OUTPUT_SIZE = 2
    NUM_GENES = (INPUT_SIZE * HIDDEN1_SIZE + HIDDEN1_SIZE
                 + HIDDEN1_SIZE * HIDDEN2_SIZE + HIDDEN2_SIZE
                 + HIDDEN2_SIZE * OUTPUT_SIZE + OUTPUT_SIZE)  # 806

    MAX_STEER = 0.5
    MAX_SPEED = 10.0

    def __init__(self, genes):
        g = np.asarray(genes, dtype=np.float64)
        self.genes = g
        i = 0
        n = self.INPUT_SIZE * self.HIDDEN1_SIZE
        self.W1 = g[i:i + n].reshape(self.INPUT_SIZE, self.HIDDEN1_SIZE)
        i += n
        self.b1 = g[i:i + self.HIDDEN1_SIZE]
        i += self.HIDDEN1_SIZE
        n = self.HIDDEN1_SIZE * self.HIDDEN2_SIZE
        self.W2 = g[i:i + n].reshape(self.HIDDEN1_SIZE, self.HIDDEN2_SIZE)
        i += n
        self.b2 = g[i:i + self.HIDDEN2_SIZE]
        i += self.HIDDEN2_SIZE
        n = self.HIDDEN2_SIZE * self.OUTPUT_SIZE
        self.W3 = g[i:i + n].reshape(self.HIDDEN2_SIZE, self.OUTPUT_SIZE)
        i += n
        self.b3 = g[i:i + self.OUTPUT_SIZE]
        self._last_speed = 0.0
        self._last_steer = 0.0
        self._prev_steer = 0.0

    def reset(self):
        self._last_speed = 0.0
        self._last_steer = 0.0
        self._prev_steer = 0.0

    def control(self, visible_cones, current_speed=None):
        """Return ``(steer, speed)`` given cones in kart frame."""
        if current_speed is not None:
            self._last_speed = current_speed

        blues = []
        yellows = []

        for cls, x, y, _z in visible_cones:
            dist = math.hypot(x, y)
            angle = math.atan2(y, x)
            if cls == "blue_cone":
                blues.append((dist, angle))
            elif cls == "yellow_cone":
                yellows.append((dist, angle))

        blues.sort()
        yellows.sort()

        inp = np.zeros(self.INPUT_SIZE)
        for j, (d, a) in enumerate(blues[:4]):
            inp[j * 2] = d / 15.0
            inp[j * 2 + 1] = a / np.pi
        for j, (d, a) in enumerate(yellows[:4]):
            inp[8 + j * 2] = d / 15.0
            inp[8 + j * 2 + 1] = a / np.pi
        inp[16] = self._last_speed / self.MAX_SPEED
        inp[17] = self._last_steer / self.MAX_STEER
        inp[18] = (self._last_steer - self._prev_steer) / self.MAX_STEER

        h1 = np.tanh(inp @ self.W1 + self.b1)
        h2 = np.tanh(h1 @ self.W2 + self.b2)
        out = h2 @ self.W3 + self.b3

        steer = float(np.tanh(out[0])) * self.MAX_STEER
        speed = float(1.0 / (1.0 + np.exp(-out[1]))) * self.MAX_SPEED

        self._prev_steer = self._last_steer
        self._last_steer = steer
        return steer, speed
