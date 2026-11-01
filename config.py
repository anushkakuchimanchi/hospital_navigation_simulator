HOST = "localhost"
PORT = 23000
ORIGIN_X = -2.5
ORIGIN_Y = -2.5
CELL_SIZE = 0.25
SAFETY_MARGIN = 1
START_XY = (-1.4, -1.8)
BASE_SPEED = 1.5
MAX_WHEEL_SPEED = 2.0
WHEEL_BASE = 0.331
LOOKAHEAD_DISTANCE = 0.45
MIN_LOOKAHEAD = 0.25
MAX_LOOKAHEAD = 0.70
FINAL_RADIUS = 0.15
NEAR_RADIUS = 0.6
TURN_ON = 0.9
TURN_OFF = 0.25
TURN_SPEED = 1.5
SIM_STEP_SLEEP = 0.02
MAX_RUN_SECONDS = 40
SPLINE_SMOOTHING = 0.08
SPLINE_SAMPLE_SPACING = 0.06
DELIVERY_PAUSE = 1.0
DOCK = "Pharmacy"
BEDS = [
    (-2.10, 1.15, -1.40, 2.45),
    (-1.25, 1.15, -0.55, 2.45),
    (-0.35, 1.15, 0.30, 2.45),
    (0.70, 1.15, 1.40, 2.45),
    (0.80, -1.70, 2.05, -1.00),
    (-2.50, -2.30, -1.65, -0.90),
]
DELIVERY_POINTS = {
    "Ward": (12, 7),
    "ICU": (14, 17),
    "Pharmacy": (3, 5),
    "Surgery": (6, 12),
}
TASKS = [
    {"name": "T1 medicines to Ward", "dest": "Ward", "priority": 1},
    {"name": "T2 blood bags to ICU", "dest": "ICU", "priority": 3},
    {"name": "T3 tools to Surgery", "dest": "Surgery", "priority": 2},
    {"name": "T4 drugs to Pharmacy", "dest": "Pharmacy", "priority": 2},
]
MARKERS = {"Ward": (1, 0.205, 1.075), "ICU": (2, 2.220, 1.725), "Pharmacy": (3, -0.895, -2.450), "Surgery": (4, 0.320, -1.025)}
WATCH_SEC = 2.0
NEED_FRAMES = 5
SCAN_TURN_SPEED = 1.0