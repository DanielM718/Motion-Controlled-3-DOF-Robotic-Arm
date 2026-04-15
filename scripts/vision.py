import cv2
import mediapipe as mp
from collections import deque
import time
import numpy as np

mp_hands = mp.solutions.hands
mp_draw = mp.solutions.drawing_utils

hands = mp_hands.Hands(
    static_image_mode=False,
    max_num_hands=1,
    model_complexity=0,
    min_detection_confidence=0.5,
    min_tracking_confidence=0.5
)

cap = cv2.VideoCapture(0)

cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)

wrist_angle_history = deque(maxlen=10)

GRID_SIZE = 81

FINGER_TIPS =    [8, 12, 16, 20]
FINGER_KNUCKLES= [6, 10, 14, 18]
FINGER_NAMES =   ["Index", "Middle", "Ring", "Pinky"]

def is_closed_fist(landmarks):
    wrist = landmarks[0]

    finger_pairs = [(8,5), (12,9), (16,13), (20,17)]

    curled = 0
    for tip_id, mcp_id in finger_pairs:
        tip = landmarks[tip_id]
        mcp = landmarks[mcp_id]

        tip_dist = ((tip.x - wrist.x)**2 + (tip.y - wrist.y)**2) ** 0.5
        mcp_dist = ((mcp.x - wrist.x)**2 + (mcp.y - wrist.y)**2) ** 0.5

        if tip_dist < mcp_dist * 1.2:
            curled += 1

    # thumb
    thumb_tip = landmarks[4]
    thumb_base = landmarks[2]

    thumb_tip_dist = ((thumb_tip.x - wrist.x)**2 + (thumb_tip.y - wrist.y)**2) ** 0.5
    thumb_base_dist = ((thumb_base.x - wrist.x)**2 + (thumb_base.y - wrist.y)**2) ** 0.5

    thumb_closed = thumb_tip_dist < thumb_base_dist * 1.2

    return curled >= 3 and thumb_closed

def draw_grid(frame, rows, cols):
    h, w, _ = frame.shape
    for i in range(1, cols):
        x = int(i * w / cols)
        cv2.line(frame, (x, 0), (x, h), (50,50,50), 1)
    for i in range(1, rows):
        y = int(i * h / rows)
        cv2.line(frame, (0, y), (w, y), (50,50,50), 1)

def get_grid_position(px, py, w, h):
    col = max(1, min(GRID_SIZE, int(px * GRID_SIZE / w) + 1))
    row = max(1, min(GRID_SIZE, int(py * GRID_SIZE / h) + 1))
    return col, row

def get_palm_facing(landmarks, handedness):
    wrist = landmarks[0]
    index_knuckle = landmarks[5]
    pinky_knuckle = landmarks[17]
    v1 = np.array([index_knuckle.x - wrist.x, index_knuckle.y - wrist.y])
    v2 = np.array([pinky_knuckle.x - wrist.x, pinky_knuckle.y - wrist.y])
    cross = v1[0] * v2[1] - v1[1] * v2[0]
    if handedness == "Right":
        return "PALM" if cross > 0 else "BACK"
    else:
        return "PALM" if cross < 0 else "BACK"

def get_finger_states(landmarks, handedness=None, facing=None):
    states = {}
    wrist = landmarks[0]

    fingers = [
        (8, 6, "Index"),
        (12, 10, "Middle"),
        (16, 14, "Ring"),
        (20, 18, "Pinky"),
    ]

    for tip_id, pip_id, name in fingers:
        tip = landmarks[tip_id]
        pip = landmarks[pip_id]

        tip_dist = ((tip.x - wrist.x)**2 + (tip.y - wrist.y)**2) ** 0.5
        pip_dist = ((pip.x - wrist.x)**2 + (pip.y - wrist.y)**2) ** 0.5

        states[name] = "OPEN" if tip_dist > pip_dist * 1.12 else "CLOSED"

    # Thumb: simple distance-based check
    thumb_tip = landmarks[4]
    thumb_ip  = landmarks[3]
    thumb_mcp = landmarks[2]

    tip_to_mcp = ((thumb_tip.x - thumb_mcp.x)**2 + (thumb_tip.y - thumb_mcp.y)**2) ** 0.5
    ip_to_mcp  = ((thumb_ip.x - thumb_mcp.x)**2 + (thumb_ip.y - thumb_mcp.y)**2) ** 0.5

    states["Thumb"] = "OPEN" if tip_to_mcp > ip_to_mcp * 1.08 else "CLOSED"

    return states

def get_pinch_distance(landmarks):
    thumb = landmarks[4]
    index = landmarks[8]
    return ((thumb.x - index.x)**2 + (thumb.y - index.y)**2) ** 0.5

def is_open_hand(states, landmarks):
    open_count = sum(1 for s in states.values() if s == "OPEN")
    return open_count >= 4

def get_wrist_angle(landmarks):
    wrist = landmarks[0]
    mid_knuckle = landmarks[9]
    dx = mid_knuckle.x - wrist.x
    dy = mid_knuckle.y - wrist.y
    return np.degrees(np.arctan2(dy, dx))

prev_time = 0

while True:
    ret, frame = cap.read()
    if not ret:
        break

    frame = cv2.resize(frame, (640, 480))

    frame = cv2.flip(frame, 1)

    h, w, _ = frame.shape

    cx, cy = w // 2, h // 2
    radius = min(w, h) // 2

    cv2.circle(frame, (cx, cy), radius, (0, 255, 0), 2)
    cv2.line(frame, (0, h//2), (w, h//2), (255, 0, 0), 2)
    
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb)


    curr_time = time.time()
    fps = 1 / (curr_time - prev_time) if prev_time else 0
    prev_time = curr_time

    draw_grid(frame, GRID_SIZE, GRID_SIZE)

    cv2.putText(frame, f'FPS: {int(fps)}', (10, 30),
                cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,0), 2)

    if results.multi_hand_landmarks:
        for hand_landmarks, hand_info in zip(results.multi_hand_landmarks, results.multi_handedness):
            mp_draw.draw_landmarks(frame, hand_landmarks, mp_hands.HAND_CONNECTIONS)

            lm = hand_landmarks.landmark
            wrist = lm[0]
            wx = int(wrist.x * w)
            wy = int(wrist.y * h)

            handedness = hand_info.classification[0].label
            display_hand = "Left" if handedness == "Right" else "Right"

            facing = get_palm_facing(lm, handedness)
            finger_states = get_finger_states(lm, handedness, facing)

            gesture = "OPEN" if is_open_hand(finger_states, lm) else "CLOSED"

            gesture_color = (0,255,0) if gesture == "OPEN" else (0,0,255)

            col, row = get_grid_position(wx, wy, w, h)

            cell_x1 = int((col - 1) * w / GRID_SIZE)
            cell_y1 = int((row - 1) * h / GRID_SIZE)
            cell_x2 = int(col * w / GRID_SIZE)
            cell_y2 = int(row * h / GRID_SIZE)

            cv2.rectangle(frame, (cell_x1, cell_y1), (cell_x2, cell_y2),
                         (0,255,255), 2)

            cv2.circle(frame, (wx, wy), 8, (0,0,255), -1)

            cv2.putText(frame, f'{display_hand} Hand', (10, h - 130),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,0), 2)
            cv2.putText(frame, f'Grid: ({col}, {row})', (10, h - 100),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,165,0), 2)
            cv2.putText(frame, gesture, (10, h - 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.9, gesture_color, 2)

            print(f"WRIST,{col},{row}\n")
            pinch = 0 if gesture == "OPEN" else 1
            print(f"PINCH,{pinch}, 0\n")

    else:
        wrist_angle_history.clear()
        print("NO_HAND\n")
        

    cv2.imshow('Hand Tracking', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()