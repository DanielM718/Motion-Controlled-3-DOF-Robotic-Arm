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

wrist_angle_history = deque(maxlen=10)

GRID_SIZE = 20

FINGER_TIPS =    [8, 12, 16, 20]
FINGER_KNUCKLES= [6, 10, 14, 18]
FINGER_NAMES =   ["Index", "Middle", "Ring", "Pinky"]

def draw_grid(frame, rows, cols):
    h, w, _ = frame.shape
    cell_w = w // cols
    cell_h = h // rows
    for i in range(1, cols):
        cv2.line(frame, (i * cell_w, 0), (i * cell_w, h), (50,50,50), 1)
    for i in range(1, rows):
        cv2.line(frame, (0, i * cell_h), (w, i * cell_h), (50,50,50), 1)
    return cell_w, cell_h

def get_grid_position(x, y):
    col = max(1, min(GRID_SIZE, int(x / (1.0 / GRID_SIZE)) + 1))
    row = max(1, min(GRID_SIZE, int(y / (1.0 / GRID_SIZE)) + 1))
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

def get_finger_states(landmarks, handedness, facing):
    states = {}
    thumb_tip = landmarks[4]
    thumb_knuckle = landmarks[3]
    if handedness == "Right":
        if facing == "PALM":
            states["Thumb"] = "OPEN" if thumb_tip.x < thumb_knuckle.x else "CLOSED"
        else:
            states["Thumb"] = "OPEN" if thumb_tip.x > thumb_knuckle.x else "CLOSED"
    else:
        if facing == "PALM":
            states["Thumb"] = "OPEN" if thumb_tip.x > thumb_knuckle.x else "CLOSED"
        else:
            states["Thumb"] = "OPEN" if thumb_tip.x < thumb_knuckle.x else "CLOSED"
    for tip_id, knuckle_id, name in zip(FINGER_TIPS, FINGER_KNUCKLES, FINGER_NAMES):
        tip = landmarks[tip_id]
        knuckle = landmarks[knuckle_id]
        states[name] = "OPEN" if tip.y < knuckle.y else "CLOSED"
    return states

def get_pinch_distance(landmarks):
    thumb = landmarks[4]
    index = landmarks[8]
    return ((thumb.x - index.x)**2 + (thumb.y - index.y)**2) ** 0.5

def is_open_hand(states, landmarks):
    if not all(s == "OPEN" for s in states.values()):
        return False
    if get_pinch_distance(landmarks) < 0.1:
        return False
    return True

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

    h, w, _ = frame.shape
    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
    results = hands.process(rgb)

    # FPS
    curr_time = time.time()
    fps = 1 / (curr_time - prev_time) if prev_time else 0
    prev_time = curr_time

    # Draw grid
    cell_w, cell_h = draw_grid(frame, GRID_SIZE, GRID_SIZE)

    # FPS top left
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

            angle = get_wrist_angle(lm)
            wrist_angle_history.append(angle)

            gesture = "OPEN" if is_open_hand(finger_states, lm) else "CLOSED"
            gesture_color = (0,255,0) if gesture == "OPEN" else (0,0,255)

            # Grid position
            col, row = get_grid_position(wrist.x, wrist.y)

            # Highlight current cell
            cell_x1 = (col - 1) * cell_w
            cell_y1 = (row - 1) * cell_h
            cell_x2 = col * cell_w
            cell_y2 = row * cell_h
            cv2.rectangle(frame, (cell_x1, cell_y1), (cell_x2, cell_y2),
                         (0,255,255), 2)

            # Wrist dot
            cv2.circle(frame, (wx, wy), 8, (0,0,255), -1)

            # Info display bottom left
            cv2.putText(frame, f'{display_hand} Hand', (10, h - 130),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,255,0), 2)
            cv2.putText(frame, f'Grid: ({col}, {row})', (10, h - 100),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0,255,255), 2)
            cv2.putText(frame, f'Angle: {int(angle)} deg', (10, h - 70),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255,165,0), 2)
            cv2.putText(frame, gesture, (10, h - 40),
                       cv2.FONT_HERSHEY_SIMPLEX, 0.9, gesture_color, 2)

            print(f"WRIST,{col},{row}")

    else:
        wrist_angle_history.clear()
        print("NO_HAND")
        

    cv2.imshow('Hand Tracking', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()