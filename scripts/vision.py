#!/usr/bin/env python3
import sys
import time
import signal
import cv2
import mediapipe as mp
from mediapipe.framework.formats import landmark_pb2

BaseOptions = mp.tasks.BaseOptions
HandLandmarker = mp.tasks.vision.HandLandmarker
HandLandmarkerOptions = mp.tasks.vision.HandLandmarkerOptions
VisionRunningMode = mp.tasks.vision.RunningMode

RUNNING = True

LANDMARK_NAMES = [
    "WRIST",
    "THUMB_CMC", "THUMB_MCP", "THUMB_IP", "THUMB_TIP",
    "INDEX_MCP", "INDEX_PIP", "INDEX_DIP", "INDEX_TIP",
    "MIDDLE_MCP", "MIDDLE_PIP", "MIDDLE_DIP", "MIDDLE_TIP",
    "RING_MCP", "RING_PIP", "RING_DIP", "RING_TIP",
    "PINKY_MCP", "PINKY_PIP", "PINKY_DIP", "PINKY_TIP",
]

# Better reduced set for control/debug
KEY_IDS = {
    "WRIST": 0,
    "THUMB_TIP": 4,
    "INDEX_MCP": 5,
    "INDEX_TIP": 8,
    "MIDDLE_MCP": 9,
    "MIDDLE_TIP": 12,
    "PINKY_MCP": 17,
    "PINKY_TIP": 20,
}


def handle_sigint(signum, frame):
    global RUNNING
    RUNNING = False


def draw_hand_landmarks(frame_bgr, hand_landmarks_list, handedness_list):
    annotated = frame_bgr.copy()

    for i, hand_landmarks in enumerate(hand_landmarks_list):
        proto = landmark_pb2.NormalizedLandmarkList()
        proto.landmark.extend([
            landmark_pb2.NormalizedLandmark(x=lm.x, y=lm.y, z=lm.z)
            for lm in hand_landmarks
        ])

        mp.solutions.drawing_utils.draw_landmarks(
            annotated,
            proto,
            mp.solutions.hands.HAND_CONNECTIONS,
            mp.solutions.drawing_styles.get_default_hand_landmarks_style(),
            mp.solutions.drawing_styles.get_default_hand_connections_style(),
        )

        # Put handedness text near wrist
        wrist = hand_landmarks[0]
        h, w, _ = annotated.shape
        px = int(wrist.x * w)
        py = int(wrist.y * h)

        label = "Unknown"
        score = 0.0
        if handedness_list and i < len(handedness_list) and handedness_list[i]:
            label = handedness_list[i][0].category_name
            score = handedness_list[i][0].score

        cv2.putText(
            annotated,
            f"{label} {score:.2f}",
            (px + 10, py - 10),
            cv2.FONT_HERSHEY_SIMPLEX,
            0.6,
            (0, 255, 0),
            2,
            cv2.LINE_AA,
        )

    return annotated


def main():
    signal.signal(signal.SIGINT, handle_sigint)

    model_path = "models/hand_landmarker.task"
    camera_index = 0
    max_num_hands = 1

    cap = cv2.VideoCapture(camera_index)
    if not cap.isOpened():
        print("ERR,failed_to_open_camera", file=sys.stderr, flush=True)
        sys.exit(1)

    options = HandLandmarkerOptions(
        base_options=BaseOptions(model_asset_path=model_path),
        running_mode=VisionRunningMode.VIDEO,
        num_hands=max_num_hands,
        min_hand_detection_confidence=0.5,
        min_hand_presence_confidence=0.5,
        min_tracking_confidence=0.5,
    )

    print("INFO,hand_landmarks_started", file=sys.stderr, flush=True)

    with HandLandmarker.create_from_options(options) as landmarker:
        while RUNNING:
            ok, frame = cap.read()
            if not ok:
                print("ERR,failed_to_read_frame", file=sys.stderr, flush=True)
                time.sleep(0.01)
                continue

            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

            mp_image = mp.Image(
                image_format=mp.ImageFormat.SRGB,
                data=frame_rgb
            )

            timestamp_ms = int(time.monotonic() * 1000)
            result = landmarker.detect_for_video(mp_image, timestamp_ms)

            display_frame = frame.copy()

            if result.hand_landmarks:
                display_frame = draw_hand_landmarks(
                    frame,
                    result.hand_landmarks,
                    result.handedness
                )

                # Use first hand for machine-readable output
                landmarks = result.hand_landmarks[0]

                handedness = "Unknown"
                score = 0.0
                if result.handedness and len(result.handedness) > 0 and len(result.handedness[0]) > 0:
                    handedness = result.handedness[0][0].category_name
                    score = result.handedness[0][0].score

                parts = [
                    "HAND",
                    str(timestamp_ms),
                    handedness,
                    f"{score:.4f}"
                ]

                for name, idx in KEY_IDS.items():
                    lm = landmarks[idx]
                    parts.extend([
                        name,
                        f"{lm.x:.6f}",
                        f"{lm.y:.6f}",
                        f"{lm.z:.6f}"
                    ])

                line = ",".join(parts)
                print(line, flush=True)

                # readable debug to stderr
                debug_str = " | ".join(
                    f"{name}=({landmarks[idx].x:.3f},{landmarks[idx].y:.3f},{landmarks[idx].z:.3f})"
                    for name, idx in KEY_IDS.items()
                )
                print(debug_str, file=sys.stderr, flush=True)
            else:
                cv2.putText(
                    display_frame,
                    "NO HAND",
                    (20, 40),
                    cv2.FONT_HERSHEY_SIMPLEX,
                    1.0,
                    (0, 0, 255),
                    2,
                    cv2.LINE_AA,
                )
                print("NO_HAND", flush=True)

            cv2.imshow("Hand Landmarks", display_frame)

            # ESC or q to quit
            key = cv2.waitKey(1) & 0xFF
            if key == 27 or key == ord('q'):
                break

    cap.release()
    cv2.destroyAllWindows()
    print("INFO,hand_landmarks_stopped", file=sys.stderr, flush=True)


if __name__ == "__main__":
    main()