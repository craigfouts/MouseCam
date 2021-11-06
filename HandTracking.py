import cv2
import mediapipe as mp


class HandDetector:
    def __init__(self, max_hands=2, detect_conf=0.5, track_conf=0.5):
        self.max_hands = max_hands
        self.detect_conf = detect_conf
        self.track_conf = track_conf
        self.mp_hands = mp.solutions.hands
        self.hands = self.mp_hands.Hands(tuple(locals().values())[1:])
        self.mp_draw = mp.solutions.drawing_utils

    def find_hands(self, img, draw=True):
        img_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        self.results = self.hands.process(img_rgb)
        if self.results.multi_hand_landmarks:
            for hand_lms in self.results.multi_hand_landmarks:
                if draw:
                    connections = self.mp_hands.HAND_CONNECTIONS
                    self.mp_draw.draw_landmarks(img, hand_lms, connections)
        return img

    def find_landmarks(self, img, hand_idx=0, draw_lms=()):
        lms = []
        if self.results.multi_hand_landmarks:
            hand = self.results.multi_hand_landmarks[hand_idx]
            for id, lm in enumerate(hand.landmark):
                h, w = img.shape[:2]
                x, y = int(lm.x * w), int(lm.y * h)
                lms.append((id, x, y))
                if id in draw_lms:
                    cv2.circle(img, (x, y), 10, (255, 0, 0), cv2.FILLED)
        return lms
