#!/usr/bin/env python3
"""
Simple gesture detection using MediaPipe and OAK-D Lite camera
All computation is performed on the camera device
"""

import cv2
import depthai as dai
import mediapipe as mp
import numpy as np
import time

# Initialize MediaPipe Hands
mp_hands = mp.solutions.hands
mp_drawing = mp.solutions.drawing_utils
mp_drawing_styles = mp.solutions.drawing_styles

# Gesture labels
GESTURE_LABELS = {
    "thumbs_up": "Thumbs Up",
    "peace": "Peace Sign", 
    "ok": "OK Sign",
    "pointing": "Pointing",
    "fist": "Fist",
    "open_palm": "Open Palm",
    "unknown": "Unknown"
}

def create_pipeline():
    """Create DepthAI pipeline for OAK-D Lite"""
    pipeline = dai.Pipeline()
    
    # Configure camera
    cam = pipeline.createColorCamera()
    cam.setBoardSocket(dai.CameraBoardSocket.RGB)
    cam.setPreviewSize(640, 480)
    cam.setInterleaved(False)
    cam.setFps(30)
    
    # Create output
    xout = pipeline.createXLinkOut()
    xout.setStreamName("rgb")
    cam.preview.link(xout.input)
    
    return pipeline

def detect_gesture(hand_landmarks):
    """
    Simple gesture detection based on hand landmarks
    Returns gesture name
    """
    if not hand_landmarks:
        return "unknown"
    
    # Get landmark positions
    landmarks = hand_landmarks.landmark
    
    # Thumb tip and base
    thumb_tip = landmarks[mp_hands.HandLandmark.THUMB_TIP]
    thumb_ip = landmarks[mp_hands.HandLandmark.THUMB_IP]
    thumb_mcp = landmarks[mp_hands.HandLandmark.THUMB_MCP]
    
    # Index finger
    index_tip = landmarks[mp_hands.HandLandmark.INDEX_FINGER_TIP]
    index_pip = landmarks[mp_hands.HandLandmark.INDEX_FINGER_PIP]
    
    # Middle finger
    middle_tip = landmarks[mp_hands.HandLandmark.MIDDLE_FINGER_TIP]
    middle_pip = landmarks[mp_hands.HandLandmark.MIDDLE_FINGER_PIP]
    
    # Ring finger
    ring_tip = landmarks[mp_hands.HandLandmark.RING_FINGER_TIP]
    ring_pip = landmarks[mp_hands.HandLandmark.RING_FINGER_PIP]
    
    # Pinky
    pinky_tip = landmarks[mp_hands.HandLandmark.PINKY_TIP]
    pinky_pip = landmarks[mp_hands.HandLandmark.PINKY_PIP]
    
    # Check if fingers are extended (simple threshold based)
    thumb_up = thumb_tip.y < thumb_ip.y
    index_up = index_tip.y < index_pip.y
    middle_up = middle_tip.y < middle_pip.y
    ring_up = ring_tip.y < ring_pip.y
    pinky_up = pinky_tip.y < pinky_pip.y
    
    # Count extended fingers
    fingers_up = sum([thumb_up, index_up, middle_up, ring_up, pinky_up])
    
    # Detect gestures
    if fingers_up == 0:
        return "fist"
    elif fingers_up == 5:
        return "open_palm"
    elif thumb_up and not index_up and not middle_up and not ring_up and not pinky_up:
        return "thumbs_up"
    elif index_up and middle_up and not ring_up and not pinky_up:
        return "peace"
    elif index_up and not middle_up and not ring_up and not pinky_up:
        return "pointing"
    elif thumb_up and index_up and middle_up and not ring_up and not pinky_up:
        # Check if thumb and index are close (OK sign)
        thumb_index_dist = np.sqrt((thumb_tip.x - index_tip.x)**2 + 
                                  (thumb_tip.y - index_tip.y)**2)
        if thumb_index_dist < 0.1:  # Threshold for OK sign
            return "ok"
    
    return "unknown"

def main():
    """Main function"""
    # Create pipeline
    pipeline = create_pipeline()
    
    # Initialize MediaPipe Hands
    with mp_hands.Hands(
        model_complexity=0,  # Use lightest model for speed
        min_detection_confidence=0.5,
        min_tracking_confidence=0.5,
        max_num_hands=2
    ) as hands:
        
        # Connect to device
        with dai.Device(pipeline) as device:
            print("Connected to OAK-D Lite")
            
            # Get output queue
            q = device.getOutputQueue(name="rgb", maxSize=4, blocking=False)
            
            # FPS calculation
            fps = 0
            prev_time = time.time()
            
            print("Press 'q' to quit")
            
            while True:
                # Get frame from camera
                in_frame = q.get()
                frame = in_frame.getCvFrame()
                
                # Flip frame horizontally for mirror effect
                frame = cv2.flip(frame, 1)
                
                # Convert BGR to RGB for MediaPipe
                rgb_frame = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                
                # Process frame with MediaPipe
                results = hands.process(rgb_frame)
                
                # Draw hand landmarks and detect gestures
                if results.multi_hand_landmarks:
                    for hand_landmarks in results.multi_hand_landmarks:
                        # Draw landmarks
                        mp_drawing.draw_landmarks(
                            frame,
                            hand_landmarks,
                            mp_hands.HAND_CONNECTIONS,
                            mp_drawing_styles.get_default_hand_landmarks_style(),
                            mp_drawing_styles.get_default_hand_connections_style()
                        )
                        
                        # Detect gesture
                        gesture = detect_gesture(hand_landmarks)
                        gesture_text = GESTURE_LABELS.get(gesture, "Unknown")
                        
                        # Get hand position for text placement
                        h, w, _ = frame.shape
                        x = int(hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].x * w)
                        y = int(hand_landmarks.landmark[mp_hands.HandLandmark.WRIST].y * h)
                        
                        # Draw gesture label
                        cv2.putText(frame, gesture_text, (x - 50, y - 20),
                                   cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                # Calculate and display FPS
                curr_time = time.time()
                fps = int(1 / (curr_time - prev_time))
                prev_time = curr_time
                
                # Display FPS and instructions
                cv2.putText(frame, f"FPS: {fps}", (10, 30),
                           cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)
                cv2.putText(frame, "Press 'q' to quit", (10, 60),
                           cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 255, 255), 1)
                
                # Show frame
                cv2.imshow("Gesture Detection - OAK-D Lite", frame)
                
                # Check for quit
                if cv2.waitKey(1) & 0xFF == ord('q'):
                    break
    
    cv2.destroyAllWindows()
    print("Gesture detection stopped")

if __name__ == "__main__":
    main() ``