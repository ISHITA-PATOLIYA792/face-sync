import cv2
import mediapipe as mp
import pyautogui
import time
import winsound
import tkinter as tk
from tkinter import messagebox
from threading import Thread

# Initialize MediaPipe Face Mesh
mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(min_detection_confidence=0.5, min_tracking_confidence=0.5)
mp_drawing = mp.solutions.drawing_utils

# Initialize variables for eye tracking and motion detection
EAR_THRESHOLD = 2.0  # ear threshold for detecting closed eyes
EYE_AR_CONSEC_FRAMES = 5  # consecutive frames to consider eyes closed
CLOSE_EYES_TIME_THRESHOLD = 5 * 60  # 5 minutes in seconds
QUICK_SLEEP_THRESHOLD = 60  # 1 minute in seconds
eyes_were_closed_since = None  # stores time when eyes were first closed
closed_eyes_counter = 0

# Function to calculate EAR (Eye Aspect Ratio)
def calculate_eye_aspect_ratio(eye):
    A = distance(eye[1], eye[5])
    B = distance(eye[2], eye[4])
    C = distance(eye[0], eye[3])
    
    # Calculate EAR
    ear = (A + B) / (2.0 * C)
    return ear

# Function to calculate the distance between two points
def distance(point1, point2):
    return ((point1.x - point2.x) ** 2 + (point1.y - point2.y) ** 2) ** 0.5

# Function to start sleep detection (eyes closed for 5 minutes)
def start_sleep_detection():
    print("Starting sleep detection...")
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera")
        return
    print("Camera opened successfully")
    global eyes_were_closed_since, closed_eyes_counter

    while cap.isOpened():
        ret, frame = cap.read()

        if not ret:
            print("Error: Could not read the frame.")
            break

        # Convert the frame to RGB for MediaPipe processing
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Process the frame with Face Mesh
        results = face_mesh.process(frame_rgb)

        # Check if faces are found
        if results.multi_face_landmarks:
            for face_landmarks in results.multi_face_landmarks:
                # Get eye landmarks
                left_eye = [face_landmarks.landmark[i] for i in range(33, 133)]  # Left eye landmarks range
                right_eye = [face_landmarks.landmark[i] for i in range(263, 293)]  # Right eye landmarks range

                # Calculate EAR for both eyes
                left_eye_ear = calculate_eye_aspect_ratio(left_eye)
                right_eye_ear = calculate_eye_aspect_ratio(right_eye)

                # Detect if both eyes are closed based on EAR threshold
                if left_eye_ear > EAR_THRESHOLD and right_eye_ear > EAR_THRESHOLD:
                    closed_eyes_counter += 1
                    if eyes_were_closed_since is None:
                        eyes_were_closed_since = time.time()
                else:
                    closed_eyes_counter = 0
                    eyes_were_closed_since = None

                # Check if eyes have been closed for the threshold time
                if closed_eyes_counter >= EYE_AR_CONSEC_FRAMES and eyes_were_closed_since:
                    elapsed_time = time.time() - eyes_were_closed_since
                    if elapsed_time >= QUICK_SLEEP_THRESHOLD and elapsed_time < CLOSE_EYES_TIME_THRESHOLD:
                        # play buzzer sound if eyes closed for 1 minute
                        winsound.Beep(2000, 500)  # higher pitch, shorter duration
                        messagebox.showwarning("Warning", "Eyes closed for 1 minute. Please stay alert!")
                        eyes_were_closed_since = None  # reset after warning
                    elif elapsed_time >= CLOSE_EYES_TIME_THRESHOLD:
                        # play buzzer sound if eyes closed for 5 minutes
                        winsound.Beep(1000, 1000)
                        messagebox.showwarning("Warning", "Eyes closed for 5 minutes. Please be attentive.")
                        eyes_were_closed_since = None  # reset after buzzer

        # Show the resulting frame in a window
        cv2.imshow('Face Mesh - Sleep Detection', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# Function to start scrolling control based on face motion (Nose to chin distance)
def start_scrolling():
    print("Starting scrolling control...")
    cap = cv2.VideoCapture(0)
    if not cap.isOpened():
        print("Error: Could not open camera")
        return
    print("Camera opened successfully")

    while cap.isOpened():
        ret, frame = cap.read()

        if not ret:
            print("Error: Could not read the frame.")
            break

        # Convert the frame to RGB for MediaPipe processing
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)

        # Process the frame with Face Mesh
        results = face_mesh.process(frame_rgb)

        # Check if faces are found
        if results.multi_face_landmarks:
            for face_landmarks in results.multi_face_landmarks:
                # Get nose and chin landmarks
                landmark1 = face_landmarks.landmark[1]  # Nose tip
                landmark152 = face_landmarks.landmark[152]  # Chin

                H, W, _ = frame.shape
                x1, y1 = int(landmark1.x * W), int(landmark1.y * H)
                x152, y152 = int(landmark152.x * W), int(landmark152.y * H)

                y_diff = y152 - y1
                if 50 < y_diff < 85:
                    pyautogui.scroll(-10)  # Scroll down
                elif y_diff > 100:
                    pyautogui.scroll(10)  # Scroll up

        # Show the resulting frame in a window
        cv2.imshow('Face Mesh - Scrolling', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

    cap.release()
    cv2.destroyAllWindows()

# Function to launch a selected feature in a separate thread
def start_feature(option):
    print(f"Starting feature: {option}")
    if option == "sleep":
        thread = Thread(target=start_sleep_detection)
        thread.daemon = True  # Make sure it stops when the app closes
        thread.start()
        print("Sleep detection thread started")
    elif option == "scroll":
        thread = Thread(target=start_scrolling)
        thread.daemon = True
        thread.start()
        print("Scrolling thread started")

# GUI Setup with Tkinter
root = tk.Tk()
root.title("Face Motion Scrolling and Sleep Detection")

# Instructions
instruction_label = tk.Label(root, text="Select a feature to use:", font=("Arial", 12))
instruction_label.pack(pady=10)

# Add buttons for both options
sleep_button = tk.Button(root, text="Start Sleep Detection", command=lambda: start_feature("sleep"), font=("Arial", 14))
sleep_button.pack(pady=10)

scroll_button = tk.Button(root, text="Start Scrolling", command=lambda: start_feature("scroll"), font=("Arial", 14))
scroll_button.pack(pady=10)

# Add a close button to exit the application
close_button = tk.Button(root, text="Exit", command=root.quit, font=("Arial", 14))
close_button.pack(pady=20)

# Run the GUI loop
root.mainloop()