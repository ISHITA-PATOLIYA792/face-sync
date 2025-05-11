import cv2
import mediapipe as mp
import time
import winsound

# Initialize MediaPipe Face Mesh
mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(min_detection_confidence=0.5, min_tracking_confidence=0.5)
mp_drawing = mp.solutions.drawing_utils

# Open the webcam (0 for the default webcam)
cap = cv2.VideoCapture(0)

# EAR threshold for closed eyes (since your EAR values increase when eyes are closed)
EAR_THRESHOLD = 2.0  # Adjusted threshold for higher EAR values when eyes are closed
EYE_AR_CONSEC_FRAMES = 5  # Consecutive frames to consider the eyes closed
CLOSE_EYES_TIME_THRESHOLD = 5  # 5 minutes in seconds

# Initialize variables for time tracking and closed eyes counter
eyes_were_closed_since = None  # Stores the time when eyes first closed
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

# Main loop for capturing frames from the webcam
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
            
            # Get eye landmarks for both eyes
            left_eye = [face_landmarks.landmark[i] for i in range(33, 133)]  # Left eye landmarks range
            right_eye = [face_landmarks.landmark[i] for i in range(263, 293)]  # Right eye landmarks range

            # Calculate the EAR for both eyes
            left_eye_ear = calculate_eye_aspect_ratio(left_eye)
            right_eye_ear = calculate_eye_aspect_ratio(right_eye)

            # Debug: Show EAR values for each eye
            cv2.putText(frame, f'Left EAR: {left_eye_ear:.2f}', (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            cv2.putText(frame, f'Right EAR: {right_eye_ear:.2f}', (50, 80), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            print(f"Left EAR: {left_eye_ear:.2f}, Right EAR: {right_eye_ear:.2f}")

            # Detect if both eyes are closed based on EAR threshold (higher EAR value means closed eyes)
            if left_eye_ear > EAR_THRESHOLD and right_eye_ear > EAR_THRESHOLD:
                closed_eyes_counter += 1
                if eyes_were_closed_since is None:
                    # Store the time when the eyes are closed
                    eyes_were_closed_since = time.time()
                    print(f"Eyes closed at {time.ctime(eyes_were_closed_since)}")
            else:
                # Reset counter when eyes are open
                closed_eyes_counter = 0
                eyes_were_closed_since = None
                print("Eyes are open.")

            # Check if eyes have been closed for the specified threshold time (e.g., 5 minutes)
            if closed_eyes_counter >= EYE_AR_CONSEC_FRAMES and eyes_were_closed_since:
                elapsed_time = time.time() - eyes_were_closed_since
                print(f"Eyes have been closed for {elapsed_time:.2f} seconds.")
                if elapsed_time >= CLOSE_EYES_TIME_THRESHOLD:
                    print(f"Eyes have been closed for 5 minutes.")
                    winsound.Beep(1000, 1000)  # Play a tone (frequency = 1000 Hz, duration = 1000 ms)
                    cv2.putText(frame, "Eyes closed for 5 minutes!", (50, 110), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)

                    # Reset the counter and timer after the buzzer rings
                    eyes_were_closed_since = None
                    closed_eyes_counter = 0  # Reset the counter

    # Show the frame with face landmarks
    cv2.imshow('Face Mesh', frame)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()
