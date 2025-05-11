import cv2
import mediapipe as mp
import pyautogui
# Initialize MediaPipe Face Mesh
mp_face_mesh = mp.solutions.face_mesh
face_mesh = mp_face_mesh.FaceMesh(min_detection_confidence=0.5, min_tracking_confidence=0.5)
mp_drawing = mp.solutions.drawing_utils

# Open the webcam (0 for the default webcam)
cap = cv2.VideoCapture(0)

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
            
            landmark1=face_landmarks.landmark[1]     #noes tip
            landmark152=face_landmarks.landmark[152] #chin
            

            #find absoulte position
            H,W,_=frame.shape
            x1,y1=int(landmark1.x*W),int(landmark1.y*H)
            x152,y152=int(landmark152.x*W),int(landmark152.y*H)
            
            
            # draw the circle
            cv2.circle(frame,(x1,y1),3,(0, 255, 0), -1)
            cv2.circle(frame,(x152,y152),3,(0, 255, 0), -1)
            

             # Display coordinates
            cv2.putText(frame, f'({x1}, {y1})', (x1 + 10, y1), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
            cv2.putText(frame, f'({x152}, {y152})', (x152 + 10, y152), cv2.FONT_HERSHEY_SIMPLEX, 0.4, (0, 255, 0), 1)
            
            y_diff=y152-y1
            cv2.putText(frame, f"y_diff: {y_diff}", (50, 100), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
            if 50<y_diff<85:
                pyautogui.scroll(-10)  # Scroll down
                cv2.putText(frame, "Scroll Down", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            elif y_diff>100:
                pyautogui.scroll(10)  # Scroll up
                cv2.putText(frame, "Scroll Up", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)


    # Show the resulting frame
    cv2.imshow('Face Mesh', frame)

    # Press 'q' to exit the loop
    if cv2.waitKey(1) & 0xFF == ord('q'):
        exit(0)

cap.release()
cv2.destroyAllWindows()
