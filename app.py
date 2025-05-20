import cv2
import mediapipe as mp
import pyautogui
import time
import winsound
import customtkinter as ctk
from PIL import Image, ImageTk
import threading
from tkinter import messagebox
import os
import sys
from customtkinter import CTkImage

class FaceSyncApp:
    def __init__(self):
        # setup window
        self.root = ctk.CTk()
        self.root.title("FaceSync")
        self.root.geometry("800x600")
        
        # set theme
        ctk.set_appearance_mode("dark")
        ctk.set_default_color_theme("blue")
        
        # initialize variables
        self.is_sleep_detection_running = False
        self.is_scrolling_running = False
        self.camera_thread = None
        self.cap = None
        self.face_mesh = None  # initialize as None
        
        # sleep detection variables
        self.EAR_THRESHOLD = 2.0  # adjusted threshold for higher EAR values when eyes are closed
        self.EYE_AR_CONSEC_FRAMES = 5  # consecutive frames to consider eyes closed
        self.CLOSE_EYES_TIME_THRESHOLD = 30  # 30 seconds threshold
        self.closed_eyes_counter = 0
        self.eyes_closed_start = None
        
        # create ui
        self.create_ui()
        
        # initialize mediapipe after ui
        self.initialize_mediapipe()
        
    def create_ui(self):
        # create main frame
        self.main_frame = ctk.CTkFrame(self.root)
        self.main_frame.pack(fill="both", expand=True, padx=20, pady=20)
        
        # title
        title_label = ctk.CTkLabel(
            self.main_frame,
            text="FaceSync",
            font=ctk.CTkFont(size=32, weight="bold")
        )
        title_label.pack(pady=20)
        
        # description
        desc_label = ctk.CTkLabel(
            self.main_frame,
            text="Control your computer with face movements and stay alert while working",
            font=ctk.CTkFont(size=14),
            wraplength=600
        )
        desc_label.pack(pady=10)
        
        # create features frame
        features_frame = ctk.CTkFrame(self.main_frame)
        features_frame.pack(fill="x", padx=40, pady=20)
        
        # sleep detection section
        sleep_frame = ctk.CTkFrame(features_frame)
        sleep_frame.pack(fill="x", padx=20, pady=10)
        
        sleep_title = ctk.CTkLabel(
            sleep_frame,
            text="Sleep Detection",
            font=ctk.CTkFont(size=20, weight="bold")
        )
        sleep_title.pack(pady=5)
        
        sleep_desc = ctk.CTkLabel(
            sleep_frame,
            text="Monitors your eyes and alerts you if you fall asleep",
            font=ctk.CTkFont(size=12)
        )
        sleep_desc.pack(pady=5)
        
        self.sleep_button = ctk.CTkButton(
            sleep_frame,
            text="Start Sleep Detection",
            command=self.start_sleep_detection,
            font=ctk.CTkFont(size=14),
            fg_color="#2E7D32",
            hover_color="#1B5E20"
        )
        self.sleep_button.pack(side="left", padx=10, pady=10)
        self.stop_sleep_button = ctk.CTkButton(
            sleep_frame,
            text="Stop Sleep Detection",
            command=self.stop_sleep_detection,
            font=ctk.CTkFont(size=14),
            fg_color="#D32F2F",
            hover_color="#B71C1C"
        )
        self.stop_sleep_button.pack(side="left", padx=10, pady=10)
        self.stop_sleep_button.configure(state="disabled")
        
        # scrolling control section
        scroll_frame = ctk.CTkFrame(features_frame)
        scroll_frame.pack(fill="x", padx=20, pady=10)
        
        scroll_title = ctk.CTkLabel(
            scroll_frame,
            text="Face Scrolling",
            font=ctk.CTkFont(size=20, weight="bold")
        )
        scroll_title.pack(pady=5)
        
        scroll_desc = ctk.CTkLabel(
            scroll_frame,
            text="Control scrolling with face movements (move your head up/down)",
            font=ctk.CTkFont(size=12)
        )
        scroll_desc.pack(pady=5)
        
        self.scroll_button = ctk.CTkButton(
            scroll_frame,
            text="Start Scrolling",
            command=self.start_scrolling,
            font=ctk.CTkFont(size=14),
            fg_color="#1976D2",
            hover_color="#1565C0"
        )
        self.scroll_button.pack(side="left", padx=10, pady=10)
        self.stop_scroll_button = ctk.CTkButton(
            scroll_frame,
            text="Stop Scrolling",
            command=self.stop_scrolling,
            font=ctk.CTkFont(size=14),
            fg_color="#D32F2F",
            hover_color="#B71C1C"
        )
        self.stop_scroll_button.pack(side="left", padx=10, pady=10)
        self.stop_scroll_button.configure(state="disabled")
        
        # status frame
        self.status_frame = ctk.CTkFrame(self.main_frame)
        self.status_frame.pack(fill="x", padx=40, pady=20)
        
        self.status_label = ctk.CTkLabel(
            self.status_frame,
            text="Status: Ready",
            font=ctk.CTkFont(size=14)
        )
        self.status_label.pack(pady=10)
        
        # add camera preview label
        self.camera_label = ctk.CTkLabel(self.main_frame, text="")
        self.camera_label.pack(pady=10)
        
        # exit button
        exit_button = ctk.CTkButton(
            self.main_frame,
            text="Exit",
            command=self.on_closing,
            font=ctk.CTkFont(size=14),
            fg_color="#D32F2F",
            hover_color="#B71C1C"
        )
        exit_button.pack(pady=20)
        
    def initialize_mediapipe(self):
        # initialize mediapipe
        self.mp_face_mesh = mp.solutions.face_mesh
        self.face_mesh = self.mp_face_mesh.FaceMesh(
            min_detection_confidence=0.5,
            min_tracking_confidence=0.5
        )
        
    def start_sleep_detection(self):
        if self.is_scrolling_running:
            self.root.after(0, lambda: messagebox.showwarning("Warning", "Please stop scrolling control first"))
            return
        self.is_sleep_detection_running = True
        self.sleep_button.configure(state="disabled")
        self.stop_sleep_button.configure(state="normal")
        self.status_label.configure(text="Status: Sleep Detection Active")
        self.cap = cv2.VideoCapture(0)
        self.sleep_detection_loop()
        
    def stop_sleep_detection(self):
        self.is_sleep_detection_running = False
        self.sleep_button.configure(state="normal")
        self.stop_sleep_button.configure(state="disabled")
        self.status_label.configure(text="Status: Ready")
        self.closed_eyes_counter = 0
        self.eyes_closed_start = None
        
        try:
            if self.cap is not None:
                if hasattr(self.cap, 'isOpened') and self.cap.isOpened():
                    self.cap.release()
                self.cap = None
            cv2.destroyAllWindows()
        except Exception as e:
            print(f"Error stopping sleep detection: {e}")
        
    def sleep_detection_loop(self):
        if not self.is_sleep_detection_running:
            try:
                cv2.destroyWindow('Sleep Detection')
            except cv2.error:
                pass  # window was not created
            return
            
        if self.cap is None or not self.cap.isOpened():
            self.root.after(0, lambda: messagebox.showerror("Error", "Could not open camera"))
            self.stop_sleep_detection()
            return
                
        ret, frame = self.cap.read()
        if not ret:
            self.stop_sleep_detection()
            return
                
        # flip frame horizontally for natural view
        frame = cv2.flip(frame, 1)
        
        frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
        results = self.face_mesh.process(frame_rgb)
        eyes_closed = False
        
        if results.multi_face_landmarks:
            for face_landmarks in results.multi_face_landmarks:
                # get eye landmarks for both eyes using the new ranges
                left_eye = [face_landmarks.landmark[i] for i in range(33, 133)]  # left eye landmarks range
                right_eye = [face_landmarks.landmark[i] for i in range(263, 293)]  # right eye landmarks range
                
                # calculate ear for both eyes
                left_ear = self.calculate_ear(left_eye)
                right_ear = self.calculate_ear(right_eye)
                avg_ear = (left_ear + right_ear) / 2.0
                
                # debug logging
                print(f"Left EAR: {left_ear:.2f}, Right EAR: {right_ear:.2f}, Avg EAR: {avg_ear:.2f}")
                
                # update status with ear values
                self.status_label.configure(text=f"Status: Sleep Detection Active | Left EAR: {left_ear:.2f} | Right EAR: {right_ear:.2f}")
                
                # draw eye landmarks
                for eye in [left_eye, right_eye]:
                    for point in eye:
                        x, y = int(point.x * frame.shape[1]), int(point.y * frame.shape[0])
                        cv2.circle(frame, (x, y), 2, (0, 255, 0), -1)  # green dots
                
                # check if both eyes are closed based on EAR threshold
                if left_ear > self.EAR_THRESHOLD and right_ear > self.EAR_THRESHOLD:
                    self.closed_eyes_counter += 1
                    if self.eyes_closed_start is None and self.closed_eyes_counter >= self.EYE_AR_CONSEC_FRAMES:
                        self.eyes_closed_start = time.time()
                        print(f"Eyes closed at {time.ctime(self.eyes_closed_start)}")
                else:
                    self.closed_eyes_counter = 0
                    if self.eyes_closed_start is not None:
                        print("Eyes opened - resetting timer")
                    self.eyes_closed_start = None
                
                # display status on frame
                if self.closed_eyes_counter >= self.EYE_AR_CONSEC_FRAMES:
                    if self.eyes_closed_start:
                        elapsed_time = time.time() - self.eyes_closed_start
                        cv2.putText(frame, f"Eyes Closed: {elapsed_time:.1f}s", (50, 50), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                    else:
                        cv2.putText(frame, "Eyes Closed", (50, 50), 
                                  cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
                else:
                    cv2.putText(frame, "Eyes Open", (50, 50), 
                              cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                
                # check if eyes have been closed for threshold time
                if (self.closed_eyes_counter >= self.EYE_AR_CONSEC_FRAMES and 
                    self.eyes_closed_start and 
                    time.time() - self.eyes_closed_start >= self.CLOSE_EYES_TIME_THRESHOLD):
                    print("Triggering alarm - eyes closed for threshold time")
                    try:
                        # play alarm multiple times
                        for _ in range(3):
                            winsound.Beep(1000, 1000)  # lower pitch for better alert
                            time.sleep(0.5)
                        # show warning message
                        self.root.after(0, lambda: messagebox.showwarning(
                            "Warning", "Eyes closed for 30 seconds. Please stay alert!"))
                    except Exception as e:
                        print(f"Error playing alarm: {e}")
                        self.root.after(0, lambda: messagebox.showwarning(
                            "Warning", "Eyes closed for 30 seconds. Please stay alert!"))
                    # reset counters
                    self.eyes_closed_start = None
                    self.closed_eyes_counter = 0
        else:
            cv2.putText(frame, "No Face Detected", (50, 50), 
                      cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 0, 255), 2)
            self.status_label.configure(text="Status: No face detected")
            self.closed_eyes_counter = 0
            self.eyes_closed_start = None
                
        # show updated frame
        cv2.imshow('Sleep Detection', frame)
        cv2.waitKey(1)
        
        # schedule next loop
        self.root.after(100, self.sleep_detection_loop)
        
    def start_scrolling(self):
        if self.is_sleep_detection_running:
            self.root.after(0, lambda: messagebox.showwarning("Warning", "Please stop sleep detection first"))
            return
        self.is_scrolling_running = True
        self.scroll_button.configure(state="disabled")
        self.stop_scroll_button.configure(state="normal")
        self.status_label.configure(text="Status: Scrolling Control Active")
        self.camera_thread = threading.Thread(target=self.run_scrolling)
        self.camera_thread.daemon = True
        self.camera_thread.start()
        
    def stop_scrolling(self):
        self.is_scrolling_running = False
        self.scroll_button.configure(state="normal")
        self.stop_scroll_button.configure(state="disabled")
        self.status_label.configure(text="Status: Ready")
        if self.cap is not None:
            if hasattr(self.cap, 'isOpened') and self.cap.isOpened():
                self.cap.release()
            self.cap = None
        cv2.destroyAllWindows()
        if self.camera_thread and self.camera_thread.is_alive():
            self.camera_thread.join(timeout=1.0)
            
    def run_scrolling(self):
        self.cap = cv2.VideoCapture(0)
        if not self.cap.isOpened():
            messagebox.showerror("Error", "Could not open camera")
            self.stop_scrolling()
            return
            
        while self.is_scrolling_running:
            ret, frame = self.cap.read()
            if not ret:
                break
                
            frame_rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
            results = self.face_mesh.process(frame_rgb)
            
            if results.multi_face_landmarks:
                for face_landmarks in results.multi_face_landmarks:
                    landmark1 = face_landmarks.landmark[1]  # nose tip
                    landmark152 = face_landmarks.landmark[152]  # chin
                    
                    H, W, _ = frame.shape
                    x1, y1 = int(landmark1.x * W), int(landmark1.y * H)
                    x152, y152 = int(landmark152.x * W), int(landmark152.y * H)
                    
                    y_diff = y152 - y1
                    if 50 < y_diff < 85:
                        pyautogui.scroll(-10)
                        cv2.putText(frame, "Scrolling Down", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                    elif y_diff > 100:
                        pyautogui.scroll(10)
                        cv2.putText(frame, "Scrolling Up", (50, 50), cv2.FONT_HERSHEY_SIMPLEX, 1, (0, 255, 0), 2)
                        
            cv2.imshow('Face Scrolling', frame)
            if cv2.waitKey(1) & 0xFF == ord('q'):
                break
                
        if self.cap is not None:
            if hasattr(self.cap, 'isOpened') and self.cap.isOpened():
                self.cap.release()
            self.cap = None
        cv2.destroyAllWindows()
        
    def calculate_ear(self, eye):
        # calculate distance between points
        def distance(p1, p2):
            return ((p1.x - p2.x) ** 2 + (p1.y - p2.y) ** 2) ** 0.5
            
        A = distance(eye[1], eye[5])
        B = distance(eye[2], eye[4])
        C = distance(eye[0], eye[3])
        
        # calculate ear
        ear = (A + B) / (2.0 * C)
        return ear
        
    def cleanup_resources(self):
        # stop any running features
        if self.is_sleep_detection_running:
            self.stop_sleep_detection()
        if self.is_scrolling_running:
            self.stop_scrolling()
        
        # release camera
        if self.cap is not None:
            if hasattr(self.cap, 'isOpened') and self.cap.isOpened():
                self.cap.release()
            self.cap = None
        
        # close all opencv windows
        cv2.destroyAllWindows()
        
        # cleanup mediapipe
        if self.face_mesh is not None:
            self.face_mesh.close()
            self.face_mesh = None

    def on_closing(self):
        if messagebox.askokcancel("Quit", "Do you want to quit?"):
            self.cleanup_resources()
            self.root.quit()
            self.root.destroy()  # ensure window is destroyed

    def run(self):
        try:
            self.root.protocol("WM_DELETE_WINDOW", self.on_closing)
            self.root.mainloop()
        finally:
            self.cleanup_resources()

if __name__ == "__main__":
    app = FaceSyncApp()
    app.run() 