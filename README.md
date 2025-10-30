# Face-Sync: Real-Time Face-Based Applications

This project provides a real-time application that integrates two features:
1. **Sleep Detection**: Monitors the user’s eye state (open or closed) to detect drowsiness. If the user has their eyes closed for a specified period (e.g., 5 minutes), an alarm will be triggered.
2. **Face Movement-Based Scrolling**: Detects facial landmarks, especially the distance between the nose and chin, to control scrolling functionality in a web page or application.

The project is built with **Python** using libraries like **OpenCV**, **MediaPipe**, and **PyAutoGUI**, and provides a GUI built using **tkinter** to control these features interactively.

---

## Features

- **Real-Time Sleep Detection**: Uses the Eye Aspect Ratio (EAR) for detecting closed eyes and triggers a notification after a set time of inactivity.
- **Face Gesture Scrolling**: Controls scrolling direction (up or down) based on the relative position of the nose tip and chin using facial landmarks.
- **Interactive GUI**: Provides buttons to control and toggle both features independently for real-time use.

---

## Requirements

To run the project, you need to have the following Python libraries installed:
- **OpenCV**: For real-time video capture and image processing.
- **MediaPipe**: To process facial landmarks and detect eye movement.
- **PyAutoGUI**: To control mouse and scrolling.
- **Tkinter**: For creating the GUI.
- **Winsound**: For playing the alarm sound on detecting drowsiness.

You can install the required libraries with the following commands:

```bash
pip install opencv-python mediapipe pyautogui tk
