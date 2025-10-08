Robostem Interface 2025
===================================

Robostem Interface 2025 is a Python-based graphical interface designed to connect computer vision and sensor subsystems used in robotics competitions and field testing.
It integrates video streaming, hazard material (HAZMAT) detection, QR-code recognition, and magnetometer readings into a unified monitoring and control tool.

--------------------------------------------------------------------------------
1. Overview
--------------------------------------------------------------------------------

This interface acts as the central dashboard for visual and magnetic sensing.
It communicates with an ESP32-CAM camera and an ESP32-based FXOS8700 magnetometer over Wi-Fi. 
The system provides real-time visualization, data annotation, and user control over camera parameters.

--------------------------------------------------------------------------------
2. Hardware Architecture
--------------------------------------------------------------------------------

The system relies on two external microcontrollers connected via Wi-Fi:

1. ESP32-CAM
   - Captures live video using the onboard OV2640 sensor.
   - Streams MJPEG video via HTTP at http://<IP>:81/stream.
   - Exposes a configuration endpoint (/set) for adjusting parameters such as brightness, contrast, exposure, and color balance.

2. ESP32 + FXOS8700 Magnetometer
   - Measures magnetic field in microteslas (µT) along X, Y, Z axes.
   - Serves readings via simple JSON (single request) or as a continuous NDJSON stream (for live plotting).

These devices communicate over a local Wi-Fi network. The Python interface consumes the data streams and provides a synchronized visualization environment.

--------------------------------------------------------------------------------
3. Software Pipeline
--------------------------------------------------------------------------------

The interface follows this processing chain:

1. Frame Acquisition
   - The ESP32-CAM continuously streams MJPEG frames.
   - The GUI captures these frames using OpenCV's VideoCapture in a background thread.

2. Object Detection (optional)
   - A YOLO model (Darknet weights loaded through OpenCV DNN) runs on each frame.
   - It detects HAZMAT placards and annotates the bounding boxes and class labels in real time.

3. QR Code Detection (optional)
   - The same frame is passed through OpenCV’s QRCodeDetector.
   - Decoded text and polygon boundaries are drawn directly over the frame.

4. Data Integration
   - A separate worker thread listens to magnetometer readings (via HTTP stream).
   - The readings are displayed numerically and optionally logged or plotted.

5. Visualization
   - All frames and data are rendered inside a PyQt6 interface with real-time overlays and control widgets.
   - User can toggle subsystems, tune camera parameters, and monitor sensor data simultaneously.

--------------------------------------------------------------------------------
4. Setup Instructions
--------------------------------------------------------------------------------

1. Install dependencies
   pip install -r requirements.txt

2. Connect both ESP32 devices to the same Wi-Fi network.

3. Find their IP addresses from the serial monitor and note them down.

4. Run the interface:
   python gui/main.py

5. Enter your stream URLs:
   - Camera: http://<CAM_IP>:81/stream
   - Magnetometer: http://<MAG_IP>/magnetometer/stream

6. Enable or disable modules using the checkboxes in the GUI.

--------------------------------------------------------------------------------
5. Extending the System
--------------------------------------------------------------------------------

- To add new detectors, create a new Python module similar to hazmat.py.
- Use OpenCV DNN or a custom ML model and connect it to the frame processing thread.
- New sensors can be added by extending magnetometer.py and defining a new worker class that reads from additional endpoints.
- The GUI is modular: each detection or data source can emit signals that are connected to the interface dynamically.

--------------------------------------------------------------------------------
6. Requirements
--------------------------------------------------------------------------------
pyqt6
opencv-python
numpy
requests
-------------------------------------------------------------------------------- 
