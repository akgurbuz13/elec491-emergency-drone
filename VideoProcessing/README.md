# Emergency Drone Video Processing System

This system processes live video feeds from a DJI Mini 2 drone (via the Litchi app) to detect fires and earthquake damage. It can automatically send emergency messages through an attached LoRa communication module when threats are detected.

## System Architecture

1.  **Drone Video Feed**: DJI Mini 2 drone sends video to the Litchi app running on an iPhone.
2.  **RTMP Streaming**: The Litchi app on the iPhone streams the video feed directly to an RTMP server running on the MacBook Pro via a local Wi-Fi network (e.g., TP-Link soft AP).
3.  **RTMP Server (MacBook)**: An Nginx server with the RTMP module, managed by `rtmp_server.py`, receives the stream.
4.  **Video Processing (MacBook)**: `video_processor.py` consumes the RTMP stream. It uses YOLOv8 models for fire and ML-based damage detection, and SSIM for structural change detection against reference images. Detections can be toggled on/off during runtime via keyboard input.
5.  **LoRa Integration (MacBook to ESP32)**: `lora_bridge.py` handles serial communication with an ESP32 (e.g., GroundTransceiver1) connected via USB. When the video processor detects an emergency, it instructs the LoRa bridge to send a formatted message to the ESP32.
6.  **LoRa Transmission (ESP32)**: The ESP32 receives commands from the LoRa bridge and transmits the emergency message over the LoRa network. It should also report ACK status back to the LoRa bridge.
7.  **User Interface**: A live video window displays the feed with detection overlays, status information (FPS, detection modes), and LoRa message ACK statuses. Console output provides detailed logs.

## Prerequisites

### Hardware
*   MacBook Pro (or other macOS device capable of running Nginx and Python ML libraries).
*   iPhone with the [Litchi for DJI app](https://flylitchi.com/) installed.
*   DJI Mini 2 drone (or other DJI drone compatible with Litchi RTMP streaming).
*   TP-Link router (or similar) to create a local, offline Wi-Fi network (Soft AP).
*   ESP32 LoRa device (configured as Ground Transceiver 1) connectable to the MacBook via USB.

### Software (macOS)
*   macOS (tested on Ventura/Sonoma).
*   Python 3.8+ (preferably 3.9+).
*   Homebrew package manager.
*   Nginx (specifically `nginx-full` with the RTMP module).
*   Python dependencies as listed in `requirements.txt`.

## Installation

### 1. Clone the Repository
```bash
git clone <your-repository-url>
cd LoraComm/VideoProcessing
```

### 2. Install System Dependencies

#### Install Homebrew (if not already installed):
```bash
/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"
```

#### Install Nginx with RTMP module:
This is crucial for the RTMP server.
```bash
brew tap denji/nginx # Tap for nginx-full
brew install nginx-full --with-rtmp-module
```
To verify Nginx installation and RTMP module:
```bash
nginx -V 
```
Look for `--add-module=nginx-rtmp-module` in the output.
If you have a standard Nginx installed, you might need to uninstall it (`brew uninstall nginx`) before installing `nginx-full`.

### 3. Install Python Dependencies
The system includes a helper script to manage this.
```bash
python3 start_system.py --install-deps
```
This will create necessary directories and install packages from `requirements.txt`.
Alternatively, to install manually:
```bash
# Ensure directories exist (start_system.py --install-deps does this too)
mkdir -p models reference_images logs nginx_config nginx_logs rtmp_streams

# Install packages
pip3 install -r requirements.txt
```

### 4. Download Detection Models (Optional but Recommended)
*   The system defaults to `yolov8n.pt` (a small, general model) if custom models aren't found. This model will be downloaded automatically by Ultralytics if not present but may not be specialized for fire/damage.
*   For improved accuracy, download pre-trained YOLOv8 models for fire and building damage detection (usually `.pt` files).
    *   Search for relevant models on resources like Hugging Face, Roboflow Universe, or train your own.
*   Place the downloaded models into the `VideoProcessing/models/` directory.
    *   Example: `VideoProcessing/models/fire_detection_yolov8.pt` and `VideoProcessing/models/damage_detection_yolov8.pt`.
    *   Update the paths in `video_processor.py` or use command-line arguments if you use different names.

### 5. Add Reference Images for SSIM Damage Detection
For structural change detection using SSIM:
1.  Capture baseline images of important campus buildings or areas from drone perspectives.
2.  Name each image descriptively, as this name will appear in alerts (e.g., `library_north_face.jpg`, `engineering_block_roof.jpg`).
3.  Store these images in the `VideoProcessing/reference_images/` directory.

## Usage

### System Setup Checklist
1.  **Network**: Power on the TP-Link router. Connect both the MacBook and the iPhone to this local Wi-Fi network.
2.  **LoRa Hardware**: Connect the ESP32 (GroundTransceiver1) to the MacBook via USB.
3.  **Identify Serial Port**: Determine the serial port for the ESP32. On macOS, it's typically like `/dev/tty.SLAB_USBtoUART` or `/dev/tty.usbserial-XXXX`.
    ```bash
    ls /dev/tty.*
    ```
4.  **Drone & Litchi**: Launch the drone. On your iPhone, open the Litchi app and connect to the drone.

### Starting the Full System

**Basic command (ensure you replace `--lora-port`):**
```bash
python3 start_system.py --lora-port /dev/tty.SLAB_USBtoUART
```

**With all options:**
```bash
python3 start_system.py \
    --lora-port /dev/tty.SLAB_USBtoUART \
    --lora-baudrate 115200 \
    --rtmp-port 1935 \
    --rtmp-key drone \
    --reference-dir "./reference_images" \
    --fire-model "./models/your_fire_model.pt" \
    --damage-model "./models/your_damage_model.pt" \
    # --disable-fire-detection \
    # --disable-damage-detection
```

*   `--disable-fire-detection`: Starts with fire detection off.
*   `--disable-damage-detection`: Starts with damage detection off.

Upon successful start, an Nginx RTMP server will be running, the LoRa bridge will be active, and the video processor will attempt to connect to the RTMP stream. A window titled "Emergency Detection Video Feed" will appear.

### iPhone Litchi App RTMP Streaming Setup
Run `python3 start_system.py --instructions` to see specific instructions including your MacBook's detected local IP. General steps:

1.  Ensure the `start_system.py` script is running on your MacBook (this starts the RTMP server).
2.  In the Litchi app (connected to your drone):
    *   Navigate to Litchi's settings.
    *   Find the "RTMP Live Streaming" or similar video output/streaming option.
    *   Enter the RTMP URL: `rtmp://<YOUR_MACBOOK_IP>:<PORT>/<KEY>/live`
        *   Example: `rtmp://192.168.1.101:1935/drone/live` (Replace IP, port, key as per your setup).
3.  Start the stream from Litchi.

### Interacting with the Video Feed Window
*   **`F` key**: Toggle Fire Detection ON/OFF.
*   **`D` key**: Toggle Damage Detection (ML & SSIM) ON/OFF.
*   **`Q` key**: Quit the video processing window (and signals the system to shut down).

### Starting Individual Components (for testing/debugging)

*   **RTMP Server Only**:
    ```bash
    python3 start_system.py --rtmp-only
    # Or directly:
    # python3 rtmp_server.py --port 1935 --key drone 
    ```

*   **LoRa Bridge Only (Interactive Mode for sending test messages)**:
    ```bash
    python3 lora_bridge.py --port /dev/tty.SLAB_USBtoUART
    ```

*   **Video Processor (requires RTMP server and optionally LoRa bridge running elsewhere or mocked)**:
    *Note: `video_processor.py` is not intended to be run standalone in the new design without a proper `LoraBridge` instance. It is started by `start_system.py`.*

## Detection Logic

*   **Fire Detection**: Uses a YOLOv8 model. Alerts are sent via LoRa if enabled.
*   **Damage Detection**:
    *   **ML-based**: Uses a YOLOv8 model for cracks, debris etc.
    *   **SSIM-based**: Compares current frame to reference images. Significant deviation below a threshold triggers a warning.
    *   Alerts are sent via LoRa if enabled.
*   **Alert Cooldown**: A 30-second cooldown per detection type (fire/damage) prevents message spam.
*   **ACK Handling**: The system displays the status of LoRa messages (e.g., "sent", "delivered", "failed") on the video feed, based on feedback from the ESP32 via `lora_bridge.py`.

## ESP32 (GroundTransceiver1) Serial JSON for ACKs
For the Python system to display LoRa ACK statuses, your `GroundTransceiver1.ino` code (or whichever ESP32 is connected to the Mac via USB) needs to send JSON messages back over serial when it gets a final ACK for a message it transmitted on behalf of the Python system.

**Example JSON from ESP32 to `lora_bridge.py`:**
```json
{"type": "ack_status", "messageId": 1678886400, "status": "delivered", "rssi": -75, "snr": 8.2}
```
*   `type`: Must be `"ack_status"`.
*   `messageId`: The *original* message ID sent by the Python system.
*   `status`: Can be `"delivered"`, `"failed"`, `"timeout"`, or other relevant status.
*   Include any other details like `rssi`, `snr` which will be logged by `lora_bridge.py`.

## Logs and Output

*   **Main System Log**: `VideoProcessing/system.log`
*   **LoRa Bridge Log**: `VideoProcessing/lora_bridge.log`
*   **Video Detection Log**: `VideoProcessing/logs/detection_log_<timestamp>.txt` (contains alert details)
*   **Saved Detection Frames**: `VideoProcessing/logs/fire_<timestamp>.jpg` and `VideoProcessing/logs/damage_<timestamp>.jpg`
*   **Nginx Logs**: `VideoProcessing/nginx_logs/` (for RTMP server debugging)

## Troubleshooting

*   **No RTMP Stream**: Check Nginx logs. Ensure Litchi has the correct RTMP URL and is actively streaming. Verify both devices are on the same local Wi-Fi. Use VLC to test the stream on the Mac: `rtmp://localhost:1935/drone/live`.
*   **Dependencies Not Found**: Ensure Homebrew and Python 3 are in your PATH. Run `python3 start_system.py --install-deps`.
*   **LoRa Issues**: Check `lora_bridge.log`. Confirm correct serial port and baud rate. Ensure ESP32 firmware is correctly handling serial JSON messages and sending ACK status updates.
*   **Model Issues**: If detections are poor, ensure your `.pt` model files are in the `models` directory and paths are correct. Consider using models fine-tuned for your specific needs.

## Standalone Video Processing Testing (Without LoRa/Full Network)

You can test the video processing capabilities (RTMP ingestion, model detection, UI) on your MacBook without setting up the full LoRa hardware or the TP-Link local Wi-Fi for the drone.

**Objective**: Stream a test video to a local RTMP server on your Mac and have `video_processor.py` process it.

**Steps:**

1.  **Ensure Nginx with RTMP is Installed:**
    *   Follow the Nginx installation steps in the "Installation" section above if you haven't already.
    *   Verify with `nginx -V` that the RTMP module is present.

2.  **Start the Local RTMP Server:**
    *   Open a terminal in the `LoraComm/VideoProcessing/` directory.
    *   Run the RTMP server script. This will use the `nginx.conf` generated in `nginx_config/`.
        ```bash
        python3 rtmp_server.py
        ```
    *   Or, use the system launcher in RTMP-only mode:
        ```bash
        python3 start_system.py --rtmp-only
        ```
    *   This will typically start an RTMP server at `rtmp://localhost:1935/drone/live` (or as configured).
    *   Keep this terminal window open. You should see logs indicating Nginx has started.

3.  **Prepare a Test Video Source to Stream via RTMP:**
    You need an application that can send an RTMP stream. Here are a few options:

    *   **Option A: Using OBS Studio (Recommended for flexibility):**
        1.  Download and install [OBS Studio](https://obsproject.com/) (free, open-source).
        2.  Open OBS. In the "Sources" panel, click `+`.
        3.  Add a source: 
            *   **Media Source**: To stream a video file. Select your video file. Check "Loop" if you want it to play continuously.
            *   **Video Capture Device**: To use your MacBook's webcam.
            *   **Window Capture** or **Display Capture**: To stream a portion of your screen.
        4.  Go to `File > Settings > Stream`.
        5.  Set `Service` to `Custom...`.
        6.  Set `Server` to `rtmp://localhost:1935/drone` (note: no `/live` here for OBS server config usually, the stream key handles that).
        7.  Set `Stream Key` to `live` (this is the `/live` part of the Litchi URL `rtmp://.../drone/live`).
        8.  Click `Apply` and `OK`.
        9.  In the main OBS window, click `Start Streaming`.

    *   **Option B: Using FFmpeg (Command-line tool):**
        1.  Ensure FFmpeg is installed (`brew install ffmpeg`).
        2.  To stream a video file (`test_video.mp4` located in the current directory) in a loop:
            ```bash
            ffmpeg -re -stream_loop -1 -i test_video.mp4 -c:v libx264 -preset veryfast -b:v 2000k -maxrate 2000k -bufsize 4000k -pix_fmt yuv420p -g 50 -c:a aac -b:a 128k -ar 44100 -f flv rtmp://localhost:1935/drone/live
            ```
            (Adjust video/audio parameters as needed. `-re` simulates a live stream.)

    *   **Option C: Using a Camera App on your Mac/Phone that supports custom RTMP output:**
        *   Some advanced camera apps allow you to specify a custom RTMP server URL. Configure it to point to `rtmp://localhost:1935/drone/live` (if streaming from the same Mac) or `rtmp://<YOUR_MAC_LOCAL_IP>:1935/drone/live` (if streaming from a phone on the *same local test network* as your Mac - for this test, your regular home Wi-Fi is fine if both devices are on it).

4.  **Run the Video Processing System (without LoRa port):**
    *   Open a **new terminal window** in the `LoraComm/VideoProcessing/` directory.
    *   Run `start_system.py`. Since you are not connecting the ESP32, omit the `--lora-port` argument. The system will warn about no LoRa port but proceed.
        ```bash
        python3 start_system.py
        ```
        *   You can also specify initial detection states:
            ```bash
            python3 start_system.py --disable-fire-detection --disable-damage-detection
            ```

5.  **Observe and Interact:**
    *   An OpenCV window titled "Emergency Detection Video Feed" should appear on your MacBook.
    *   You should see the video you are streaming from OBS, FFmpeg, or your camera app.
    *   **Keyboard Controls in Video Window:**
        *   `F`: Toggle Fire Detection ON/OFF.
        *   `D`: Toggle Damage Detection ON/OFF.
        *   `Q`: Quit the video processor and shut down the system.
    *   Check the console output in both terminals (RTMP server and `start_system.py`) for logs, detection messages, and status updates.
    *   If detection models are present in the `models` folder and detections are enabled, you should see bounding boxes and alerts if the test video contains relevant content.

6.  **Stopping the Test:**
    *   Press `Q` in the OpenCV video window, or `Ctrl+C` in the terminal running `start_system.py`.
    *   Then, stop your RTMP stream (e.g., click `Stop Streaming` in OBS, or `Ctrl+C` in the FFmpeg terminal).
    *   Finally, `Ctrl+C` in the terminal running `rtmp_server.py` (if you ran it directly) to stop Nginx.

This setup allows you to verify the video pipeline, the detection algorithms, and the user interface components of the Python application independently.

## License
This project is licensed under the MIT License. See the `LICENSE` file in the main project directory. 