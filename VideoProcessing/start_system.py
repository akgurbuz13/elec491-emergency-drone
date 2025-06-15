#!/usr/bin/env python3
import os
import sys
import time
import argparse
import subprocess
import threading
import signal
import queue # Added for thread-safe communication
# import json # No longer directly used here for message construction
from lora_bridge import LoraBridge, EmergencyMessage # EmergencyMessage might still be used for test msg
from video_processor import VideoProcessor, FIRE_MODEL_PATH, DAMAGE_MODEL_PATH, DEFAULT_MODEL_DIR, DEFAULT_REFERENCE_DIR # Import the updated VideoProcessor and constants
import logging
import cv2

# Configure logging
logging.basicConfig(
    level=logging.INFO,
    format='%(asctime)s - %(name)s - %(levelname)s - %(message)s',
    handlers=[
        logging.StreamHandler(sys.stdout),
        logging.FileHandler(os.path.join(os.path.dirname(os.path.abspath(__file__)), "system.log"))
    ]
)
logger = logging.getLogger('System Launcher')

# Global variables for managing components and threads
# rtmp_server_instance: RTMPServer = None # Removed
lora_bridge_instance: LoraBridge = None
video_processor_instance: VideoProcessor = None

lora_bridge_thread: threading.Thread = None
video_processor_thread: threading.Thread = None

# Global args for access in cleanup functions
args = None
# Queues for GUI mode, need to be global for cleanup as well
frame_queue = None
keypress_queue = None

# Keep track of subprocesses for Nginx (managed by RTMPServer)
# child_processes = [] # Replaced by direct management within RTMPServer


# def start_rtmp_server_component(stream_key="drone", port=1935): # Removed function
#     """Start the RTMP server component."""
#     global rtmp_server_instance
#     logger.info("Initializing RTMP server...")
#     rtmp_server_instance = RTMPServer(port=port, stream_key=stream_key)
#     if not rtmp_server_instance.start_server(): 
#         logger.error("Failed to start RTMP server component")
#         return False, None
#     logger.info(f"RTMP Server component started. URL: {rtmp_server_instance.get_rtmp_url()}")
#     return True, rtmp_server_instance.get_rtmp_url()


def start_lora_bridge_component(serial_port, baudrate=115200):
    """Initialize and start the LoRa bridge component in a new thread."""
    global lora_bridge_instance, lora_bridge_thread
    logger.info(f"Initializing LoRa bridge on {serial_port}...")
    lora_bridge_instance = LoraBridge(port=serial_port, baudrate=baudrate)
    
    if not lora_bridge_instance.connect(): # Try initial connection
        logger.error(f"Initial connection to LoRa module on {serial_port} failed. Bridge will attempt to connect in its thread.")
        # We can still start the thread, it will keep trying to connect.

    lora_bridge_thread = threading.Thread(target=lora_bridge_instance._run, daemon=True)
    lora_bridge_instance.running = True # Set running before starting thread
    lora_bridge_thread.start()
    logger.info("LoRa bridge component thread started.")
    
    # Optional: Send a test message to confirm connectivity if initially connected
    if lora_bridge_instance.connected:
        test_msg = EmergencyMessage(
            text="System Launcher: LoRa Bridge Online", 
            emergency_code=3  # Code 3 = Other/System
        )
        lora_bridge_instance.send_message(test_msg)
        logger.info("Sent LoRa bridge startup test message.")
    return True


def start_video_processor_component(lora_bridge_ref: LoraBridge,
                                   fire_model=None, damage_model=None, reference_dir=None,
                                   fire_enabled=True, damage_enabled=True, headless_mode=False,
                                   frame_q=None, key_q=None, person_enabled=True,
                                   person_model=None, 
                                   preferred_camera_index=-1):
    """Initialize and start the video processor component in a new thread."""
    global video_processor_instance, video_processor_thread
    logger.info("Initializing video processor...")
    
    video_processor_instance = VideoProcessor(
        lora_bridge_instance=lora_bridge_ref,
        fire_model_path=fire_model,
        damage_model_path=damage_model,
        reference_dir=reference_dir,
        initial_fire_detection_enabled=fire_enabled,
        initial_damage_detection_enabled=damage_enabled,
        headless_mode=headless_mode,
        frame_queue=frame_q,
        keypress_queue=key_q,
        initial_person_detection_enabled=person_enabled,
        person_model_path=person_model,
        preferred_camera_index=preferred_camera_index
    )
    
    video_processor_thread = threading.Thread(target=video_processor_instance.start_processing_loop, daemon=True)
    video_processor_thread.start()
    logger.info("Video processor component thread started.")
    return True


def cleanup_components():
    """Clean up all components and threads."""
    logger.info("Initiating component cleanup...")
    global shutdown_in_progress, args, frame_queue, keypress_queue
    shutdown_in_progress = True # Ensure this is set early
    
    # 1. Signal VideoProcessor to stop and join its thread
    if video_processor_instance:
        logger.info("Stopping video processor instance...")
        video_processor_instance.running = False
        # If GUI queues exist, send sentinel to video_processor's keypress_queue 
        # to ensure it unblocks from keypress_queue.get() if it's waiting.
        # Also send sentinel to frame_queue from video_processor if it didn't exit cleanly.
        if not args.headless and keypress_queue: # Check if keypress_queue was initialized
            try: keypress_queue.put_nowait(None) # Sentinel for keypress queue
            except queue.Full:
                 logger.warning("Keypress queue full during shutdown signal.")
            except Exception as e:
                 logger.warning(f"Exception putting None to keypress_queue: {e}")

        if video_processor_thread and video_processor_thread.is_alive():
            logger.info("Waiting for video processor thread to join...")
            video_processor_thread.join(timeout=7) # Increased timeout slightly
            if video_processor_thread.is_alive():
                 logger.warning("Video processor thread did not terminate gracefully after 7s.")
        video_processor_instance.stop() # Call its internal stop for CV windows etc.
        logger.info("Video processor component stopped.")

    # 2. Signal LoRaBridge to stop and join its thread
    if lora_bridge_instance:
        logger.info("Stopping LoRa bridge instance...")
        lora_bridge_instance.running = False
        if lora_bridge_thread and lora_bridge_thread.is_alive():
            logger.info("Waiting for LoRa bridge thread to join...")
            lora_bridge_thread.join(timeout=5)
            if lora_bridge_thread.is_alive():
                 logger.warning("LoRa bridge thread did not terminate gracefully.")
        lora_bridge_instance.stop()
        logger.info("LoRa bridge component stopped.")

    # 3. Stop RTMP Server (Nginx) # Removed section
    # if rtmp_server_instance:
    #     logger.info("Stopping RTMP server instance...")
    #     rtmp_server_instance.stop_server()
    #     logger.info("RTMP server component stopped.")
    
    logger.info("Component cleanup complete.")

# Global flag to indicate if shutdown is in progress
shutdown_in_progress = False

def signal_handler_main(sig, frame):
    global shutdown_in_progress, args
    if shutdown_in_progress:
        logger.info("Shutdown already in progress, ignoring signal.")
        return
    shutdown_in_progress = True
    logger.info(f"Received signal {signal.Signals(sig).name if isinstance(sig, int) else sig}, shutting down all components...")
    cleanup_components()
    logger.info("Exiting system launcher.")
    sys.exit(0)

def check_dependencies():
    """Check if the system has all required dependencies"""
    dependencies = [
        ('python3', 'Python 3 is required'),
        ('pip3', 'Pip3 is required to install Python packages'),
        # ('nginx', 'Nginx is required for RTMP server (install with: brew install nginx-full --with-rtmp-module)') # Removed nginx check
        ('ffmpeg', 'FFmpeg is required for video processing (install with: brew install ffmpeg).') # ffmpeg might still be used by OpenCV backend for cameras
    ]
    
    missing = []
    for cmd, msg in dependencies:
        # Use shutil.which for better cross-platform compatibility if available, else fallback
        try:
            import shutil
            if shutil.which(cmd) is None:
                missing.append(f"{cmd}: {msg}")
        except ImportError:
            if subprocess.run(['which', cmd], stdout=subprocess.PIPE, stderr=subprocess.PIPE).returncode != 0:
                missing.append(f"{cmd}: {msg}")
    
    python_packages = [
        'opencv-python',
        'numpy',
        'ultralytics',
        'pyserial',
        'scikit-image'
        # 'pkg_resources' is part of setuptools, usually available
    ]
    
    try:
        import pkg_resources # For checking python package versions
        installed_packages = {pkg.key for pkg in pkg_resources.working_set}
        for package_name in python_packages:
            # Normalize package name for checking (e.g., opencv-python -> opencv_python)
            normalized_name = package_name.lower().replace('-', '_')
            if normalized_name not in installed_packages and package_name not in installed_packages:
                missing.append(f"Python package '{package_name}' is missing. Install with 'pip3 install {package_name}' or use --install-deps")
    except ImportError:
        logger.warning("Module 'pkg_resources' not found. Cannot check Python package versions accurately. Please ensure all packages from requirements.txt are installed.")
    except Exception as e:
        logger.warning(f"Could not check Python packages due to an error: {e}")
    
    return missing

def install_requirements():
    """Install required Python packages from requirements.txt"""
    req_file = os.path.join(os.path.dirname(os.path.abspath(__file__)), "requirements.txt")
    if not os.path.exists(req_file):
        logger.error(f"requirements.txt not found at {req_file}. Cannot install dependencies.")
        return False
    try:
        logger.info(f"Installing Python packages from {req_file}...")
        subprocess.check_call([sys.executable, "-m", "pip", "install", "-r", req_file])
        logger.info("Installation complete.")
        # Verify ultralytics install (as it can be tricky)
        try:
            import ultralytics
            logger.info(f"Ultralytics version {ultralytics.__version__} detected.")
        except ImportError:
            logger.error("Ultralytics (YOLO) still not found after pip install. Please check installation manually.")
        return True
    except subprocess.CalledProcessError as e:
        logger.error(f"Failed to install requirements: {e}. Check pip output for details.")
        return False
    except Exception as e:
        logger.error(f"An unexpected error occurred during pip install: {e}")
        return False

def create_dir_structure():
    """Create necessary directory structure"""
    base_path = os.path.dirname(os.path.abspath(__file__))
    dirs_to_create = [
        os.path.join(base_path, "models"),
        os.path.join(base_path, "reference_images"),
        os.path.join(base_path, "logs"),
        # os.path.join(base_path, "nginx_config"), # Removed
        # os.path.join(base_path, "nginx_logs"),   # Removed
        # os.path.join(base_path, "rtmp_streams") # Removed
    ]
    
    for d_path in dirs_to_create:
        try:
            os.makedirs(d_path, exist_ok=True)
            logger.info(f"Ensured directory exists: {d_path}")
        except OSError as e:
            logger.error(f"Failed to create directory {d_path}: {e}")

# def show_rtmp_instructions(rtmp_port=1935, rtmp_key="drone"): # Removed function
#     """Show instructions for streaming from Litchi to the RTMP server"""
#     # Try to get local IP (best effort for user convenience)
#     local_ip = "<YOUR_MACBOOK_IP_ON_TP_LINK_NETWORK>"
#     try:
#         # This is a common way but might not always work or be the correct IP for the isolated network
#         import socket
#         s = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
#         s.connect(("8.8.8.8", 80)) # Connect to a known address to find local IP for that route
#         local_ip = s.getsockname()[0]
#         s.close()
#     except Exception:
#         pass # Stick to placeholder if detection fails

#     print("\n=======================================================================")
#     print("                Litchi App RTMP Streaming Instructions                   ")
#     print("=======================================================================")
#     print("1. Ensure your MacBook and iPhone are connected to the TP-Link WiFi network")
#     print("   (e.g., SSID: 'Emergency_KU')")
#     print("2. On your MacBook, ensure the Video Processing System is running.")
#     print("   The RTMP server should now be active on your MacBook.")
#     print("3. On your iPhone, open the Litchi for DJI app and connect to your drone.")
#     print("4. In Litchi's settings, find the RTMP streaming option.")
#     print("   (This might be under 'Video Settings' or a dedicated 'Streaming' section)")
#     print("5. Configure the RTMP stream with the following URL:")
#     print(f"   URL: rtmp://{local_ip}:{rtmp_port}/{rtmp_key}/live")
#     print("   If the IP above is incorrect, find your MacBook's IP address on the")
#     print("   TP-Link network (System Settings > Wi-Fi > Details... > IP Address)." )
#     print("6. Start the stream from Litchi.")
#     print("   You should see the video feed in the 'Emergency Detection Video Feed' window on your MacBook.")
#     print("=======================================================================")
#     print("Note: The exact steps in Litchi might vary slightly based on the app version.")
#     print("Refer to Litchi documentation if you cannot find the RTMP settings.")
#     print("=======================================================================\n")


def main():
    parser = argparse.ArgumentParser(description="Video Processing System Launcher")
    parser.add_argument("--lora-port", type=str, default=None, 
                        help="Serial port for LoRa module (e.g., /dev/tty.SLAB_USBtoUART or COM3). If not provided, LoRa operations will be simulated.")
    parser.add_argument("--fire-model", type=str, default=FIRE_MODEL_PATH,
                        help=f"Path to fire detection model or Roboflow model ID (e.g., project/version). Default: {FIRE_MODEL_PATH}")
    parser.add_argument("--damage-model", type=str, default=DAMAGE_MODEL_PATH,
                        help=f"Path to damage detection model. Default: {DAMAGE_MODEL_PATH}")
    parser.add_argument("--person-model", type=str, default=os.path.join(DEFAULT_MODEL_DIR, "yolov8s.pt"), # Default to yolov8s.pt for person
                        help=f"Path to person detection model. Default: {os.path.join(DEFAULT_MODEL_DIR, 'yolov8s.pt')}")
    parser.add_argument("--reference-dir", type=str, default=DEFAULT_REFERENCE_DIR,
                        help=f"Directory for reference images (SSIM). Default: {DEFAULT_REFERENCE_DIR}")
    parser.add_argument("--disable-fire", action="store_false", dest="fire_enabled",
                        help="Disable fire detection on startup")
    parser.add_argument("--disable-damage", action="store_false", dest="damage_enabled",
                        help="Disable damage detection on startup")
    parser.add_argument("--disable-person", action="store_false", dest="person_enabled", # New argument to disable person detection
                        help="Disable person detection on startup")
    parser.add_argument("--headless", action="store_true",
                        help="Run in headless mode (no GUI display, logs detections).")
    parser.add_argument("--camera-index", type=int, default=-1,
                        help="Preferred camera index for video input (e.g., 0, 1, -1 for auto). Default: -1")
    
    # Arguments for RTMP server (if managed by this script) # These are being removed
    # parser.add_argument("--rtmp-server-port", type=int, default=1935, help="Port for the internal RTMP server (default: 1935)") # Removed
    # parser.add_argument("--instructions", action="store_true", help="Show iPhone Litchi RTMP streaming setup instructions and exit") # Removed
    parser.add_argument("--install-deps", action="store_true", help="Install/update Python dependencies from requirements.txt and exit")
    # parser.add_argument("--rtmp-only", action="store_true", help="Start only the RTMP server (for testing streaming setup)") # Removed
    
    parser.set_defaults(fire_detection_enabled=True, damage_detection_enabled=True, person_detection_enabled=True)
    # Make args global so cleanup_components can access args.headless
    global args
    args = parser.parse_args()
    
    # Initialize queues for GUI mode - make them global for access in cleanup
    global frame_queue, keypress_queue
    if not args.headless:
        frame_queue = queue.Queue(maxsize=2)
        keypress_queue = queue.Queue(maxsize=5)
    else:
        frame_queue = None
        keypress_queue = None
    
    # Register signal handlers for graceful shutdown
    signal.signal(signal.SIGINT, signal_handler_main)
    signal.signal(signal.SIGTERM, signal_handler_main)
    
    # if args.instructions: # Removed call
    #     show_rtmp_instructions(args.rtmp_server_port, "drone")
    #     return 0
        
    create_dir_structure() # Ensure all necessary directories exist
    
    if args.install_deps:
        logger.info("Attempting to install dependencies...")
        if install_requirements():
            logger.info("Dependencies installed successfully. Please re-run the script without --install-deps to start the system.")
        else:
            logger.error("Failed to install all dependencies. Please check the logs.")
        return 0 if install_requirements() else 1
            
    missing_deps = check_dependencies()
    if missing_deps:
        logger.error("Critical dependencies are missing:")
        for dep_msg in missing_deps:
            logger.error(f"  - {dep_msg}")
        logger.error("Please install them or use --install-deps for Python packages and re-run.")
        return 1
    
    # rtmp_url_for_processor = args.rtmp_url # Removed, no longer used

    try:
        # rtmp_started, _ = start_rtmp_server_component(stream_key="drone", port=args.rtmp_server_port) # Removed RTMP server start
        # if not rtmp_started: # Removed
        #     logger.error("RTMP server failed to start. Exiting.") # Removed
        #     return 1 # Removed

        # if args.rtmp_only: # Removed
        #     logger.info("RTMP server started. Running in RTMP-only mode.") # Removed
        #     print("\nRTMP server is running. Press Ctrl+C to stop.") # Removed
        #     while not shutdown_in_progress: # Removed
        #         time.sleep(1) # Removed
        #     return 0 # Removed

        if args.lora_port:
            if not start_lora_bridge_component(args.lora_port):
                logger.error("LoRa bridge failed to start. Video processing will run without LoRa integration.")
                # lora_bridge_instance will be None, VideoProcessor should handle this
        else:
            logger.warning("No LoRa serial port specified. Video processing will run without LoRa integration.")

        if args.headless:
            logger.info("Headless mode enabled. No GUI window will be shown.")
            # Start video processor in headless mode (no queues needed from main)
            start_video_processor_component(
                lora_bridge_ref=lora_bridge_instance,
                fire_model=args.fire_model, damage_model=args.damage_model, reference_dir=args.reference_dir,
                fire_enabled=args.fire_enabled, damage_enabled=args.damage_enabled, headless_mode=True,
                person_enabled=args.person_enabled, person_model=args.person_model, 
                preferred_camera_index=args.camera_index
            )
        else: # GUI Mode - Main thread handles OpenCV window
            logger.info("Running in GUI mode. Main thread will handle video display and input.")
            
            # Start the video processor component first, so it can begin populating the queues
            start_video_processor_component(
                lora_bridge_ref=lora_bridge_instance,
                fire_model=args.fire_model, 
                damage_model=args.damage_model, 
                reference_dir=args.reference_dir,
                fire_enabled=args.fire_enabled, 
                damage_enabled=args.damage_enabled, 
                headless_mode=False, # Explicitly False for GUI mode
                frame_q=frame_queue,   # Pass the initialized frame_queue
                key_q=keypress_queue,  # Pass the initialized keypress_queue
                person_enabled=args.person_enabled, 
                person_model=args.person_model,
                preferred_camera_index=args.camera_index
            )

            window_name = "Emergency Detection Video Feed"
            try:
                cv2.namedWindow(window_name, cv2.WINDOW_NORMAL)
                cv2.setWindowProperty(window_name, cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_NORMAL)
                cv2.waitKey(1) # Allow window to draw

                while not shutdown_in_progress:
                    try:
                        frame = frame_queue.get(timeout=0.1) # Timeout to allow checking shutdown_in_progress
                        if frame is None: # Sentinel for end of stream
                            logger.info("Received sentinel from video processor. Closing display.")
                            break
                        cv2.imshow(window_name, frame)
                    except queue.Empty:
                        # No new frame, just continue to process keypresses and check shutdown
                        pass 
                    except Exception as e_gui_loop:
                        logger.error(f"Error in main GUI loop processing frame: {e_gui_loop}", exc_info=True)
                        # Potentially break or signal shutdown here if GUI fails catastrophically
                        # For now, we continue, VideoProcessor might go headless on its own or stop
                        pass # Continue loop to allow clean shutdown via other means

                    key = cv2.waitKey(1) & 0xFF
                    if key != 255: # A key was pressed
                        if key == ord('q') or key == ord('Q'):
                            logger.info("'Q' pressed in main window. Initiating shutdown.")
                            if not shutdown_in_progress: signal_handler_main(signal.SIGTERM, None)
                            break # Exit GUI loop
                        else:
                            # Send other keys to video processor thread
                            try:
                                keypress_queue.put_nowait(key)
                            except queue.Full:
                                logger.warning("Keypress queue full, video processor might be busy.")
                    
                    # Check if video processor thread died (e.g. if it had an internal error)
                    if video_processor_thread and not video_processor_thread.is_alive():
                        logger.error("Video processor thread died unexpectedly during GUI mode.")
                        if not shutdown_in_progress: signal_handler_main(signal.SIGTERM, None)
                        break # Exit GUI loop

            except Exception as e_main_gui:
                logger.error(f"Fatal error in main GUI setup or loop: {e_main_gui}", exc_info=True)
                if not shutdown_in_progress: signal_handler_main(signal.SIGTERM, None)
            finally:
                logger.info("Main GUI loop ended. Ensuring cleanup.")
                
                # Attempt to close OpenCV windows first
                try:
                    cv2.destroyAllWindows()
                    # Add a few waitKeys to help macOS process window closing events
                    for _ in range(5):
                        cv2.waitKey(1)
                    logger.info("OpenCV windows destroyed.")
                except Exception as e_destroy:
                    logger.error(f"Exception during cv2.destroyAllWindows(): {e_destroy}")

                # Then, signal and wait for the video processor thread to stop
                if video_processor_instance and video_processor_instance.running:
                    logger.info("Signaling video processor to stop from main GUI finally block.")
                    video_processor_instance.running = False
                    if keypress_queue: 
                        try: keypress_queue.put_nowait(ord('q')) 
                        except queue.Full:
                            logger.warning("Keypress queue full when trying to send final 'q'.")
                        except Exception as e_kp_q:
                            logger.warning(f"Exception sending final 'q' to keypress_queue: {e_kp_q}")
                
                if video_processor_thread and video_processor_thread.is_alive():
                    logger.info("Waiting for video_processor_thread to complete after GUI loop exit...")
                    video_processor_thread.join(timeout=5) 
                    if video_processor_thread.is_alive():
                        logger.warning("Video processor thread still alive after GUI loop exit and join attempt.")
                    else:
                        logger.info("Video processor thread successfully joined.")
                
                logger.info("Main GUI cleanup finished.")
                if not shutdown_in_progress:
                    logger.info("Shutdown not initiated by signal, calling cleanup_components from GUI finally.")
                    cleanup_components() 

    except KeyboardInterrupt:
        logger.info("KeyboardInterrupt received in main launcher thread.")
        if not shutdown_in_progress: signal_handler_main(signal.SIGINT, None)
    except Exception as e:
        logger.error(f"An unhandled exception occurred in the main launcher: {e}", exc_info=True)
    finally:
        if not shutdown_in_progress: # Ensure cleanup if not already done by signal handler
            logger.info("Ensuring cleanup in finally block...")
            cleanup_components()
        logger.info("System launcher has shut down.")
    
    return 0

if __name__ == "__main__":
    sys.exit(main()) 