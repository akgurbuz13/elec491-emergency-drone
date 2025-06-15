#!/usr/bin/env python3
import cv2
import numpy as np
import time
import argparse
import os
import json
import datetime
import queue # Added for thread-safe communication
import traceback # Added for printing exception info
from skimage.metrics import structural_similarity as ssim
from ultralytics import YOLO
import torch  # Added import for torch

# Assuming lora_bridge is in the same directory or PYTHONPATH
from lora_bridge import LoraBridge, EmergencyMessage # Ensure EmergencyMessage is imported if used directly

# Default paths and parameters
DEFAULT_MODEL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "models")
FIRE_MODEL_PATH = os.path.join(DEFAULT_MODEL_DIR, "best.pt")
DAMAGE_MODEL_PATH = os.path.join(DEFAULT_MODEL_DIR, "damage_detection_yolov8.pt")
DEFAULT_REFERENCE_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "reference_images")
DEFAULT_LOG_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), "logs")

# Default detection thresholds
FIRE_DETECTION_THRESHOLD = 0.75
DAMAGE_DETECTION_THRESHOLD = 0.50
SSIM_THRESHOLD = 0.65

# Alerting Configuration
ALERT_RETRY_INTERVAL_SECONDS = 4
ALERT_MAX_RETRIES = 4
ALERT_CHECK_INTERVAL_SECONDS = 1 # How often to check ACK status

class VideoProcessor:
    def __init__(self, lora_bridge_instance: LoraBridge,
                 fire_model_path=None, damage_model_path=None,
                 reference_dir=None, log_dir=None,
                 fire_threshold=FIRE_DETECTION_THRESHOLD,
                 damage_threshold=DAMAGE_DETECTION_THRESHOLD,
                 ssim_threshold=SSIM_THRESHOLD,
                 initial_fire_detection_enabled=True,
                 initial_damage_detection_enabled=True,
                 headless_mode=False,
                 frame_queue=None,
                 keypress_queue=None,
                 initial_person_detection_enabled=True,
                 person_model_path=None,
                 preferred_camera_index=-1):
        """
        Initialize the video processor.
        
        Args:
            lora_bridge_instance (LoraBridge): Instance of the LoraBridge for communication
            fire_model_path (str): Path to the fire detection model
            damage_model_path (str): Path to the damage detection model
            reference_dir (str): Directory containing reference images
            log_dir (str): Directory to save logs and detection images
            fire_threshold (float): Confidence threshold for fire detection
            damage_threshold (float): Confidence threshold for damage detection
            ssim_threshold (float): SSIM threshold for damage detection
            initial_fire_detection_enabled (bool): Whether fire detection should be enabled on startup
            initial_damage_detection_enabled (bool): Whether damage detection should be enabled on startup
            headless_mode (bool): If True, don't try to display video windows (save frames to disk instead)
            frame_queue (queue.Queue): Queue for sending frames to main thread
            keypress_queue (queue.Queue): Queue for receiving keypresses from main thread
            initial_person_detection_enabled (bool): Whether person detection should be enabled on startup
            person_model_path (str): Path to the person detection model (defaults to yolov8s.pt)
            preferred_camera_index (int): Preferred camera index to try first. Default -1 attempts fallbacks.
        """
        self.lora_bridge = lora_bridge_instance
        self.fire_model_path = fire_model_path or FIRE_MODEL_PATH
        self.damage_model_path = damage_model_path or DAMAGE_MODEL_PATH
        self.person_model_path = person_model_path or os.path.join(DEFAULT_MODEL_DIR, "yolov8s.pt")
        self.reference_dir = reference_dir or DEFAULT_REFERENCE_DIR
        self.log_dir = log_dir or DEFAULT_LOG_DIR
        self.fire_threshold = fire_threshold
        self.damage_threshold = damage_threshold
        self.ssim_threshold = ssim_threshold
        self.preferred_camera_index = preferred_camera_index
        
        # Initialize logger
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        self.log_file = os.path.join(self.log_dir, f"detection_log_{timestamp}.txt")
        
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
            
        with open(self.log_file, 'w') as f:
            f.write(f"# Detection Log - Started {datetime.datetime.now().isoformat()}\n")
            f.write(f"# Preferred Camera Index: {self.preferred_camera_index}\n")
            f.write(f"# Fire Model: {self.fire_model_path}\n")
            f.write(f"# Damage Model: {self.damage_model_path}\n")
            f.write(f"# References: {self.reference_dir}\n")
            f.write(f"# Thresholds - Fire: {fire_threshold}, Damage: {damage_threshold}, SSIM: {ssim_threshold}\n\n")
        
        # Resource initialization
        self.fire_model = None
        self.damage_model = None
        self.person_model = None
        self.reference_images = {}
        self.cap = None 
        
        # Processing state
        self.running = False
        self.fire_detected_in_frame = False
        self.damage_detected_in_frame = False
        self.last_time = time.time()
        self.frame_count = 0
        self.fps = 0
        self.headless_mode = headless_mode
        self.frame_queue = frame_queue
        self.keypress_queue = keypress_queue
        
        # Detection toggles
        self.fire_detection_enabled = initial_fire_detection_enabled
        self.damage_detection_enabled = initial_damage_detection_enabled
        self.person_detection_enabled = initial_person_detection_enabled
        
        self.total_frames = 0
        self.fire_detections_count = 0
        self.damage_detections_count = 0
        self.person_detections_count = 0

        # Stateful Alerting
        self.active_alert = None # Stores {'id': ..., 'type': ..., 'details': ..., 'sent_time': ..., 'retry_count': ..., 'acked': False}
        self.alert_cooldown_time = 10  # Seconds between new alerts of the same type
        self.last_alert_times = {'fire': 0, 'damage': 0}  # Last time each alert type was sent
        self.pending_alerts_display = {}  # For displaying status in UI (uses actual or sim IDs)
        self.last_alert_check_time = time.time()

        # Fire detection frequency control
        self.last_fire_check_time = 0.0
        self.fire_check_interval = 0.7  # Seconds

        # For headless mode, setup frame saving directory
        if self.headless_mode:
            self.frames_dir = os.path.join(self.log_dir, f"frames_{timestamp}")
            if not os.path.exists(self.frames_dir):
                os.makedirs(self.frames_dir)
            self.frame_save_interval = 5  # Save every 5th frame to avoid filling disk
            self.frame_counter = 0

    def load_models(self):
        """Load ML models for detection"""
        try:
            # --- Fire Model Loading --- 
            self.fire_model = None # Ensure it's None initially
            
            # --- Load Ultralytics Models (Fire, Damage, Person) --- 
            # Apply patch only if loading *any* Ultralytics model
            original_load = torch.load
            def safe_load(*args, **kwargs):
                # Force weights_only=False for compatibility with potentially older models
                kwargs['weights_only'] = False
                return original_load(*args, **kwargs)
            
            torch.load = safe_load
            
            try:
                # Load Fire Model (Ultralytics)
                fire_path_to_load = self.fire_model_path
                if not os.path.exists(fire_path_to_load) and (os.path.basename(fire_path_to_load) == fire_path_to_load or fire_path_to_load.endswith('.pt')):
                     print(f"Ultralytics fire model '{fire_path_to_load}' not found locally. YOLO will attempt download if it's an official model name.")
                elif not os.path.exists(fire_path_to_load):
                      print(f"Error: Specified Ultralytics fire model path not found: {fire_path_to_load}. Defaulting to yolov8n.pt.")
                      fire_path_to_load = "yolov8n.pt"
                print(f"Loading Ultralytics fire model: {fire_path_to_load}")
                self.fire_model = YOLO(fire_path_to_load)
                 
                # Load Damage Model (Ultralytics)
                damage_path_to_load = self.damage_model_path
                if not os.path.exists(damage_path_to_load):
                     print(f"Damage model '{damage_path_to_load}' not found. Defaulting to yolov8n.pt.")
                     damage_path_to_load = "yolov8n.pt"
                print(f"Loading Ultralytics damage model: {damage_path_to_load}")
                self.damage_model = YOLO(damage_path_to_load)
                 
                # Load Person Model (Ultralytics)
                person_path_to_load = self.person_model_path
                if not os.path.exists(person_path_to_load):
                     print(f"Person model '{person_path_to_load}' not found. Defaulting to yolov8s.pt.")
                     person_path_to_load = "yolov8s.pt"
                print(f"Loading Ultralytics person model: {person_path_to_load}")
                self.person_model = YOLO(person_path_to_load)
            except Exception as e_yolo_load:
                print(f"Error during Ultralytics model loading (inside patch): {e_yolo_load}")
                traceback.print_exc()
                # Restore patch even on error before re-raising
                torch.load = original_load 
                raise # Re-raise the exception to be caught by the outer block
            finally:
                # --- Restore original torch.load --- 
                torch.load = original_load 

            print("Models loaded successfully")
            return True
                
        except Exception as e:
            print(f"Error during model loading phase (outer): {e}")
            traceback.print_exc() 
            self.fire_model = None
            self.damage_model = None
            self.person_model = None
            return False
    
    def load_reference_images(self):
        """Load reference images for damage comparison"""
        if not self.damage_detection_enabled: # Only load if damage detection is on
            print("Damage detection (SSIM) is disabled, skipping reference image loading.")
            return True # Not an error if disabled

        if not os.path.exists(self.reference_dir):
            os.makedirs(self.reference_dir)
            print(f"Created reference directory: {self.reference_dir}")
            print("Please add reference images of the campus to this directory for SSIM comparison.")
            return False # Return false as no images are present yet
        
        images = [f for f in os.listdir(self.reference_dir) if f.endswith(('.jpg', '.jpeg', '.png'))]
        if not images:
            print("No reference images found for SSIM. Damage detection may rely solely on ML model.")
            return True # Not a critical error, ML can still work
        
        print(f"Loading {len(images)} reference images...")
        for img_file in images:
            img_path = os.path.join(self.reference_dir, img_file)
            img = cv2.imread(img_path)
            if img is not None:
                location_name = os.path.splitext(img_file)[0]
                self.reference_images[location_name] = cv2.resize(img, (640, 480))
        
        print(f"Loaded {len(self.reference_images)} reference images successfully")
        return len(self.reference_images) > 0
    
    def connect_to_stream(self):
        """Connect to a camera: preferred index, then fallbacks (MacBook, iPhone, other virtual)."""
        self.cap = None # Ensure cap is None initially
        cameras_to_try = []

        # 1. If a specific camera index is provided (preferred_camera_index), try that first.
        if self.preferred_camera_index >= 0:
            cameras_to_try.append({"index": self.preferred_camera_index, "name": f"Preferred Camera (Index {self.preferred_camera_index})"})

        # 2. Add common fallback cameras (ensure no duplicates if preferred_camera_index is one of these)
        # Order can be adjusted. e.g., 0 (often default/Continuity), 1 (often built-in FaceTime)
        common_fallbacks = [
            {"index": 0, "name": "Default Camera (Index 0 / Continuity Camera)"},
            {"index": 1, "name": "FaceTime HD Camera (MacBook - Index 1)"},
            {"index": 2, "name": "Camera (Index 2)"}, # Generic fallback
            # Add more potential indices if needed
        ]

        for fallback in common_fallbacks:
            # Only add if not already added as preferred and not already in the list by index
            if self.preferred_camera_index != fallback["index"] and \
               not any(cam_to_try["index"] == fallback["index"] for cam_to_try in cameras_to_try):
                cameras_to_try.append(fallback)
        
        # Ensure preferred_camera_index is genuinely tried first if it was also a common fallback index
        # (The above logic should handle this by not adding duplicates, but this re-sorts just in case)
        if self.preferred_camera_index >= 0:
            preferred_entry = next((item for item in cameras_to_try if item["index"] == self.preferred_camera_index), None)
            if preferred_entry: # Should always be true if preferred_camera_index >= 0
                cameras_to_try.remove(preferred_entry)
                cameras_to_try.insert(0, preferred_entry)
        
        # Iterate through the prioritized list of cameras
        for cam_info in cameras_to_try:
            idx, name = cam_info["index"], cam_info["name"]
            if self.cap and self.cap.isOpened(): # Already connected
                break
            print(f"Attempting to connect to {name} at index: {idx}")
            try:
                # Using cv2.CAP_AVFOUNDATION for macOS specific devices might be more stable
                capture = cv2.VideoCapture(idx, cv2.CAP_AVFOUNDATION)
                if capture.isOpened():
                    self.cap = capture
                    print(f"Successfully connected to {name} (index {idx}).")
                    break # Exit loop once a camera is successfully opened
                else:
                    print(f"Failed to open {name} (index {idx}).")
                    capture.release()
            except Exception as e_cam:
                print(f"Exception while trying {name} (index {idx}): {e_cam}")
                if 'capture' in locals() and capture and capture.isOpened():
                    capture.release()
        
        if not self.cap or not self.cap.isOpened():
            print("All video source attempts failed.")
            return False

        # Get video properties from the successfully opened capture
        width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        fps = self.cap.get(cv2.CAP_PROP_FPS) # May not be accurate for all sources

        # (Check if source is NOT the rtmp_url, implying it's a webcam)
        # This needs a more reliable way to check if it's a webcam vs RTMP
        # For now, let's assume if it's not the original RTMP URL string, it's a webcam
        is_webcam = True # A simple flag, refine if necessary. Now always true as RTMP is removed.
        try: 
            if self.cap.getBackendName() == 'AVFOUNDATION':
                is_webcam = True
            else: 
                is_webcam = False # Might be a file or other non-AVFoundation source
        except Exception:
            is_webcam = True # Default to assuming it's a webcam if backend check fails

        if is_webcam: # Apply only to webcams (or what we assume are webcams)
            print(f"Source identified as webcam. Attempting to set resolution to 640x480.")
            target_width = 640
            target_height = 480
            # Try setting resolution
            set_w = self.cap.set(cv2.CAP_PROP_FRAME_WIDTH, target_width)
            set_h = self.cap.set(cv2.CAP_PROP_FRAME_HEIGHT, target_height)
            
            # Read properties again after setting
            actual_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
            actual_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
            
            if set_w and set_h:
                print(f"Successfully set webcam resolution to {target_width}x{target_height}. Actual: {actual_width}x{actual_height}")
            else:
                print(f"Could not set webcam resolution to {target_width}x{target_height}. Using native: {actual_width}x{actual_height}")
            
            # Store the resolution we will be working with (either set or native)
            self.processing_width = actual_width
            self.processing_height = actual_height

        print(f"Connected to video source: {self.processing_width}x{self.processing_height} @ {fps if fps > 0 else 'N/A'} fps")
        return True
    
    def detect_fire(self, frame):
        """
        Detect fire in a frame using the loaded fire model (YOLO or Roboflow).
        """
        if not self.fire_detection_enabled or not self.fire_model:
            return False, []

        detections = []
        raw_detections_for_debug = []
        # Check if fire_model is an instance of the Roboflow client
        # Since we removed Roboflow, this check is no longer needed and self.fire_model is assumed to be Ultralytics YOLO
        is_roboflow_model = False # self.fire_model will not be a Roboflow client

        try:
            if is_roboflow_model: # This block will now effectively be skipped
                # Roboflow Model Inference using inference-sdk
                # Confidence threshold is handled server-side by Roboflow based on deployment settings,
                # but we can set a client-side confidence filter too.
                # Let's use the configured self.fire_threshold for filtering.
                api_results = self.fire_model.infer(frame, model_id=self.fire_model_path, confidence=self.fire_threshold)
                
                # Results format from inference_sdk might be different.
                # Assuming api_results is a dictionary or object with a 'predictions' list/attribute.
                # Check the actual format based on library documentation if this fails.
                if hasattr(api_results, 'predictions'):
                    predictions = api_results.predictions
                elif isinstance(api_results, list) and len(api_results) > 0 and hasattr(api_results[0], 'predictions'): # Older format?
                     predictions = api_results[0].predictions
                elif isinstance(api_results, dict) and 'predictions' in api_results:
                     predictions = api_results['predictions']
                else:
                     print("[DEBUG Fire Detect] Roboflow response format not recognized or no predictions.")
                     predictions = [] # No predictions found in expected format

                for pred in predictions:
                    # Accessing attributes directly if it's an object, or keys if dict
                    try: 
                        conf = pred.confidence
                        class_name = pred.class_name
                        # Convert center x, y, width, height to x1, y1, x2, y2
                        x1 = int(pred.x - pred.width / 2)
                        y1 = int(pred.y - pred.height / 2)
                        x2 = int(pred.x + pred.width / 2)
                        y2 = int(pred.y + pred.height / 2)
                        bbox = [x1, y1, x2, y2]
                    except AttributeError:
                         # Try accessing as dict keys if not object attributes
                         try:
                            conf = pred['confidence']
                            class_name = pred['class']
                            x1 = int(pred['x'] - pred['width'] / 2)
                            y1 = int(pred['y'] - pred['height'] / 2)
                            x2 = int(pred['x'] + pred['width'] / 2)
                            y2 = int(pred['y'] + pred['height'] / 2)
                            bbox = [x1, y1, x2, y2]
                         except (KeyError, TypeError) as e_dict:
                             print(f"[DEBUG Fire Detect] Could not parse Roboflow prediction: {pred}, Error: {e_dict}")
                             continue # Skip this prediction

                    raw_detections_for_debug.append(f"{class_name} (Conf: {conf:.2f})")
                    # Confidence filter already applied via API call or happens client-side
                    # Assuming `infer(confidence=...)` filters results
                    detections.append({
                        'class': class_name,
                        'confidence': conf,
                        'bbox': bbox
                    })
            else:
                # Ultralytics YOLO Model Inference (existing logic)
                results = self.fire_model(frame, conf=0.05, verbose=False)[0] # Low conf for debug capture
                for detection_data in results.boxes.data.tolist():
                    x1, y1, x2, y2, conf, cls = detection_data
                    class_name = results.names[int(cls)]
                    
                    # --- ADDED DETAILED DEBUG PRINT ---
                    print(f"[DEBUG Fire Model - All Detections] Class: {class_name} (ID: {int(cls)}), Confidence: {conf:.2f}, BBox: {[int(x1), int(y1), int(x2), int(y2)]}")

                    if class_name.lower() in ['fire', 'smoke', 'flame']: 
                        raw_detections_for_debug.append(f"{class_name} (Conf: {conf:.2f})")
                        if conf >= self.fire_threshold:
                            detections.append({
                                'class': class_name,
                                'confidence': conf,
                                'bbox': [int(x1), int(y1), int(x2), int(y2)]
                            })
        except Exception as e_infer:
            print(f"Error during fire model inference: {e_infer}", exc_info=True)
            return False, [] 

        if raw_detections_for_debug:
            print(f"[DEBUG Fire Detect] Raw model outputs: {raw_detections_for_debug}")
            
        return len(detections) > 0, detections
    
    def detect_damage_ml(self, frame):
        """
        Detect building damage in a frame using YOLOv8
        """
        if not self.damage_detection_enabled:
            return False, []

        results = self.damage_model(frame, conf=self.damage_threshold, verbose=False)[0]
        detections = []
        for detection_data in results.boxes.data.tolist():
            x1, y1, x2, y2, conf, cls = detection_data
            class_name = results.names[int(cls)]
            if class_name.lower() in ['damage', 'crack', 'debris', 'collapsed', 'rubble'] and conf >= self.damage_threshold:
                detections.append({
                    'class': class_name,
                    'confidence': conf,
                    'bbox': [int(x1), int(y1), int(x2), int(y2)]
                })
        return len(detections) > 0, detections
    
    def detect_damage_ssim(self, frame):
        """
        Detect building damage by comparing with reference images using SSIM
        """
        if not self.damage_detection_enabled or not self.reference_images:
            return False, {}
        
        frame_resized = cv2.resize(frame, (640, 480))
        frame_gray = cv2.cvtColor(frame_resized, cv2.COLOR_BGR2GRAY)
        
        best_match = {'score': 1.0, 'location': None, 'threshold_breached': False}
        
        for location, ref_img in self.reference_images.items():
            ref_gray = cv2.cvtColor(ref_img, cv2.COLOR_BGR2GRAY)
            # Ensure data_range is appropriate for uint8 images
            score, _ = ssim(frame_gray, ref_gray, full=True, data_range=frame_gray.max() - frame_gray.min())
            
            if score < best_match['score']: # Lower score = less similar
                best_match['score'] = score
                best_match['location'] = location
        
        is_damaged = best_match['score'] < self.ssim_threshold and best_match['location'] is not None
        best_match['threshold_breached'] = is_damaged
        
        return is_damaged, {
            'score': best_match['score'], 
            'location': best_match['location'],
            'threshold': self.ssim_threshold,
            'threshold_breached': is_damaged
        }
    
    def detect_person(self, frame):
        """
        Detect persons in a frame using the general model (e.g., self.fire_model or a dedicated person model if loaded)
        For now, uses self.fire_model which is yolov8n.pt by default.
        Detect persons in a frame using the dedicated person model.
        """
        if not self.person_detection_enabled or not self.person_model: # Use self.person_model
            return False, []
            
        # Using a common threshold, can be made specific if needed
        # We use self.fire_model here assuming it's the general YOLO model if no specific fire model is loaded
        results = self.person_model(frame, conf=0.3, classes=[0], verbose=False)[0] # Use self.person_model, Class 0 is 'person'
        detections = []
        
        # Debug print for all detected 'person' classes before thresholding by main confidence
        raw_person_detections_debug = []

        for detection_data in results.boxes.data.tolist():
            x1, y1, x2, y2, conf, cls = detection_data
            class_name = results.names[int(cls)] # Should be 'person' due to classes=[0]
            raw_person_detections_debug.append(f"{class_name} (Conf: {conf:.2f})")
            
            # The main confidence threshold for person detection can be different if needed
            # For now, let's use a fixed one or make it configurable like fire_threshold
            if conf >= 0.4: # Example threshold for person
                detections.append({
                    'class': class_name,
                    'confidence': conf,
                    'bbox': [int(x1), int(y1), int(x2), int(y2)]
                })
        
        if raw_person_detections_debug:
            print(f"[DEBUG Person Detect] Raw model outputs: {raw_person_detections_debug}")
            
        return len(detections) > 0, detections

    def save_detection_frame(self, frame, detection_type, details):
        """Save a frame with detection for logging purposes"""
        if not os.path.exists(self.log_dir):
            os.makedirs(self.log_dir)
        
        timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
        filename = f"{detection_type}_{timestamp}.jpg"
        filepath = os.path.join(self.log_dir, filename)
        
        # Add detection info to the frame
        height, width = frame.shape[:2]
        info_text = f"{detection_type.upper()} DETECTED - {details}"
        cv2.putText(frame, info_text, (10, height - 20),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)
        
        # Save the frame
        cv2.imwrite(filepath, frame)
        print(f"Detection frame saved to {filepath}")
        
        # Log the detection
        with open(self.log_file, 'a') as f:
            f.write(f"{datetime.datetime.now().isoformat()} - {detection_type.upper()} DETECTION: {details} (Image: {filename})\n")
    
    def process_frame(self, frame):
        original_frame = frame.copy() # For saving clean frame if needed
        
        # Fire detection
        current_time = time.time()
        if (current_time - self.last_fire_check_time) >= self.fire_check_interval:
            self.last_fire_check_time = current_time
            current_fire_detected, fire_instances = self.detect_fire(frame)
        else:
            # If not performing check, assume no new fire detected in this specific interval
            current_fire_detected, fire_instances = False, []
        
        # Damage detection
        current_damage_ml, damage_instances_ml = self.detect_damage_ml(frame)
        current_damage_ssim, damage_info_ssim = self.detect_damage_ssim(frame)
        current_damage_detected = current_damage_ml or current_damage_ssim
        
        # Person detection (new)
        current_person_detected, person_instances = self.detect_person(frame)
        
        # Update overall detection state for the frame (used for sending one alert per event)
        self.fire_detected_in_frame = current_fire_detected
        self.damage_detected_in_frame = current_damage_detected
        self.person_detected_in_frame = current_person_detected

        # Stats update
        if current_fire_detected: self.fire_detections_count += 1
        if current_damage_detected: self.damage_detections_count += 1
        if current_person_detected: self.person_detections_count += 1 # New
        
        alert_details_fire = ""
        alert_details_damage = ""

        # Process and draw fire detections
        if current_fire_detected:
            alert_details_fire = f"{len(fire_instances)} fire/smoke instances"
            for detection in fire_instances:
                x1, y1, x2, y2 = detection['bbox']
                label = f"{detection['class']}: {detection['confidence']:.2f}"
                cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 0, 255), 2) # Red
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
            
            # Send alert only if no active alert or cooldown period has passed
            current_time = time.time()
            if (self.active_alert is None or 
                (self.active_alert['type'] != 'fire' and 
                 current_time - self.last_alert_times['fire'] > self.alert_cooldown_time)):
                self.send_emergency_message("fire", alert_details_fire)
                self.last_alert_times['fire'] = current_time


        # Process and draw damage detections
        if current_damage_detected:
            details_parts = []
            if current_damage_ml:
                details_parts.append(f"{len(damage_instances_ml)} ML detects")
                for detection in damage_instances_ml:
                    x1, y1, x2, y2 = detection['bbox']
                    label = f"{detection['class']}: {detection['confidence']:.2f}"
                    cv2.rectangle(frame, (x1, y1), (x2, y2), (0, 165, 255), 2) # Orange
                    cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 165, 255), 2)

            if current_damage_ssim and damage_info_ssim.get('threshold_breached'):
                loc = damage_info_ssim.get('location', "Unknown")
                score = damage_info_ssim.get('score', 0)
                details_parts.append(f"SSIM change at {loc} ({score:.2f})")
                cv2.putText(frame, f"SSIM Low: {loc} ({score:.2f})", (10, frame.shape[0] - 70), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 165, 255), 2)
            
            alert_details_damage = "; ".join(details_parts)
            
            # Send alert only if no active alert or cooldown period has passed
            current_time = time.time()
            if (self.active_alert is None or 
                (self.active_alert['type'] != 'damage' and 
                 current_time - self.last_alert_times['damage'] > self.alert_cooldown_time)):
                self.send_emergency_message("damage", alert_details_damage)
                self.last_alert_times['damage'] = current_time

        # Process and draw person detections (new)
        if current_person_detected:
            for detection in person_instances:
                x1, y1, x2, y2 = detection['bbox']
                label = f"Person: {detection['confidence']:.2f}" # 'person' class
                cv2.rectangle(frame, (x1, y1), (x2, y2), (255, 0, 0), 2) # Blue for persons
                cv2.putText(frame, label, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 2)
            # We are not sending LoRa alerts for persons in this example, just visual display

        # Save frame if a significant detection occurred in this frame
        if self.fire_detected_in_frame and alert_details_fire:
             self.save_detection_frame(original_frame, "fire", alert_details_fire) # Save original frame
        if self.damage_detected_in_frame and alert_details_damage:
             self.save_detection_frame(original_frame, "damage", alert_details_damage) # Save original frame

        # Check and update active alert status
        current_time = time.time()
        if self.active_alert and current_time - self.last_alert_check_time > ALERT_CHECK_INTERVAL_SECONDS:
            self.last_alert_check_time = current_time
            self._check_active_alert_status()

        # FPS Calculation
        current_time_fps = time.time()
        self.frame_count += 1
        self.total_frames += 1
        if current_time_fps - self.last_time >= 1.0:
            self.fps = self.frame_count / (current_time_fps - self.last_time)
            self.frame_count = 0
            self.last_time = current_time_fps
        
        # Overlay Status Information
        y_offset = 20
        cv2.putText(frame, f"FPS: {self.fps:.1f}", (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0), 2); y_offset += 20
        cv2.putText(frame, f"Fire Detect: {'ON' if self.fire_detection_enabled else 'OFF'} (Press 'F')", (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if self.fire_detection_enabled else (0,0,255), 2); y_offset += 20
        cv2.putText(frame, f"Damage Detect: {'ON' if self.damage_detection_enabled else 'OFF'} (Press 'D')", (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if self.damage_detection_enabled else (0,0,255), 2); y_offset += 20
        cv2.putText(frame, f"Person Detect: {'ON' if self.person_detection_enabled else 'OFF'} (Press 'P')", (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (0, 255, 0) if self.person_detection_enabled else (0,0,255), 2); y_offset += 20
        # Display count for the CURRENT frame, not cumulative
        persons_in_current_frame = len(person_instances) if current_person_detected else 0
        cv2.putText(frame, f"Persons This Frame: {persons_in_current_frame}", (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.6, (255, 0, 0) if persons_in_current_frame > 0 else (0, 255, 0), 2); y_offset += 20
        
        # Display status of active alert if any
        if self.active_alert:
            alert_status = f"Active Alert: {self.active_alert['type']} - Retry {self.active_alert['retry_count']}/{ALERT_MAX_RETRIES}"
            cv2.putText(frame, alert_status, (10, y_offset), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 2)
            y_offset += 20
        
        # Display status of tracked alerts (from self.pending_alerts_display)
        alerts_to_display = list(self.pending_alerts_display.values())[-3:] # Get last 3 alerts being tracked
        for i, alert_text in enumerate(alerts_to_display):
             cv2.putText(frame, alert_text, (10, y_offset + i*20), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255,255,0), 2)
        
        return frame

    def _check_active_alert_status(self):
        """
        Check if active alert has been acknowledged or needs retrying
        """
        if not self.active_alert:
            return
        
        current_time = time.time()
        alert_id = self.active_alert['id']
        
        # TODO: In the future, implement actual checking of acknowledgment from LoRa
        # For now, just retry based on time
        
        # Check if retry interval has passed
        if current_time - self.active_alert['sent_time'] > ALERT_RETRY_INTERVAL_SECONDS:
            # Check if max retries reached
            if self.active_alert['retry_count'] >= ALERT_MAX_RETRIES:
                print(f"Alert {alert_id} reached max retries ({ALERT_MAX_RETRIES}). Marking as failed.")
                self.pending_alerts_display[alert_id] = f"Alert {self.active_alert['type']} ID:{alert_id} - failed (max retries)"
                self.active_alert = None  # Clear active alert
            else:
                # Retry sending
                self.active_alert['retry_count'] += 1
                print(f"Retrying alert {alert_id} (attempt {self.active_alert['retry_count']}/{ALERT_MAX_RETRIES})")
                
                # Resend emergency message
                emergency_code = 1 if self.active_alert['type'] == "fire" else 2
                emergency_msg = EmergencyMessage(text=f"{self.active_alert['type'].upper()}: {self.active_alert['details']}", 
                                               emergency_code=emergency_code)
                
                if self.lora_bridge and hasattr(self.lora_bridge, 'send_message') and self.lora_bridge.connected:
                    new_id = self.lora_bridge.send_message(emergency_msg)
                    if new_id:
                        # Update active alert with new message ID and time
                        self.active_alert['id'] = new_id
                        self.active_alert['sent_time'] = current_time
                        self.pending_alerts_display[new_id] = f"Alert {self.active_alert['type']} ID:{new_id} - retry {self.active_alert['retry_count']}"
                    else:
                        print(f"Failed to resend alert {alert_id}")
                else:
                    # Simulate for UI
                    sim_id = f"{alert_id}_retry{self.active_alert['retry_count']}"
                    self.active_alert['id'] = sim_id
                    self.active_alert['sent_time'] = current_time
                    self.pending_alerts_display[sim_id] = f"SimAlert {self.active_alert['type']} ID:{sim_id} - retry {self.active_alert['retry_count']}"

    def send_emergency_message(self, alert_type: str, details: str, image_path: str = None):
        """
        Sends an emergency message via LoRa bridge if available, otherwise logs.
        Implements stateful alerting to avoid duplicate messages.
        """
        # First, define emergency_code outside of any conditionals to avoid UnboundLocalError
        emergency_code = 1 if alert_type == "fire" else 2  # Code 1 for fire, 2 for damage
        
        # Generate a message ID based on timestamp
        message_id_prefix = f"msg_{int(time.time())}"
        log_message = f"EMERGENCY ALERT ({alert_type}): {details}"
        if image_path:
            log_message += f" (Image: {os.path.basename(image_path)})"
        
        # If there's already an active alert of the same type, don't send another
        if self.active_alert and self.active_alert['type'] == alert_type:
            # Just update the display to show we detected again but didn't resend
            display_id = f"{message_id_prefix}_ignored"
            self.pending_alerts_display[display_id] = f"Alert {alert_type} - ignored (active alert exists)"
            print(f"Ignoring {alert_type} alert as there's already an active one: {log_message}")
            return
            
        # Create a new active alert
        if self.lora_bridge and hasattr(self.lora_bridge, 'send_message') and self.lora_bridge.connected:
            print(f"Attempting to send LoRa message: {log_message}")
            emergency_msg = EmergencyMessage(text=f"{alert_type.upper()}: {details}", emergency_code=emergency_code)
            actual_msg_id = self.lora_bridge.send_message(emergency_msg)
            
            if actual_msg_id:
                # Store as active alert
                self.active_alert = {
                    'id': actual_msg_id,
                    'type': alert_type,
                    'details': details,
                    'sent_time': time.time(),
                    'retry_count': 0,
                    'acked': False
                }
                
                self.pending_alerts_display[actual_msg_id] = f"Alert {alert_type} ID:{actual_msg_id} - sent"
                print(f"LoRa message sent with ID: {actual_msg_id}. Details: {log_message}")
                # Log to file that it was sent via LoRa
                with open(self.log_file, 'a') as f:
                    f.write(f"{datetime.datetime.now().isoformat()} - LORA SENT - {log_message} (ID: {actual_msg_id})\n")
            else:
                print(f"LoRa message sending failed (no ID returned). Details: {log_message}")
                # Log to file about the failure
                with open(self.log_file, 'a') as f:
                    f.write(f"{datetime.datetime.now().isoformat()} - LORA SEND FAIL - {log_message}\n")
        else:
            # LoRa bridge not available or not connected, or send_message attribute missing
            sim_message_id = f"{message_id_prefix}_{alert_type}_sim"
            
            # Create simulated active alert
            self.active_alert = {
                'id': sim_message_id,
                'type': alert_type,
                'details': details,
                'sent_time': time.time(),
                'retry_count': 0,
                'acked': False
            }
            
            print(f"[LoRa SIMULATOR / Bridge Not Ready] Alert: {log_message} (Simulated ID: {sim_message_id})")
            # Log to file that it was a simulation / bridge not ready
            with open(self.log_file, 'a') as f:
                f.write(f"{datetime.datetime.now().isoformat()} - LORA SIMULATED/NOT_READY - {log_message} (ID: {sim_message_id})\n")
            # Add to pending_alerts for UI display
            self.pending_alerts_display[sim_message_id] = f"SimAlert {alert_type} ID:{sim_message_id} - sim_lora_unavailable"

    def handle_keypress(self, key):
        if key == ord('f') or key == ord('F'):
            self.fire_detection_enabled = not self.fire_detection_enabled
            print(f"Fire detection {'ENABLED' if self.fire_detection_enabled else 'DISABLED'}")
        elif key == ord('d') or key == ord('D'):
            self.damage_detection_enabled = not self.damage_detection_enabled
            if self.damage_detection_enabled and not self.reference_images:
                self.load_reference_images() # Try to load if just enabled and not loaded
            print(f"Damage detection {'ENABLED' if self.damage_detection_enabled else 'DISABLED'}")
        elif key == ord('p') or key == ord('P'): # New keybind for Person detection
            self.person_detection_enabled = not self.person_detection_enabled
            print(f"Person detection {'ENABLED' if self.person_detection_enabled else 'DISABLED'}")
        elif key == ord('q') or key == ord('Q'):
            self.running = False
            print("Quit signal received. Shutting down video processor.")

    def start_processing_loop(self): # Renamed from start
        if not self.load_models():
            print("Failed to load detection models. Video processing cannot start.")
            return False
        
        # Load reference images if damage detection is initially enabled
        if self.damage_detection_enabled:
            self.load_reference_images()
        
        if not self.connect_to_stream():
            print("Failed to connect to video stream. Video processing cannot start.")
            return False
        
        self.running = True
        # Message updated based on whether a frame_queue is present (GUI vs headless)
        if self.frame_queue:
            print("Video processor started. Frames will be sent to the main thread for display.")
            print("Ensure the main application is handling 'F', 'D', 'P', 'Q' keys.")
        else:
            print("Video processor started in headless mode (no frame queue). Detections will be logged.")
            if not self.headless_mode:
                 print("Warning: frame_queue is None, but headless_mode was not explicitly set. Forcing headless.")
                 self.headless_mode = True # Force headless if no queue
                 # Ensure frames_dir for saving if forced headless here
                 if not hasattr(self, 'frames_dir') or not self.frames_dir:
                    timestamp = datetime.datetime.now().strftime("%Y%m%d_%H%M%S")
                    self.frames_dir = os.path.join(self.log_dir, f"frames_{timestamp}")
                    if not os.path.exists(self.frames_dir): os.makedirs(self.frames_dir)
                    self.frame_save_interval = getattr(self, 'frame_save_interval', 5)
                    self.frame_counter = getattr(self, 'frame_counter', 0)

        try:
            while self.running:
                # Check for keypresses from the main thread
                if self.keypress_queue and not self.keypress_queue.empty():
                    try:
                        key = self.keypress_queue.get_nowait()
                        self.handle_keypress(key) # Process keypress
                    except queue.Empty:
                        pass # Should not happen with not empty check, but good practice

                ret, frame = self.cap.read()
                if not ret:
                    print("Failed to read frame. Attempting to reconnect to stream...")
                    self.cap.release()
                    time.sleep(2) # Wait before retrying connection
                    if not self.connect_to_stream():
                        print("Reconnection failed. Exiting video processing loop.")
                        self.running = False # Stop if reconnection fails
                    continue
                
                processed_frame = self.process_frame(frame)
                
                if self.frame_queue: # If there's a queue, send the frame
                    try:
                        self.frame_queue.put_nowait(processed_frame)
                    except queue.Full:
                        # Optionally, log this or handle if the main thread isn't consuming fast enough
                        print("Warning: Frame queue is full. Main thread might be slow.")
                        pass 
                elif self.headless_mode: # Original headless logic (saving to disk)
                    if hasattr(self, 'frames_dir') and self.frames_dir: 
                        self.frame_counter += 1
                        if self.frame_counter % self.frame_save_interval == 0:
                            frame_path = os.path.join(self.frames_dir, f"frame_{self.total_frames:06d}.jpg")
                            try:
                                cv2.imwrite(frame_path, processed_frame)
                            except Exception as e:
                                print(f"Warning: Failed to save frame to {frame_path}: {e}")
                
                # If not using frame_queue (headless), a small sleep is good to prevent busy-waiting
                if not self.frame_queue:
                    time.sleep(0.01) # Adjust as needed if CPU usage is high in headless

                if not self.running: # Check if self.running was set to False
                    break
            
        except KeyboardInterrupt:
            print("Video processing interrupted by user (KeyboardInterrupt).")
        except Exception as e:
            print(f"An error occurred in the video processing loop: {e}")
            traceback.print_exc()
        finally:
            # Signal main thread that processing is done if using queue
            if self.frame_queue:
                self.frame_queue.put_nowait(None) # Sentinel value to indicate end of stream
            self.stop()
            
        return True
    
    def stop(self):
        """Stop video processing and release resources"""
        print("Stopping video processor...")
        self.running = False
        
        if self.cap:
            self.cap.release()
            self.cap = None # Ensure it's None
        
        try:
            cv2.destroyAllWindows()
        except Exception as e:
            print(f"Note: Exception during cv2.destroyAllWindows(): {e}")
        
        # LoRa bridge is managed externally now
        
        print("\n--- Video Processor Final Statistics ---")
        print(f"Total frames processed: {self.total_frames}")
        print(f"Fire alerts triggered: {self.fire_detections_count}")
        print(f"Damage alerts triggered: {self.damage_detections_count}")
        print(f"Person alerts triggered: {self.person_detections_count}")
        print(f"Total frames with person detection: {self.person_detections_count}")
        print(f"Detection log saved to: {self.log_file}")

# Removed main() function as this script will be run as a component
# Argument parsing will be handled by start_system.py

# Example of how it might be called by start_system.py (conceptual)
# if __name__ == '__main__':
#     # This is for standalone testing if needed, but not the primary execution path
#     parser = argparse.ArgumentParser(description="Process drone video feed for emergency detection")
#     parser.add_argument("--rtmp", type=str, required=True, help="RTMP URL for video stream")
#     # ... other arguments as before, but lora_bridge would need to be mocked or set up
#     args = parser.parse_args()
#
#     # For standalone test, you'd need a mock or real LoraBridge
#     # from lora_bridge import LoraBridge # Assuming LoraBridge is in the same directory or install path
#     # test_lora_port = "/dev/tty.SLAB_USBtoUART" # Example port
#     # mock_lora_bridge = LoraBridge(port=test_lora_port)
#     # if not mock_lora_bridge.start():
#     # print(f"Failed to start mock LoraBridge on {test_lora_port}")
#     # exit()
#
#     processor = VideoProcessor(
#         rtmp_url=args.rtmp,
#         lora_bridge_instance=None, # mock_lora_bridge, # Pass the instance
#         # ... other args
#     )
#     processor.start_processing_loop()
#     # mock_lora_bridge.stop()