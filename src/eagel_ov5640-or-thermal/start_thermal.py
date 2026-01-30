import socket
import struct
import numpy as np
import cv2
from scipy import ndimage

# pip install numpy opencv-python scipy

# Configuration
ESP32_IP = "eagel_1.local"  # or ip
ESP32_PORT = 81
DISPLAY_SIZE = 780
FULLSCREEN = True  # Set True for Raspberry Pi fullscreen mode (1080x1080)

# some apply_interpolation options below (just comment out) 
PIXEL_SIZE = 45  # Size of each thermal pixel in screen pixels (1080/45 = 24x24 grid)

# ctrl+f colormap to find places to change it 

# Temperature mapping 
# values that seemed to work fairly well:
# min,  max
# 20    32
# Lower TEMP_MIN = darker baseline, more contrast with warm objects
# Lower TEMP_MAX = people appear brighter/whiter (more sensitive)
# Higher TEMP_MAX = more headroom for very hot objects (lighter, etc.)
# Keep the range around 12-20°C wide for good color gradient
TEMP_MIN = 20.0 
TEMP_MAX = 32.0 

show_temp = False # set True for text on screen to help with callibration

# Alert threshold for highlighting hot spots
ALERT_THRESHOLD = 32.0  # Celsius (adjust based on environment)

# Pre-compute circular mask once
_CIRCULAR_MASK = None

def create_circular_mask(size):
    """Create a circular mask for the round display"""
    global _CIRCULAR_MASK
    if _CIRCULAR_MASK is None or _CIRCULAR_MASK.shape[0] != size:
        y, x = np.ogrid[:size, :size]
        center = size // 2
        _CIRCULAR_MASK = (x - center)**2 + (y - center)**2 <= center**2
    return _CIRCULAR_MASK

"""
def apply_interpolation(data, target_size):
    # Upscale 8x8 thermal data to target size with smooth cubic interpolation
    # Reshape to 8x8
    grid = data.reshape(8, 8)
    
    # Use scipy's zoom for smooth cubic interpolation
    upscaled = ndimage.zoom(grid, target_size/8, order=3)
    
    return upscaled
"""

# def apply_interpolation(data, target_size):
    # """
    # Fast upscale using OpenCV instead of scipy (much faster on Pi)
    # """
    # # Reshape to 8x8
    # grid = data.reshape(8, 8).astype(np.float32)
    
    # # Use OpenCV's fast bilinear interpolation
    # upscaled = cv2.resize(grid, (target_size, target_size), interpolation=cv2.INTER_LINEAR)
    
    # return upscaled

def apply_interpolation(data, target_size):
    """
    Pixelated upscale - PIXEL_SIZE controls visible grid resolution
    """
    # Reshape to 8x8
    grid = data.reshape(8, 8).astype(np.float32)
    
    # Determine grid resolution (how many blocks across the screen)
    grid_resolution = max(8, target_size // PIXEL_SIZE)
    
    # First: upscale from 8x8 to desired grid resolution (smooth)
    intermediate = cv2.resize(grid, (grid_resolution, grid_resolution), 
                             interpolation=cv2.INTER_LINEAR)
    
    # Second: scale to display size (blocky to maintain pixel look)
    upscaled = cv2.resize(intermediate, (target_size, target_size), 
                         interpolation=cv2.INTER_NEAREST)
    
    return upscaled


def create_heatmap(thermal_data, size=DISPLAY_SIZE, show_alert=True):
    """
    Create a colorized heatmap from thermal data
    """
    # Interpolate to display size
    upscaled = apply_interpolation(thermal_data, size)
    
    # Normalize to 0-255 for colormap
    normalized = np.clip((upscaled - TEMP_MIN) / (TEMP_MAX - TEMP_MIN) * 255, 0, 255).astype(np.uint8)
    
    # Apply colormap (INFERNO)
    colored = cv2.applyColorMap(normalized, cv2.COLORMAP_INFERNO)
    
    # Highlight hot spots if alert threshold exceeded
    if show_alert:
        hot_spots = upscaled > ALERT_THRESHOLD
        if np.any(hot_spots):
            # Add white overlay on hot spots
            overlay = colored.copy()
            overlay[hot_spots] = [255, 255, 255]
            colored = cv2.addWeighted(colored, 0.7, overlay, 0.3, 0)
    
    # Apply circular mask for round display
    mask = create_circular_mask(size)
    colored[~mask] = [0, 0, 0]  # Black background outside circle
       
    return colored

def receive_thermal_data(sock):
    """
    Receive and parse thermal camera data
    Returns: numpy array of 64 floats or None if error
    """
    try:
        # Expecting 256 bytes (64 floats × 4 bytes each)
        data = b''
        while len(data) < 256:
            chunk = sock.recv(256 - len(data))
            if not chunk:
                return None
            data += chunk
        
        # Unpack as 64 floats (little-endian)
        temps = struct.unpack('<64f', data)
        return np.array(temps, dtype=np.float32)
    
    except Exception as e:
        print(f"Error receiving data: {e}")
        return None

def main():
    print(f"Connecting to ESP32 at {ESP32_IP}:{ESP32_PORT}...")
    
    # Socket settings for lower latency
    sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)  # Disable Nagle's algorithm
    
    try:
        sock.connect((ESP32_IP, ESP32_PORT))
        print("Connected! Starting thermal display...")
        
        # Setup window based on fullscreen mode
        if FULLSCREEN:
            cv2.namedWindow("Thermal Camera", cv2.WINDOW_NORMAL)
            cv2.setWindowProperty("Thermal Camera", cv2.WND_PROP_FULLSCREEN, cv2.WINDOW_FULLSCREEN)
            display_size = 1080  # Full resolution for round display
        else:
            cv2.namedWindow("Thermal Camera", cv2.WINDOW_NORMAL)
            cv2.resizeWindow("Thermal Camera", DISPLAY_SIZE, DISPLAY_SIZE)
            display_size = DISPLAY_SIZE
        
        frame_count = 0
        
        while True:
            # Receive thermal data
            thermal_data = receive_thermal_data(sock)
            
            if thermal_data is None:
                print("Connection lost. Reconnecting...")
                sock.close()
                sock = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                sock.setsockopt(socket.IPPROTO_TCP, socket.TCP_NODELAY, 1)
                sock.connect((ESP32_IP, ESP32_PORT))
                continue
            
            # Create and display heatmap
            heatmap = create_heatmap(thermal_data, size=display_size)
            
            # for live callibration: Show current temp range on screen
            if show_temp:
                cv2.putText(heatmap, f"Range: {thermal_data.min():.1f}-{thermal_data.max():.1f}C", (10, 30), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (255, 255, 255), 2)
    
            
            cv2.imshow("Thermal Camera", heatmap)
            
            # Print stats every 30 frames
            frame_count += 1
            if frame_count % 30 == 0:
                print(f"Min: {thermal_data.min():.1f}°C | "
                      f"Max: {thermal_data.max():.1f}°C | "
                      f"Avg: {thermal_data.mean():.1f}°C")
            
            # ESC to exit, 's' to save screenshot
            key = cv2.waitKey(1) & 0xFF
            if key == 27:  # ESC
                break
            elif key == ord('s'):
                filename = f"thermal_snapshot_{frame_count}.png"
                cv2.imwrite(filename, heatmap)
                print(f"Saved {filename}")
    
    except KeyboardInterrupt:
        print("\nStopping...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        sock.close()
        cv2.destroyAllWindows()

if __name__ == "__main__":
    main()