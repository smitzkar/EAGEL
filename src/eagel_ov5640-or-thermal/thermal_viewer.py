import socket
import numpy as np
import matplotlib.pyplot as plt

HOST = "eagel_2.local"  # or the ESP32 IP (e.g., "192.168.43.112")
PORT = 81

# Create TCP client
s = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
s.connect((HOST, PORT))
print(f"Connected to {HOST}:{PORT}")

# Prepare plot
plt.ion()
fig, ax = plt.subplots()
im = ax.imshow(np.zeros((8,8)), cmap='inferno', vmin=15, vmax=40)
plt.title("AMG8833 Thermal Stream")
plt.colorbar(im)

try:
    while True:
        data = s.recv(64)
        if len(data) < 64:
            continue

        # Convert bytes (0–255) to temperatures
        frame = np.frombuffer(data, dtype=np.uint8).reshape((8,8))
        im.set_data(frame)
        plt.draw()
        plt.pause(0.01)

        # Optional alert message
        if (frame > 200).any():  # adjust threshold
            print("🔥 Heat detected!")

except KeyboardInterrupt:
    print("\nExiting...")
finally:
    s.close()
    plt.close(fig)
