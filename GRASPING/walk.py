import time
import tkinter as tk
import threading
from dynamixel_sdk import *  # Uses Dynamixel SDK library

# Constants
PORT = "/dev/tty.usbserial-FT9HDAWY"  # Your port location
BAUDRATE = 57600
DXL_ID = 15
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
ADDR_MOVING_SPEED = 112
ADDR_TORQUE_ENABLE = 64
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
PROTOCOL_VERSION = 2.0

def degrees_to_dxl_units(degrees):
    """Convert degrees to Dynamixel position units (0-4095)."""
    return int((degrees / 360.0) * 4095)

def dxl_units_to_degrees(units):
    """Convert Dynamixel position units to degrees."""
    return (units / 4095.0) * 360

class MotorControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("Motor Oscillation Control")
        self.dxl_id = DXL_ID

        # Dynamixel Initialization
        self.port_handler = PortHandler(PORT)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)

        if not self.port_handler.openPort() or not self.port_handler.setBaudRate(BAUDRATE):
            raise Exception("Failed to open port or set baudrate!")

        # Enable Torque
        self.packet_handler.write1ByteTxRx(self.port_handler, self.dxl_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)
        print("Torque enabled. Motor ready.")

        # Read Zero Position
        pos, _, _ = self.packet_handler.read4ByteTxRx(self.port_handler, self.dxl_id, ADDR_PRESENT_POSITION)
        self.zero_position = pos
        self.running = False
        print(f"Zero position: {self.zero_position}")

        # UI Elements
        tk.Label(root, text="Motor Oscillation Control", font=("Arial", 16)).pack(pady=10)

        tk.Label(root, text="Amplitude (degrees):").pack()
        self.amplitude_entry = tk.Entry(root)
        self.amplitude_entry.insert(0, "30")  # Default 30 degrees
        self.amplitude_entry.pack()

        tk.Label(root, text="Speed (steps/sec):").pack()
        self.speed_entry = tk.Entry(root)
        self.speed_entry.insert(0, "5")  # Default 5 steps/sec
        self.speed_entry.pack()

        self.start_button = tk.Button(root, text="Start Oscillation", command=self.start_oscillation)
        self.start_button.pack(pady=5)

        self.stop_button = tk.Button(root, text="Stop Oscillation", command=self.stop_oscillation)
        self.stop_button.pack(pady=5)

        self.log_text = tk.Text(root, height=10, width=40)
        self.log_text.pack(pady=10)

    def move_to_position(self, position):
        """Command the motor to move to a specific position."""
        self.packet_handler.write4ByteTxRx(self.port_handler, self.dxl_id, ADDR_GOAL_POSITION, position)

    def set_speed(self, speed):
        """Set the motor's movement speed."""
        self.packet_handler.write2ByteTxRx(self.port_handler, self.dxl_id, ADDR_MOVING_SPEED, speed)
        print(f"Speed set to: {speed}")

    def start_oscillation(self):
        """Start oscillating the motor."""
        amplitude_deg = float(self.amplitude_entry.get())  # Amplitude in degrees
        speed = int(self.speed_entry.get())  # Steps/sec

        if amplitude_deg <= 0 or speed <= 0:
            self.log_text.insert(tk.END, "Invalid amplitude or speed.\n")
            return

        amplitude_units = degrees_to_dxl_units(amplitude_deg)  # Convert to units
        self.running = True
        self.set_speed(speed)

        high_pos = self.zero_position + amplitude_units
        low_pos = self.zero_position - amplitude_units
        steps = 10  # Number of intermediate steps for smooth motion

        def oscillate():
            while self.running:
                # Move up
                for i in range(steps + 1):
                    if not self.running:
                        return
                    position = int(self.zero_position + amplitude_units * (i / steps))
                    self.move_to_position(position)
                    time.sleep(1 / (speed * 10))  # Smooth timing

                # Move down
                for i in range(steps + 1):
                    if not self.running:
                        return
                    position = int(self.zero_position + amplitude_units * ((steps - i) / steps))
                    self.move_to_position(position)
                    time.sleep(1 / (speed * 10))

        threading.Thread(target=oscillate, daemon=True).start()
        self.log_text.insert(tk.END, "Oscillation started.\n")

    def stop_oscillation(self):
        """Stop the motor oscillation."""
        self.running = False
        self.move_to_position(self.zero_position)
        self.log_text.insert(tk.END, "Oscillation stopped.\n")

    def cleanup(self):
        """Disable torque and close the port."""
        self.packet_handler.write1ByteTxRx(self.port_handler, self.dxl_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        self.port_handler.closePort()
        print("Cleanup done. Motor torque disabled and port closed.")

# Run Application
if __name__ == "__main__":
    try:
        root = tk.Tk()
        app = MotorControlApp(root)
        root.mainloop()
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if 'app' in locals():
            app.cleanup()
