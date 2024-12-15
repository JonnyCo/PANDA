import serial
import tkinter as tk
from tkinter import messagebox
from dynamixel_sdk import *  # Uses Dynamixel SDK library
import threading
import time

# ----- RFID Configuration -----
RFID_PORT = "/dev/tty.usbmodem1101"  # Replace with RFID serial port
BAUDRATE_RFID = 115200

# ----- Motor Configuration -----
SERVO_PORT = "/dev/tty.usbserial-FT9HDAWY"  # Replace with Dynamixel serial port
BAUDRATE_SERVO = 57600
PROTOCOL_VERSION = 2.0
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
DXL_IDS = [15, 16, 17, 18, 19]

# Dynamixel joint limits
JOINT_LIMITS = {motor_id: {"min": -45, "max": 45} for motor_id in DXL_IDS}

# Conversion functions
def degrees_to_dxl_units(degrees):
    return max(0, min(4095, int((degrees / 360) * 4095)))

def dxl_units_to_degrees(units):
    return (units / 4095) * 360

# ----- RFID Data Map -----
rfid_map = {
    "49:b4:1b:2f": {"name": "Jonny Cohen", "address": "75 Amherst St, Cambridge, MA", "weight": "1kg"},
    "93:da:46:da": {"name": "Neil Gershenfeld", "address": "Center For Bits and Atoms, Cambridge, MA", "weight": "5kg"},
}

# ----- Main GUI and Logic -----
class RobotControlApp:
    def __init__(self, root):
        self.root = root
        self.root.title("RFID and Robot Control")
        self.root.geometry("500x500")

        # RFID Labels
        tk.Label(root, text="RFID Package Recognition", font=("Arial", 16)).pack(pady=5)
        self.label_name = tk.Label(root, text="Name: ", font=("Arial", 12))
        self.label_name.pack(pady=2)
        self.label_address = tk.Label(root, text="Address: ", font=("Arial", 12))
        self.label_address.pack(pady=2)
        self.label_weight = tk.Label(root, text="Weight: ", font=("Arial", 12))
        self.label_weight.pack(pady=2)

        # Motor Control UI
        tk.Label(root, text="Motor Control", font=("Arial", 16)).pack(pady=10)
        self.motor_id_entry = tk.Entry(root, width=10)
        self.motor_id_entry.pack(pady=2)
        self.motor_id_entry.insert(0, "Motor ID")

        self.angle_entry = tk.Entry(root, width=10)
        self.angle_entry.pack(pady=2)
        self.angle_entry.insert(0, "Angle (deg)")

        self.set_button = tk.Button(root, text="Set Motor Angle", command=self.set_motor_angle)
        self.set_button.pack(pady=5)

        self.position_text = tk.Text(root, height=10, width=40)
        self.position_text.pack(pady=5)

        # Start threads
        threading.Thread(target=self.read_rfid, daemon=True).start()
        threading.Thread(target=self.update_motor_positions, daemon=True).start()

        # Initialize servo control
        self.init_servos()

    def read_rfid(self):
        try:
            rfid_ser = serial.Serial(RFID_PORT, BAUDRATE_RFID)
            while True:
                if rfid_ser.in_waiting > 0:
                    uid = rfid_ser.readline().decode().strip()
                    data = rfid_map.get(uid, {"name": "Unknown", "address": "N/A", "weight": "N/A"})
                    self.label_name.config(text=f"Name: {data['name']}")
                    self.label_address.config(text=f"Address: {data['address']}")
                    self.label_weight.config(text=f"Weight: {data['weight']}")
        except serial.SerialException:
            messagebox.showerror("Error", "RFID serial port issue.")

    def init_servos(self):
        self.port_handler = PortHandler(SERVO_PORT)
        self.packet_handler = PacketHandler(PROTOCOL_VERSION)
        if not self.port_handler.openPort() or not self.port_handler.setBaudRate(BAUDRATE_SERVO):
            messagebox.showerror("Error", "Failed to initialize Dynamixel connection.")
            exit()
        for motor_id in DXL_IDS:
            self.packet_handler.write1ByteTxRx(self.port_handler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE)

    def set_motor_angle(self):
        try:
            motor_id = int(self.motor_id_entry.get())
            desired_angle = float(self.angle_entry.get())

            if motor_id not in DXL_IDS:
                messagebox.showerror("Error", "Invalid Motor ID.")
                return

            limits = JOINT_LIMITS[motor_id]
            if not (limits["min"] <= desired_angle <= limits["max"]):
                messagebox.showerror("Error", f"Angle out of range: {limits['min']} to {limits['max']} degrees.")
                return

            position = degrees_to_dxl_units(desired_angle)
            self.packet_handler.write4ByteTxRx(self.port_handler, motor_id, ADDR_GOAL_POSITION, position)
        except ValueError:
            messagebox.showerror("Error", "Enter valid Motor ID and Angle.")

    def update_motor_positions(self):
        while True:
            self.position_text.delete("1.0", tk.END)
            for motor_id in DXL_IDS:
                position, _, _ = self.packet_handler.read4ByteTxRx(self.port_handler, motor_id, ADDR_PRESENT_POSITION)
                angle = dxl_units_to_degrees(position)
                self.position_text.insert(tk.END, f"Motor {motor_id}: {angle:.2f} degrees\n")
            time.sleep(1)

# ----- Run the Application -----
if __name__ == "__main__":
    root = tk.Tk()
    app = RobotControlApp(root)
    root.mainloop()
