# Remember to make sure the serial port is correct. ls /dev/tty.* is a good command

import serial
import tkinter as tk
from tkinter import messagebox

# Map RFID UIDs to people and addresses
rfid_map = {
    "49:b4:1b:2f": {
        "name": "Jonny Cohen",
        "address": "75 Amherst St, Cambridge, MA 02139",
        "weight": "1kg",
        "dims": "0.1 x 0.1 x 0.1m",
        "refrigerate": "No",
        "fragile": "No"
    },
    "93:da:46:da": {
        "name": "Neil Gershenfeld",
        "address": "Center For Bits and Atoms, Cambridge, MA",
        "weight": "5kg",
        "dims": "0.2 x 0.3 x 0.5m",
        "refrigerate": "Keep Below 10.0C",
        "fragile": "Yes"
    },

    "e2:e:97:3f": {
        "name": "Kent Larson",
        "address": "City Science Group, Camrbidge, MA",
        "weight": "0.5Kg",
        "dims": "0.5 x 0.1 x 0.2m",
        "refrigerate": "Keep Below 40.0C",
        "fragile": "No"

    },

    # Add more UIDs as needed
}

# Function to display info
def display_info(uid):
    person = rfid_map.get(uid, None)
    if person:
        # Update all labels with the respective values
        label_name.config(text=f"Name: {person['name']}")
        label_address.config(text=f"Address: {person['address']}")
        label_weight.config(text=f"Weight: {person['weight']}")
        label_dims.config(text=f"Dimensions: {person['dims']}")
        label_refrigerate.config(text=f"Refrigerate: {person['refrigerate']}")
        label_fragile.config(text=f"Fragile: {person['fragile']}")
    else:
        # Default text for unknown UID
        label_name.config(text="Unknown UID")
        label_address.config(text="No information available.")
        label_weight.config(text="Weight: N/A")
        label_dims.config(text="Dimensions: N/A")
        label_refrigerate.config(text="Refrigerate: N/A")
        label_fragile.config(text="Fragile: N/A")


# Set up the GUI
root = tk.Tk()
root.title("RFID Package Recognition Demo")
root.geometry("400x265")

label_title = tk.Label(root, text="RFID Package Recognition", font=("Arial", 16))
label_title.pack(pady=10)

label_name = tk.Label(root, text="Name: ", font=("Arial", 14))
label_name.pack(pady=5)

label_address = tk.Label(root, text="Address: ", font=("Arial", 14))
label_address.pack(pady=5)

label_weight = tk.Label(root, text="Weight: ", font=("Arial", 14))
label_weight.pack(pady=5)

label_dims = tk.Label(root, text="Dimensions: ", font=("Arial", 14))
label_dims.pack(pady=5)

label_refrigerate = tk.Label(root, text="Refrigerate: ", font=("Arial", 14))
label_refrigerate.pack(pady=5)

label_fragile = tk.Label(root, text="Fragile: ", font=("Arial", 14))
label_fragile.pack(pady=5)

# Set up serial connection
try:
    ser = serial.Serial('/dev/tty.usbmodem1101', 115200)  # Replace 'COM3' with your serial port
except serial.SerialException:
    messagebox.showerror("Error", "Unable to open serial port. Check the connection.")
    exit()

# Continuously read from serial
def read_serial():
    if ser.in_waiting > 0:
        uid = ser.readline().decode('utf-8').strip()
        display_info(uid)
    root.after(100, read_serial)

root.after(100, read_serial)
root.mainloop()
