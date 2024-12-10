from dynamixel_sdk import *  # Uses Dynamixel SDK library
import time

# Control table addresses
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116
ADDR_PRESENT_POSITION = 132

# Protocol and connection settings
PROTOCOL_VERSION = 2.0
DEVICENAME = "/dev/tty.usbserial-FT9HDAWY"  # Replace with your port
BAUDRATE = 57600
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0

# Motor IDs
DXL_IDS = [15, 16, 17, 18, 19]

# Joint limits
JOINT_LIMITS = {
    motor_id: {"min": -45, "max": 45} for motor_id in DXL_IDS
}

# Conversion functions
def degrees_to_dxl_units(degrees, range_min=0, range_max=4095):
    """
    Convert degrees to Dynamixel position units.
    Degrees are clamped to the valid range defined by range_min and range_max.
    """
    dxl_units = int((degrees / 360) * 4095)
    return max(range_min, min(range_max, dxl_units))

def dxl_units_to_degrees(dxl_units):
    """
    Convert Dynamixel position units to degrees.
    Assumes 0–4095 maps to 0–360 degrees.
    """
    return (dxl_units / 4095) * 360

def main():
    # Initialize PortHandler and PacketHandler
    port_handler = PortHandler(DEVICENAME)
    packet_handler = PacketHandler(PROTOCOL_VERSION)

    # Open port
    if not port_handler.openPort():
        print("Failed to open port")
        return

    # Set baudrate
    if not port_handler.setBaudRate(BAUDRATE):
        print("Failed to set baudrate")
        return

    # Enable torque for all motors
    for motor_id in DXL_IDS:
        dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(
            port_handler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_ENABLE
        )
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Failed to enable torque for motor {motor_id}: {packet_handler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"Error enabling torque for motor {motor_id}: {packet_handler.getRxPacketError(dxl_error)}")
        else:
            print(f"Torque enabled for motor {motor_id}")

    # Read and display initial positions
    zero_positions = {}
    print("\nInitial Motor Angles (Degrees):")
    for motor_id in DXL_IDS:
        dxl_present_position, dxl_comm_result, dxl_error = packet_handler.read4ByteTxRx(
            port_handler, motor_id, ADDR_PRESENT_POSITION
        )
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Failed to read position for motor {motor_id}: {packet_handler.getTxRxResult(dxl_comm_result)}")
            continue
        elif dxl_error != 0:
            print(f"Error reading position for motor {motor_id}: {packet_handler.getRxPacketError(dxl_error)}")
            continue

        initial_angle = dxl_units_to_degrees(dxl_present_position)
        zero_positions[motor_id] = dxl_present_position  # Store initial position as zero
        print(f"Motor {motor_id}: {initial_angle:.2f} degrees (set as zero)")

    # Special handling for motor 19
    if 19 in zero_positions:
        motor_19_zero = zero_positions[19]
        print(f"Motor 19 raw zero position: {motor_19_zero}")
        if motor_19_zero > 3000 or motor_19_zero < 1000:
            print("Adjusting motor 19 zero position to midpoint (2048)")
            zero_positions[19] = 2048  # Set to the middle of the range for safety

    # Main loop for monitoring and commanding motors
    try:
        while True:
            # Print current relative positions
            print("\nCurrent Motor Positions (Relative to Zero):")
            for motor_id in DXL_IDS:
                dxl_present_position, dxl_comm_result, dxl_error = packet_handler.read4ByteTxRx(
                    port_handler, motor_id, ADDR_PRESENT_POSITION
                )
                if dxl_comm_result != COMM_SUCCESS:
                    print(f"Failed to read position for motor {motor_id}: {packet_handler.getTxRxResult(dxl_comm_result)}")
                    continue
                elif dxl_error != 0:
                    print(f"Error reading position for motor {motor_id}: {packet_handler.getRxPacketError(dxl_error)}")
                    continue

                relative_position = dxl_units_to_degrees(dxl_present_position - zero_positions[motor_id])
                print(f"Motor {motor_id}: {relative_position:.2f} degrees")

            # Accept user input for motor control
            print("\nEnter motor command in format '[motor_id] [desired_angle]'. Type 'exit' to quit.")
            command = input("> ").strip()
            if command.lower() == "exit":
                break

            try:
                motor_id, desired_angle = map(float, command.split())
                motor_id = int(motor_id)

                if motor_id not in DXL_IDS:
                    print(f"Invalid motor ID: {motor_id}")
                    continue

                # Apply joint limits
                min_limit = JOINT_LIMITS[motor_id]["min"]
                max_limit = JOINT_LIMITS[motor_id]["max"]
                if not (min_limit <= desired_angle <= max_limit):
                    print(f"Desired angle out of range for motor {motor_id}: {min_limit} to {max_limit} degrees")
                    continue

                # Convert desired angle to Dynamixel units
                desired_position = degrees_to_dxl_units(desired_angle + dxl_units_to_degrees(zero_positions[motor_id]))

                # Debugging: print the computed desired position
                print(f"Motor {motor_id}: Computed position = {desired_position}")

                if not (0 <= desired_position <= 4095):
                    print(f"Computed position for motor {motor_id} exceeds valid range (0–4095).")
                    continue

                # Send position command
                dxl_comm_result, dxl_error = packet_handler.write4ByteTxRx(
                    port_handler, motor_id, ADDR_GOAL_POSITION, desired_position
                )
                if dxl_comm_result != COMM_SUCCESS:
                    print(f"Failed to set goal position for motor {motor_id}: {packet_handler.getTxRxResult(dxl_comm_result)}")
                elif dxl_error != 0:
                    print(f"Error setting goal position for motor {motor_id}: {packet_handler.getRxPacketError(dxl_error)}")
                else:
                    print(f"Motor {motor_id} moving to {desired_angle:.2f} degrees")
            except ValueError:
                print("Invalid command format. Use '[motor_id] [desired_angle]'.")
    finally:
        # Disable torque and close port
        for motor_id in DXL_IDS:
            packet_handler.write1ByteTxRx(port_handler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE)
        port_handler.closePort()
        print("Program terminated. Port closed.")


if __name__ == "__main__":
    main()
