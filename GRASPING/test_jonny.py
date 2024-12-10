from dynamixel_sdk import *  # Uses Dynamixel SDK library

# Control table addresses (ensure these match your motor's datasheet)
ADDR_TORQUE_ENABLE = 64
ADDR_GOAL_POSITION = 116  # Commonly 4 bytes
ADDR_PRESENT_POSITION = 132  # Commonly 4 bytes
ADDR_OPERATING_MODE = 11

# Protocol and connection settings
PROTOCOL_VERSION = 2.0  # Protocol version
DEVICENAME = "/dev/tty.usbserial-FT9HDAWY"  # Replace with your port
BAUDRATE = 57600
TORQUE_ENABLE = 1
TORQUE_DISABLE = 0
DXL_MOVING_STATUS_THRESHOLD = 20  # Threshold for movement completion

# Motor IDs
DXL_IDS = [15, 16, 17, 18, 19]

# Helper to convert degrees to Dynamixel units
def degrees_to_dxl_units(degrees):
    return int(degrees / 360 * 4095)

# Helper to convert Dynamixel units to degrees
def dxl_units_to_degrees(dxl_units):
    return dxl_units / 4095 * 360

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

    # Read and display the current position of each motor
    current_positions = {}
    print("\nCurrent Motor Angles:")
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

        # Convert position to degrees and display
        current_angle = dxl_units_to_degrees(dxl_present_position)
        current_positions[motor_id] = current_angle
        print(f"Motor {motor_id}: {current_angle:.2f} degrees")

    # Ask user for desired angles
    desired_positions = {}
    print("\nEnter desired angles for each motor:")
    for motor_id in DXL_IDS:
        while True:
            try:
                desired_angle = float(input(f"Motor {motor_id} (current: {current_positions[motor_id]:.2f} degrees): "))
                desired_positions[motor_id] = degrees_to_dxl_units(desired_angle)
                break
            except ValueError:
                print("Invalid input. Please enter a numeric value.")

    # Move motors to desired angles
    for motor_id in DXL_IDS:
        dxl_comm_result, dxl_error = packet_handler.write4ByteTxRx(
            port_handler, motor_id, ADDR_GOAL_POSITION, desired_positions[motor_id]
        )
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Failed to set goal position for motor {motor_id}: {packet_handler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"Error setting goal position for motor {motor_id}: {packet_handler.getRxPacketError(dxl_error)}")
        else:
            print(f"Motor {motor_id} moving to {dxl_units_to_degrees(desired_positions[motor_id]):.2f} degrees")

    # Wait until all motors reach their target positions
    reached_motors = set()  # Track which motors have reached their position
    print("\nWaiting for motors to reach their target positions...")
    while len(reached_motors) < len(DXL_IDS):
        for motor_id in DXL_IDS:
            if motor_id in reached_motors:
                continue  # Skip motors that have already reached their position

            dxl_present_position, dxl_comm_result, dxl_error = packet_handler.read4ByteTxRx(
                port_handler, motor_id, ADDR_PRESENT_POSITION
            )
            if dxl_comm_result != COMM_SUCCESS:
                print(f"Failed to read position for motor {motor_id}: {packet_handler.getTxRxResult(dxl_comm_result)}")
                continue
            elif dxl_error != 0:
                print(f"Error reading position for motor {motor_id}: {packet_handler.getRxPacketError(dxl_error)}")
                continue

            if abs(desired_positions[motor_id] - dxl_present_position) < DXL_MOVING_STATUS_THRESHOLD:
                print(f"Motor {motor_id} reached target position: {dxl_units_to_degrees(dxl_present_position):.2f} degrees")
                reached_motors.add(motor_id)

    # Disable torque after movement
    for motor_id in DXL_IDS:
        dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(
            port_handler, motor_id, ADDR_TORQUE_ENABLE, TORQUE_DISABLE
        )
        if dxl_comm_result != COMM_SUCCESS:
            print(f"Failed to disable torque for motor {motor_id}: {packet_handler.getTxRxResult(dxl_comm_result)}")
        elif dxl_error != 0:
            print(f"Error disabling torque for motor {motor_id}: {packet_handler.getRxPacketError(dxl_error)}")
        else:
            print(f"Torque disabled for motor {motor_id}")

    # Close port
    port_handler.closePort()

if __name__ == "__main__":
    main()
