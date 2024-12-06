from DXL_motor_control import DXL_Conmunication
import time
import json
import threading
from collections import deque
import os
import traceback

# Constants
DEVICE_NAME = "/dev/ttyUSB0"
B_RATE = 57600
LED_ADDR_LEN = (65, 1)
LED_ON = 1
LED_OFF = 0
TIME_RECORD = 0.0025
TIME_REPLAY = 0.0025
RIME_WAIT_WRITE = 0.04
class ControlCmd:
    def __init__(self):
        # File paths
        self.record_path = os.path.expanduser('./data_local/joint.json')
        #print(f"Resolved path: {self.record_path}")
        
        # State variables
        self.is_recording = False
        self.stop_replay_event = threading.Event()
        
        # Dynamixel setup
        self.dynamixel = DXL_Conmunication(DEVICE_NAME, B_RATE)
        self.dynamixel.activateDXLConnection()

        # Create motors
        self.motor_list = [
            self.dynamixel.createMotor(f'motor{i}', motor_number=i)
            for i in range(1, 17)
        ]
        self.motor_position = {f"motor{i}": 0 for i in range(1, 17)}

        # Initialize motors
        self.dynamixel.rebootAllMotor()
        self.dynamixel.updateMotorData()

    def enable_all_motor(self):
        for motor in self.motor_list:
            motor.enableMotor()

    def disable_all_motor(self):
        for motor in self.motor_list:
            motor.disableMotor()

    def motor_led_control(self, state=LED_OFF):
        for motor in self.motor_list:
            motor.directWriteData(state, *LED_ADDR_LEN)

    def read_all_motor_data(self):
        for motor in self.motor_list:
            position, _ = motor.directReadData(132, 4)
            while abs(position) > 4095:
                position -= (position / abs(position)) * 4095
            self.motor_position[motor.name] = int(position * 360 / 4095)
        print(self.motor_position)
        return self.motor_position

    def motor_position_control(self, position):
        for motor in self.motor_list:
            motor.writePosition(position[motor.name])
        self.dynamixel.sentAllCmd()
        time.sleep(0.05)

    def process_record_action_points(self):
        """
        Records action points (timestamps and motor positions) into a deque.
        Each recorded action point is independent to avoid overwriting issues.
        """
        self.motor_led_control(LED_ON)
        action_points = deque()
        start_time = time.time()

        print("Start recording action points...")
        while self.is_recording:
            elapsed_time = time.time() - start_time
            
            # Read motor data and make a copy to store fresh data
            all_servo_position = self.read_all_motor_data()
            action_points.append({
                "timestamp": elapsed_time,
                "positions": all_servo_position.copy()  # Ensure a new dictionary is stored
            })
            
            time.sleep(0.05)

        self.motor_led_control(LED_OFF)
        print("Finished recording. Saving to file...")

        # Write all action points to the file
        with open(self.record_path, 'w') as f:
            for action in action_points:
                f.write(json.dumps(action) + '\n')
        print("Recording saved.")


    def start_record_action_points(self):
        self.is_recording = True
        self.recording_thread = threading.Thread(target=self.process_record_action_points, daemon=True)
        self.recording_thread.start()

    def stop_record_action_points(self):
        self.is_recording = False
        if self.recording_thread.is_alive():
            self.recording_thread.join()

    def replay_recorded_data(self):
        """
        Replay recorded motor movements from the new JSON format.
        Each action point includes a timestamp and positions dictionary.
        """
        self.enable_all_motor()
        time.sleep(0.1)
        
        with open(self.record_path) as f: 
            one_action_point = f.readline()
            
            while one_action_point and (not self.stop_replay_event.is_set()):
                # Parse the JSON line into a dictionary
                one_action_point = json.loads(one_action_point)
                
                # Extract positions from the JSON data
                positions = one_action_point["positions"]
                
                # Convert positions to the required format for motor control
                motor_positions = {
                    motor: int((positions[motor] * 4095) / 360) for motor in positions
                }
                
                # Send motor positions
                self.motor_position_control(motor_positions)
                        
                time.sleep(0.04)  
                
                # Read next action point
                one_action_point = f.readline()
        
        self.disable_all_motor()
        self.replaying = False
        self.send_real_time_data = True




    def replay_interpolation(self): #NOT WORKING - It will Kill the Panda !!!!
        """
        Replay motor movements with interpolation and dynamic timing based on motor speed and distance.
        """
        self.enable_all_motor()
        time.sleep(0.1)

        # Load recorded action points
        with open(self.record_path) as f:
            action_points = [json.loads(line) for line in f]

        for i in range(len(action_points) - 1):
            if self.stop_replay_event.is_set():
                break

            # Current and next motor positions
            current_positions = action_points[i]["positions"]
            next_positions = action_points[i + 1]["positions"]

            # Calculate interpolation steps based on maximum distance and a fixed speed
            max_distance = max(abs(next_positions[motor] - current_positions[motor]) for motor in current_positions)
            steps = max(int(max_distance / 5), 10)  # Minimum 10 steps

            for step in range(steps):
                interpolated_positions = {
                    motor: int(
                        current_positions[motor] + 
                        (step / steps) * (next_positions[motor] - current_positions[motor])
                    )
                    for motor in current_positions
                }

                # Send interpolated positions to motors
                #self.motor_position_control(interpolated_positions)

                # Dynamic sleep: Calculate time based on distance and speed
                avg_distance = sum(abs(next_positions[motor] - current_positions[motor]) for motor in current_positions) / len(current_positions)
                sleep_time = max(0.01, avg_distance / 1000)  # Dynamic wait proportional to distance
                time.sleep(sleep_time)


        self.disable_all_motor()


    def stop_replay_all(self):
        self.stop_replay_event.set()

if __name__ == "__main__":
    controlcmd = ControlCmd()

    command_dict = {
        "read": controlcmd.read_all_motor_data,
        "record": controlcmd.start_record_action_points,
        "stop": controlcmd.stop_record_action_points,
        "replay": controlcmd.replay_recorded_data,
        "disable": controlcmd.disable_all_motor,
        "replayi": controlcmd.replay_interpolation,
    }

    while True:
        try:
            cmd = input("CMD : ")
            if cmd in command_dict:
                command_dict[cmd]()
            elif cmd == "exit":
                break
        except Exception as e:
            traceback.print_exc()
            break
