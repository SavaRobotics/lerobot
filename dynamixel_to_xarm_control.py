#!/usr/bin/env python3
"""
Continuously read five Dynamixel servo angles and control xarm joints 1, 2, 3, 4, and 5 accordingly.
The xarm will follow the Dynamixel movements within a safe range.
Press Ctrl+C to stop.

Usage: python dynamixel_to_xarm_control.py
"""

import time
import math
from lerobot.common.motors import Motor, MotorNormMode
from lerobot.common.motors.dynamixel import DynamixelMotorsBus
from xarm.wrapper import XArmAPI


# Configuration
PORT = "/dev/ttyACM0"
UPDATE_RATE = 10  # Hz
XARM_IP = "192.168.1.197"
SAFETY_RANGE = 60.0  # degrees - max deviation from initial xarm position

print(f"=== Five Dynamixel to xarm Control ===")
print(f"Dynamixel Port: {PORT}")
print(f"Dynamixel IDs: 1 (→ xarm J1), 2 (→ xarm J2), 3 (→ xarm J3), 4 (→ xarm J4), 5 (→ xarm J5)")
print(f"xarm IP: {XARM_IP}")  
print(f"Update rate: {UPDATE_RATE} Hz")
print(f"Safety range: ±{SAFETY_RANGE}°")
print()

# Setup five Dynamixel motors
motors = {
    "servo1": Motor(id=1, model="xl330-m077", norm_mode=MotorNormMode.RANGE_M100_100),
    "servo2": Motor(id=2, model="xl330-m077", norm_mode=MotorNormMode.RANGE_M100_100),
    "servo3": Motor(id=3, model="xl330-m077", norm_mode=MotorNormMode.RANGE_M100_100),
    "servo4": Motor(id=4, model="xl330-m077", norm_mode=MotorNormMode.RANGE_M100_100),
    "servo5": Motor(id=5, model="xl330-m077", norm_mode=MotorNormMode.RANGE_M100_100)
}
bus = DynamixelMotorsBus(port=PORT, motors=motors)

# Setup xarm
arm = XArmAPI(XARM_IP)

# Variables to store initial positions
dynamixel1_initial = None
dynamixel2_initial = None
dynamixel3_initial = None
dynamixel4_initial = None
dynamixel5_initial = None
xarm_j1_initial = None
xarm_j2_initial = None
xarm_j3_initial = None
xarm_j4_initial = None
xarm_j5_initial = None

try:
    # Connect to Dynamixel
    print("Connecting to Dynamixel servos...")
    bus.connect()
    print("Dynamixel servos connected!")
    
    # Connect to xarm
    print("Connecting to xarm...")
    arm.connect()
    print("xarm connected!")
    
    # Set xarm to servo mode and enable
    arm.set_mode(1)  # servo motion mode
    arm.set_state(0)  # enable the arm
    print("xarm set to servo mode and enabled")
    
    # Record initial positions
    print("\nRecording initial positions...")
    
    # Get initial Dynamixel 1 position
    position1 = bus.read("Present_Position", "servo1", normalize=False)
    if position1 > 2147483647:  # 2^31 - 1
        decoded_position1 = position1 - 4294967296  # 2^32
    else:
        decoded_position1 = position1
    dynamixel1_initial = (decoded_position1 / 4096) * 360
    
    # Get initial Dynamixel 2 position  
    position2 = bus.read("Present_Position", "servo2", normalize=False)
    if position2 > 2147483647:  # 2^31 - 1
        decoded_position2 = position2 - 4294967296  # 2^32
    else:
        decoded_position2 = position2
    dynamixel2_initial = (decoded_position2 / 4096) * 360
    
    # Get initial Dynamixel 3 position  
    position3 = bus.read("Present_Position", "servo3", normalize=False)
    if position3 > 2147483647:  # 2^31 - 1
        decoded_position3 = position3 - 4294967296  # 2^32
    else:
        decoded_position3 = position3
    dynamixel3_initial = (decoded_position3 / 4096) * 360
    
    # Get initial Dynamixel 4 position  
    position4 = bus.read("Present_Position", "servo4", normalize=False)
    if position4 > 2147483647:  # 2^31 - 1
        decoded_position4 = position4 - 4294967296  # 2^32
    else:
        decoded_position4 = position4
    dynamixel4_initial = (decoded_position4 / 4096) * 360
    
    # Get initial Dynamixel 5 position  
    position5 = bus.read("Present_Position", "servo5", normalize=False)
    if position5 > 2147483647:  # 2^31 - 1
        decoded_position5 = position5 - 4294967296  # 2^32
    else:
        decoded_position5 = position5
    dynamixel5_initial = (decoded_position5 / 4096) * 360
    
    # Get initial xarm positions (all joints)
    code, xarm_angles = arm.get_servo_angle(is_radian=False)
    if code != 0:
        raise Exception(f"Failed to get xarm angles, code: {code}")
    xarm_j1_initial = xarm_angles[0]  # Store J1 initial position
    xarm_j2_initial = xarm_angles[1]  # Store J2 initial position
    xarm_j3_initial = xarm_angles[2]  # Store J3 initial position
    xarm_j4_initial = xarm_angles[3]  # Store J4 initial position
    xarm_j5_initial = xarm_angles[4]  # Store J5 initial position
    
    print(f"Initial Dynamixel 1 angle: {dynamixel1_initial:.2f}°")
    print(f"Initial Dynamixel 2 angle: {dynamixel2_initial:.2f}°")
    print(f"Initial Dynamixel 3 angle: {dynamixel3_initial:.2f}°")
    print(f"Initial Dynamixel 4 angle: {dynamixel4_initial:.2f}°")
    print(f"Initial Dynamixel 5 angle: {dynamixel5_initial:.2f}°")
    print(f"Initial xarm J1 angle: {xarm_j1_initial:.2f}°")
    print(f"Initial xarm J2 angle: {xarm_j2_initial:.2f}°")
    print(f"Initial xarm J3 angle: {xarm_j3_initial:.2f}°")
    print(f"Initial xarm J4 angle: {xarm_j4_initial:.2f}°")
    print(f"Initial xarm J5 angle: {xarm_j5_initial:.2f}°")
    print(f"xarm J1 will move within [{xarm_j1_initial-SAFETY_RANGE:.1f}°, {xarm_j1_initial+SAFETY_RANGE:.1f}°]")
    print(f"xarm J2 will move within [{xarm_j2_initial-SAFETY_RANGE:.1f}°, {xarm_j2_initial+SAFETY_RANGE:.1f}°]")
    print(f"xarm J3 will move within [{xarm_j3_initial-SAFETY_RANGE:.1f}°, {xarm_j3_initial+SAFETY_RANGE:.1f}°]")
    print(f"xarm J4 will move within [{xarm_j4_initial-SAFETY_RANGE:.1f}°, {xarm_j4_initial+SAFETY_RANGE:.1f}°]")
    print(f"xarm J5 will move within [{xarm_j5_initial-SAFETY_RANGE:.1f}°, {xarm_j5_initial+SAFETY_RANGE:.1f}°]")
    print("\nStarting control loop (Ctrl+C to stop)...")
    print("J1 | J2 | J3 | J4 | J5 | Status")
    print("-" * 45)
    
    # Continuous control loop
    while True:
        # Read current Dynamixel 1 position
        position1 = bus.read("Present_Position", "servo1", normalize=False)
        if position1 > 2147483647:  # 2^31 - 1
            decoded_position1 = position1 - 4294967296  # 2^32
        else:
            decoded_position1 = position1
        dynamixel1_current = (decoded_position1 / 4096) * 360
        
        # Read current Dynamixel 2 position
        position2 = bus.read("Present_Position", "servo2", normalize=False)
        if position2 > 2147483647:  # 2^31 - 1
            decoded_position2 = position2 - 4294967296  # 2^32
        else:
            decoded_position2 = position2
        dynamixel2_current = (decoded_position2 / 4096) * 360
        
        # Read current Dynamixel 3 position
        position3 = bus.read("Present_Position", "servo3", normalize=False)
        if position3 > 2147483647:  # 2^31 - 1
            decoded_position3 = position3 - 4294967296  # 2^32
        else:
            decoded_position3 = position3
        dynamixel3_current = (decoded_position3 / 4096) * 360
        
        # Read current Dynamixel 4 position
        position4 = bus.read("Present_Position", "servo4", normalize=False)
        if position4 > 2147483647:  # 2^31 - 1
            decoded_position4 = position4 - 4294967296  # 2^32
        else:
            decoded_position4 = position4
        dynamixel4_current = (decoded_position4 / 4096) * 360
        
        # Read current Dynamixel 5 position
        position5 = bus.read("Present_Position", "servo5", normalize=False)
        if position5 > 2147483647:  # 2^31 - 1
            decoded_position5 = position5 - 4294967296  # 2^32
        else:
            decoded_position5 = position5
        dynamixel5_current = (decoded_position5 / 4096) * 360
        
        # Calculate changes from initial positions
        dynamixel1_change = dynamixel1_current - dynamixel1_initial
        dynamixel2_change = dynamixel2_current - dynamixel2_initial
        dynamixel3_change = dynamixel3_current - dynamixel3_initial
        dynamixel4_change = dynamixel4_current - dynamixel4_initial
        dynamixel5_change = dynamixel5_current - dynamixel5_initial
        
        # Calculate target xarm angles
        xarm_j1_target = xarm_j1_initial + dynamixel1_change
        xarm_j2_target = xarm_j2_initial - dynamixel2_change
        xarm_j3_target = xarm_j3_initial - dynamixel3_change
        xarm_j4_target = xarm_j4_initial + dynamixel4_change
        xarm_j5_target = xarm_j5_initial + dynamixel5_change
        
        # Apply safety limits
        xarm_j1_limited = max(xarm_j1_initial - SAFETY_RANGE, 
                             min(xarm_j1_initial + SAFETY_RANGE, xarm_j1_target))
        xarm_j2_limited = max(xarm_j2_initial - SAFETY_RANGE, 
                             min(xarm_j2_initial + SAFETY_RANGE, xarm_j2_target))
        xarm_j3_limited = max(xarm_j3_initial - SAFETY_RANGE, 
                             min(xarm_j3_initial + SAFETY_RANGE, xarm_j3_target))
        xarm_j4_limited = max(xarm_j4_initial - SAFETY_RANGE, 
                             min(xarm_j4_initial + SAFETY_RANGE, xarm_j4_target))
        xarm_j5_limited = max(xarm_j5_initial - SAFETY_RANGE, 
                             min(xarm_j5_initial + SAFETY_RANGE, xarm_j5_target))
        
        # Send movement command (no threshold check)
        # Get current angles for all joints
        code, current_angles = arm.get_servo_angle(is_radian=False)
        if code == 0:
            # Update J1, J2, J3, J4, and J5
            current_angles[0] = xarm_j1_limited
            current_angles[1] = xarm_j2_limited
            current_angles[2] = xarm_j3_limited
            current_angles[3] = xarm_j4_limited
            current_angles[4] = xarm_j5_limited
            # Convert degrees to radians (xarm expects radians despite is_radian parameter)
            angles_rad = [math.radians(angle) for angle in current_angles]
            # Send command to xarm with all joint angles
            code = arm.set_servo_angle_j(angles_rad, wait=False, is_radian=True)
            if code == 9:
                # Error 9 means not ready - try to re-enable
                arm.set_state(0)
                status = "NOT_READY"
            else:
                status = "MOVING" if code == 0 else f"ERROR({code})"
        else:
            status = f"READ_ERROR({code})"
        
        # Determine if clamped
        j1_clamped = abs(xarm_j1_target - xarm_j1_limited) > 0.01
        j2_clamped = abs(xarm_j2_target - xarm_j2_limited) > 0.01
        j3_clamped = abs(xarm_j3_target - xarm_j3_limited) > 0.01
        j4_clamped = abs(xarm_j4_target - xarm_j4_limited) > 0.01
        j5_clamped = abs(xarm_j5_target - xarm_j5_limited) > 0.01
        if j1_clamped or j2_clamped or j3_clamped or j4_clamped or j5_clamped:
            clamp_status = ""
            if j1_clamped: clamp_status += "1"
            if j2_clamped: clamp_status += "2"
            if j3_clamped: clamp_status += "3"
            if j4_clamped: clamp_status += "4"
            if j5_clamped: clamp_status += "5"
            status += f" CLAMP(J{clamp_status})"
        
        # Display status
        print(f"\r{xarm_j1_limited:2.0f}°|{xarm_j2_limited:2.0f}°|{xarm_j3_limited:2.0f}°|{xarm_j4_limited:2.0f}°|{xarm_j5_limited:2.0f}°|{status:12s}", 
              end='', flush=True)
        
        # Control update rate
        time.sleep(1.0 / UPDATE_RATE)
        
except KeyboardInterrupt:
    print("\n\nStopping...")
    
except Exception as e:
    print(f"\nError: {e}")
    
finally:
    # Always disconnect
    try:
        if bus.is_connected:
            bus.disconnect()
            print("Dynamixel disconnected")
    except:
        pass
    
    try:
        if arm.connected:
            arm.disconnect()
            print("xarm disconnected")
    except:
        pass
