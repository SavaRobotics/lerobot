#!/usr/bin/env python3
"""
Continuously read two Dynamixel servo angles and control xarm joints 1 and 2 accordingly.
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

print(f"=== Dual Dynamixel to xarm Control ===")
print(f"Dynamixel Port: {PORT}")
print(f"Dynamixel IDs: 1 (→ xarm J1), 2 (→ xarm J2)")
print(f"xarm IP: {XARM_IP}")  
print(f"Update rate: {UPDATE_RATE} Hz")
print(f"Safety range: ±{SAFETY_RANGE}°")
print()

# Setup both Dynamixel motors
motors = {
    "servo1": Motor(id=1, model="xl330-m077", norm_mode=MotorNormMode.RANGE_M100_100),
    "servo2": Motor(id=2, model="xl330-m077", norm_mode=MotorNormMode.RANGE_M100_100)
}
bus = DynamixelMotorsBus(port=PORT, motors=motors)

# Setup xarm
arm = XArmAPI(XARM_IP)

# Variables to store initial positions
dynamixel1_initial = None
dynamixel2_initial = None
xarm_j1_initial = None
xarm_j2_initial = None

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
    
    # Get initial xarm positions (all joints)
    code, xarm_angles = arm.get_servo_angle(is_radian=False)
    if code != 0:
        raise Exception(f"Failed to get xarm angles, code: {code}")
    xarm_j1_initial = xarm_angles[0]  # Store J1 initial position
    xarm_j2_initial = xarm_angles[1]  # Store J2 initial position
    
    print(f"Initial Dynamixel 1 angle: {dynamixel1_initial:.2f}°")
    print(f"Initial Dynamixel 2 angle: {dynamixel2_initial:.2f}°")
    print(f"Initial xarm J1 angle: {xarm_j1_initial:.2f}°")
    print(f"Initial xarm J2 angle: {xarm_j2_initial:.2f}°")
    print(f"xarm J1 will move within [{xarm_j1_initial-SAFETY_RANGE:.1f}°, {xarm_j1_initial+SAFETY_RANGE:.1f}°]")
    print(f"xarm J2 will move within [{xarm_j2_initial-SAFETY_RANGE:.1f}°, {xarm_j2_initial+SAFETY_RANGE:.1f}°]")
    print("\nStarting control loop (Ctrl+C to stop)...")
    print("Dyn1→xArm1 | Dyn2→xArm2 | Status")
    print("-" * 40)
    
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
        
        # Calculate changes from initial positions
        dynamixel1_change = dynamixel1_current - dynamixel1_initial
        dynamixel2_change = dynamixel2_current - dynamixel2_initial
        
        # Calculate target xarm angles
        xarm_j1_target = xarm_j1_initial + dynamixel1_change
        xarm_j2_target = xarm_j2_initial - dynamixel2_change
        
        # Apply safety limits
        xarm_j1_limited = max(xarm_j1_initial - SAFETY_RANGE, 
                             min(xarm_j1_initial + SAFETY_RANGE, xarm_j1_target))
        xarm_j2_limited = max(xarm_j2_initial - SAFETY_RANGE, 
                             min(xarm_j2_initial + SAFETY_RANGE, xarm_j2_target))
        
        # Send movement command (no threshold check)
        # Get current angles for all joints
        code, current_angles = arm.get_servo_angle(is_radian=False)
        if code == 0:
            # Update J1 and J2
            current_angles[0] = xarm_j1_limited
            current_angles[1] = xarm_j2_limited
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
        if j1_clamped or j2_clamped:
            clamp_status = ""
            if j1_clamped: clamp_status += "J1"
            if j2_clamped: clamp_status += "J2"
            status += f" CLAMP({clamp_status})"
        
        # Display status
        print(f"\r{dynamixel1_current:5.1f}°→{xarm_j1_limited:5.1f}° | {dynamixel2_current:5.1f}°→{xarm_j2_limited:5.1f}° | {status:15s}", 
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
