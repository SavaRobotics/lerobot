#!/usr/bin/env python3
"""
Continuously read Dynamixel servo angle and control xarm joint 1 accordingly.
The xarm will follow the Dynamixel movements within a safe range.
Press Ctrl+C to stop.

Usage: python dynamixel_to_xarm_control.py [port] [motor_id]
"""

import sys
import time
import math
from lerobot.common.motors import Motor, MotorNormMode
from lerobot.common.motors.dynamixel import DynamixelMotorsBus
from xarm.wrapper import XArmAPI


# Default configuration
DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_MOTOR_ID = 1
UPDATE_RATE = 10  # Hz
XARM_IP = "192.168.1.197"
SAFETY_RANGE = 60.0  # degrees - max deviation from initial xarm position


# Parse command line arguments
port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
motor_id = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_MOTOR_ID

print(f"=== Dynamixel to xarm Control ===")
print(f"Dynamixel Port: {port}")
print(f"Dynamixel Motor ID: {motor_id}")
print(f"xarm IP: {XARM_IP}")
print(f"Update rate: {UPDATE_RATE} Hz")
print(f"Safety range: ±{SAFETY_RANGE}°")
print()

# Setup Dynamixel motor
motors = {"servo": Motor(id=motor_id, model="xl330-m077", norm_mode=MotorNormMode.RANGE_M100_100)}
bus = DynamixelMotorsBus(port=port, motors=motors)

# Setup xarm
arm = XArmAPI(XARM_IP)

# Variables to store initial positions
dynamixel_initial = None
xarm_initial = None
last_xarm_target = None

try:
    # Connect to Dynamixel
    print("Connecting to Dynamixel servo...")
    bus.connect()
    print("Dynamixel connected!")
    
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
    
    # Get initial Dynamixel position
    position = bus.read("Present_Position", "servo", normalize=False)
    if position > 2147483647:  # 2^31 - 1
        decoded_position = position - 4294967296  # 2^32
    else:
        decoded_position = position
    dynamixel_initial = (decoded_position / 4096) * 360
    
    # Get initial xarm positions (all joints)
    code, xarm_angles = arm.get_servo_angle(is_radian=False)
    if code != 0:
        raise Exception(f"Failed to get xarm angles, code: {code}")
    xarm_initial = xarm_angles[0]  # Store J1 initial position
    last_xarm_target = xarm_initial
    
    print(f"Initial Dynamixel angle: {dynamixel_initial:.2f}°")
    print(f"Initial xarm J1 angle: {xarm_initial:.2f}°")
    print(f"xarm will move within [{xarm_initial-SAFETY_RANGE:.1f}°, {xarm_initial+SAFETY_RANGE:.1f}°]")
    print("\nStarting control loop (Ctrl+C to stop)...")
    print("Dynamixel → xarm | Target | Status")
    print("-" * 40)
    
    # Continuous control loop
    while True:
        # Read current Dynamixel position
        position = bus.read("Present_Position", "servo", normalize=False)
        
        # Decode two's complement if negative (for Extended Position Mode)
        if position > 2147483647:  # 2^31 - 1
            decoded_position = position - 4294967296  # 2^32
        else:
            decoded_position = position
        
        # Convert to degrees
        dynamixel_current = (decoded_position / 4096) * 360
        
        # Calculate change from initial position
        dynamixel_change = dynamixel_current - dynamixel_initial
        
        # Calculate target xarm angle
        xarm_target = xarm_initial + dynamixel_change
        
        # Apply safety limits
        xarm_target_limited = max(xarm_initial - SAFETY_RANGE, 
                                 min(xarm_initial + SAFETY_RANGE, xarm_target))
        
        # Send movement command (no threshold check)
        # Get current angles for all joints
        code, current_angles = arm.get_servo_angle(is_radian=False)
        if code == 0:
            # Update only J1
            current_angles[0] = xarm_target_limited
            # Convert degrees to radians (xarm expects radians despite is_radian parameter)
            angles_rad = [math.radians(angle) for angle in current_angles]
            # Send command to xarm with all joint angles
            code = arm.set_servo_angle_j(angles_rad, wait=False, is_radian=True)
            if code == 9:
                # Error 9 means not ready - try to re-enable
                arm.set_state(0)
                status = "NOT_READY"
            else:
                last_xarm_target = xarm_target_limited
                status = "MOVING" if code == 0 else f"ERROR({code})"
        else:
            status = f"READ_ERROR({code})"
        
        # Determine if clamped
        clamped = abs(xarm_target - xarm_target_limited) > 0.01
        if clamped:
            status += " CLAMPED"
        
        # Display status
        print(f"\r{dynamixel_current:6.1f}° → {xarm_target_limited:6.1f}° | {xarm_target:6.1f}° | {status:12s}", 
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
