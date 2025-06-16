#!/usr/bin/env python3
"""
Continuously read and display joint angle from a Dynamixel servo.
Press Ctrl+C to stop.

Usage: python read_angle_continuous.py [port] [motor_id]
"""

import sys
import time
from lerobot.common.motors import Motor, MotorNormMode
from lerobot.common.motors.dynamixel import DynamixelMotorsBus


# Default configuration
DEFAULT_PORT = "/dev/ttyACM0"
DEFAULT_MOTOR_ID = 1
UPDATE_RATE = 10  # Hz

# Parse command line arguments
port = sys.argv[1] if len(sys.argv) > 1 else DEFAULT_PORT
motor_id = int(sys.argv[2]) if len(sys.argv) > 2 else DEFAULT_MOTOR_ID

print(f"=== Continuous Angle Reader ===")
print(f"Port: {port}")
print(f"Motor ID: {motor_id}")
print(f"Update rate: {UPDATE_RATE} Hz")
print()

# Setup motor
motors = {"servo": Motor(id=motor_id, model="xl330-m077", norm_mode=MotorNormMode.RANGE_M100_100)}
bus = DynamixelMotorsBus(port=port, motors=motors)

try:
    # Connect
    print("Connecting to Dynamixel servo...")
    bus.connect()
    print("Connected! Reading angles (Ctrl+C to stop)\n")
    
    # Continuous reading loop
    while True:
        # Read position
        position = bus.read("Present_Position", "servo", normalize=False)
        
        # Decode two's complement if negative (for Extended Position Mode)
        if position > 2147483647:  # 2^31 - 1
            decoded_position = position - 4294967296  # 2^32
        else:
            decoded_position = position
        
        # Convert to degrees
        degrees = (decoded_position / 4096) * 360
        
        # Clear line and print - just show the angle cleanly
        print(f"\rAngle: {degrees:8.2f}°", end='', flush=True)
        
        # Control update rate
        time.sleep(1.0 / UPDATE_RATE)
        
except KeyboardInterrupt:
    print("\n\nStopping...")
    
except Exception as e:
    print(f"\nError: {e}")
    
finally:
    # Always disconnect
    if bus.is_connected:
        bus.disconnect()
        print("Disconnected")
