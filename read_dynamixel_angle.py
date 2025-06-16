#!/usr/bin/env python3
"""
Simple script to read joint angle from a Dynamixel servo.
Minimal implementation using LeRobot's Dynamixel interface.
"""

import sys
from lerobot.common.motors import Motor, MotorNormMode
from lerobot.common.motors.dynamixel import DynamixelMotorsBus


def read_servo_angle(port="/dev/ttyACM0", motor_id=1, motor_name="servo"):
    """
    Read the current joint angle from a single Dynamixel servo.
    
    Args:
        port: Serial port for the Dynamixel (default: /dev/ttyACM0)
        motor_id: ID of the Dynamixel motor (default: 1)
        motor_name: Name for the motor (default: "servo")
    
    Returns:
        float: Current position in raw encoder units (0-4095 for XL330)
    """
    # Create motor configuration for single XL330-M077
    motors = {motor_name: Motor(id=motor_id, model="xl330-m077", norm_mode=MotorNormMode.RANGE_M100_100)}
    
    # Create bus instance
    bus = DynamixelMotorsBus(port=port, motors=motors)
    
    try:
        # Connect to the motor
        print(f"Connecting to motor ID {motor_id} on port {port}...")
        bus.connect()
        
        # Read present position
        position = bus.read("Present_Position", motor_name, normalize=False)
        
        # Decode two's complement if negative (for Extended Position Mode)
        # Check if this looks like a negative value encoded as unsigned
        if position > 2147483647:  # 2^31 - 1
            # This is a negative value in two's complement
            decoded_position = position - 4294967296  # 2^32
        else:
            decoded_position = position
        
        # Convert to degrees (optional)
        # XL330 has 4096 positions per 360 degrees
        position_degrees = (decoded_position / 4096) * 360
        
        print(f"\nMotor Status:")
        print(f"  Raw position: {position}")
        print(f"  Decoded position: {decoded_position}")
        print(f"  Angle: {position_degrees:.1f}°")
        
        return position
        
    except Exception as e:
        print(f"Error: {e}")
        return None
        
    finally:
        # Always disconnect
        if bus.is_connected:
            bus.disconnect()
            print("\nDisconnected from motor")


def main():
    """Main function with optional command line arguments."""
    # Default values
    port = "/dev/ttyACM0"
    motor_id = 1
    
    # Parse command line arguments
    if len(sys.argv) > 1:
        port = sys.argv[1]
    if len(sys.argv) > 2:
        motor_id = int(sys.argv[2])
    
    print("=== Dynamixel Servo Angle Reader ===")
    print(f"Port: {port}")
    print(f"Motor ID: {motor_id}")
    print()
    
    # Read the angle
    position = read_servo_angle(port=port, motor_id=motor_id)
    
    if position is None:
        print("\nFailed to read motor position")
        sys.exit(1)


if __name__ == "__main__":
    main()
