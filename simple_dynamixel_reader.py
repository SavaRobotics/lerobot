#!/usr/bin/env python3
"""
Ultra-minimal Dynamixel angle reader.
Can be imported or run directly.
"""

from lerobot.common.motors import Motor, MotorNormMode
from lerobot.common.motors.dynamixel import DynamixelMotorsBus


class SimpleDynamixelReader:
    """Minimal class to read from a single Dynamixel servo."""
    
    def __init__(self, port="/dev/ttyACM0", motor_id=1, model="xl330-m077"):
        """Initialize with motor configuration."""
        self.motors = {"servo": Motor(id=motor_id, model=model, norm_mode=MotorNormMode.RANGE_M100_100)}
        self.bus = DynamixelMotorsBus(port=port, motors=self.motors)
        self.connected = False
    
    def connect(self):
        """Connect to the motor."""
        if not self.connected:
            self.bus.connect()
            self.connected = True
    
    def read_angle(self):
        """Read current angle in degrees."""
        if not self.connected:
            raise RuntimeError("Not connected. Call connect() first.")
        
        # Read raw position (0-4095 for XL330)
        position = self.bus.read("Present_Position", "servo", normalize=False)
        
        # Convert to degrees
        degrees = (position / 4096) * 360
        
        return degrees
    
    def read_raw(self):
        """Read raw encoder position (0-4095)."""
        if not self.connected:
            raise RuntimeError("Not connected. Call connect() first.")
        
        return self.bus.read("Present_Position", "servo", normalize=False)
    
    def disconnect(self):
        """Disconnect from the motor."""
        if self.connected:
            self.bus.disconnect()
            self.connected = False
    
    def __enter__(self):
        """Context manager entry."""
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        """Context manager exit."""
        self.disconnect()


# Example usage when run directly
if __name__ == "__main__":
    import sys
    
    # Parse command line arguments
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    motor_id = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    
    print(f"=== Simple Dynamixel Reader ===")
    print(f"Port: {port}")
    print(f"Motor ID: {motor_id}")
    print()
    
    # Method 1: Using context manager (recommended)
    with SimpleDynamixelReader(port=port, motor_id=motor_id) as reader:
        angle = reader.read_angle()
        raw = reader.read_raw()
        print(f"Angle: {angle:.1f}°")
        print(f"Raw position: {raw}")
    
    # Method 2: Manual connect/disconnect
    # reader = SimpleDynamixelReader(port=port, motor_id=motor_id)
    # reader.connect()
    # print(f"Angle: {reader.read_angle():.1f}°")
    # reader.disconnect()
