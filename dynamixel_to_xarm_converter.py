#!/usr/bin/env python3
"""
Dynamixel to XArm angle converter with proper two's complement handling.
Handles negative angles and provides bounded output for XArm.
"""

import sys
from lerobot.common.motors import Motor, MotorNormMode
from lerobot.common.motors.dynamixel import DynamixelMotorsBus
from lerobot.common.utils.encoding_utils import decode_twos_complement


class DynamixelToXArmConverter:
    """Convert Dynamixel positions to XArm-compatible angles with bounds."""
    
    def __init__(self, 
                 port="/dev/ttyACM0", 
                 motor_id=1,
                 min_angle=-180.0,  # Minimum angle in degrees
                 max_angle=180.0):   # Maximum angle in degrees
        """
        Initialize converter with angle bounds.
        
        Args:
            port: Serial port for Dynamixel
            motor_id: Dynamixel motor ID
            min_angle: Minimum allowed angle in degrees
            max_angle: Maximum allowed angle in degrees
        """
        self.min_angle = min_angle
        self.max_angle = max_angle
        self.motor_id = motor_id
        
        # Setup motor
        self.motors = {"servo": Motor(
            id=motor_id, 
            model="xl330-m077", 
            norm_mode=MotorNormMode.RANGE_M100_100
        )}
        self.bus = DynamixelMotorsBus(port=port, motors=self.motors)
        self.connected = False
        
    def connect(self):
        """Connect to the motor."""
        if not self.connected:
            self.bus.connect()
            self.connected = True
            print(f"Connected to motor ID {self.motor_id}")
            
            # Check operating mode
            mode = self.bus.read("Operating_Mode", "servo")
            if mode == 4:  # Extended Position Mode
                print("Motor is in Extended Position Mode (multi-turn)")
            elif mode == 3:  # Position Mode
                print("Motor is in Position Mode (single-turn)")
            else:
                print(f"Motor is in mode {mode}")
    
    def read_raw_position(self):
        """Read raw position with two's complement decoding."""
        if not self.connected:
            raise RuntimeError("Not connected. Call connect() first.")
        
        # Read raw position
        raw_pos = self.bus.read("Present_Position", "servo", normalize=False)
        
        # Decode two's complement for negative values
        # Extended position mode uses 32-bit signed values
        decoded_pos = decode_twos_complement(raw_pos, n_bytes=4)
        
        return raw_pos, decoded_pos
    
    def read_bounded_angle(self):
        """
        Read angle and apply bounds for XArm compatibility.
        
        Returns:
            tuple: (raw_position, decoded_position, bounded_angle_degrees, was_clamped)
        """
        raw_pos, decoded_pos = self.read_raw_position()
        
        # Convert to degrees
        # XL330 has 4096 positions per revolution
        angle_degrees = (decoded_pos / 4096.0) * 360.0
        
        # Apply bounds
        bounded_angle = angle_degrees
        was_clamped = False
        
        if angle_degrees < self.min_angle:
            bounded_angle = self.min_angle
            was_clamped = True
        elif angle_degrees > self.max_angle:
            bounded_angle = self.max_angle
            was_clamped = True
        
        return raw_pos, decoded_pos, bounded_angle, was_clamped
    
    def continuous_read(self, update_rate=10):
        """Continuously read and display bounded angles."""
        import time
        
        print(f"\nContinuous reading at {update_rate}Hz (Ctrl+C to stop)")
        print(f"Angle bounds: [{self.min_angle}°, {self.max_angle}°]\n")
        
        try:
            while True:
                raw, decoded, angle, clamped = self.read_bounded_angle()
                
                # Format output
                status = "CLAMPED" if clamped else "OK"
                
                print(f"\rRaw: {raw:10d} | Decoded: {decoded:7d} | "
                      f"Angle: {angle:7.1f}° | Status: {status:8s}", 
                      end='', flush=True)
                
                time.sleep(1.0 / update_rate)
                
        except KeyboardInterrupt:
            print("\n\nStopped.")
    
    def disconnect(self):
        """Disconnect from motor."""
        if self.connected:
            self.bus.disconnect()
            self.connected = False
            print("Disconnected")
    
    def __enter__(self):
        self.connect()
        return self
    
    def __exit__(self, exc_type, exc_val, exc_tb):
        self.disconnect()


def main():
    """Example usage with command line arguments."""
    # Parse arguments
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    motor_id = int(sys.argv[2]) if len(sys.argv) > 2 else 1
    min_angle = float(sys.argv[3]) if len(sys.argv) > 3 else -180.0
    max_angle = float(sys.argv[4]) if len(sys.argv) > 4 else 180.0
    
    print("=== Dynamixel to XArm Converter ===")
    print(f"Port: {port}")
    print(f"Motor ID: {motor_id}")
    print(f"Angle bounds: [{min_angle}°, {max_angle}°]")
    
    with DynamixelToXArmConverter(port, motor_id, min_angle, max_angle) as converter:
        # Single read example
        raw, decoded, angle, clamped = converter.read_bounded_angle()
        print(f"\nSingle read:")
        print(f"  Raw value: {raw}")
        print(f"  Decoded position: {decoded}")
        print(f"  Angle: {angle:.1f}°")
        print(f"  Clamped: {clamped}")
        
        # Continuous read
        print("\nStarting continuous read...")
        converter.continuous_read(update_rate=10)


if __name__ == "__main__":
    print("Usage: python dynamixel_to_xarm_converter.py [port] [motor_id] [min_angle] [max_angle]")
    print("Example: python dynamixel_to_xarm_converter.py /dev/ttyACM0 6 -90 90")
    print()
    main()
