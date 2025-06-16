#!/usr/bin/env python3
"""
Super simple servo ID setter using lerobot classes.
Connect only ONE servo at a time!
"""

import sys
from lerobot.common.motors import Motor, MotorNormMode
from lerobot.common.motors.dynamixel import DynamixelMotorsBus


def main():
    # Get port from command line or use default
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    
    print("=== Simple Servo ID Setter ===")
    print(f"Port: {port}")
    print("\n⚠️  Connect only ONE servo!")
    
    # Create a temporary motor config (ID doesn't matter, we'll scan for it)
    temp_motor = {"temp": Motor(id=1, model="xl330-m077", norm_mode=MotorNormMode.RANGE_M100_100)}
    bus = DynamixelMotorsBus(port=port, motors=temp_motor)
    
    try:
        # Connect and scan
        print("\nScanning for servo...")
        bus._connect(handshake=False)
        
        # Find connected servo
        id_model = bus.broadcast_ping()
        
        if not id_model:
            print("No servo found! Check connection and power.")
            return
        
        # Get current ID
        current_id = list(id_model.keys())[0]
        print(f"\nFound servo with ID: {current_id}")
        
        # Ask for new ID
        new_id = input("Enter new ID (1-253): ")
        
        try:
            new_id = int(new_id)
            if not 1 <= new_id <= 253:
                print("ID must be between 1 and 253")
                return
        except ValueError:
            print("Invalid input")
            return
        
        if new_id == current_id:
            print(f"ID is already {new_id}")
            return
        
        # Disable torque and set new ID
        bus._disable_torque(current_id, "xl330-m077")
        bus._write(7, 1, current_id, new_id)  # 7 is ID address, 1 byte
        
        print(f"\nChanged ID from {current_id} to {new_id}")
        
        # Verify
        if bus.ping(new_id):
            print(f"✓ Verified: Servo responds at ID {new_id}")
        else:
            print("⚠ Could not verify new ID")
            
    except Exception as e:
        print(f"\nError: {e}")
    finally:
        bus.disconnect()
        print("\nDone!")


if __name__ == "__main__":
    main()
