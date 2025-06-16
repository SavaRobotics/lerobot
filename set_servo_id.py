#!/usr/bin/env python3
"""
Simple script to set the ID of a Dynamixel servo.
Connect only ONE servo at a time when using this script.
"""

import sys
import dynamixel_sdk as dxl

# Configuration
PROTOCOL_VERSION = 2.0
BAUDRATE = 1_000_000
ADDR_ID = 7  # Address for ID in control table
ADDR_TORQUE_ENABLE = 64  # Address for Torque Enable

def set_servo_id(port="/dev/ttyACM0"):
    """Set ID for a single connected Dynamixel servo."""
    
    # Create port and packet handlers
    port_handler = dxl.PortHandler(port)
    packet_handler = dxl.PacketHandler(PROTOCOL_VERSION)
    
    # Open port
    if not port_handler.openPort():
        print(f"Failed to open port {port}")
        return False
    
    # Set baudrate
    if not port_handler.setBaudRate(BAUDRATE):
        print(f"Failed to set baudrate to {BAUDRATE}")
        port_handler.closePort()
        return False
    
    print(f"Connected to port {port}")
    print("Scanning for servo...")
    
    # Broadcast ping to find servo
    data_list, comm = packet_handler.broadcastPing(port_handler)
    
    if comm != dxl.COMM_SUCCESS:
        print(f"Failed to scan: {packet_handler.getTxRxResult(comm)}")
        port_handler.closePort()
        return False
    
    if not data_list:
        print("No servo found! Make sure ONE servo is connected and powered.")
        port_handler.closePort()
        return False
    
    # Get current ID
    current_id = list(data_list.keys())[0]
    model_number = data_list[current_id][0]
    
    print(f"\nFound servo:")
    print(f"  Current ID: {current_id}")
    print(f"  Model: {model_number}")
    
    # Ask for new ID
    while True:
        try:
            new_id = input("\nEnter new ID (1-253) or 'q' to quit: ")
            
            if new_id.lower() == 'q':
                print("Cancelled.")
                port_handler.closePort()
                return False
            
            new_id = int(new_id)
            
            if 1 <= new_id <= 253:
                break
            else:
                print("ID must be between 1 and 253")
                
        except ValueError:
            print("Please enter a valid number")
    
    if new_id == current_id:
        print(f"ID is already {new_id}. No change needed.")
        port_handler.closePort()
        return True
    
    # Disable torque before changing ID
    comm, error = packet_handler.write1ByteTxRx(port_handler, current_id, ADDR_TORQUE_ENABLE, 0)
    if comm != dxl.COMM_SUCCESS:
        print(f"Failed to disable torque: {packet_handler.getTxRxResult(comm)}")
        port_handler.closePort()
        return False
    
    # Write new ID
    comm, error = packet_handler.write1ByteTxRx(port_handler, current_id, ADDR_ID, new_id)
    
    if comm != dxl.COMM_SUCCESS:
        print(f"Failed to set ID: {packet_handler.getTxRxResult(comm)}")
        port_handler.closePort()
        return False
    
    print(f"\nSuccessfully changed ID from {current_id} to {new_id}")
    
    # Verify by pinging new ID
    model, comm, error = packet_handler.ping(port_handler, new_id)
    if comm == dxl.COMM_SUCCESS:
        print(f"Verified: Servo now responds at ID {new_id}")
    else:
        print(f"Warning: Could not verify servo at new ID {new_id}")
    
    port_handler.closePort()
    return True


def main():
    """Main function."""
    # Get port from command line or use default
    port = sys.argv[1] if len(sys.argv) > 1 else "/dev/ttyACM0"
    
    print("=== Dynamixel Servo ID Setter ===")
    print(f"Port: {port}")
    print("\n⚠️  WARNING: Connect only ONE servo at a time!")
    print("Press Enter to continue...")
    input()
    
    # Set the ID
    if set_servo_id(port):
        print("\nDone!")
    else:
        print("\nFailed to set servo ID")
        sys.exit(1)


if __name__ == "__main__":
    main()
