import dynamixel_sdk as dxl
import sys

# Configuration
PORT = '/dev/ttyACM0'  # Change this to your port
BAUDRATE = 1000000  # Default baudrate
PROTOCOL_VERSION = 2.0

# Control table addresses (common for most Dynamixel X series)
ADDR_ID = 7              # Address for ID
ADDR_TORQUE_ENABLE = 64  # Address for Torque Enable

# Get current and new motor ID from command line arguments
if len(sys.argv) > 2:
    CURRENT_ID = int(sys.argv[1])
    NEW_ID = int(sys.argv[2])
else:
    print("Usage: python clearer_test.py <current_id> <new_id>")
    print("Example: python clearer_test.py 1 5")
    print("This will change the motor ID from 1 to 5")
    quit()

# Create port and packet handlers
port_handler = dxl.PortHandler(PORT)
packet_handler = dxl.PacketHandler(PROTOCOL_VERSION)

# Open port
if port_handler.openPort():
    print(f"Succeeded to open the port {PORT}")
else:
    print(f"Failed to open the port {PORT}")
    quit()

# Set baudrate
if port_handler.setBaudRate(BAUDRATE):
    print(f"Succeeded to set the baudrate to {BAUDRATE}")
else:
    print(f"Failed to set the baudrate to {BAUDRATE}")
    quit()

# First, disable torque to allow ID change
dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, CURRENT_ID, ADDR_TORQUE_ENABLE, 0)
if dxl_comm_result != dxl.COMM_SUCCESS:
    print(f"Failed to disable torque: {packet_handler.getTxRxResult(dxl_comm_result)}")
    port_handler.closePort()
    quit()

print(f"Torque disabled on motor ID {CURRENT_ID}")

# Write new ID
dxl_comm_result, dxl_error = packet_handler.write1ByteTxRx(port_handler, CURRENT_ID, ADDR_ID, NEW_ID)

# Check for communication errors
if dxl_comm_result != dxl.COMM_SUCCESS:
    print(f"Communication error: {packet_handler.getTxRxResult(dxl_comm_result)}")
elif dxl_error != 0:
    print(f"Hardware error: {packet_handler.getRxPacketError(dxl_error)}")
else:
    print(f"Successfully changed motor ID from {CURRENT_ID} to {NEW_ID}")
    
    # Verify by pinging the new ID
    model_number, dxl_comm_result, dxl_error = packet_handler.ping(port_handler, NEW_ID)
    if dxl_comm_result == dxl.COMM_SUCCESS:
        print(f"Verified: Motor now responds at ID {NEW_ID}")
    else:
        print(f"Warning: Could not verify motor at new ID {NEW_ID}")

# Close port
port_handler.closePort()
print("Port closed")
