#!/usr/bin/env python3

"""
Interactive test script for Koch Leader arm
- Configuration: Set up individual motors
- Calibration: Record motor ranges and save to file
- Teleoperation: Display real-time motor positions
"""

import os
import sys
import time
import select
import termios
import tty
import json
from datetime import datetime
from typing import Dict, Tuple, Optional, List

# Import only from dynamixel module
from lerobot.common.motors.dynamixel import (
    DynamixelMotorsBus,
    OperatingMode,
    DriveMode,
    TorqueMode,
)
from lerobot.common.motors import Motor, MotorCalibration, MotorNormMode

# Constants
PORT = "/dev/ttyACM0"
CALIBRATION_FILE = "calibration.txt"
CHAIN_TEST_FILE = "chain_test_results.txt"
BAUDRATE = 1_000_000

# Motor configuration
MOTOR_CONFIG = {
    "shoulder_pan": {"id": 1, "model": "xl330-m077", "norm_mode": MotorNormMode.RANGE_M100_100},
    "shoulder_lift": {"id": 2, "model": "xl330-m077", "norm_mode": MotorNormMode.RANGE_M100_100},
    "elbow_flex": {"id": 3, "model": "xl330-m077", "norm_mode": MotorNormMode.RANGE_M100_100},
    "wrist_flex": {"id": 4, "model": "xl330-m077", "norm_mode": MotorNormMode.RANGE_M100_100},
    "wrist_roll": {"id": 5, "model": "xl330-m077", "norm_mode": MotorNormMode.RANGE_M100_100},
    "gripper": {"id": 6, "model": "xl330-m077", "norm_mode": MotorNormMode.RANGE_0_100},
}

# Full turn motors (360 degree range)
FULL_TURN_MOTORS = ["shoulder_pan", "wrist_roll"]


def clear_screen():
    """Clear terminal screen"""
    os.system('clear' if os.name == 'posix' else 'cls')


def get_key_press(timeout: float = 0.1) -> Optional[str]:
    """Get a single key press with timeout"""
    old_settings = termios.tcgetattr(sys.stdin)
    try:
        tty.setraw(sys.stdin.fileno())
        rlist, _, _ = select.select([sys.stdin], [], [], timeout)
        if rlist:
            key = sys.stdin.read(1)
            # Handle escape sequences
            if key == '\x1b':
                # Check for additional escape sequence characters
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                if rlist:
                    key += sys.stdin.read(2)
            return key
    finally:
        termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
    return None


def wait_for_enter():
    """Wait for Enter key press"""
    input("Press ENTER to continue...")


def save_calibration(calibration: Dict[str, Dict[str, int]]):
    """Save calibration data to text file"""
    with open(CALIBRATION_FILE, 'w') as f:
        f.write("motor_name,homing_offset,range_min,range_max,drive_mode\n")
        for motor_name, data in calibration.items():
            f.write(f"{motor_name},{data['homing_offset']},{data['range_min']},{data['range_max']},{data['drive_mode']}\n")
    print(f"\nCalibration saved to {CALIBRATION_FILE}")


def load_calibration() -> Optional[Dict[str, MotorCalibration]]:
    """Load calibration data from text file"""
    if not os.path.exists(CALIBRATION_FILE):
        return None
    
    calibration = {}
    try:
        with open(CALIBRATION_FILE, 'r') as f:
            lines = f.readlines()
            if len(lines) < 2:  # Header + at least one motor
                return None
            
            for line in lines[1:]:  # Skip header
                parts = line.strip().split(',')
                if len(parts) == 5:
                    motor_name = parts[0]
                    calibration[motor_name] = MotorCalibration(
                        id=MOTOR_CONFIG[motor_name]["id"],
                        homing_offset=int(parts[1]),
                        range_min=int(parts[2]),
                        range_max=int(parts[3]),
                        drive_mode=int(parts[4])
                    )
        return calibration
    except Exception as e:
        print(f"Error loading calibration: {e}")
        return None


def decode_if_negative(value: int, n_bytes: int = 4) -> int:
    """Decode value as two's complement if it appears to be a negative value interpreted as unsigned"""
    max_signed = (1 << (n_bytes * 8 - 1)) - 1  # Max positive value for signed integer
    if value > max_signed:
        # This is likely a negative value interpreted as unsigned
        return value - (1 << (n_bytes * 8))
    return value


def is_position_valid(raw_pos: int, motor_name: str) -> bool:
    """Check if a position reading is valid (not corrupted)"""
    # First, try to decode if it's a negative value interpreted as unsigned
    decoded_pos = decode_if_negative(raw_pos)
    
    # For extended position mode, valid range is much larger (±256 revolutions)
    # XL330 has 4096 positions per revolution, so ±256 rev = ±1,048,576 positions
    max_extended_position = 256 * 4096  # 1,048,576
    
    # Check if decoded position is within reasonable extended position range
    if abs(decoded_pos) > max_extended_position:
        return False
    
    # If we decoded a negative value, update the raw_pos for further checks
    if decoded_pos != raw_pos:
        # This was a negative value, which is valid in extended position mode
        return True
    
    # For positive values, check normal ranges
    if raw_pos > 1000000:  # Anything over 1 million positive is suspicious
        return False
        
    return True


def read_positions_with_retry(bus: DynamixelMotorsBus, max_retries: int = 3) -> dict:
    """Read positions with retry logic and error detection"""
    for attempt in range(max_retries + 1):
        try:
            positions = bus.sync_read("Present_Position", normalize=False)
            
            # Decode any negative values (two's complement) and validate
            decoded_positions = {}
            invalid_motors = []
            
            for motor_name, raw_pos in positions.items():
                # First decode if it's a negative value
                decoded_pos = decode_if_negative(raw_pos)
                
                # Check if position is valid (using decoded value)
                if not is_position_valid(raw_pos, motor_name):
                    invalid_motors.append((motor_name, raw_pos))
                else:
                    # Store the decoded position
                    decoded_positions[motor_name] = decoded_pos
            
            # If we have invalid readings and retries left, try again
            if invalid_motors and attempt < max_retries:
                print(f"\n⚠️  Invalid position readings detected (attempt {attempt + 1}):")
                for motor, pos in invalid_motors:
                    decoded = decode_if_negative(pos)
                    print(f"   {motor}: {pos} (decoded: {decoded}) - likely corrupted")
                print("   Retrying...")
                time.sleep(0.01)  # Small delay before retry
                continue
            
            # If we still have invalid readings on final attempt, mark them
            if invalid_motors:
                print(f"\n❌ Persistent communication errors after {max_retries} retries:")
                for motor, pos in invalid_motors:
                    decoded = decode_if_negative(pos)
                    print(f"   {motor}: {pos} (decoded: {decoded}) - corrupted data")
                    decoded_positions[motor] = None  # Mark as invalid
            
            return decoded_positions
            
        except Exception as e:
            if attempt < max_retries:
                print(f"\n⚠️  Communication error (attempt {attempt + 1}): {e}")
                print("   Retrying...")
                time.sleep(0.01)
                continue
            else:
                print(f"\n❌ Communication failed after {max_retries} retries: {e}")
                return {}
    
    return {}


def display_motor_values(bus: DynamixelMotorsBus, show_normalized: bool = True):
    """Display current motor positions with error detection"""
    try:
        # Read raw positions directly for accurate display
        raw_positions = bus.sync_read("Present_Position", normalize=False)
        
        if not raw_positions:
            print("\n❌ Failed to read motor positions")
            return
        
        print("\nMotor Positions:")
        print("-" * 90)
        print(f"{'Motor':<15} {'Position':<20} {'Normalized':<15} {'Status'}")
        print("-" * 90)
        
        error_count = 0
        for motor_name in MOTOR_CONFIG.keys():
            raw_pos = raw_positions.get(motor_name, None)
            
            # Handle missing readings
            if raw_pos is None:
                print(f"{motor_name:<15} {'ERROR':<15} {'N/A':<15} {'COMM ERROR'}")
                error_count += 1
                continue
            
            # Decode if negative value (two's complement)
            decoded_pos = decode_if_negative(raw_pos)
            
            # Debug for specific value
            if raw_pos == 4294966982:
                print(f"\n[DEBUG] Found 4294966982 → Decoded to: {decoded_pos}")
            
            # Check if position is valid
            if not is_position_valid(raw_pos, motor_name):
                print(f"{motor_name:<15} {str(raw_pos):<15} {'CORRUPTED':<15} {'DATA ERROR'}")
                error_count += 1
                continue
            
            # Display position - show decoded value if it was negative
            if decoded_pos != raw_pos:
                # This was a negative value - show decoded
                pos_str = f"{decoded_pos}"
            else:
                pos_str = str(decoded_pos)  # Always use decoded value
            
            # Get normalized value if calibrated
            norm_str = "N/A"
            status = "OK"
            
            if show_normalized and bus.is_calibrated:
                try:
                    norm_pos = bus.read("Present_Position", motor_name, normalize=True)
                    if MOTOR_CONFIG[motor_name]["norm_mode"] == MotorNormMode.RANGE_M100_100:
                        norm_str = f"{norm_pos:7.1f}%"
                    elif MOTOR_CONFIG[motor_name]["norm_mode"] == MotorNormMode.RANGE_0_100:
                        norm_str = f"{norm_pos:6.1f}%"
                    
                    # Check if at limits using decoded position
                    calib = bus.calibration.get(motor_name)
                    if calib:
                        if decoded_pos <= calib.range_min:
                            status = "MIN LIMIT"
                        elif decoded_pos >= calib.range_max:
                            status = "MAX LIMIT"
                except:
                    status = "NORM ERROR"
            
            print(f"{motor_name:<15} {pos_str:<20} {norm_str:<15} {status}")
        
        # Note about two's complement if we see any negative positions
        has_negative = any(decode_if_negative(pos) != pos for pos in raw_positions.values() if pos is not None)
        if has_negative:
            print("-" * 80)
            print("📍 Note: Negative positions detected and decoded from two's complement")
        
        if error_count > 0:
            print("-" * 80)
            print(f"⚠️  {error_count} motor(s) with communication errors")
        
    except Exception as e:
        print(f"❌ Error in display function: {e}")


def optimize_communication(bus: DynamixelMotorsBus):
    """Optimize communication settings to reduce errors"""
    print("\n🔧 Optimizing communication settings...")
    
    try:
        # Reduce Return_Delay_Time to minimum (from lerobot implementation)
        # Default is usually 250 (500µs), we set it to 0 (2µs minimum)
        for motor_name in MOTOR_CONFIG.keys():
            try:
                bus.write("Return_Delay_Time", motor_name, 0)
                print(f"   ✓ {motor_name}: Return delay optimized")
            except Exception as e:
                print(f"   ⚠ {motor_name}: Failed to optimize ({e})")
        
        print("✓ Communication optimization complete")
        return True
        
    except Exception as e:
        print(f"❌ Communication optimization failed: {e}")
        return False


def test_communication_stability(bus: DynamixelMotorsBus, test_duration: int = 10):
    """Test communication stability over time"""
    print(f"\n🔍 Testing communication stability for {test_duration} seconds...")
    print("   This will help identify if errors are consistent or intermittent.")
    
    total_reads = 0
    error_count = 0
    error_motors = {}
    
    start_time = time.time()
    
    try:
        while time.time() - start_time < test_duration:
            total_reads += 1
            positions = read_positions_with_retry(bus, max_retries=0)  # No retries for this test
            
            # Check for errors
            for motor_name, raw_pos in positions.items():
                if raw_pos is None or not is_position_valid(raw_pos, motor_name):
                    error_count += 1
                    error_motors[motor_name] = error_motors.get(motor_name, 0) + 1
            
            # Progress indicator
            if total_reads % 20 == 0:
                elapsed = time.time() - start_time
                print(f"\r   Progress: {elapsed:.1f}s - {total_reads} reads - {error_count} errors", end='', flush=True)
            
            time.sleep(0.05)  # 20Hz sampling rate
    
    except KeyboardInterrupt:
        print("\n   Test interrupted by user")
    
    print(f"\n\n📊 Communication Test Results:")
    print(f"   Total reads: {total_reads}")
    print(f"   Total errors: {error_count}")
    
    if total_reads > 0:
        error_rate = (error_count / total_reads) * 100
        print(f"   Error rate: {error_rate:.2f}%")
        
        if error_rate > 10:
            print("   🔴 HIGH ERROR RATE - Significant communication issues")
        elif error_rate > 5:
            print("   🟡 MODERATE ERROR RATE - Some communication issues")
        elif error_rate > 0:
            print("   🟢 LOW ERROR RATE - Minor communication issues")
        else:
            print("   ✅ NO ERRORS - Communication is stable")
    
    if error_motors:
        print(f"\n   Errors by motor:")
        for motor, count in error_motors.items():
            rate = (count / total_reads) * 100
            print(f"     {motor}: {count} errors ({rate:.1f}%)")
    
    return error_count == 0


def diagnose_system(bus: DynamixelMotorsBus):
    """Run comprehensive system diagnostics"""
    clear_screen()
    print("=== SYSTEM DIAGNOSTICS ===")
    print("\nRunning comprehensive diagnostics to identify issues...")
    
    if not bus.is_connected:
        print("❌ Bus not connected. Please connect first.")
        wait_for_enter()
        return
    
    # Test 1: Basic connectivity
    print("\n1️⃣ Testing basic connectivity...")
    try:
        ping_results = bus.broadcast_ping()
        if ping_results:
            print(f"   ✅ Found {len(ping_results)} motors responding")
            for motor_id, model in ping_results.items():
                print(f"      ID {motor_id}: Model {model}")
        else:
            print("   ❌ No motors responding to ping")
    except Exception as e:
        print(f"   ❌ Ping failed: {e}")
    
    # Test 2: Single position read
    print("\n2️⃣ Testing single position reads...")
    try:
        positions = bus.sync_read("Present_Position", normalize=False)
        valid_count = 0
        for motor_name, pos in positions.items():
            if is_position_valid(pos, motor_name):
                print(f"   ✅ {motor_name}: {pos} (valid)")
                valid_count += 1
            else:
                print(f"   ❌ {motor_name}: {pos} (CORRUPTED)")
        
        print(f"   Summary: {valid_count}/{len(positions)} motors with valid readings")
    except Exception as e:
        print(f"   ❌ Position read failed: {e}")
    
    # Test 3: Communication stability
    print("\n3️⃣ Testing communication stability...")
    stability_ok = test_communication_stability(bus, test_duration=5)
    
    # Test 4: Check Return_Delay_Time settings
    print("\n4️⃣ Checking communication timing settings...")
    try:
        for motor_name in MOTOR_CONFIG.keys():
            try:
                delay_time = bus.read("Return_Delay_Time", motor_name)
                if delay_time > 10:
                    print(f"   ⚠ {motor_name}: Return delay {delay_time} (should be 0-2)")
                else:
                    print(f"   ✅ {motor_name}: Return delay {delay_time} (optimal)")
            except:
                print(f"   ❌ {motor_name}: Cannot read return delay time")
    except Exception as e:
        print(f"   ❌ Timing check failed: {e}")
    
    # Recommendations
    print(f"\n💡 RECOMMENDATIONS:")
    if not stability_ok:
        print("   • Power supply: Check voltage stability (12V ±5%)")
        print("   • Cables: Inspect for damage, ensure tight connections")
        print("   • Environment: Check for electrical interference")
        print("   • Load: Try reducing mechanical load on motors")
        print("   • Optimize communication settings (option in main menu)")
    else:
        print("   • System appears stable!")
        print("   • If issues persist, they may be load/position dependent")
    
    wait_for_enter()


def configure_motors(bus: DynamixelMotorsBus):
    """Interactive motor configuration - setup individual motors with proper IDs"""
    # Disconnect to allow reconfiguration
    was_connected = bus.is_connected
    if was_connected:
        print("\nDisconnecting for motor configuration...")
        bus.disconnect()
        time.sleep(0.5)
    
    while True:
        clear_screen()
        print("=== MOTOR CONFIGURATION ===")
        print("\nThis will configure individual motors with their proper IDs.")
        print("You will need to connect ONE motor at a time.\n")
        
        print("Select motor to configure:")
        for i, motor_name in enumerate(MOTOR_CONFIG.keys(), 1):
            motor_id = MOTOR_CONFIG[motor_name]["id"]
            print(f"{i}. {motor_name} (Target ID: {motor_id})")
        
        print("\nA. Configure ALL motors in sequence")
        print("ESC - Return to main menu")
        print("\nPress a number (1-6) or A: ", end='', flush=True)
        
        key = get_key_press(timeout=None)
        
        if key == '\x1b' or key == '\x1b[':  # ESC
            break
        
        if key and key.upper() == 'A':
            # Configure all motors in reverse order like in koch_leader.py
            motor_list = list(reversed(list(MOTOR_CONFIG.keys())))
            for motor_name in motor_list:
                if not configure_single_motor(bus, motor_name):
                    print(f"\n✗ Failed to configure {motor_name}. Stopping sequence.")
                    wait_for_enter()
                    break
            continue
        
        if key and key.isdigit():
            motor_idx = int(key) - 1
            motor_list = list(MOTOR_CONFIG.keys())
            
            if 0 <= motor_idx < len(motor_list):
                motor_name = motor_list[motor_idx]
                configure_single_motor(bus, motor_name)
    
    # Reconnect if was connected before
    if was_connected:
        print("\nReconnecting motors...")
        try:
            # Create temporary bus to reconnect all motors
            temp_bus = DynamixelMotorsBus(
                port=PORT,
                motors=bus.motors,
                calibration=bus.calibration
            )
            temp_bus.connect()
            print("✓ Successfully reconnected all motors")
            bus.port_handler = temp_bus.port_handler
            bus.packet_handler = temp_bus.packet_handler
            bus.sync_reader = temp_bus.sync_reader
            bus.sync_writer = temp_bus.sync_writer
        except Exception as e:
            print(f"✗ Failed to reconnect: {e}")
            wait_for_enter()


def configure_single_motor(bus: DynamixelMotorsBus, motor_name: str) -> bool:
    """Configure a single motor - returns True if successful"""
    clear_screen()
    print(f"=== CONFIGURING {motor_name.upper()} ===")
    
    target_id = MOTOR_CONFIG[motor_name]["id"]
    model = MOTOR_CONFIG[motor_name]["model"]
    
    print(f"\nTarget motor: {motor_name}")
    print(f"Target ID: {target_id}")
    print(f"Expected model: {model}")
    
    print("\n" + "="*60)
    print(f"IMPORTANT: Connect ONLY the '{motor_name}' motor to the controller.")
    print("Disconnect all other motors!")
    print("="*60)
    
    print(f"\nConnect the controller board to the '{motor_name}' motor ONLY and press ENTER...")
    wait_for_enter()
    
    try:
        # Create a temporary bus instance for single motor configuration
        temp_bus = DynamixelMotorsBus(
            port=PORT,
            motors={motor_name: Motor(target_id, model, MOTOR_CONFIG[motor_name]["norm_mode"])},
            calibration=None
        )
        
        print(f"\nSearching for motor on port {PORT}...")
        
        # Try to setup the motor (this will scan baudrates and set proper ID)
        temp_bus._connect(handshake=False)
        temp_bus.setup_motor(motor_name)
        
        print(f"\n✓ SUCCESS: '{motor_name}' motor configured with ID {target_id}")
        
        # Verify configuration
        model_num = temp_bus.ping(target_id)
        if model_num:
            print(f"✓ Verification successful - Motor responding at ID {target_id}")
            print(f"  Model number: {model_num}")
        else:
            print(f"✗ WARNING: Motor not responding at ID {target_id}")
            temp_bus.disconnect()
            return False
        
        temp_bus.disconnect()
        return True
        
    except Exception as e:
        print(f"\n✗ ERROR configuring motor: {e}")
        print("\nPossible issues:")
        print("- Motor not connected properly")
        print("- Motor not powered")
        print("- Wrong motor connected")
        print("- Multiple motors connected (only ONE should be connected)")
        return False
    
    finally:
        wait_for_enter()


def calibrate_motors(bus: DynamixelMotorsBus):
    """Full calibration process"""
    clear_screen()
    print("=== MOTOR CALIBRATION ===")
    print("\nThis will calibrate all motors.")
    print("Make sure all motors are connected and powered.")
    
    wait_for_enter()
    
    try:
        # Step 1: Disable torque on all motors
        print("\nDisabling torque on all motors...")
        bus.disable_torque()
        print("✓ Torque disabled")
        
        # Step 2: Set operating modes
        print("\nSetting operating modes...")
        for motor_name in MOTOR_CONFIG.keys():
            if motor_name == "gripper":
                bus.write("Operating_Mode", motor_name, OperatingMode.CURRENT_POSITION.value)
            else:
                bus.write("Operating_Mode", motor_name, OperatingMode.EXTENDED_POSITION.value)
        print("✓ Operating modes set")
        
        # Step 3: Set drive mode for elbow_flex
        print("\nSetting drive modes...")
        bus.write("Drive_Mode", "elbow_flex", DriveMode.INVERTED.value)
        print("✓ Drive modes set")
        
        # Step 4: Record homing positions
        print("\n" + "="*60)
        print("STEP 1: HOMING POSITION")
        print("="*60)
        print("\nMove all joints to their MIDDLE/HOME position.")
        print("This will be the zero reference point.")
        wait_for_enter()
        
        # Read current positions
        positions = bus.sync_read("Present_Position", normalize=False)
        homing_offsets = {}
        
        for motor_name, pos in positions.items():
            model = MOTOR_CONFIG[motor_name]["model"]
            max_resolution = 4096 - 1  # xl330-m077 resolution
            homing_offset = int(max_resolution / 2) - pos
            homing_offsets[motor_name] = homing_offset
            bus.write("Homing_Offset", motor_name, homing_offset)
        
        print("✓ Homing offsets recorded")
        
        # Step 5: Record range of motion
        print("\n" + "="*60)
        print("STEP 2: RANGE OF MOTION")
        print("="*60)
        print(f"\nMove the following joints through their FULL range:")
        
        # Only record ranges for non-full-turn motors
        unknown_range_motors = [m for m in MOTOR_CONFIG.keys() if m not in FULL_TURN_MOTORS]
        print("  " + ", ".join(unknown_range_motors))
        print(f"\nFull-turn motors ({', '.join(FULL_TURN_MOTORS)}) will use full range automatically.")
        print("\nPress ENTER when ready to start recording...")
        wait_for_enter()
        
        print("\nRecording... Move the joints now. Press ENTER when done.")
        
        # Initialize min/max with current positions
        positions = bus.sync_read("Present_Position", normalize=False)
        range_mins = {motor: pos for motor, pos in positions.items()}
        range_maxes = {motor: pos for motor, pos in positions.items()}
        
        # Record ranges
        old_settings = termios.tcgetattr(sys.stdin)
        try:
            while True:
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                if rlist:
                    key = sys.stdin.read(1)
                    if key == '\n' or key == '\r':
                        break
                
                # Update ranges
                positions = bus.sync_read("Present_Position", normalize=False)
                for motor in unknown_range_motors:
                    if motor in positions:
                        range_mins[motor] = min(range_mins[motor], positions[motor])
                        range_maxes[motor] = max(range_maxes[motor], positions[motor])
                
                # Display current ranges
                print("\r" + " " * 80, end='')  # Clear line
                print(f"\rRecording... {len(positions)} motors active", end='', flush=True)
                
        finally:
            termios.tcsetattr(sys.stdin, termios.TCSADRAIN, old_settings)
        
        # Set full range for full-turn motors
        for motor in FULL_TURN_MOTORS:
            range_mins[motor] = 0
            range_maxes[motor] = 4095
        
        print("\n\n✓ Range recording complete")
        
        # Display recorded ranges
        print("\nRecorded ranges:")
        for motor in MOTOR_CONFIG.keys():
            print(f"  {motor}: {range_mins[motor]} - {range_maxes[motor]}")
        
        # Step 6: Create calibration data
        calibration_data = {}
        calibration_dict = {}
        
        for motor_name in MOTOR_CONFIG.keys():
            drive_mode = 1 if motor_name == "elbow_flex" else 0
            
            calibration_data[motor_name] = {
                'homing_offset': homing_offsets[motor_name],
                'range_min': range_mins[motor_name],
                'range_max': range_maxes[motor_name],
                'drive_mode': drive_mode
            }
            
            calibration_dict[motor_name] = MotorCalibration(
                id=MOTOR_CONFIG[motor_name]["id"],
                homing_offset=homing_offsets[motor_name],
                range_min=range_mins[motor_name],
                range_max=range_maxes[motor_name],
                drive_mode=drive_mode
            )
        
        # Write calibration to motors
        bus.write_calibration(calibration_dict)
        
        # Save to file
        save_calibration(calibration_data)
        
        print("\n✓ Calibration complete and saved!")
        
    except Exception as e:
        print(f"\nError during calibration: {e}")
    
    wait_for_enter()


def test_teleoperation(bus: DynamixelMotorsBus):
    """Display real-time motor positions"""
    clear_screen()
    print("=== TELEOPERATION TEST MODE ===")
    print("\nDisplaying real-time motor positions.")
    print("Press ESC to return to main menu.")
    
    # Check if calibrated
    if not bus.is_calibrated:
        print("\nWARNING: Motors not calibrated. Showing raw values only.")
        time.sleep(2)
    
    try:
        while True:
            # Check for ESC key
            key = get_key_press(timeout=0.05)
            if key == '\x1b' or key == '\x1b[':  # ESC
                break
            
            # Clear and display values
            print("\033[H\033[J", end='')  # Clear screen keeping cursor position
            print("=== TELEOPERATION TEST MODE ===")
            print("Press ESC to return to main menu.")
            
            display_motor_values(bus, show_normalized=bus.is_calibrated)
            
            # Small delay to prevent flickering
            time.sleep(0.05)
            
    except KeyboardInterrupt:
        pass
    except Exception as e:
        print(f"\nError in teleoperation: {e}")
        wait_for_enter()


def save_chain_state(chain_data: dict):
    """Save chain test results to JSON file"""
    try:
        with open(CHAIN_TEST_FILE, 'w') as f:
            json.dump(chain_data, f, indent=2)
        print(f"Chain state saved to {CHAIN_TEST_FILE}")
    except Exception as e:
        print(f"Error saving chain state: {e}")


def load_chain_state() -> dict:
    """Load chain test results from JSON file"""
    if not os.path.exists(CHAIN_TEST_FILE):
        # Initialize new chain state
        return {
            "chain_version": "1.0",
            "test_date": datetime.now().isoformat(),
            "current_chain": [],
            "problematic_motors": [],
            "test_sessions": [],
            "available_motors": list(MOTOR_CONFIG.keys())
        }
    
    try:
        with open(CHAIN_TEST_FILE, 'r') as f:
            return json.load(f)
    except Exception as e:
        print(f"Error loading chain state: {e}")
        return load_chain_state()  # Return default if error


def test_single_motor_isolated(motor_name: str, ensure_configured: bool = True) -> Tuple[bool, float, str]:
    """Test a single motor in isolation
    Returns: (success, error_rate, error_details)
    """
    print(f"\n🔍 Testing {motor_name} in isolation...")
    
    # Create a single motor bus
    single_motor = {motor_name: Motor(
        id=MOTOR_CONFIG[motor_name]["id"],
        model=MOTOR_CONFIG[motor_name]["model"],
        norm_mode=MOTOR_CONFIG[motor_name]["norm_mode"]
    )}
    
    bus = DynamixelMotorsBus(
        port=PORT,
        motors=single_motor,
        calibration=None
    )
    
    try:
        # Connect to the motor
        bus.connect()
        
        # Configure if needed
        if ensure_configured:
            try:
                bus.configure_motors()
                print(f"   ✓ Motor configured")
            except:
                print(f"   ⚠ Motor already configured or configuration skipped")
        
        # Phase 1: Stationary test
        print(f"\n📍 Phase 1: Stationary Communication Test")
        print(f"   Testing communication while motor is stationary...")
        stationary_error_count = 0
        stationary_total = 10
        stationary_corrupted = []
        
        for i in range(stationary_total):
            try:
                positions = bus.sync_read("Present_Position", normalize=False)
                if motor_name in positions:
                    pos = positions[motor_name]
                    if not is_position_valid(pos, motor_name):
                        stationary_error_count += 1
                        stationary_corrupted.append(pos)
                else:
                    stationary_error_count += 1
            except:
                stationary_error_count += 1
            
            time.sleep(0.05)
        
        stationary_error_rate = (stationary_error_count / stationary_total) * 100
        
        if stationary_error_count == 0:
            print(f"   ✅ Stationary test: PASSED (0% error rate)")
        else:
            print(f"   ❌ Stationary test: FAILED ({stationary_error_rate:.1f}% error rate)")
            print(f"      Corrupted values: {stationary_corrupted}")
        
        # Phase 2: Movement/torque test
        print(f"\n🔄 Phase 2: Movement & Torque Test")
        print(f"   Now you can manually move/spin the {motor_name} motor.")
        print(f"   Watch for communication errors during movement.")
        print(f"   Try different positions, apply torque, etc.")
        print(f"   Press ENTER when done testing movement...")
        
        movement_error_count = 0
        movement_total = 0
        movement_corrupted = []
        position_history = []
        
        # Start continuous monitoring
        print(f"\n   🔍 Monitoring {motor_name} - Move the motor now!")
        print(f"   {'Read#':<6} {'Position':<10} {'Status':<12} {'Notes'}")
        print(f"   {'-'*6} {'-'*10} {'-'*12} {'-'*20}")
        
        try:
            while True:
                # Check if user pressed enter
                rlist, _, _ = select.select([sys.stdin], [], [], 0.1)
                if rlist:
                    key = sys.stdin.read(1)
                    if key == '\n' or key == '\r':
                        break
                
                movement_total += 1
                
                try:
                    positions = bus.sync_read("Present_Position", normalize=False)
                    if motor_name in positions:
                        pos = positions[motor_name]
                        position_history.append(pos)
                        
                        if not is_position_valid(pos, motor_name):
                            movement_error_count += 1
                            movement_corrupted.append(pos)
                            status = "❌ CORRUPTED"
                            notes = f"Value: {pos}"
                        else:
                            status = "✅ Valid"
                            # Check if position changed significantly (movement detected)
                            if len(position_history) > 5:
                                recent_pos = position_history[-5:]
                                pos_range = max(recent_pos) - min(recent_pos)
                                if pos_range > 50:  # Significant movement
                                    notes = "Movement detected"
                                else:
                                    notes = "Stationary"
                            else:
                                notes = "Starting..."
                        
                        print(f"\r   {movement_total:<6} {pos:<10} {status:<12} {notes}", end='', flush=True)
                    else:
                        movement_error_count += 1
                        print(f"\r   {movement_total:<6} {'NO READ':<10} {'❌ COMM ERR':<12} {'No response'}", end='', flush=True)
                        
                except Exception as e:
                    movement_error_count += 1
                    print(f"\r   {movement_total:<6} {'ERROR':<10} {'❌ EXCEPTION':<12} {str(e)[:15]}", end='', flush=True)
                
                time.sleep(0.1)  # 10Hz monitoring
                
        except KeyboardInterrupt:
            print(f"\n   Test interrupted by user")
        
        print(f"\n\n   📊 Movement Test Results:")
        print(f"      Total reads: {movement_total}")
        print(f"      Errors: {movement_error_count}")
        
        if movement_total > 0:
            movement_error_rate = (movement_error_count / movement_total) * 100
            print(f"      Error rate: {movement_error_rate:.2f}%")
            
            if movement_corrupted:
                print(f"      Sample corrupted values: {movement_corrupted[:5]}")
            
            # Analyze position changes
            if len(position_history) > 10:
                valid_positions = [p for p in position_history if is_position_valid(p, motor_name)]
                if valid_positions:
                    pos_min = min(valid_positions)
                    pos_max = max(valid_positions)
                    pos_range = pos_max - pos_min
                    print(f"      Position range: {pos_min} to {pos_max} (range: {pos_range})")
                    
                    if pos_range > 100:
                        print(f"      ✅ Motor movement detected during test")
                    else:
                        print(f"      ⚠️  Limited movement detected")
        
        # Overall results
        overall_error_count = stationary_error_count + movement_error_count
        overall_total = stationary_total + movement_total
        overall_error_rate = (overall_error_count / overall_total) * 100 if overall_total > 0 else 100
        
        print(f"\n   🎯 Overall Test Summary:")
        print(f"      Stationary errors: {stationary_error_count}/{stationary_total} ({stationary_error_rate:.1f}%)")
        if movement_total > 0:
            print(f"      Movement errors: {movement_error_count}/{movement_total} ({movement_error_rate:.1f}%)")
        print(f"      Combined error rate: {overall_error_rate:.1f}%")
        
        all_corrupted = stationary_corrupted + movement_corrupted
        
        if overall_error_rate == 0:
            print(f"\n   ✅ {motor_name}: PASSED - No errors during stationary or movement testing")
            error_details = ""
            success = True
        elif stationary_error_count > 0 and movement_error_count == 0:
            print(f"\n   ⚠️  {motor_name}: Errors only when stationary - possible hardware issue")
            error_details = f"Stationary errors: {stationary_error_rate:.1f}%, Movement: 0%"
            success = False
        elif stationary_error_count == 0 and movement_error_count > 0:
            print(f"\n   ⚠️  {motor_name}: Errors only during movement - load/torque related")
            error_details = f"Movement errors: {movement_error_rate:.1f}%, Stationary: 0%"
            success = False
        else:
            print(f"\n   ❌ {motor_name}: Errors in both stationary and movement - serious communication issue")
            error_details = f"Combined error rate: {overall_error_rate:.1f}%"
            success = False
        
        if all_corrupted:
            error_details += f", Sample corrupted: {all_corrupted[:3]}"
        
        bus.disconnect()
        return success, overall_error_rate, error_details
            
    except Exception as e:
        print(f"   ❌ {motor_name}: Connection failed - {e}")
        return False, 100.0, str(e)
    finally:
        if bus.is_connected:
            bus.disconnect()


def test_motor_chain(chain_motors: List[str], test_duration: int = 5) -> Tuple[bool, dict]:
    """Test all motors in the current chain
    Returns: (success, test_results)
    """
    if not chain_motors:
        return True, {"error": "No motors in chain"}
    
    print(f"\n🔗 Testing chain with {len(chain_motors)} motor(s)...")
    print(f"   Chain: {' → '.join(chain_motors)}")
    
    # Create bus with chain motors
    motors = {}
    for motor_name in chain_motors:
        motors[motor_name] = Motor(
            id=MOTOR_CONFIG[motor_name]["id"],
            model=MOTOR_CONFIG[motor_name]["model"],
            norm_mode=MOTOR_CONFIG[motor_name]["norm_mode"]
        )
    
    bus = DynamixelMotorsBus(
        port=PORT,
        motors=motors,
        calibration=None
    )
    
    try:
        # Connect to the chain
        print("   Connecting to chain...")
        bus.connect()
        print("   ✓ Connected")
        
        # Test communication stability
        print(f"   Testing stability for {test_duration} seconds...")
        
        total_reads = 0
        error_count = 0
        motor_errors = {motor: 0 for motor in chain_motors}
        corrupted_readings = {motor: [] for motor in chain_motors}
        
        start_time = time.time()
        
        while time.time() - start_time < test_duration:
            total_reads += 1
            
            try:
                positions = bus.sync_read("Present_Position", normalize=False)
                
                for motor_name in chain_motors:
                    if motor_name in positions:
                        pos = positions[motor_name]
                        if not is_position_valid(pos, motor_name):
                            error_count += 1
                            motor_errors[motor_name] += 1
                            corrupted_readings[motor_name].append(pos)
                    else:
                        error_count += 1
                        motor_errors[motor_name] += 1
                        
            except Exception as e:
                error_count += 1
                for motor in chain_motors:
                    motor_errors[motor] += 1
            
            # Progress update
            if total_reads % 20 == 0:
                elapsed = time.time() - start_time
                print(f"\r   Progress: {elapsed:.1f}s - {total_reads} reads - {error_count} errors", end='', flush=True)
            
            time.sleep(0.05)
        
        print()  # New line after progress
        
        # Calculate results
        overall_error_rate = (error_count / (total_reads * len(chain_motors))) * 100
        
        motor_error_rates = {}
        for motor, errors in motor_errors.items():
            rate = (errors / total_reads) * 100
            motor_error_rates[motor] = rate
        
        # Display results
        print(f"\n   📊 Chain Test Results:")
        print(f"      Total reads: {total_reads}")
        print(f"      Overall error rate: {overall_error_rate:.2f}%")
        
        success = overall_error_rate < 5  # Less than 5% is considered stable
        
        if motor_errors:
            print(f"\n   Motor-specific errors:")
            for motor, errors in motor_errors.items():
                rate = motor_error_rates[motor]
                if errors > 0:
                    print(f"      {motor}: {errors} errors ({rate:.1f}%)")
                    if corrupted_readings[motor]:
                        print(f"         Sample corrupted values: {corrupted_readings[motor][:3]}")
                else:
                    print(f"      {motor}: ✅ No errors")
        
        bus.disconnect()
        
        return success, {
            "total_reads": total_reads,
            "error_count": error_count,
            "overall_error_rate": overall_error_rate,
            "motor_error_rates": motor_error_rates,
            "corrupted_readings": corrupted_readings
        }
        
    except Exception as e:
        print(f"   ❌ Chain test failed: {e}")
        return False, {"error": str(e)}
    finally:
        if bus.is_connected:
            bus.disconnect()


def chain_builder_mode():
    """Interactive chain builder mode for systematic motor testing"""
    chain_state = load_chain_state()
    session_id = len(chain_state["test_sessions"]) + 1
    
    while True:
        clear_screen()
        print("=== CHAIN BUILDER MODE ===")
        print("\nSystematically build and test motor chains to identify communication issues.")
        
        # Display current chain
        current_chain = chain_state["current_chain"]
        if current_chain:
            print(f"\n🔗 Current Chain ({len(current_chain)} motors):")
            chain_display = " → ".join([m["motor_name"] for m in current_chain])
            print(f"   {chain_display}")
            
            # Show chain health
            error_motors = [m for m in current_chain if m.get("error_count", 0) > 0]
            if error_motors:
                print(f"   ⚠️  {len(error_motors)} motor(s) with errors")
            else:
                print(f"   ✅ All motors stable")
        else:
            print("\n🔗 Current Chain: Empty")
        
        # Display problematic motors
        if chain_state["problematic_motors"]:
            print(f"\n❌ Problematic Motors ({len(chain_state['problematic_motors'])}):")
            for motor in chain_state["problematic_motors"]:
                print(f"   {motor['motor_name']} - Failed at chain length {motor['failed_at_chain_length']}")
        
        # Available motors
        motors_in_chain = [m["motor_name"] for m in current_chain]
        problematic_names = [m["motor_name"] for m in chain_state["problematic_motors"]]
        available = [m for m in MOTOR_CONFIG.keys() 
                    if m not in motors_in_chain and m not in problematic_names]
        
        if available:
            print(f"\n📦 Available Motors: {', '.join(available)}")
            if available:
                print(f"   Next suggested: {available[0]}")
        
        # Menu options
        print("\nOptions:")
        if available:
            print("1. Test Next Motor (Isolated)")
            print("2. Add Next Motor to Chain")
        if current_chain:
            print("3. Test Current Chain")
            print("4. Remove Last Motor")
        print("5. View Detailed Results")
        print("6. Reset Chain")
        print("7. Save & Exit")
        print("ESC - Return to Main Menu")
        
        print("\nInstructions:")
        if not current_chain:
            print("   • Start by testing motors individually")
            print("   • Successfully tested motors can be added to the chain")
        else:
            print(f"   • Current entry point: {current_chain[0]['motor_name']}")
            print("   • Connect motors in order shown above")
        
        print("\nSelect option: ", end='', flush=True)
        
        key = get_key_press(timeout=None)
        
        if key == '\x1b' or key == '\x1b[':  # ESC
            save_chain_state(chain_state)
            break
        
        elif key == '1' and available:
            # Test motor isolated - let user select which one
            clear_screen()
            print("=== SELECT MOTOR FOR ISOLATED TESTING ===")
            print("\nAvailable motors for testing:")
            
            for i, motor in enumerate(available, 1):
                motor_id = MOTOR_CONFIG[motor]["id"]
                print(f"{i}. {motor} (ID: {motor_id})")
            
            print("\nESC - Return to main menu")
            print("\nSelect motor to test (1-{}): ".format(len(available)), end='', flush=True)
            
            motor_key = get_key_press(timeout=None)
            
            if motor_key == '\x1b' or motor_key == '\x1b[':  # ESC
                continue
            
            if motor_key and motor_key.isdigit():
                motor_idx = int(motor_key) - 1
                if 0 <= motor_idx < len(available):
                    selected_motor = available[motor_idx]
                    
                    print(f"\n\n=== TESTING {selected_motor.upper()} (ISOLATED) ===")
                    print(f"\nConnect ONLY the {selected_motor} motor and press ENTER...")
                    wait_for_enter()
                    
                    success, error_rate, error_details = test_single_motor_isolated(selected_motor)
                    
                    if success:
                        print(f"\n✅ {selected_motor} passed isolated test!")
                        print("You can now add it to the chain.")
                    else:
                        print(f"\n❌ {selected_motor} failed isolated test!")
                        print("This motor has communication issues even when alone.")
                        
                        # Add to problematic motors
                        chain_state["problematic_motors"].append({
                            "motor_name": selected_motor,
                            "id": MOTOR_CONFIG[selected_motor]["id"],
                            "failed_at_chain_length": 0,
                            "error_type": "isolated_test_failure",
                            "error_details": error_details,
                            "error_rate": error_rate
                        })
                    
                    wait_for_enter()
                else:
                    print("\nInvalid selection!")
                    time.sleep(1)
            else:
                print("\nInvalid selection!")
                time.sleep(1)
        
        elif key == '2' and available:
            # Add next motor to chain
            next_motor = available[0]
            
            # First test it isolated
            print(f"\n\n=== ADDING {next_motor.upper()} TO CHAIN ===")
            print(f"\nFirst, testing {next_motor} in isolation...")
            print(f"Connect ONLY the {next_motor} motor and press ENTER...")
            wait_for_enter()
            
            success, error_rate, error_details = test_single_motor_isolated(next_motor)
            
            if not success:
                print(f"\n❌ {next_motor} failed isolated test! Cannot add to chain.")
                chain_state["problematic_motors"].append({
                    "motor_name": next_motor,
                    "id": MOTOR_CONFIG[next_motor]["id"],
                    "failed_at_chain_length": len(current_chain),
                    "error_type": "pre_chain_test_failure",
                    "error_details": error_details,
                    "error_rate": error_rate
                })
                wait_for_enter()
                continue
            
            # Now test with chain
            print(f"\n✅ {next_motor} passed isolated test!")
            print(f"\nNow connect the chain:")
            if current_chain:
                chain_order = [m["motor_name"] for m in current_chain] + [next_motor]
                print(f"   {' → '.join(chain_order)}")
            else:
                print(f"   Just {next_motor}")
            print(f"\nEntry point: {current_chain[0]['motor_name'] if current_chain else next_motor}")
            print("\nPress ENTER when chain is connected...")
            wait_for_enter()
            
            # Test the new chain
            test_chain = [m["motor_name"] for m in current_chain] + [next_motor]
            success, results = test_motor_chain(test_chain)
            
            if success:
                print(f"\n✅ Chain test PASSED! {next_motor} works well in the chain.")
                
                # Add to chain
                chain_state["current_chain"].append({
                    "motor_name": next_motor,
                    "id": MOTOR_CONFIG[next_motor]["id"],
                    "position": len(current_chain) + 1,
                    "added_timestamp": datetime.now().isoformat(),
                    "test_status": "pass",
                    "error_count": 0,
                    "error_rate": results.get("motor_error_rates", {}).get(next_motor, 0)
                })
            else:
                print(f"\n❌ Chain test FAILED! Adding {next_motor} causes communication errors.")
                
                # Determine which motor(s) failed
                if "motor_error_rates" in results:
                    failed_motors = [(m, rate) for m, rate in results["motor_error_rates"].items() if rate > 5]
                    
                    for motor, rate in failed_motors:
                        if motor == next_motor:
                            print(f"   • {motor} is causing new errors ({rate:.1f}% error rate)")
                        else:
                            print(f"   • {motor} started failing after adding {next_motor} ({rate:.1f}% error rate)")
                
                # Add to problematic motors
                chain_state["problematic_motors"].append({
                    "motor_name": next_motor,
                    "id": MOTOR_CONFIG[next_motor]["id"],
                    "failed_at_chain_length": len(current_chain) + 1,
                    "error_type": "chain_communication_failure",
                    "error_details": str(results),
                    "error_rate": results.get("motor_error_rates", {}).get(next_motor, 100)
                })
            
            # Save session data
            chain_state["test_sessions"].append({
                "session_id": session_id,
                "timestamp": datetime.now().isoformat(),
                "action": "add_motor",
                "motor": next_motor,
                "chain_before": [m["motor_name"] for m in current_chain],
                "success": success,
                "results": results
            })
            
            save_chain_state(chain_state)
            wait_for_enter()
        
        elif key == '3' and current_chain:
            # Test current chain
            print("\n\n=== TESTING CURRENT CHAIN ===")
            chain_motors = [m["motor_name"] for m in current_chain]
            print(f"\nEnsure chain is connected: {' → '.join(chain_motors)}")
            print(f"Entry point: {chain_motors[0]}")
            print("\nPress ENTER to start test...")
            wait_for_enter()
            
            success, results = test_motor_chain(chain_motors, test_duration=10)
            
            # Update chain state with results
            if "motor_error_rates" in results:
                for motor in current_chain:
                    motor["error_rate"] = results["motor_error_rates"].get(motor["motor_name"], 0)
                    motor["test_status"] = "pass" if motor["error_rate"] < 5 else "fail"
            
            save_chain_state(chain_state)
            wait_for_enter()
        
        elif key == '4' and current_chain:
            # Remove last motor
            removed = current_chain.pop()
            print(f"\n✓ Removed {removed['motor_name']} from chain")
            
            # Remove from problematic if it was there
            chain_state["problematic_motors"] = [
                m for m in chain_state["problematic_motors"] 
                if m["motor_name"] != removed["motor_name"]
            ]
            
            save_chain_state(chain_state)
            time.sleep(1)
        
        elif key == '5':
            # View detailed results
            clear_screen()
            print("=== DETAILED CHAIN TEST RESULTS ===")
            
            print(f"\nChain Version: {chain_state['chain_version']}")
            print(f"Test Date: {chain_state['test_date']}")
            
            if current_chain:
                print(f"\n✅ Working Chain ({len(current_chain)} motors):")
                for i, motor in enumerate(current_chain, 1):
                    status = "✅" if motor.get("test_status") == "pass" else "❌"
                    error_rate = motor.get("error_rate", 0)
                    print(f"   {i}. {motor['motor_name']} - {status} ({error_rate:.1f}% errors)")
            
            if chain_state["problematic_motors"]:
                print(f"\n❌ Problematic Motors ({len(chain_state['problematic_motors'])}):")
                for motor in chain_state["problematic_motors"]:
                    print(f"\n   {motor['motor_name']} (ID: {motor['id']})")
                    print(f"      Failed at chain length: {motor['failed_at_chain_length']}")
                    print(f"      Error type: {motor['error_type']}")
                    print(f"      Error rate: {motor.get('error_rate', 'N/A')}%")
                    if motor.get("error_details"):
                        print(f"      Details: {motor['error_details'][:100]}...")
            
            if chain_state["test_sessions"]:
                print(f"\n📊 Recent Test Sessions (last 5):")
                for session in chain_state["test_sessions"][-5:]:
                    print(f"\n   Session {session['session_id']} - {session['timestamp'][:19]}")
                    print(f"      Action: {session['action']}")
                    if "motor" in session:
                        print(f"      Motor: {session['motor']}")
                    print(f"      Success: {'✅' if session['success'] else '❌'}")
            
            wait_for_enter()
        
        elif key == '6':
            # Reset chain
            print("\n\n⚠️  This will clear all chain data and start fresh.")
            print("Are you sure? (y/N): ", end='', flush=True)
            confirm = input().strip().lower()
            
            if confirm == 'y':
                chain_state = {
                    "chain_version": "1.0",
                    "test_date": datetime.now().isoformat(),
                    "current_chain": [],
                    "problematic_motors": [],
                    "test_sessions": [],
                    "available_motors": list(MOTOR_CONFIG.keys())
                }
                save_chain_state(chain_state)
                print("✓ Chain reset complete!")
                time.sleep(1)
        
        elif key == '7':
            # Save and exit
            save_chain_state(chain_state)
            print("\n✓ Chain state saved!")
            time.sleep(1)
            break


def main():
    """Main program loop"""
    # Initialize motor configuration
    motors = {}
    for motor_name, config in MOTOR_CONFIG.items():
        motors[motor_name] = Motor(
            id=config["id"],
            model=config["model"],
            norm_mode=config["norm_mode"]
        )
    
    # Try to load existing calibration
    calibration = load_calibration()
    
    # Create bus instance
    bus = DynamixelMotorsBus(
        port=PORT,
        motors=motors,
        calibration=calibration
    )
    
    # Main menu loop
    while True:
        clear_screen()
        print("=== KOCH LEADER ARM TEST SCRIPT ===")
        print(f"\nPort: {PORT}")
        print(f"Motors: {len(motors)}")
        
        # Check connection status
        if bus.is_connected:
            print("Status: CONNECTED")
            if bus.is_calibrated:
                print("Calibration: LOADED")
            else:
                print("Calibration: NOT LOADED")
        else:
            print("Status: NOT CONNECTED")
        
        print("\nMain Menu:")
        print("1. Configure Motors")
        print("2. Calibrate Motors")
        print("3. Test Teleoperation")
        if bus.is_connected:
            print("4. System Diagnostics")
            print("5. Optimize Communication")
        print("6. Chain Builder Mode")
        if bus.is_connected:
            print("D. Disconnect")
        else:
            print("C. Connect to motors")
        print("Q. Quit")
        
        print("\nSelect option: ", end='', flush=True)
        
        key = get_key_press(timeout=None)
        
        if key and key.upper() == 'Q':
            if bus.is_connected:
                print("\n\nDisconnecting...")
                bus.disconnect()
            print("Goodbye!")
            break
        
        elif key == '1':
            # Motor configuration doesn't require connection first
            configure_motors(bus)
        
        elif key == '2':
            if not bus.is_connected:
                print("\n\nConnecting to motors...")
                try:
                    bus.connect()
                    calibrate_motors(bus)
                except Exception as e:
                    print(f"Connection failed: {e}")
                    wait_for_enter()
            else:
                calibrate_motors(bus)
        
        elif key == '3':
            if not bus.is_connected:
                print("\n\nConnecting to motors...")
                try:
                    bus.connect()
                    test_teleoperation(bus)
                except Exception as e:
                    print(f"Connection failed: {e}")
                    wait_for_enter()
            else:
                test_teleoperation(bus)
        
        elif key == '4' and bus.is_connected:
            diagnose_system(bus)
        
        elif key == '5' and bus.is_connected:
            clear_screen()
            print("=== COMMUNICATION OPTIMIZATION ===")
            print("\nThis will optimize communication settings to reduce errors.")
            print("It will set Return_Delay_Time to minimum on all motors.")
            wait_for_enter()
            
            success = optimize_communication(bus)
            if success:
                print("\n✅ Communication settings optimized!")
                print("This should reduce communication errors significantly.")
            else:
                print("\n❌ Optimization failed. Check connection and try again.")
            
            wait_for_enter()
        
        elif key == '6':
            # Chain builder mode
            chain_builder_mode()
        
        elif key and key.upper() == 'C' and not bus.is_connected:
            print("\n\nConnecting to motors...")
            try:
                bus.connect()
                print("Connected successfully!")
                time.sleep(1)
            except Exception as e:
                print(f"Connection failed: {e}")
                wait_for_enter()
        
        elif key and key.upper() == 'D' and bus.is_connected:
            print("\n\nDisconnecting...")
            bus.disconnect()
            print("Disconnected.")
            time.sleep(1)


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\n\nProgram interrupted.")
    except Exception as e:
        print(f"\n\nUnexpected error: {e}")
