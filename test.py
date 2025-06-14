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
from typing import Dict, Tuple, Optional

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


def is_position_valid(raw_pos: int, motor_name: str) -> bool:
    """Check if a position reading is valid (not corrupted)"""
    # Check for obviously corrupted values (large numbers that indicate communication errors)
    # These are typically close to 2^32 - 1 or negative values interpreted as unsigned
    if raw_pos > 4290000000:  # Close to 2^32 - 1, indicates corrupted data
        return False
    
    # Check for reasonable range (Dynamixel XL330 has 0-4095 range, extended position allows more)
    # But anything over 1 million is likely corrupted for normal operation
    if raw_pos > 1000000 and raw_pos < 4290000000:
        return False
        
    return True


def read_positions_with_retry(bus: DynamixelMotorsBus, max_retries: int = 3) -> dict:
    """Read positions with retry logic and error detection"""
    for attempt in range(max_retries + 1):
        try:
            positions = bus.sync_read("Present_Position", normalize=False)
            
            # Validate all position readings
            invalid_motors = []
            for motor_name, raw_pos in positions.items():
                if not is_position_valid(raw_pos, motor_name):
                    invalid_motors.append((motor_name, raw_pos))
            
            # If we have invalid readings and retries left, try again
            if invalid_motors and attempt < max_retries:
                print(f"\n⚠️  Invalid position readings detected (attempt {attempt + 1}):")
                for motor, pos in invalid_motors:
                    print(f"   {motor}: {pos} (likely corrupted)")
                print("   Retrying...")
                time.sleep(0.01)  # Small delay before retry
                continue
            
            # If we still have invalid readings on final attempt, mark them
            if invalid_motors:
                print(f"\n❌ Persistent communication errors after {max_retries} retries:")
                for motor, pos in invalid_motors:
                    print(f"   {motor}: {pos} (corrupted data)")
                    positions[motor] = None  # Mark as invalid
            
            return positions
            
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
        positions = read_positions_with_retry(bus, max_retries=2)
        
        if not positions:
            print("\n❌ Failed to read motor positions")
            return
        
        print("\nMotor Positions:")
        print("-" * 70)
        print(f"{'Motor':<15} {'Raw':<10} {'Normalized':<15} {'Status'}")
        print("-" * 70)
        
        error_count = 0
        for motor_name in MOTOR_CONFIG.keys():
            raw_pos = positions.get(motor_name, None)
            
            # Handle corrupted/invalid readings
            if raw_pos is None:
                print(f"{motor_name:<15} {'ERROR':<10} {'N/A':<15} {'COMM ERROR'}")
                error_count += 1
                continue
            elif not is_position_valid(raw_pos, motor_name):
                print(f"{motor_name:<15} {raw_pos:<10} {'CORRUPTED':<15} {'DATA ERROR'}")
                error_count += 1
                continue
            
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
                    
                    # Check if at limits
                    calib = bus.calibration.get(motor_name)
                    if calib:
                        if raw_pos <= calib.range_min:
                            status = "MIN LIMIT"
                        elif raw_pos >= calib.range_max:
                            status = "MAX LIMIT"
                except:
                    status = "NORM ERROR"
            
            print(f"{motor_name:<15} {raw_pos:<10} {norm_str:<15} {status}")
        
        if error_count > 0:
            print("-" * 70)
            print(f"⚠️  {error_count} motor(s) with communication errors")
            print("   This suggests:")
            print("   • Power supply issues under load")
            print("   • Cable connection problems")
            print("   • Electrical interference")
            print("   • Timing issues in serial communication")
        
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
