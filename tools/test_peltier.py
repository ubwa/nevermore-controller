#!/usr/bin/env python3
"""
Unified test script for Nevermore Controller Peltier control.
Supports both USB CDC Serial and BLE connections.

Usage:
    Serial: python test_peltier.py --serial /dev/ttyACM1
    BLE:    python test_peltier.py --ble [device_name]

Requirements:
    Serial: pyserial (pip install pyserial)
    BLE:    bleak (pip install bleak)
"""

import argparse
import struct
import sys
import time
from abc import ABC, abstractmethod
from typing import Optional

# =============================================================================
# GATT Characteristic Handles (from build/generated/nevermore.h)
# =============================================================================

# Environmental Sensing Service - Temperature (BLE Temperature, sint16)
HANDLE_TEMP_INTAKE = 0x0012
HANDLE_TEMP_EXHAUST = 0x0016
HANDLE_TEMP_MCU = 0x001A

# Environmental Sensing Service - Humidity (BLE Humidity, uint16)
HANDLE_HUMIDITY_INTAKE = 0x001E
HANDLE_HUMIDITY_EXHAUST = 0x0022

# Environmental Sensing Service - Pressure (BLE Pressure, uint32)
HANDLE_PRESSURE_INTAKE = 0x0026
HANDLE_PRESSURE_EXHAUST = 0x002A

# Environmental Sensing Service - VOC (uint16)
HANDLE_VOC_INDEX_INTAKE = 0x002E
HANDLE_VOC_INDEX_EXHAUST = 0x0034

# Fan Service
HANDLE_FAN_PWM = 0x0049
HANDLE_FAN_TACHOMETER = 0x0061

# Peltier Characteristic Handles
HANDLE_PELTIER_TEMP_COLD = 0x00A3
HANDLE_PELTIER_TEMP_HOT = 0x00A7
HANDLE_PELTIER_POWER = 0x00AB
HANDLE_PELTIER_ERROR = 0x00AF
HANDLE_PELTIER_TARGET_TEMP = 0x00B3
HANDLE_PELTIER_ENABLE = 0x00B6
HANDLE_PELTIER_MIN_TEMP_COLD = 0x00B9
HANDLE_PELTIER_MAX_TEMP_COLD = 0x00BC
HANDLE_PELTIER_MIN_TEMP_HOT = 0x00BF
HANDLE_PELTIER_MAX_TEMP_HOT = 0x00C2
HANDLE_PELTIER_MAX_DEVIATION = 0x00C5
HANDLE_PELTIER_ENABLE_DELAY = 0x00C8
HANDLE_PELTIER_CYCLE_TIME = 0x00CB
HANDLE_PELTIER_KP = 0x00CE
HANDLE_PELTIER_KI = 0x00D1
HANDLE_PELTIER_KD = 0x00D4
HANDLE_PELTIER_SMOOTH_TIME = 0x00D7
HANDLE_PELTIER_DEW_POINT_SAFETY = 0x00DA
HANDLE_PELTIER_HOT_SIDE_SAFETY = 0x00DD
HANDLE_PELTIER_CONTROL_MODE = 0x00E0
HANDLE_PELTIER_WATERMARK_HIGH = 0x00E3
HANDLE_PELTIER_WATERMARK_LOW = 0x00E6
HANDLE_PELTIER_AUTOTUNE = 0x00E9
HANDLE_CONFIG_RESET_SETTINGS = 0x00FA  # Reset settings (write uint8 flags: 1=calibration, 2=policies, 4=hardware)

# BLE Service UUID
PELTIER_SERVICE_UUID = "3f8a2c1d-4e5b-6a7c-8d9e-0f1a2b3c4d5e"

# USB CDC GATT Protocol Commands
CMD_ATTR_READ = 0xFB
CMD_ATTR_WRITE = 0xFC


# =============================================================================
# Data Encoding/Decoding
# =============================================================================

def decode_temperature(data: bytes) -> float:
    """Decode BLE Temperature (sint16, 0.01°C resolution)"""
    value = struct.unpack("<h", data)[0]
    return value * 0.01

def encode_temperature(temp: float) -> bytes:
    """Encode temperature to BLE format"""
    return struct.pack("<h", int(temp * 100))

def decode_percentage(data: bytes) -> float:
    """Decode BLE Percentage8 (uint8, 0.5% resolution)"""
    return struct.unpack("<B", data)[0] * 0.5

def decode_uint8(data: bytes) -> int:
    return struct.unpack("<B", data)[0]

def decode_uint16(data: bytes) -> int:
    return struct.unpack("<H", data)[0]

def decode_float32(data: bytes) -> float:
    return struct.unpack("<f", data)[0]

def decode_time_second16(data: bytes) -> float:
    """Decode BLE TimeSecond16 (uint16, 0.01s resolution)"""
    return struct.unpack("<H", data)[0] * 0.01

def decode_humidity(data: bytes) -> float:
    """Decode BLE Humidity (uint16, 0.01% resolution)"""
    return struct.unpack("<H", data)[0] * 0.01


# =============================================================================
# Abstract Client Interface
# =============================================================================

class NevermoreClient(ABC):
    """Abstract base class for Nevermore connections"""
    
    @abstractmethod
    def read_attr(self, handle: int) -> Optional[bytes]:
        """Read a GATT attribute by handle"""
        pass
    
    @abstractmethod
    def write_attr(self, handle: int, data: bytes) -> bool:
        """Write a GATT attribute by handle"""
        pass
    
    @abstractmethod
    def close(self):
        """Close the connection"""
        pass
    
    # High-level Peltier functions
    def read_peltier_status(self):
        """Read and display comprehensive Peltier status"""
        print("\n=== Peltier Status ===")
        
        # Monitoring
        print("\n-- Monitoring --")
        data = self.read_attr(HANDLE_PELTIER_TEMP_COLD)
        if data:
            print(f"Cold Side Temperature: {decode_temperature(data):.2f}°C")
        
        data = self.read_attr(HANDLE_PELTIER_TEMP_HOT)
        if data:
            print(f"Hot Side Temperature: {decode_temperature(data):.2f}°C")
        
        data = self.read_attr(HANDLE_TEMP_INTAKE)
        if data:
            print(f"Intake Temperature: {decode_temperature(data):.2f}°C")
        
        data = self.read_attr(HANDLE_PELTIER_POWER)
        if data:
            print(f"Power: {decode_percentage(data):.1f}%")
        
        data = self.read_attr(HANDLE_PELTIER_ERROR)
        if data:
            error = decode_uint8(data)
            error_names = {
                0: "None", 1: "Cold temp too low", 2: "Cold temp too high",
                3: "Hot temp too low", 4: "Hot temp too high",
                5: "Temp deviation exceeded", 6: "Sensor read failure",
                7: "PWM init failure", 8: "Sensor unstable"
            }
            print(f"Error: {error_names.get(error, f'Unknown ({error})')}")
        
        # Control
        print("\n-- Control --")
        data = self.read_attr(HANDLE_PELTIER_TARGET_TEMP)
        if data:
            print(f"Target Temperature: {decode_temperature(data):.2f}°C")
        
        data = self.read_attr(HANDLE_PELTIER_ENABLE)
        if data:
            print(f"Enabled: {'Yes' if decode_uint8(data) else 'No'}")
        
        data = self.read_attr(HANDLE_PELTIER_CONTROL_MODE)
        if data:
            mode = decode_uint8(data)
            print(f"Control Mode: {'PID' if mode == 0 else 'Watermark'}")
        
        # PID parameters
        print("\n-- PID Parameters --")
        data = self.read_attr(HANDLE_PELTIER_KP)
        if data:
            print(f"Kp: {decode_float32(data):.4f}")
        
        data = self.read_attr(HANDLE_PELTIER_KI)
        if data:
            print(f"Ki: {decode_float32(data):.4f}")
        
        data = self.read_attr(HANDLE_PELTIER_KD)
        if data:
            print(f"Kd: {decode_float32(data):.4f}")
        
        # Safety parameters
        print("\n-- Safety Parameters --")
        data = self.read_attr(HANDLE_PELTIER_MAX_DEVIATION)
        if data:
            print(f"Max Temp Deviation: {decode_temperature(data):.2f}°C")
        
        data = self.read_attr(HANDLE_PELTIER_DEW_POINT_SAFETY)
        if data:
            print(f"Dew Point Safety: {decode_temperature(data):.2f}°C")
        
        data = self.read_attr(HANDLE_PELTIER_HOT_SIDE_SAFETY)
        if data:
            print(f"Hot Side Safety: {decode_temperature(data):.2f}°C")
        
        # Auto-tune status
        print("\n-- Auto-Tune --")
        data = self.read_attr(HANDLE_PELTIER_AUTOTUNE)
        if data:
            status = decode_uint8(data)
            status_names = {0: "Idle", 1: "Running", 2: "Complete"}
            print(f"Auto-Tune Status: {status_names.get(status, f'Unknown ({status})')}")
    
    def read_environmental_status(self):
        """Read and display environmental sensor status"""
        print("\n=== Environmental Sensors ===")
        
        print("\n-- Temperature --")
        data = self.read_attr(HANDLE_TEMP_INTAKE)
        if data:
            print(f"Intake:  {decode_temperature(data):6.2f}°C")
        
        data = self.read_attr(HANDLE_TEMP_EXHAUST)
        if data:
            print(f"Exhaust: {decode_temperature(data):6.2f}°C")
        
        data = self.read_attr(HANDLE_TEMP_MCU)
        if data:
            print(f"MCU:     {decode_temperature(data):6.2f}°C")
        
        print("\n-- Humidity --")
        data = self.read_attr(HANDLE_HUMIDITY_INTAKE)
        if data:
            print(f"Intake:  {decode_humidity(data):6.2f}%")
        
        data = self.read_attr(HANDLE_HUMIDITY_EXHAUST)
        if data:
            print(f"Exhaust: {decode_humidity(data):6.2f}%")
    
    def set_target_temperature(self, temp: float):
        """Set target temperature"""
        if self.write_attr(HANDLE_PELTIER_TARGET_TEMP, encode_temperature(temp)):
            print(f"Target temperature set to {temp:.2f}°C")
        else:
            print("Failed to set target temperature")
    
    def enable_peltier(self):
        """Enable Peltier"""
        if self.write_attr(HANDLE_PELTIER_ENABLE, bytes([1])):
            print("Peltier enabled")
        else:
            print("Failed to enable Peltier")
    
    def disable_peltier(self):
        """Disable Peltier"""
        if self.write_attr(HANDLE_PELTIER_ENABLE, bytes([0])):
            print("Peltier disabled")
        else:
            print("Failed to disable Peltier")
    
    def clear_error(self):
        """Clear error status"""
        if self.write_attr(HANDLE_PELTIER_ERROR, bytes([0])):
            print("Error cleared")
        else:
            print("Failed to clear error")
    
    def set_control_mode(self, mode: int):
        """Set control mode: 0=PID, 1=Watermark"""
        if self.write_attr(HANDLE_PELTIER_CONTROL_MODE, bytes([mode])):
            print(f"Control mode set to {'PID' if mode == 0 else 'Watermark'}")
        else:
            print("Failed to set control mode")
    
    def set_pid_parameters(self, kp: float, ki: float, kd: float):
        """Set PID parameters"""
        success = True
        if self.write_attr(HANDLE_PELTIER_KP, struct.pack("<f", kp)):
            print(f"Kp set to {kp:.4f}")
        else:
            success = False
        
        if self.write_attr(HANDLE_PELTIER_KI, struct.pack("<f", ki)):
            print(f"Ki set to {ki:.4f}")
        else:
            success = False
        
        if self.write_attr(HANDLE_PELTIER_KD, struct.pack("<f", kd)):
            print(f"Kd set to {kd:.4f}")
        else:
            success = False
        
        return success
    
    def set_safety_parameters(self, dew_point_safety: float, hot_side_safety: float):
        """Set safety parameters"""
        success = True
        if self.write_attr(HANDLE_PELTIER_DEW_POINT_SAFETY, encode_temperature(dew_point_safety)):
            print(f"Dew Point Safety set to {dew_point_safety:.2f}°C")
        else:
            print("Failed to set Dew Point Safety")
            success = False
        
        if self.write_attr(HANDLE_PELTIER_HOT_SIDE_SAFETY, encode_temperature(hot_side_safety)):
            print(f"Hot Side Safety set to {hot_side_safety:.2f}°C")
        else:
            print("Failed to set Hot Side Safety")
            success = False
        
        return success
    
    def set_max_deviation(self, max_deviation: float):
        """Set maximum temperature deviation (hot-cold) before error"""
        if self.write_attr(HANDLE_PELTIER_MAX_DEVIATION, encode_temperature(max_deviation)):
            print(f"Max Temp Deviation set to {max_deviation:.2f}°C")
            return True
        else:
            print("Failed to set Max Temp Deviation")
            return False
    
    def reset_settings(self, flags: int = 2):
        """Reset settings. Flags: 1=calibration, 2=policies (default), 4=hardware"""
        if self.write_attr(HANDLE_CONFIG_RESET_SETTINGS, struct.pack("<B", flags)):
            flag_names = []
            if flags & 1: flag_names.append("calibration")
            if flags & 2: flag_names.append("policies")
            if flags & 4: flag_names.append("hardware")
            print(f"Settings reset: {', '.join(flag_names)}")
            print("Note: Some settings may require a device restart to take effect.")
            return True
        else:
            print("Failed to reset settings")
            return False
    
    def start_autotune(self):
        """Start PID auto-tune calibration"""
        if self.write_attr(HANDLE_PELTIER_AUTOTUNE, bytes([1])):
            print("PID auto-tune started")
            print("The system will oscillate around the target temperature.")
            print("This will take several minutes to collect enough data.")
        else:
            print("Failed to start auto-tune (check Peltier is enabled)")
    
    def stop_autotune(self):
        """Stop/cancel auto-tune"""
        if self.write_attr(HANDLE_PELTIER_AUTOTUNE, bytes([0])):
            print("PID auto-tune stopped")
        else:
            print("Failed to stop auto-tune")
    
    def apply_autotune(self):
        """Apply auto-tune calculated PID values"""
        if self.write_attr(HANDLE_PELTIER_AUTOTUNE, bytes([2])):
            print("Auto-tune PID values applied")
        else:
            print("Failed to apply auto-tune values")
    
    def monitor_autotune(self, interval: float = 1.0):
        """Monitor auto-tune progress"""
        print("\n=== PID Auto-Tune Monitor (Ctrl+C to stop) ===\n")
        try:
            while True:
                status_data = self.read_attr(HANDLE_PELTIER_AUTOTUNE)
                status = decode_uint8(status_data) if status_data else -1
                status_names = {0: "Idle", 1: "Running", 2: "Complete", -1: "Error"}
                
                intake_data = self.read_attr(HANDLE_TEMP_INTAKE)
                intake = decode_temperature(intake_data) if intake_data else float("nan")
                
                target_data = self.read_attr(HANDLE_PELTIER_TARGET_TEMP)
                target = decode_temperature(target_data) if target_data else float("nan")
                
                power_data = self.read_attr(HANDLE_PELTIER_POWER)
                power = decode_percentage(power_data) if power_data else 0.0
                
                print(f"AutoTune: {status_names.get(status, 'Unknown'):8s} | "
                      f"Intake: {intake:5.2f}°C | Target: {target:5.2f}°C | "
                      f"Power: {power:5.1f}%")
                
                if status == 2:
                    print("\n✓ Auto-tune complete! Use 'apply' to apply the calculated PID values.")
                    break
                
                time.sleep(interval)
        except KeyboardInterrupt:
            print("\nMonitoring stopped")
    
    def monitor(self, interval: float = 1.0):
        """Continuous monitoring"""
        print(f"\n=== Monitoring (Ctrl+C to stop) ===\n")
        try:
            while True:
                # Environmental
                intake_data = self.read_attr(HANDLE_TEMP_INTAKE)
                intake = decode_temperature(intake_data) if intake_data else float("nan")
                
                # Peltier
                cold_data = self.read_attr(HANDLE_PELTIER_TEMP_COLD)
                cold = decode_temperature(cold_data) if cold_data else float("nan")
                
                hot_data = self.read_attr(HANDLE_PELTIER_TEMP_HOT)
                hot = decode_temperature(hot_data) if hot_data else float("nan")
                
                target_data = self.read_attr(HANDLE_PELTIER_TARGET_TEMP)
                target = decode_temperature(target_data) if target_data else float("nan")
                
                power_data = self.read_attr(HANDLE_PELTIER_POWER)
                power = decode_percentage(power_data) if power_data else 0.0
                
                enable_data = self.read_attr(HANDLE_PELTIER_ENABLE)
                enabled = decode_uint8(enable_data) if enable_data else 0
                
                print(f"In={intake:5.1f}°C | Cold={cold:5.1f}°C Hot={hot:5.1f}°C | "
                      f"Target={target:5.1f}°C | PWR={power:5.1f}% | E={enabled}")
                
                time.sleep(interval)
        except KeyboardInterrupt:
            print("\nMonitoring stopped")


# =============================================================================
# Serial Client Implementation
# =============================================================================

class SerialClient(NevermoreClient):
    """Serial (USB CDC) connection to Nevermore controller"""
    
    def __init__(self, port: str, baudrate: int = 115200):
        import serial
        self.ser = serial.Serial(
            port=port, baudrate=baudrate, timeout=2,
            bytesize=serial.EIGHTBITS, parity=serial.PARITY_NONE,
            stopbits=serial.STOPBITS_ONE
        )
        time.sleep(0.2)
        self.ser.reset_input_buffer()
        self.ser.reset_output_buffer()
        print(f"Connected to {port} at {baudrate} baud")
    
    def close(self):
        self.ser.close()
    
    def read_attr(self, handle: int) -> Optional[bytes]:
        # Clear stale data
        if self.ser.in_waiting > 0:
            self.ser.read(self.ser.in_waiting)
        
        # Send read command
        self.ser.write(bytes([CMD_ATTR_READ]) + struct.pack("<H", handle))
        self.ser.flush()
        time.sleep(0.01)
        
        # Read response
        cmd = self.ser.read(1)
        if len(cmd) == 0 or cmd[0] != CMD_ATTR_READ:
            return None
        
        handle_bytes = self.ser.read(2)
        if len(handle_bytes) != 2:
            return None
        
        success = self.ser.read(1)
        if len(success) == 0 or success[0] == 0:
            self.ser.read(2)  # consume size
            return None
        
        size_bytes = self.ser.read(2)
        if len(size_bytes) != 2:
            return None
        size = struct.unpack("<H", size_bytes)[0]
        
        if size == 0 or size > 256:
            return None
        
        data = self.ser.read(size)
        return data if len(data) == size else None
    
    def write_attr(self, handle: int, data: bytes) -> bool:
        self.ser.write(bytes([CMD_ATTR_WRITE]))
        self.ser.write(struct.pack("<H", handle))
        self.ser.write(struct.pack("<H", len(data)))
        self.ser.write(data)
        self.ser.flush()
        
        # Read response
        cmd = self.ser.read(1)
        if len(cmd) == 0 or cmd[0] != CMD_ATTR_WRITE:
            return False
        
        handle_bytes = self.ser.read(2)
        if len(handle_bytes) != 2:
            return False
        
        success = self.ser.read(1)
        return len(success) > 0 and success[0] != 0


# =============================================================================
# BLE Client Implementation
# =============================================================================

class BLEClient(NevermoreClient):
    """BLE connection to Nevermore controller (async wrapper)"""
    
    def __init__(self, client, chars):
        self._client = client
        self._chars = chars
        self._handle_to_char = {}
        
        # Build handle to characteristic mapping
        handle_map = {
            HANDLE_PELTIER_TEMP_COLD: "temp_cold",
            HANDLE_PELTIER_TEMP_HOT: "temp_hot",
            HANDLE_PELTIER_POWER: "power",
            HANDLE_PELTIER_ERROR: "error",
            HANDLE_PELTIER_TARGET_TEMP: "target_temp",
            HANDLE_PELTIER_ENABLE: "enable",
            HANDLE_PELTIER_CONTROL_MODE: "control_mode",
            HANDLE_PELTIER_KP: "kp",
            HANDLE_PELTIER_KI: "ki",
            HANDLE_PELTIER_KD: "kd",
            HANDLE_PELTIER_DEW_POINT_SAFETY: "dew_point_safety",
            HANDLE_PELTIER_HOT_SIDE_SAFETY: "hot_side_safety",
            HANDLE_PELTIER_AUTOTUNE: "autotune",
        }
        for handle, name in handle_map.items():
            if name in chars:
                self._handle_to_char[handle] = chars[name]
    
    def close(self):
        pass  # Handled by async context manager
    
    def read_attr(self, handle: int) -> Optional[bytes]:
        import asyncio
        if handle in self._handle_to_char:
            try:
                return asyncio.get_event_loop().run_until_complete(
                    self._client.read_gatt_char(self._handle_to_char[handle])
                )
            except Exception:
                return None
        return None
    
    def write_attr(self, handle: int, data: bytes) -> bool:
        import asyncio
        if handle in self._handle_to_char:
            try:
                asyncio.get_event_loop().run_until_complete(
                    self._client.write_gatt_char(self._handle_to_char[handle], data)
                )
                return True
            except Exception:
                return False
        return False


# =============================================================================
# Interactive Menu
# =============================================================================

def interactive_menu(client: NevermoreClient):
    """Interactive menu for testing"""
    while True:
        print("\n" + "=" * 50)
        print("=== Nevermore Controller Menu ===")
        print("=" * 50)
        print("\n--- Status ---")
        print("1. Read Peltier status")
        print("2. Read environmental status")
        print("3. Monitor (continuous)")
        
        print("\n--- Peltier Control ---")
        print("10. Set target temperature")
        print("11. Enable Peltier")
        print("12. Disable Peltier")
        print("13. Clear error")
        print("14. Set PID mode")
        print("15. Set Watermark mode")
        
        print("\n--- PID Configuration ---")
        print("20. Set PID parameters (Kp, Ki, Kd)")
        
        print("\n--- Safety Configuration ---")
        print("25. Set safety parameters (dew point, hot side)")
        print("26. Set max temperature deviation")
        
        print("\n--- PID Auto-Tune ---")
        print("30. Start auto-tune")
        print("31. Monitor auto-tune progress")
        print("32. Apply auto-tune results")
        print("33. Stop/cancel auto-tune")
        
        print("\n--- Settings ---")
        print("40. Reset policy settings to defaults")
        print("41. Reset all settings (policies + calibration + hardware)")
        
        print("\n0. Exit")
        
        choice = input("\nEnter choice: ").strip()
        
        if choice == "1":
            client.read_peltier_status()
        elif choice == "2":
            client.read_environmental_status()
        elif choice == "3":
            client.monitor()
        elif choice == "10":
            try:
                temp = float(input("Enter target temperature (°C): "))
                client.set_target_temperature(temp)
            except ValueError:
                print("Invalid temperature")
        elif choice == "11":
            client.enable_peltier()
        elif choice == "12":
            client.disable_peltier()
        elif choice == "13":
            client.clear_error()
        elif choice == "14":
            client.set_control_mode(0)
        elif choice == "15":
            client.set_control_mode(1)
        elif choice == "20":
            try:
                kp = float(input("Enter Kp: "))
                ki = float(input("Enter Ki: "))
                kd = float(input("Enter Kd: "))
                client.set_pid_parameters(kp, ki, kd)
            except ValueError:
                print("Invalid value")
        elif choice == "25":
            try:
                print("\nSafety Parameters:")
                print("  Dew Point Safety: margin above dew point (prevents condensation)")
                print("  Hot Side Safety: margin below max hot temp (thermal protection)")
                dew = float(input("Enter Dew Point Safety margin (°C, e.g. 5): "))
                hot = float(input("Enter Hot Side Safety margin (°C, e.g. 10): "))
                client.set_safety_parameters(dew, hot)
            except ValueError:
                print("Invalid value")
        elif choice == "26":
            try:
                print("\nMax Temperature Deviation:")
                print("  Maximum allowed difference between hot and cold sides.")
                print("  Exceeding this triggers an error and shuts down the Peltier.")
                print("  Typical Peltier modules can handle 40-60°C differential.")
                dev = float(input("Enter max deviation (°C, e.g. 40): "))
                client.set_max_deviation(dev)
            except ValueError:
                print("Invalid value")
        elif choice == "30":
            client.start_autotune()
        elif choice == "31":
            client.monitor_autotune()
        elif choice == "32":
            client.apply_autotune()
        elif choice == "33":
            client.stop_autotune()
        elif choice == "40":
            confirm = input("Reset policy settings to defaults? (y/n): ").strip().lower()
            if confirm == 'y':
                client.reset_settings(2)  # policies only
        elif choice == "41":
            confirm = input("Reset ALL settings (policies + calibration + hardware)? (y/n): ").strip().lower()
            if confirm == 'y':
                client.reset_settings(7)  # all flags
        elif choice == "0":
            break
        else:
            print("Invalid choice")


# =============================================================================
# BLE Connection Helpers
# =============================================================================

async def find_ble_device(name: str = "Nevermore"):
    """Scan for BLE device"""
    from bleak import BleakScanner
    
    print(f"Scanning for {name}...")
    devices = await BleakScanner.discover(timeout=10.0)
    
    for device in devices:
        if device.name and name in device.name:
            print(f"Found: {device.name} ({device.address})")
            return device.address
    
    print(f"Device '{name}' not found")
    return None


def get_peltier_characteristics(client):
    """Get Peltier service characteristics"""
    chars = {}
    
    for service in client.services:
        if service.uuid.lower() == PELTIER_SERVICE_UUID.lower():
            char_list = list(service.characteristics)
            if len(char_list) >= 23:
                chars["temp_cold"] = char_list[0]
                chars["temp_hot"] = char_list[1]
                chars["power"] = char_list[2]
                chars["error"] = char_list[3]
                chars["target_temp"] = char_list[4]
                chars["enable"] = char_list[5]
                chars["kp"] = char_list[13]
                chars["ki"] = char_list[14]
                chars["kd"] = char_list[15]
                chars["dew_point_safety"] = char_list[17]
                chars["hot_side_safety"] = char_list[18]
                chars["control_mode"] = char_list[19]
                chars["autotune"] = char_list[22]
            break
    
    return chars


async def run_ble(device_name: str):
    """Run with BLE connection"""
    from bleak import BleakClient as BleakClientClass
    
    address = await find_ble_device(device_name)
    if not address:
        sys.exit(1)
    
    print(f"Connecting to {address}...")
    async with BleakClientClass(address, timeout=60.0) as bleak_client:
        if bleak_client.is_connected:
            print("Connected!")
            chars = get_peltier_characteristics(bleak_client)
            client = BLEClient(bleak_client, chars)
            interactive_menu(client)
        else:
            print("Failed to connect")


# =============================================================================
# Main
# =============================================================================

def main():
    parser = argparse.ArgumentParser(
        description="Nevermore Controller Peltier Test Tool",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  Serial:  python test_peltier.py --serial /dev/ttyACM1
  BLE:     python test_peltier.py --ble
  BLE:     python test_peltier.py --ble "Nevermore"
        """
    )
    parser.add_argument("--serial", "-s", metavar="PORT",
                        help="Serial port (e.g., /dev/ttyACM1, COM5)")
    parser.add_argument("--ble", "-b", nargs="?", const="Nevermore", metavar="NAME",
                        help="BLE device name (default: Nevermore)")
    
    args = parser.parse_args()
    
    if not args.serial and not args.ble:
        parser.print_help()
        print("\nError: Specify either --serial or --ble")
        sys.exit(1)
    
    if args.serial:
        try:
            client = SerialClient(args.serial)
            interactive_menu(client)
            client.close()
        except Exception as e:
            print(f"Serial error: {e}")
            sys.exit(1)
    
    elif args.ble:
        import asyncio
        try:
            asyncio.run(run_ble(args.ble))
        except KeyboardInterrupt:
            print("\nExiting...")


if __name__ == "__main__":
    main()
