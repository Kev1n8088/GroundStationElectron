#!/usr/bin/env python3
"""
LFS Telemetry Parser and Viewer
Based on the JavaScript serial-comm.js implementation
Parses binary telemetry files and provides a GUI to step through timestamps
"""

import struct
import tkinter as tk
from tkinter import ttk, filedialog, messagebox
import json
import math
from typing import Dict, List, Optional, Tuple, Any
from dataclasses import dataclass
from datetime import datetime
import matplotlib.pyplot as plt
from matplotlib.backends.backend_tkagg import FigureCanvasTkAgg
import mplcursors
import numpy as np

# Protocol Constants (from JavaScript)
PACKET_HEADER = 0xAA
CRC_POLYNOMIAL = 0xEDB88320
MAX_PACKET_SIZE = 255
HEADER_SIZE = 3
CHECKSUM_SIZE = 4

# Message Types
MESSAGE_TYPES = {
    51: "RTCM_FRAGMENT_1",
    52: "RTCM_FRAGMENT_2", 
    53: "RTCM_FRAGMENT_3",
    54: "RTCM_FRAGMENT_4",
    55: "RTCM_FINAL",
    155: "STATE_TELEMETRY",
    156: "SENSORS",
    157: "GPS",
    158: "LANDER",
    159: "KALMAN"
}

# Vehicle States
VEHICLE_STATES = {
    0: 'DISARMED',
    1: 'ARMED', 
    2: 'ALIGN CALC',
    3: 'ASC GUIDE',
    4: 'ASC STAB',
    5: 'DES COAST',
    6: 'DES GUIDE',
    7: 'LANDED',
    64: 'WHEEL TEST',
    65: 'STAB TEST',
    66: 'POS TEST',
    67: 'TRAJ TEST',
    68: 'ALIGN TEST',
    69: 'TEST PREP',
    70: 'PYRO TEST'
}

# Sensor Fail Bits
SENSOR_FAIL_BITS = {
    'BAROMETER': 0x01,
    'SCH1': 0x02,
    'ICM': 0x04,
    'MAGNETOMETER': 0x08,
    'GPS': 0x10
}

@dataclass
class ParsedPacket:
    valid: bool
    message_type: int
    data: bytes
    error: Optional[str] = None
    timestamp: Optional[float] = None

@dataclass
class EulerAngles:
    roll: float
    pitch: float
    yaw: float

class LFSTelemetryParser:
    def __init__(self):
        self.packets: List[ParsedPacket] = []
        self.data_version = None  # Will be set to "1.0" or "1.1"
        
    @staticmethod
    def quaternion_to_euler(q_w: float, q_x: float, q_y: float, q_z: float) -> EulerAngles:
        """Convert quaternion to Euler angles using the provided formula"""
        # Roll (x-axis rotation)
        sinr_cosp = 2 * (q_w * q_x + q_y * q_z)
        cosr_cosp = 1 - 2 * (q_x * q_x + q_y * q_y)
        roll = math.atan2(sinr_cosp, cosr_cosp)
        
        # Pitch (y-axis rotation)
        sinp = 2 * (q_w * q_y - q_z * q_x)
        if abs(sinp) >= 1:
            pitch = math.copysign(math.pi / 2, sinp)  # Use 90 degrees if out of range
        else:
            pitch = math.asin(sinp)
        
        # Yaw (z-axis rotation)
        siny_cosp = 2 * (q_w * q_z + q_x * q_y)
        cosy_cosp = 1 - 2 * (q_y * q_y + q_z * q_z)
        yaw = math.atan2(siny_cosp, cosy_cosp)
        
        return EulerAngles(roll=roll, pitch=pitch, yaw=yaw)
        
    def detect_data_version(self, data: bytes) -> str:
        """Detect data version based on packet sizes and types"""
        # Find first few valid packets and analyze their sizes
        offset = 0
        packet_sizes = []
        message_types = []
        
        # Check first 10 packets or so
        for _ in range(min(10, len(data) // 50)):  # Rough estimate
            header_pos = data.find(PACKET_HEADER, offset)
            if header_pos == -1:
                break
                
            if header_pos + HEADER_SIZE > len(data):
                break
                
            payload_length = data[header_pos + 1]
            packet_length = HEADER_SIZE + payload_length
            
            if header_pos + packet_length > len(data):
                break
                
            # Try to parse this packet to get message type
            packet_data = data[header_pos:header_pos + packet_length]
            parsed = self.unpack_lfs_packet(packet_data)
            
            if parsed.valid and len(parsed.data) >= 1:
                packet_sizes.append(payload_length)
                message_types.append(parsed.message_type)
            
            offset = header_pos + packet_length
        
        if not packet_sizes:
            return "1.0"  # Default to old version
        
        # v1.1 characteristics:
        # - STATE_TELEMETRY (155) has payload ~181 bytes
        # - GPS (157) has payload ~162 bytes 
        # - No SENSORS (156), LANDER (158), KALMAN (159) packets
        
        # v1.0 characteristics:
        # - STATE_TELEMETRY (155) has payload ~65 bytes
        # - Multiple packet types including 156, 158, 159
        
        has_large_state_packets = any(
            msg == 155 and size > 150 
            for msg, size in zip(message_types, packet_sizes)
        )
        
        has_old_packet_types = any(
            msg in [156, 158, 159] 
            for msg in message_types
        )
        
        if has_large_state_packets and not has_old_packet_types:
            return "1.1"
        else:
            return "1.0"
    
    def parse_state_telemetry_v11(self, data: bytes) -> Dict[str, Any]:
        """Parse state telemetry data for v1.1 (combined packet)"""
        if len(data) < 181:
            raise ValueError(f'State telemetry v1.1 data too short: {len(data)} bytes, expected 181')
        
        offset = 0
        
        # State telemetry portion (matches old format)
        vehicle_state = struct.unpack_from('<b', data, offset)[0]; offset += 1
        quat_w = struct.unpack_from('<f', data, offset)[0]; offset += 4
        quat_x = struct.unpack_from('<f', data, offset)[0]; offset += 4
        quat_y = struct.unpack_from('<f', data, offset)[0]; offset += 4
        quat_z = struct.unpack_from('<f', data, offset)[0]; offset += 4
        accel_x = struct.unpack_from('<f', data, offset)[0]; offset += 4
        accel_y = struct.unpack_from('<f', data, offset)[0]; offset += 4
        accel_z = struct.unpack_from('<f', data, offset)[0]; offset += 4
        vel_x = struct.unpack_from('<f', data, offset)[0]; offset += 4
        vel_y = struct.unpack_from('<f', data, offset)[0]; offset += 4
        vel_z = struct.unpack_from('<f', data, offset)[0]; offset += 4
        pos_x = struct.unpack_from('<f', data, offset)[0]; offset += 4
        pos_y = struct.unpack_from('<f', data, offset)[0]; offset += 4
        pos_z = struct.unpack_from('<f', data, offset)[0]; offset += 4
        time_since_launch = struct.unpack_from('<f', data, offset)[0]; offset += 4
        
        # Sensor data portion (previously separate SENSORS packet)
        failmask = struct.unpack_from('<b', data, offset)[0]; offset += 1
        sd_good = struct.unpack_from('<B', data, offset)[0] != 0; offset += 1
        gyro_yaw = struct.unpack_from('<f', data, offset)[0]; offset += 4
        gyro_pitch = struct.unpack_from('<f', data, offset)[0]; offset += 4
        gyro_roll = struct.unpack_from('<f', data, offset)[0]; offset += 4
        accelerometer_x = struct.unpack_from('<f', data, offset)[0]; offset += 4
        accelerometer_y = struct.unpack_from('<f', data, offset)[0]; offset += 4
        accelerometer_z = struct.unpack_from('<f', data, offset)[0]; offset += 4
        baro_altitude = struct.unpack_from('<f', data, offset)[0]; offset += 4
        gyro_bias_yaw = struct.unpack_from('<f', data, offset)[0]; offset += 4
        gyro_bias_pitch = struct.unpack_from('<f', data, offset)[0]; offset += 4
        gyro_bias_roll = struct.unpack_from('<f', data, offset)[0]; offset += 4
        
        # Kalman uncertainty data (previously separate KALMAN packet)
        acc_unc_x = struct.unpack_from('<f', data, offset)[0]; offset += 4
        acc_unc_y = struct.unpack_from('<f', data, offset)[0]; offset += 4
        acc_unc_z = struct.unpack_from('<f', data, offset)[0]; offset += 4
        vel_unc_x = struct.unpack_from('<f', data, offset)[0]; offset += 4
        vel_unc_y = struct.unpack_from('<f', data, offset)[0]; offset += 4
        vel_unc_z = struct.unpack_from('<f', data, offset)[0]; offset += 4
        pos_unc_x = struct.unpack_from('<f', data, offset)[0]; offset += 4
        pos_unc_y = struct.unpack_from('<f', data, offset)[0]; offset += 4
        pos_unc_z = struct.unpack_from('<f', data, offset)[0]; offset += 4
        
        # Kalman measurement data
        accel_meas_x = struct.unpack_from('<f', data, offset)[0]; offset += 4
        accel_meas_y = struct.unpack_from('<f', data, offset)[0]; offset += 4
        accel_meas_z = struct.unpack_from('<f', data, offset)[0]; offset += 4
        vel_meas_x = struct.unpack_from('<f', data, offset)[0]; offset += 4
        vel_meas_y = struct.unpack_from('<f', data, offset)[0]; offset += 4
        vel_meas_z = struct.unpack_from('<f', data, offset)[0]; offset += 4
        pos_meas_x = struct.unpack_from('<f', data, offset)[0]; offset += 4
        pos_meas_y = struct.unpack_from('<f', data, offset)[0]; offset += 4
        pos_meas_z = struct.unpack_from('<f', data, offset)[0]; offset += 4
        
        vehicle_ms = struct.unpack_from('<I', data, offset)[0]; offset += 4
        down_count = struct.unpack_from('<I', data, offset)[0]
        
        # Parse sensor failures
        sensor_status = {}
        for name, bit in SENSOR_FAIL_BITS.items():
            sensor_status[name] = not bool(failmask & bit)
        
        return {
            'type': 'STATE_TELEMETRY_V11',
            'vehicleState': vehicle_state,
            'vehicleStateName': VEHICLE_STATES.get(vehicle_state, f'UNKNOWN({vehicle_state})'),
            'quaternion': {'w': quat_w, 'x': quat_x, 'y': quat_y, 'z': quat_z},
            'eulerAngles': self.quaternion_to_euler(quat_w, quat_x, quat_y, quat_z),
            'acceleration': {'x': accel_x, 'y': accel_y, 'z': accel_z},
            'velocity': {'x': vel_x, 'y': vel_y, 'z': vel_z},
            'position': {'x': pos_x, 'y': pos_y, 'z': pos_z},
            'timeSinceLaunch': time_since_launch,
            
            # Sensor data
            'failmask': failmask,
            'sensorStatus': sensor_status,
            'sdGood': sd_good,
            'gyro': {'yaw': gyro_yaw, 'pitch': gyro_pitch, 'roll': gyro_roll},
            'accelerometer': {'x': accelerometer_x, 'y': accelerometer_y, 'z': accelerometer_z},
            'baroAltitude': baro_altitude,
            'gyroBias': {'yaw': gyro_bias_yaw, 'pitch': gyro_bias_pitch, 'roll': gyro_bias_roll},
            
            # Kalman data
            'uncertainty': {
                'acceleration': {'x': acc_unc_x, 'y': acc_unc_y, 'z': acc_unc_z},
                'velocity': {'x': vel_unc_x, 'y': vel_unc_y, 'z': vel_unc_z},
                'position': {'x': pos_unc_x, 'y': pos_unc_y, 'z': pos_unc_z}
            },
            'measurements': {
                'acceleration': {'x': accel_meas_x, 'y': accel_meas_y, 'z': accel_meas_z},
                'velocity': {'x': vel_meas_x, 'y': vel_meas_y, 'z': vel_meas_z},
                'position': {'x': pos_meas_x, 'y': pos_meas_y, 'z': pos_meas_z}
            },
            
            'vehicleMs': vehicle_ms,
            'downCount': down_count
        }
    
    def parse_gps_data_v11(self, data: bytes) -> Dict[str, Any]:
        """Parse GPS data for v1.1 (combined packet with lander data)"""
        if len(data) < 162:
            raise ValueError(f'GPS v1.1 data too short: {len(data)} bytes, expected 162')
        
        offset = 0
        
        # GPS data portion
        sats_in_view = struct.unpack_from('<B', data, offset)[0]; offset += 1
        sats_used = struct.unpack_from('<B', data, offset)[0]; offset += 1
        gps_quality = struct.unpack_from('<B', data, offset)[0]; offset += 1
        current_lat = struct.unpack_from('<f', data, offset)[0]; offset += 4
        current_lon = struct.unpack_from('<f', data, offset)[0]; offset += 4
        current_alt = struct.unpack_from('<f', data, offset)[0]; offset += 4
        accuracy_2d = struct.unpack_from('<f', data, offset)[0]; offset += 4
        accuracy_3d = struct.unpack_from('<f', data, offset)[0]; offset += 4
        pdop = struct.unpack_from('<f', data, offset)[0]; offset += 4
        gps_ms = struct.unpack_from('<I', data, offset)[0]; offset += 4
        last_rtcm = struct.unpack_from('<I', data, offset)[0]; offset += 4
        home_lat = struct.unpack_from('<f', data, offset)[0]; offset += 4
        home_lon = struct.unpack_from('<f', data, offset)[0]; offset += 4
        home_alt = struct.unpack_from('<f', data, offset)[0]; offset += 4
        down_vel = struct.unpack_from('<f', data, offset)[0]; offset += 4
        east_vel = struct.unpack_from('<f', data, offset)[0]; offset += 4
        north_vel = struct.unpack_from('<f', data, offset)[0]; offset += 4
        rel_x = struct.unpack_from('<f', data, offset)[0]; offset += 4
        rel_y = struct.unpack_from('<f', data, offset)[0]; offset += 4
        rel_z = struct.unpack_from('<f', data, offset)[0]; offset += 4
        
        # Lander data portion (previously separate LANDER packet)
        y_target = struct.unpack_from('<f', data, offset)[0]; offset += 4
        z_target = struct.unpack_from('<f', data, offset)[0]; offset += 4
        ignition_alt = struct.unpack_from('<f', data, offset)[0]; offset += 4
        apogee_alt = struct.unpack_from('<f', data, offset)[0]; offset += 4
        yaw_setpoint = struct.unpack_from('<f', data, offset)[0]; offset += 4
        pitch_setpoint = struct.unpack_from('<f', data, offset)[0]; offset += 4
        yaw_command = struct.unpack_from('<f', data, offset)[0]; offset += 4
        pitch_command = struct.unpack_from('<f', data, offset)[0]; offset += 4
        roll_mixed_yaw = struct.unpack_from('<f', data, offset)[0]; offset += 4
        roll_mixed_pitch = struct.unpack_from('<f', data, offset)[0]; offset += 4
        yaw_misalign = struct.unpack_from('<f', data, offset)[0]; offset += 4
        pitch_misalign = struct.unpack_from('<f', data, offset)[0]; offset += 4
        roll_command = struct.unpack_from('<f', data, offset)[0]; offset += 4
        y_projected = struct.unpack_from('<f', data, offset)[0]; offset += 4
        z_projected = struct.unpack_from('<f', data, offset)[0]; offset += 4
        vbat = struct.unpack_from('<f', data, offset)[0]; offset += 4
        thrust = struct.unpack_from('<f', data, offset)[0]; offset += 4
        mass = struct.unpack_from('<f', data, offset)[0]; offset += 4
        mmoi = struct.unpack_from('<f', data, offset)[0]; offset += 4
        moment_arm = struct.unpack_from('<f', data, offset)[0]; offset += 4
        pyro_status = struct.unpack_from('<B', data, offset)[0]; offset += 1
        vehicle_ms = struct.unpack_from('<I', data, offset)[0]; offset += 4
        down_count = struct.unpack_from('<I', data, offset)[0]
        
        # Parse pyro status bits
        chute_good = bool(pyro_status & (1 << 0))
        pyro_good = bool(pyro_status & (1 << 1))
        
        return {
            'type': 'GPS_V11',
            
            # GPS data
            'satellites': {'inView': sats_in_view, 'used': sats_used},
            'quality': gps_quality,
            'position': {'latitude': current_lat, 'longitude': current_lon, 'altitude': current_alt},
            'homePosition': {'latitude': home_lat, 'longitude': home_lon, 'altitude': home_alt},
            'velocity': {'down': down_vel, 'east': east_vel, 'north': north_vel},
            'accuracy': {'2D': accuracy_2d, '3D': accuracy_3d},
            'pdop': pdop,
            'lastRTCM': last_rtcm,
            'relativePosition': {'x': rel_x, 'y': rel_y, 'z': rel_z},
            
            # Lander data  
            'target': {'Y': y_target, 'Z': z_target},
            'ignitionAltitude': ignition_alt,
            'apogeeAltitude': apogee_alt,
            'setpoints': {'yaw': yaw_setpoint, 'pitch': pitch_setpoint},
            'commands': {'yaw': yaw_command, 'pitch': pitch_command, 'roll': roll_command},
            'rollMixed': {'yaw': roll_mixed_yaw, 'pitch': roll_mixed_pitch},
            'misaligns': {'yaw': yaw_misalign, 'pitch': pitch_misalign},
            'projected': {'Y': y_projected, 'Z': z_projected},
            'batteryVoltage': vbat,
            'thrust': thrust,
            'mass': mass,
            'momentOfInertia': mmoi,
            'momentArm': moment_arm,
            'pyroStatus': {
                'raw': pyro_status,
                'chute': chute_good,
                'pyro': pyro_good
            },
            
            'vehicleMs': vehicle_ms,
            'downCount': down_count
        }
        
    def calculate_crc32(self, data: bytes) -> int:
        """Calculate CRC32 checksum (matches JavaScript implementation)"""
        crc = 0xFFFFFFFF
        
        for byte in data:
            crc ^= byte
            for _ in range(8):
                if crc & 1:
                    crc = (crc >> 1) ^ CRC_POLYNOMIAL
                else:
                    crc >>= 1
        
        return (~crc) & 0xFFFFFFFF
    
    def apply_cobs_decoding(self, data: bytearray, first_cobs_offset: int):
        """Apply COBS decoding (matches JavaScript implementation)"""
        cobs_pos = first_cobs_offset
        
        while cobs_pos < len(data):
            offset = data[cobs_pos]
            data[cobs_pos] = PACKET_HEADER  # Restore original 0xAA
            
            if offset == 0:
                break
                
            cobs_pos += offset
    
    def unpack_lfs_packet(self, packet: bytes) -> ParsedPacket:
        """Unpack LFS packet (matches JavaScript implementation)"""
        result = ParsedPacket(valid=False, message_type=0, data=b'')
        
        if len(packet) < HEADER_SIZE + CHECKSUM_SIZE:
            result.error = 'Packet too short'
            return result
        
        if packet[0] != PACKET_HEADER:
            result.error = 'Invalid packet header'
            return result
        
        payload_length = packet[1]
        cobs_offset = packet[2]
        
        if len(packet) != HEADER_SIZE + payload_length:
            result.error = 'Packet size mismatch'
            return result
        
        if payload_length < CHECKSUM_SIZE + 2:
            result.error = 'Payload too short'
            return result
        
        # Extract payload and checksum section
        payload_and_checksum = bytearray(packet[HEADER_SIZE:])
        
        if cobs_offset > 0:
            self.apply_cobs_decoding(payload_and_checksum, cobs_offset)
        
        # Split payload and checksum
        actual_payload_size = payload_length - CHECKSUM_SIZE
        payload = bytes(payload_and_checksum[:actual_payload_size])
        received_crc = struct.unpack('<I', payload_and_checksum[actual_payload_size:actual_payload_size + 4])[0]
        
        # Verify checksum
        calculated_crc = self.calculate_crc32(payload)
        if calculated_crc != received_crc:
            result.error = f'CRC mismatch: calculated={calculated_crc:08x}, received={received_crc:08x}'
            return result
        
        if actual_payload_size < 2:
            result.error = 'Payload too short for message type'
            return result
        
        result.valid = True
        result.message_type = payload[1]
        result.data = payload[2:]  # Skip addressing and message type
        
        return result
    
    def parse_state_telemetry(self, data: bytes) -> Dict[str, Any]:
        """Parse state telemetry data"""
        if len(data) < 65:
            raise ValueError('State telemetry data too short')
        
        offset = 0
        vehicle_state = struct.unpack_from('<b', data, offset)[0]; offset += 1
        quat_w = struct.unpack_from('<f', data, offset)[0]; offset += 4
        quat_x = struct.unpack_from('<f', data, offset)[0]; offset += 4
        quat_y = struct.unpack_from('<f', data, offset)[0]; offset += 4
        quat_z = struct.unpack_from('<f', data, offset)[0]; offset += 4
        accel_x = struct.unpack_from('<f', data, offset)[0]; offset += 4
        accel_y = struct.unpack_from('<f', data, offset)[0]; offset += 4
        accel_z = struct.unpack_from('<f', data, offset)[0]; offset += 4
        vel_x = struct.unpack_from('<f', data, offset)[0]; offset += 4
        vel_y = struct.unpack_from('<f', data, offset)[0]; offset += 4
        vel_z = struct.unpack_from('<f', data, offset)[0]; offset += 4
        pos_x = struct.unpack_from('<f', data, offset)[0]; offset += 4
        pos_y = struct.unpack_from('<f', data, offset)[0]; offset += 4
        pos_z = struct.unpack_from('<f', data, offset)[0]; offset += 4
        time_since_launch = struct.unpack_from('<f', data, offset)[0]; offset += 4
        vehicle_ms = struct.unpack_from('<I', data, offset)[0]; offset += 4
        down_count = struct.unpack_from('<I', data, offset)[0]
        
        return {
            'type': 'STATE_TELEMETRY',
            'vehicleState': vehicle_state,
            'vehicleStateName': VEHICLE_STATES.get(vehicle_state, f'UNKNOWN({vehicle_state})'),
            'quaternion': {'w': quat_w, 'x': quat_x, 'y': quat_y, 'z': quat_z},
            'eulerAngles': self.quaternion_to_euler(quat_w, quat_x, quat_y, quat_z),
            'acceleration': {'x': accel_x, 'y': accel_y, 'z': accel_z},
            'velocity': {'x': vel_x, 'y': vel_y, 'z': vel_z},
            'position': {'x': pos_x, 'y': pos_y, 'z': pos_z},
            'timeSinceLaunch': time_since_launch,
            'vehicleMs': vehicle_ms,
            'downCount': down_count
        }
    
    def parse_sensor_data(self, data: bytes) -> Dict[str, Any]:
        """Parse sensor data"""
        if len(data) < 50:
            raise ValueError('Sensor data too short')
        
        offset = 0
        failmask = struct.unpack_from('<b', data, offset)[0]; offset += 1
        sd_good = struct.unpack_from('<B', data, offset)[0] != 0; offset += 1
        gyro_yaw = struct.unpack_from('<f', data, offset)[0]; offset += 4
        gyro_pitch = struct.unpack_from('<f', data, offset)[0]; offset += 4
        gyro_roll = struct.unpack_from('<f', data, offset)[0]; offset += 4
        accel_x = struct.unpack_from('<f', data, offset)[0]; offset += 4
        accel_y = struct.unpack_from('<f', data, offset)[0]; offset += 4
        accel_z = struct.unpack_from('<f', data, offset)[0]; offset += 4
        baro_altitude = struct.unpack_from('<f', data, offset)[0]; offset += 4
        gyro_bias_yaw = struct.unpack_from('<f', data, offset)[0]; offset += 4
        gyro_bias_pitch = struct.unpack_from('<f', data, offset)[0]; offset += 4
        gyro_bias_roll = struct.unpack_from('<f', data, offset)[0]; offset += 4
        vehicle_ms = struct.unpack_from('<I', data, offset)[0]; offset += 4
        down_count = struct.unpack_from('<I', data, offset)[0]
        
        # Parse sensor failures
        sensor_status = {}
        for name, bit in SENSOR_FAIL_BITS.items():
            sensor_status[name] = not bool(failmask & bit)
        
        return {
            'type': 'SENSORS',
            'failmask': failmask,
            'sensorStatus': sensor_status,
            'sdGood': sd_good,
            'gyro': {'yaw': gyro_yaw, 'pitch': gyro_pitch, 'roll': gyro_roll},
            'accelerometer': {'x': accel_x, 'y': accel_y, 'z': accel_z},
            'baroAltitude': baro_altitude,
            'gyroBias': {'yaw': gyro_bias_yaw, 'pitch': gyro_bias_pitch, 'roll': gyro_bias_roll},
            'vehicleMs': vehicle_ms,
            'downCount': down_count
        }
    
    def parse_gps_data(self, data: bytes) -> Dict[str, Any]:
        """Parse GPS data"""
        if len(data) < 79:
            raise ValueError('GPS data too short')
        
        offset = 0
        sats_in_view = struct.unpack_from('<B', data, offset)[0]; offset += 1
        sats_used = struct.unpack_from('<B', data, offset)[0]; offset += 1
        gps_quality = struct.unpack_from('<B', data, offset)[0]; offset += 1
        current_lat = struct.unpack_from('<f', data, offset)[0]; offset += 4
        current_lon = struct.unpack_from('<f', data, offset)[0]; offset += 4
        current_alt = struct.unpack_from('<f', data, offset)[0]; offset += 4
        accuracy_2d = struct.unpack_from('<f', data, offset)[0]; offset += 4
        accuracy_3d = struct.unpack_from('<f', data, offset)[0]; offset += 4
        pdop = struct.unpack_from('<f', data, offset)[0]; offset += 4
        gps_ms = struct.unpack_from('<I', data, offset)[0]; offset += 4
        last_rtcm = struct.unpack_from('<I', data, offset)[0]; offset += 4
        home_lat = struct.unpack_from('<f', data, offset)[0]; offset += 4
        home_lon = struct.unpack_from('<f', data, offset)[0]; offset += 4
        home_alt = struct.unpack_from('<f', data, offset)[0]; offset += 4
        down_vel = struct.unpack_from('<f', data, offset)[0]; offset += 4
        east_vel = struct.unpack_from('<f', data, offset)[0]; offset += 4
        north_vel = struct.unpack_from('<f', data, offset)[0]; offset += 4
        rel_x = struct.unpack_from('<f', data, offset)[0]; offset += 4
        rel_y = struct.unpack_from('<f', data, offset)[0]; offset += 4
        rel_z = struct.unpack_from('<f', data, offset)[0]; offset += 4
        vehicle_ms = struct.unpack_from('<I', data, offset)[0]; offset += 4
        down_count = struct.unpack_from('<I', data, offset)[0]
        
        return {
            'type': 'GPS',
            'satellites': {'inView': sats_in_view, 'used': sats_used},
            'quality': gps_quality,
            'position': {'latitude': current_lat, 'longitude': current_lon, 'altitude': current_alt},
            'homePosition': {'latitude': home_lat, 'longitude': home_lon, 'altitude': home_alt},
            'velocity': {'down': down_vel, 'east': east_vel, 'north': north_vel},
            'accuracy': {'2D': accuracy_2d, '3D': accuracy_3d},
            'pdop': pdop,
            'lastRTCM': last_rtcm,
            'relativePosition': {'x': rel_x, 'y': rel_y, 'z': rel_z},
            'vehicleMs': vehicle_ms,
            'downCount': down_count
        }
    
    def parse_lander_data(self, data: bytes) -> Dict[str, Any]:
        """Parse lander data"""
        if len(data) < 89:
            raise ValueError('Lander data too short')
        
        offset = 0
        y_target = struct.unpack_from('<f', data, offset)[0]; offset += 4
        z_target = struct.unpack_from('<f', data, offset)[0]; offset += 4
        ignition_alt = struct.unpack_from('<f', data, offset)[0]; offset += 4
        apogee_alt = struct.unpack_from('<f', data, offset)[0]; offset += 4
        yaw_setpoint = struct.unpack_from('<f', data, offset)[0]; offset += 4
        pitch_setpoint = struct.unpack_from('<f', data, offset)[0]; offset += 4
        yaw_command = struct.unpack_from('<f', data, offset)[0]; offset += 4
        pitch_command = struct.unpack_from('<f', data, offset)[0]; offset += 4
        roll_mixed_yaw = struct.unpack_from('<f', data, offset)[0]; offset += 4
        roll_mixed_pitch = struct.unpack_from('<f', data, offset)[0]; offset += 4
        yaw_misalign = struct.unpack_from('<f', data, offset)[0]; offset += 4
        pitch_misalign = struct.unpack_from('<f', data, offset)[0]; offset += 4
        roll_command = struct.unpack_from('<f', data, offset)[0]; offset += 4
        y_projected = struct.unpack_from('<f', data, offset)[0]; offset += 4
        z_projected = struct.unpack_from('<f', data, offset)[0]; offset += 4
        vbat = struct.unpack_from('<f', data, offset)[0]; offset += 4
        thrust = struct.unpack_from('<f', data, offset)[0]; offset += 4
        mass = struct.unpack_from('<f', data, offset)[0]; offset += 4
        mmoi = struct.unpack_from('<f', data, offset)[0]; offset += 4
        moment_arm = struct.unpack_from('<f', data, offset)[0]; offset += 4
        pyro_status = struct.unpack_from('<B', data, offset)[0]; offset += 1
        vehicle_ms = struct.unpack_from('<I', data, offset)[0]; offset += 4
        down_count = struct.unpack_from('<I', data, offset)[0]
        
        # Parse pyro status bits
        chute_good = bool(pyro_status & (1 << 0))
        pyro_good = bool(pyro_status & (1 << 1))
        
        return {
            'type': 'LANDER',
            'target': {'Y': y_target, 'Z': z_target},
            'ignitionAltitude': ignition_alt,
            'apogeeAltitude': apogee_alt,
            'setpoints': {'yaw': yaw_setpoint, 'pitch': pitch_setpoint},
            'commands': {'yaw': yaw_command, 'pitch': pitch_command, 'roll': roll_command},
            'rollMixed': {'yaw': roll_mixed_yaw, 'pitch': roll_mixed_pitch},
            'misaligns': {'yaw': yaw_misalign, 'pitch': pitch_misalign},
            'projected': {'Y': y_projected, 'Z': z_projected},
            'batteryVoltage': vbat,
            'thrust': thrust,
            'mass': mass,
            'momentOfInertia': mmoi,
            'momentArm': moment_arm,
            'pyroStatus': {
                'raw': pyro_status,
                'chute': chute_good,
                'pyro': pyro_good
            },
            'vehicleMs': vehicle_ms,
            'downCount': down_count
        }
    
    def parse_kalman_data(self, data: bytes) -> Dict[str, Any]:
        """Parse Kalman filter data"""
        if len(data) < 80:
            raise ValueError('Kalman data too short')
        
        offset = 0
        acc_unc_x = struct.unpack_from('<f', data, offset)[0]; offset += 4
        acc_unc_y = struct.unpack_from('<f', data, offset)[0]; offset += 4
        acc_unc_z = struct.unpack_from('<f', data, offset)[0]; offset += 4
        vel_unc_x = struct.unpack_from('<f', data, offset)[0]; offset += 4
        vel_unc_y = struct.unpack_from('<f', data, offset)[0]; offset += 4
        vel_unc_z = struct.unpack_from('<f', data, offset)[0]; offset += 4
        pos_unc_x = struct.unpack_from('<f', data, offset)[0]; offset += 4
        pos_unc_y = struct.unpack_from('<f', data, offset)[0]; offset += 4
        pos_unc_z = struct.unpack_from('<f', data, offset)[0]; offset += 4
        acc_meas_x = struct.unpack_from('<f', data, offset)[0]; offset += 4
        acc_meas_y = struct.unpack_from('<f', data, offset)[0]; offset += 4
        acc_meas_z = struct.unpack_from('<f', data, offset)[0]; offset += 4
        vel_meas_x = struct.unpack_from('<f', data, offset)[0]; offset += 4
        vel_meas_y = struct.unpack_from('<f', data, offset)[0]; offset += 4
        vel_meas_z = struct.unpack_from('<f', data, offset)[0]; offset += 4
        pos_meas_x = struct.unpack_from('<f', data, offset)[0]; offset += 4
        pos_meas_y = struct.unpack_from('<f', data, offset)[0]; offset += 4
        pos_meas_z = struct.unpack_from('<f', data, offset)[0]; offset += 4
        vehicle_ms = struct.unpack_from('<I', data, offset)[0]; offset += 4
        down_count = struct.unpack_from('<I', data, offset)[0]
        
        return {
            'type': 'KALMAN',
            'uncertainty': {
                'acceleration': {'x': acc_unc_x, 'y': acc_unc_y, 'z': acc_unc_z},
                'velocity': {'x': vel_unc_x, 'y': vel_unc_y, 'z': vel_unc_z},
                'position': {'x': pos_unc_x, 'y': pos_unc_y, 'z': pos_unc_z}
            },
            'measurements': {
                'acceleration': {'x': acc_meas_x, 'y': acc_meas_y, 'z': acc_meas_z},
                'velocity': {'x': vel_meas_x, 'y': vel_meas_y, 'z': vel_meas_z},
                'position': {'x': pos_meas_x, 'y': pos_meas_y, 'z': pos_meas_z}
            },
            'vehicleMs': vehicle_ms,
            'downCount': down_count
        }
    
    def parse_packet_data(self, packet: ParsedPacket) -> Optional[Dict[str, Any]]:
        """Parse packet data based on message type and version"""
        try:
            if self.data_version == "1.1":
                # v1.1 parsing (consolidated packets)
                if packet.message_type == 155:  # STATE_TELEMETRY
                    return self.parse_state_telemetry_v11(packet.data)
                elif packet.message_type == 157:  # GPS (includes lander data)
                    return self.parse_gps_data_v11(packet.data)
                else:
                    return {
                        'type': MESSAGE_TYPES.get(packet.message_type, f'UNKNOWN({packet.message_type})'),
                        'raw_data': packet.data.hex()
                    }
            else:
                # v1.0 parsing (original separate packets)
                if packet.message_type == 155:  # STATE_TELEMETRY
                    return self.parse_state_telemetry(packet.data)
                elif packet.message_type == 156:  # SENSORS
                    return self.parse_sensor_data(packet.data)
                elif packet.message_type == 157:  # GPS
                    return self.parse_gps_data(packet.data)
                elif packet.message_type == 158:  # LANDER
                    return self.parse_lander_data(packet.data)
                elif packet.message_type == 159:  # KALMAN
                    return self.parse_kalman_data(packet.data)
                else:
                    return {
                        'type': MESSAGE_TYPES.get(packet.message_type, f'UNKNOWN({packet.message_type})'),
                        'raw_data': packet.data.hex()
                    }
        except Exception as e:
            return {
                'type': 'PARSE_ERROR',
                'error': str(e),
                'raw_data': packet.data.hex()
            }
    
    def parse_file(self, filename: str) -> List[Dict[str, Any]]:
        """Parse binary telemetry file with version detection"""
        self.packets.clear()
        parsed_data = []
        
        with open(filename, 'rb') as f:
            data = f.read()
        
        # Detect version automatically
        detected_version = self.detect_data_version(data)
        
        # Ask user to confirm or override version
        version_dialog = tk.Toplevel()
        version_dialog.title("Data Version Selection")
        version_dialog.geometry("400x200")
        version_dialog.transient()
        version_dialog.grab_set()
        
        selected_version = tk.StringVar(value=detected_version)
        result = {'version': None}
        
        def on_confirm():
            result['version'] = selected_version.get()
            version_dialog.destroy()
        
        def on_cancel():
            version_dialog.destroy()
        
        # Create dialog content
        ttk.Label(version_dialog, text=f"Auto-detected version: {detected_version}").pack(pady=10)
        ttk.Label(version_dialog, text="Select telemetry data version:").pack(pady=5)
        
        radio_frame = ttk.Frame(version_dialog)
        radio_frame.pack(pady=10)
        
        ttk.Radiobutton(radio_frame, text="v1.0 (5 separate packet types)", 
                       variable=selected_version, value="1.0").pack(anchor='w')
        ttk.Radiobutton(radio_frame, text="v1.1 (2 consolidated packet types)", 
                       variable=selected_version, value="1.1").pack(anchor='w')
        
        button_frame = ttk.Frame(version_dialog)
        button_frame.pack(pady=20)
        
        ttk.Button(button_frame, text="OK", command=on_confirm).pack(side='left', padx=5)
        ttk.Button(button_frame, text="Cancel", command=on_cancel).pack(side='left', padx=5)
        
        # Center the dialog
        version_dialog.update_idletasks()
        x = (version_dialog.winfo_screenwidth() // 2) - (version_dialog.winfo_width() // 2)
        y = (version_dialog.winfo_screenheight() // 2) - (version_dialog.winfo_height() // 2)
        version_dialog.geometry(f"+{x}+{y}")
        
        # Wait for user response
        version_dialog.wait_window()
        
        if result['version'] is None:
            return []  # User cancelled
        
        self.data_version = result['version']
        
        # Find packet boundaries
        offset = 0
        packet_count = 0
        
        while offset < len(data):
            # Find next packet header
            header_pos = data.find(PACKET_HEADER, offset)
            if header_pos == -1:
                break
            
            # Check if we have enough data for a header
            if header_pos + HEADER_SIZE > len(data):
                break
            
            payload_length = data[header_pos + 1]
            packet_length = HEADER_SIZE + payload_length
            
            # Check if we have the complete packet
            if header_pos + packet_length > len(data):
                break
            
            # Extract packet
            packet_data = data[header_pos:header_pos + packet_length]
            
            # Parse packet
            parsed_packet = self.unpack_lfs_packet(packet_data)
            parsed_packet.timestamp = packet_count  # Use packet index as timestamp for now
            
            if parsed_packet.valid:
                packet_data_dict = self.parse_packet_data(parsed_packet)
                if packet_data_dict:
                    packet_data_dict['timestamp'] = packet_count
                    packet_data_dict['packet_index'] = packet_count
                    packet_data_dict['data_version'] = self.data_version
                    parsed_data.append(packet_data_dict)
                    packet_count += 1
            
            offset = header_pos + packet_length
        
        return parsed_data

class TelemetryViewer:
    def __init__(self, root):
        self.root = root
        self.root.title("LFS Telemetry Viewer")
        self.root.geometry("1400x900")
        
        self.parser = LFSTelemetryParser()
        self.telemetry_data = []
        self.current_index = 0
        
        # Plotting window reference
        self.plot_window = None
        
        # Persistent vehicle state - updated as packets arrive
        self.vehicle_state = {
            # State telemetry
            'vehicleState': None,
            'vehicleStateName': 'UNKNOWN',
            'quaternion': {'w': 0.0, 'x': 0.0, 'y': 0.0, 'z': 0.0},
            'eulerAngles': EulerAngles(0.0, 0.0, 0.0),
            'acceleration': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'timeSinceLaunch': 0.0,
            
            # Sensor data
            'failmask': 0,
            'sensorStatus': {},
            'sdGood': False,
            'gyro': {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0},
            'accelerometer': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'baroAltitude': 0.0,
            'gyroBias': {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0},
            
            # GPS data
            'satellites': {'inView': 0, 'used': 0},
            'gpsQuality': 0,
            'gpsPosition': {'latitude': 0.0, 'longitude': 0.0, 'altitude': 0.0},
            'homePosition': {'latitude': 0.0, 'longitude': 0.0, 'altitude': 0.0},
            'gpsVelocity': {'down': 0.0, 'east': 0.0, 'north': 0.0},
            'gpsAccuracy': {'2D': 0.0, '3D': 0.0},
            'pdop': 0.0,
            'lastRTCM': 0,
            'relativePosition': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            
            # Lander data
            'target': {'Y': 0.0, 'Z': 0.0},
            'ignitionAltitude': 0.0,
            'apogeeAltitude': 0.0,
            'setpoints': {'yaw': 0.0, 'pitch': 0.0},
            'commands': {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0},
            'rollMixed': {'yaw': 0.0, 'pitch': 0.0},
            'misaligns': {'yaw': 0.0, 'pitch': 0.0},
            'projected': {'Y': 0.0, 'Z': 0.0},
            'batteryVoltage': 0.0,
            'thrust': 0.0,
            'mass': 0.0,
            'momentOfInertia': 0.0,
            'momentArm': 0.0,
            'pyroStatus': {'raw': 0, 'chute': False, 'pyro': False},
            
            # Kalman data
            'kalmanUncertainty': {
                'acceleration': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'position': {'x': 0.0, 'y': 0.0, 'z': 0.0}
            },
            'kalmanMeasurements': {
                'acceleration': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'position': {'x': 0.0, 'y': 0.0, 'z': 0.0}
            },
            
            # Common fields
            'vehicleMs': 0,
            'downCount': 0,
            
            # Packet tracking
            'lastUpdated': {
                'STATE_TELEMETRY': -1,
                'SENSORS': -1,
                'GPS': -1,
                'LANDER': -1,
                'KALMAN': -1
            }
        }
        
        self.setup_ui()
    
    def setup_ui(self):
        # File selection frame
        file_frame = ttk.Frame(self.root)
        file_frame.pack(fill='x', padx=5, pady=5)
        
        ttk.Button(file_frame, text="Open Telemetry File", command=self.open_file).pack(side='left')
        self.file_label = ttk.Label(file_frame, text="No file selected")
        self.file_label.pack(side='left', padx=(10, 0))
        
        # Navigation frame
        nav_frame = ttk.Frame(self.root)
        nav_frame.pack(fill='x', padx=5, pady=5)
        
        ttk.Button(nav_frame, text="First", command=self.first_packet).pack(side='left')
        ttk.Button(nav_frame, text="Previous", command=self.prev_packet).pack(side='left')
        ttk.Button(nav_frame, text="Next", command=self.next_packet).pack(side='left')
        ttk.Button(nav_frame, text="Last", command=self.last_packet).pack(side='left')
        
        self.index_label = ttk.Label(nav_frame, text="0 / 0")
        self.index_label.pack(side='left', padx=(10, 0))
        
        # Jump to index
        ttk.Label(nav_frame, text="Jump to:").pack(side='left', padx=(20, 0))
        self.jump_var = tk.StringVar()
        self.jump_entry = ttk.Entry(nav_frame, textvariable=self.jump_var, width=10)
        self.jump_entry.pack(side='left', padx=(5, 0))
        ttk.Button(nav_frame, text="Go", command=self.jump_to_index).pack(side='left')
        
        # Add plot button
        ttk.Button(nav_frame, text="Show Plots", command=self.show_plots).pack(side='left', padx=(20, 0))
        
        # Data display frame
        data_frame = ttk.Frame(self.root)
        data_frame.pack(fill='both', expand=True, padx=5, pady=5)
        
        # Create text widget with scrollbar
        text_frame = ttk.Frame(data_frame)
        text_frame.pack(fill='both', expand=True)
        
        self.text_widget = tk.Text(text_frame, wrap='none', font=('Courier', 10))
        scrollbar_v = ttk.Scrollbar(text_frame, orient='vertical', command=self.text_widget.yview)
        scrollbar_h = ttk.Scrollbar(text_frame, orient='horizontal', command=self.text_widget.xview)
        
        self.text_widget.configure(yscrollcommand=scrollbar_v.set, xscrollcommand=scrollbar_h.set)
        
        self.text_widget.grid(row=0, column=0, sticky='nsew')
        scrollbar_v.grid(row=0, column=1, sticky='ns')
        scrollbar_h.grid(row=1, column=0, sticky='ew')
        
        text_frame.grid_rowconfigure(0, weight=1)
        text_frame.grid_columnconfigure(0, weight=1)
        
        # Status bar
        self.status_var = tk.StringVar()
        self.status_var.set("Ready")
        status_bar = ttk.Label(self.root, textvariable=self.status_var, relief='sunken')
        status_bar.pack(fill='x', side='bottom')
        
        # Bind keyboard events
        self.root.bind('<Left>', lambda e: self.prev_packet())
        self.root.bind('<Right>', lambda e: self.next_packet())
        self.root.bind('<Home>', lambda e: self.first_packet())
        self.root.bind('<End>', lambda e: self.last_packet())
        self.root.focus_set()
    
    def open_file(self):
        filename = filedialog.askopenfilename(
            title="Select Telemetry File",
            filetypes=[("All files", "*.*"), ("Binary files", "*.bin"), ("Data files", "*.dat")]
        )
        
        if filename:
            try:
                self.status_var.set("Parsing file...")
                self.root.update()
                
                self.telemetry_data = self.parser.parse_file(filename)
                self.current_index = 0
                
                self.file_label.config(text=f"File: {filename}")
                self.status_var.set(f"Loaded {len(self.telemetry_data)} packets")
                
                self.update_display()
                
            except Exception as e:
                messagebox.showerror("Error", f"Failed to parse file:\n{str(e)}")
                self.status_var.set("Error loading file")
    
    def update_vehicle_state(self, packet: Dict[str, Any], packet_index: int):
        """Update persistent vehicle state with data from current packet"""
        packet_type = packet.get('type', 'UNKNOWN')
        data_version = packet.get('data_version', '1.0')
        
        # Update last updated timestamp for this packet type
        if packet_type in self.vehicle_state['lastUpdated']:
            self.vehicle_state['lastUpdated'][packet_type] = packet_index
        
        # Update state based on packet type and version
        if packet_type in ['STATE_TELEMETRY', 'STATE_TELEMETRY_V11']:
            self.vehicle_state['vehicleState'] = packet['vehicleState']
            self.vehicle_state['vehicleStateName'] = packet['vehicleStateName']
            self.vehicle_state['quaternion'] = packet['quaternion'].copy()
            self.vehicle_state['eulerAngles'] = packet['eulerAngles']
            self.vehicle_state['acceleration'] = packet['acceleration'].copy()
            self.vehicle_state['velocity'] = packet['velocity'].copy()
            self.vehicle_state['position'] = packet['position'].copy()
            self.vehicle_state['timeSinceLaunch'] = packet['timeSinceLaunch']
            
            # For v1.1, sensor and kalman data is in the same packet
            if packet_type == 'STATE_TELEMETRY_V11':
                self.vehicle_state['failmask'] = packet['failmask']
                self.vehicle_state['sensorStatus'] = packet['sensorStatus'].copy()
                self.vehicle_state['sdGood'] = packet['sdGood']
                self.vehicle_state['gyro'] = packet['gyro'].copy()
                self.vehicle_state['accelerometer'] = packet['accelerometer'].copy()
                self.vehicle_state['baroAltitude'] = packet['baroAltitude']
                self.vehicle_state['gyroBias'] = packet['gyroBias'].copy()
                
                self.vehicle_state['kalmanUncertainty'] = packet['uncertainty'].copy()
                self.vehicle_state['kalmanMeasurements'] = packet['measurements'].copy()
            
        elif packet_type == 'SENSORS':
            self.vehicle_state['failmask'] = packet['failmask']
            self.vehicle_state['sensorStatus'] = packet['sensorStatus'].copy()
            self.vehicle_state['sdGood'] = packet['sdGood']
            self.vehicle_state['gyro'] = packet['gyro'].copy()
            self.vehicle_state['accelerometer'] = packet['accelerometer'].copy()
            self.vehicle_state['baroAltitude'] = packet['baroAltitude']
            self.vehicle_state['gyroBias'] = packet['gyroBias'].copy()
            
        elif packet_type in ['GPS', 'GPS_V11']:
            self.vehicle_state['satellites'] = packet['satellites'].copy()
            self.vehicle_state['gpsQuality'] = packet['quality']
            self.vehicle_state['gpsPosition'] = packet['position'].copy()
            self.vehicle_state['homePosition'] = packet['homePosition'].copy()
            self.vehicle_state['gpsVelocity'] = packet['velocity'].copy()
            self.vehicle_state['gpsAccuracy'] = packet['accuracy'].copy()
            self.vehicle_state['pdop'] = packet['pdop']
            self.vehicle_state['lastRTCM'] = packet['lastRTCM']
            self.vehicle_state['relativePosition'] = packet['relativePosition'].copy()
            
            # For v1.1, lander data is in the same packet
            if packet_type == 'GPS_V11':
                self.vehicle_state['target'] = packet['target'].copy()
                self.vehicle_state['ignitionAltitude'] = packet['ignitionAltitude']
                self.vehicle_state['apogeeAltitude'] = packet['apogeeAltitude']
                self.vehicle_state['setpoints'] = packet['setpoints'].copy()
                self.vehicle_state['commands'] = packet['commands'].copy()
                self.vehicle_state['rollMixed'] = packet['rollMixed'].copy()
                self.vehicle_state['misaligns'] = packet['misaligns'].copy()
                self.vehicle_state['projected'] = packet['projected'].copy()
                self.vehicle_state['batteryVoltage'] = packet['batteryVoltage']
                self.vehicle_state['thrust'] = packet['thrust']
                self.vehicle_state['mass'] = packet['mass']
                self.vehicle_state['momentOfInertia'] = packet['momentOfInertia']
                self.vehicle_state['momentArm'] = packet['momentArm']
                self.vehicle_state['pyroStatus'] = packet['pyroStatus'].copy()
            
        elif packet_type == 'LANDER':
            self.vehicle_state['target'] = packet['target'].copy()
            self.vehicle_state['ignitionAltitude'] = packet['ignitionAltitude']
            self.vehicle_state['apogeeAltitude'] = packet['apogeeAltitude']
            self.vehicle_state['setpoints'] = packet['setpoints'].copy()
            self.vehicle_state['commands'] = packet['commands'].copy()
            self.vehicle_state['rollMixed'] = packet['rollMixed'].copy()
            self.vehicle_state['misaligns'] = packet['misaligns'].copy()
            self.vehicle_state['projected'] = packet['projected'].copy()
            self.vehicle_state['batteryVoltage'] = packet['batteryVoltage']
            self.vehicle_state['thrust'] = packet['thrust']
            self.vehicle_state['mass'] = packet['mass']
            self.vehicle_state['momentOfInertia'] = packet['momentOfInertia']
            self.vehicle_state['momentArm'] = packet['momentArm']
            self.vehicle_state['pyroStatus'] = packet['pyroStatus'].copy()
            
        elif packet_type == 'KALMAN':
            self.vehicle_state['kalmanUncertainty'] = packet['uncertainty'].copy()
            self.vehicle_state['kalmanMeasurements'] = packet['measurements'].copy()
        
        # Update common fields if present
        if 'vehicleMs' in packet:
            self.vehicle_state['vehicleMs'] = packet['vehicleMs']
        if 'downCount' in packet:
            self.vehicle_state['downCount'] = packet['downCount']

    def update_display(self):
        """Update the display with current comprehensive vehicle state"""
        if not self.telemetry_data:
            self.text_widget.delete(1.0, tk.END)
            self.index_label.config(text="0 / 0")
            return
        
        # Update vehicle state up to current packet index
        self.reset_vehicle_state()
        for i in range(self.current_index + 1):
            if i < len(self.telemetry_data):
                self.update_vehicle_state(self.telemetry_data[i], i)
        
        if 0 <= self.current_index < len(self.telemetry_data):
            current_packet = self.telemetry_data[self.current_index]
            
            # Format comprehensive vehicle state display
            formatted_text = f"=== COMPREHENSIVE VEHICLE STATE ===\n"
            formatted_text += f"Current Packet Index: {self.current_index}\n"
            formatted_text += f"Current Packet Type: {current_packet.get('type', 'UNKNOWN')}\n"
            formatted_text += f"Total Packets Processed: {self.current_index + 1}\n"
            formatted_text += "=" * 60 + "\n\n"
            
            # Format the comprehensive state
            formatted_text += self.format_comprehensive_state()
            
            self.text_widget.delete(1.0, tk.END)
            self.text_widget.insert(1.0, formatted_text)
        
        self.index_label.config(text=f"{self.current_index + 1} / {len(self.telemetry_data)}")
    
    def reset_vehicle_state(self):
        """Reset vehicle state to initial values"""
        self.vehicle_state = {
            # State telemetry
            'vehicleState': None,
            'vehicleStateName': 'UNKNOWN',
            'quaternion': {'w': 0.0, 'x': 0.0, 'y': 0.0, 'z': 0.0},
            'eulerAngles': EulerAngles(0.0, 0.0, 0.0),
            'acceleration': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'position': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'timeSinceLaunch': 0.0,
            
            # Sensor data
            'failmask': 0,
            'sensorStatus': {},
            'sdGood': False,
            'gyro': {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0},
            'accelerometer': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            'baroAltitude': 0.0,
            'gyroBias': {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0},
            
            # GPS data
            'satellites': {'inView': 0, 'used': 0},
            'gpsQuality': 0,
            'gpsPosition': {'latitude': 0.0, 'longitude': 0.0, 'altitude': 0.0},
            'homePosition': {'latitude': 0.0, 'longitude': 0.0, 'altitude': 0.0},
            'gpsVelocity': {'down': 0.0, 'east': 0.0, 'north': 0.0},
            'gpsAccuracy': {'2D': 0.0, '3D': 0.0},
            'pdop': 0.0,
            'lastRTCM': 0,
            'relativePosition': {'x': 0.0, 'y': 0.0, 'z': 0.0},
            
            # Lander data
            'target': {'Y': 0.0, 'Z': 0.0},
            'ignitionAltitude': 0.0,
            'apogeeAltitude': 0.0,
            'setpoints': {'yaw': 0.0, 'pitch': 0.0},
            'commands': {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0},
            'rollMixed': {'yaw': 0.0, 'pitch': 0.0},
            'misaligns': {'yaw': 0.0, 'pitch': 0.0},
            'projected': {'Y': 0.0, 'Z': 0.0},
            'batteryVoltage': 0.0,
            'thrust': 0.0,
            'mass': 0.0,
            'momentOfInertia': 0.0,
            'momentArm': 0.0,
            'pyroStatus': {'raw': 0, 'chute': False, 'pyro': False},
            
            # Kalman data
            'kalmanUncertainty': {
                'acceleration': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'position': {'x': 0.0, 'y': 0.0, 'z': 0.0}
            },
            'kalmanMeasurements': {
                'acceleration': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'velocity': {'x': 0.0, 'y': 0.0, 'z': 0.0},
                'position': {'x': 0.0, 'y': 0.0, 'z': 0.0}
            },
            
            # Common fields
            'vehicleMs': 0,
            'downCount': 0,
            
            # Packet tracking
            'lastUpdated': {
                'STATE_TELEMETRY': -1,
                'SENSORS': -1,
                'GPS': -1,
                'LANDER': -1,
                'KALMAN': -1
            }
        }
    
    def format_comprehensive_state(self) -> str:
        """Format comprehensive vehicle state for display"""
        formatted = ""
        
        # Header with last update information
        formatted += "LAST PACKET UPDATES:\n"
        for packet_type, last_idx in self.vehicle_state['lastUpdated'].items():
            if last_idx >= 0:
                formatted += f"  {packet_type}: Packet {last_idx}\n"
            else:
                formatted += f"  {packet_type}: No data\n"
        formatted += "\n" + "="*50 + "\n\n"
        
        # Vehicle State & Time
        formatted += "VEHICLE STATE & TIMING:\n"
        formatted += f"  State: {self.vehicle_state['vehicleStateName']}"
        if self.vehicle_state['vehicleState'] is not None:
            formatted += f" ({self.vehicle_state['vehicleState']})"
        formatted += "\n"
        formatted += f"  Time Since Launch: {self.vehicle_state['timeSinceLaunch']:.3f} s\n"
        formatted += f"  Vehicle Time: {self.vehicle_state['vehicleMs']} ms\n"
        formatted += f"  Down Count: {self.vehicle_state['downCount']}\n\n"
        
        # Position & Motion (State Telemetry)
        formatted += "POSITION & MOTION (State Telemetry):\n"
        q = self.vehicle_state['quaternion']
        formatted += f"  Quaternion: w={q['w']:.6f}, x={q['x']:.6f}, y={q['y']:.6f}, z={q['z']:.6f}\n"
        
        # Add Euler angles
        euler = self.vehicle_state['eulerAngles']
        formatted += f"  Euler Angles (rad): Roll={euler.roll:.6f}, Pitch={euler.pitch:.6f}, Yaw={euler.yaw:.6f}\n"
        formatted += f"  Euler Angles (deg): Roll={math.degrees(euler.roll):.2f}°, Pitch={math.degrees(euler.pitch):.2f}°, Yaw={math.degrees(euler.yaw):.2f}°\n"
        
        pos = self.vehicle_state['position']
        formatted += f"  Position (m): X={pos['x']:.3f}, Y={pos['y']:.3f}, Z={pos['z']:.3f}\n"
        
        vel = self.vehicle_state['velocity']
        formatted += f"  Velocity (m/s): X={vel['x']:.3f}, Y={vel['y']:.3f}, Z={vel['z']:.3f}\n"
        
        acc = self.vehicle_state['acceleration']
        formatted += f"  Acceleration (m/s²): X={acc['x']:.3f}, Y={acc['y']:.3f}, Z={acc['z']:.3f}\n\n"
        
        # Sensor Data
        formatted += "SENSOR DATA:\n"
        if self.vehicle_state['sensorStatus']:
            formatted += f"  Sensor Status: {'OK' if not self.vehicle_state['failmask'] else 'FAILURES'}\n"
            for sensor, status in self.vehicle_state['sensorStatus'].items():
                formatted += f"    {sensor}: {'OK' if status else 'FAIL'}\n"
        else:
            formatted += "  Sensor Status: No data\n"
        formatted += f"  SD Card: {'OK' if self.vehicle_state['sdGood'] else 'FAIL'}\n"
        
        gyro = self.vehicle_state['gyro']
        formatted += f"  Gyroscope (rad/s): Yaw={gyro['yaw']:.6f}, Pitch={gyro['pitch']:.6f}, Roll={gyro['roll']:.6f}\n"
        
        acc_sensor = self.vehicle_state['accelerometer']
        formatted += f"  Accelerometer (m/s²): X={acc_sensor['x']:.3f}, Y={acc_sensor['y']:.3f}, Z={acc_sensor['z']:.3f}\n"
        
        formatted += f"  Barometer Altitude: {self.vehicle_state['baroAltitude']:.3f} m\n"
        
        bias = self.vehicle_state['gyroBias']
        formatted += f"  Gyro Bias (rad/s): Yaw={bias['yaw']:.6f}, Pitch={bias['pitch']:.6f}, Roll={bias['roll']:.6f}\n\n"
        
        # GPS Data
        formatted += "GPS DATA:\n"
        formatted += f"  Quality: {self.vehicle_state['gpsQuality']}\n"
        sats = self.vehicle_state['satellites']
        formatted += f"  Satellites: {sats['used']}/{sats['inView']}\n"
        formatted += f"  PDOP: {self.vehicle_state['pdop']:.2f}\n"
        
        gps_pos = self.vehicle_state['gpsPosition']
        formatted += f"  Position: Lat={gps_pos['latitude']:.8f}°, Lon={gps_pos['longitude']:.8f}°, Alt={gps_pos['altitude']:.3f}m\n"
        
        home_pos = self.vehicle_state['homePosition']
        formatted += f"  Home: Lat={home_pos['latitude']:.8f}°, Lon={home_pos['longitude']:.8f}°, Alt={home_pos['altitude']:.3f}m\n"
        
        gps_vel = self.vehicle_state['gpsVelocity']
        formatted += f"  GPS Velocity (m/s): N={gps_vel['north']:.3f}, E={gps_vel['east']:.3f}, D={gps_vel['down']:.3f}\n"
        
        gps_acc = self.vehicle_state['gpsAccuracy']
        formatted += f"  GPS Accuracy: 2D={gps_acc['2D']:.3f}m, 3D={gps_acc['3D']:.3f}m\n"
        
        rel_pos = self.vehicle_state['relativePosition']
        formatted += f"  Relative Position (m): X={rel_pos['x']:.3f}, Y={rel_pos['y']:.3f}, Z={rel_pos['z']:.3f}\n"
        formatted += f"  Last RTCM: {self.vehicle_state['lastRTCM']}\n\n"
        
        # Lander/Control Data
        formatted += "LANDER/CONTROL DATA:\n"
        formatted += f"  Battery Voltage: {self.vehicle_state['batteryVoltage']:.2f} V\n"
        formatted += f"  Thrust: {self.vehicle_state['thrust']:.3f} N\n"
        formatted += f"  Mass: {self.vehicle_state['mass']:.3f} kg\n"
        formatted += f"  Moment of Inertia: {self.vehicle_state['momentOfInertia']:.6f} kg⋅m²\n"
        formatted += f"  Moment Arm: {self.vehicle_state['momentArm']:.3f} m\n"
        
        target = self.vehicle_state['target']
        formatted += f"  Target Position: Y={target['Y']:.3f}m, Z={target['Z']:.3f}m\n"
        
        projected = self.vehicle_state['projected']
        formatted += f"  Projected Position: Y={projected['Y']:.3f}m, Z={projected['Z']:.3f}m\n"
        
        formatted += f"  Ignition Altitude: {self.vehicle_state['ignitionAltitude']:.3f} m\n"
        formatted += f"  Apogee Altitude: {self.vehicle_state['apogeeAltitude']:.3f} m\n"
        
        setpoints = self.vehicle_state['setpoints']
        formatted += f"  Attitude Setpoints: Yaw={setpoints['yaw']:.3f}rad, Pitch={setpoints['pitch']:.3f}rad\n"
        
        commands = self.vehicle_state['commands']
        formatted += f"  Control Commands: Yaw={commands['yaw']:.3f}, Pitch={commands['pitch']:.3f}, Roll={commands['roll']:.3f}\n"
        
        roll_mixed = self.vehicle_state['rollMixed']
        formatted += f"  Roll Mixed: Yaw={roll_mixed['yaw']:.3f}, Pitch={roll_mixed['pitch']:.3f}\n"
        
        misaligns = self.vehicle_state['misaligns']
        formatted += f"  Misalignments: Yaw={misaligns['yaw']:.3f}, Pitch={misaligns['pitch']:.3f}\n"
        
        pyro = self.vehicle_state['pyroStatus']
        formatted += f"  Pyro Status: Chute={'OK' if pyro['chute'] else 'NO CONTINUITY'}, "
        formatted += f"Pyro={'OK' if pyro['pyro'] else 'NO CONTINUITY'} (Raw: {pyro['raw']})\n\n"
        
        # Kalman Filter Data
        formatted += "KALMAN FILTER DATA:\n"
        formatted += "  Measurements:\n"
        k_meas = self.vehicle_state['kalmanMeasurements']
        formatted += f"    Position (m): X={k_meas['position']['x']:.3f}, Y={k_meas['position']['y']:.3f}, Z={k_meas['position']['z']:.3f}\n"
        formatted += f"    Velocity (m/s): X={k_meas['velocity']['x']:.3f}, Y={k_meas['velocity']['y']:.3f}, Z={k_meas['velocity']['z']:.3f}\n"
        formatted += f"    Acceleration (m/s²): X={k_meas['acceleration']['x']:.3f}, Y={k_meas['acceleration']['y']:.3f}, Z={k_meas['acceleration']['z']:.3f}\n"
        
        formatted += "  Uncertainties (1-sigma):\n"
        k_unc = self.vehicle_state['kalmanUncertainty']
        formatted += f"    Position (m): X={k_unc['position']['x']:.6f}, Y={k_unc['position']['y']:.6f}, Z={k_unc['position']['z']:.6f}\n"
        formatted += f"    Velocity (m/s): X={k_unc['velocity']['x']:.6f}, Y={k_unc['velocity']['y']:.6f}, Z={k_unc['velocity']['z']:.6f}\n"
        formatted += f"    Acceleration (m/s²): X={k_unc['acceleration']['x']:.6f}, Y={k_unc['acceleration']['y']:.6f}, Z={k_unc['acceleration']['z']:.6f}\n"
        
        return formatted
    
    def first_packet(self):
        if self.telemetry_data:
            self.current_index = 0
            self.update_display()
    
    def last_packet(self):
        if self.telemetry_data:
            self.current_index = len(self.telemetry_data) - 1
            self.update_display()
    
    def prev_packet(self):
        if self.telemetry_data and self.current_index > 0:
            self.current_index -= 1
            self.update_display()
    
    def next_packet(self):
        if self.telemetry_data and self.current_index < len(self.telemetry_data) - 1:
            self.current_index += 1
            self.update_display()
    
    def jump_to_index(self):
        try:
            index = int(self.jump_var.get()) - 1  # Convert to 0-based index
            if 0 <= index < len(self.telemetry_data):
                self.current_index = index
                self.update_display()
            else:
                messagebox.showerror("Error", f"Index must be between 1 and {len(self.telemetry_data)}")
        except ValueError:
            messagebox.showerror("Error", "Please enter a valid number")
    
    def show_plots(self):
        """Show plotting window with gimbal control analysis"""
        if not self.telemetry_data:
            messagebox.showwarning("No Data", "Please load a telemetry file first")
            return
        
        if self.plot_window is not None:
            try:
                self.plot_window.destroy()
            except:
                pass
        
        self.plot_window = tk.Toplevel(self.root)
        self.plot_window.title("Gimbal Control Analysis")
        self.plot_window.geometry("1200x800")
        
        # Create plots
        self.create_gimbal_plots()
    
    def create_gimbal_plots(self):
        """Create plots for gimbal control analysis"""
        # Extract time series data
        timestamps = []
        yaw_setpoints = []
        pitch_setpoints = []
        yaw_misaligns = []
        pitch_misaligns = []
        euler_yaw = []
        euler_pitch = []
        rollmixed_yaw = []
        rollmixed_pitch = []
        yaw_control = []
        pitch_control = []
        yaw_corrected = []
        pitch_corrected = []
        pos_y = []
        pos_z = []
        proj_y = []
        proj_z = []

        # Process all data up to current point for complete time series
        temp_state = self.reset_temp_vehicle_state()

        for i, packet in enumerate(self.telemetry_data):
            # Update temp state
            self.update_temp_vehicle_state(temp_state, packet, i)

            # Extract timestamp (use time since launch if available, otherwise packet index)
            if temp_state.get('timeSinceLaunch', 0) > 0:
                timestamp = temp_state['timeSinceLaunch']
            else:
                timestamp = i * 0.1  # Assume 10Hz if no time data

            timestamps.append(timestamp)

            # Extract control data
            yaw_setpoints.append(temp_state['setpoints']['yaw'])
            pitch_setpoints.append(temp_state['setpoints']['pitch'])
            yaw_misaligns.append(temp_state['misaligns']['yaw'])
            pitch_misaligns.append(temp_state['misaligns']['pitch'])

            # Extract Euler angles
            euler = temp_state['eulerAngles']
            euler_yaw.append(euler.yaw)
            euler_pitch.append(euler.pitch)

            # Extract rollMixed angles
            rollmixed_yaw.append(temp_state['rollMixed']['yaw'])
            rollmixed_pitch.append(temp_state['rollMixed']['pitch'])

            commands = temp_state['commands']
            yaw_control.append(commands['yaw'])
            pitch_control.append(commands['pitch'])

            # Calculate corrected angles (rollMixed angle - misalignment)
            yaw_corrected.append(temp_state['rollMixed']['yaw'] - temp_state['misaligns']['yaw'])
            pitch_corrected.append(temp_state['rollMixed']['pitch'] - temp_state['misaligns']['pitch'])

            # Extract Y/Z position and projected position from packet if available
            # Try to get from packet, fallback to 0.0 if not present
            y_val = 0.0
            z_val = 0.0
            y_proj = 0.0
            z_proj = 0.0
            if 'position' in packet and isinstance(packet['position'], dict):
                y_val = packet['position'].get('y', 0.0)
                z_val = packet['position'].get('z', 0.0)
            if 'projected' in packet and isinstance(packet['projected'], dict):
                y_proj = packet['projected'].get('Y', 0.0)
                z_proj = packet['projected'].get('Z', 0.0)
            pos_y.append(y_val)
            pos_z.append(z_val)
            proj_y.append(y_proj)
            proj_z.append(z_proj)

        # Convert to numpy arrays for plotting
        timestamps = np.array(timestamps)
        yaw_setpoints = np.array(yaw_setpoints)
        pitch_setpoints = np.array(pitch_setpoints)
        yaw_misaligns = np.array(yaw_misaligns)
        pitch_misaligns = np.array(pitch_misaligns)
        euler_yaw = np.array(euler_yaw)
        euler_pitch = np.array(euler_pitch)

        yaw_control = np.array(yaw_control) / 100.0  # Scale if necessary
        pitch_control = np.array(pitch_control) / 100.0

        rollmixed_yaw = np.array(rollmixed_yaw)
        rollmixed_pitch = np.array(rollmixed_pitch)
        yaw_corrected = np.array(yaw_corrected)
        pitch_corrected = np.array(pitch_corrected)
        pos_y = np.array(pos_y)
        pos_z = np.array(pos_z)
        proj_y = np.array(proj_y)
        proj_z = np.array(proj_z)

        print(proj_y)
        print(proj_z)

        # Create figure with subplots (2x3 grid for 5 plots)
        fig, axs = plt.subplots(2, 2, figsize=(18, 10))
        fig.suptitle('Gimbal Control Analysis', fontsize=16)
        ax1 = axs[0, 0]
        ax2 = axs[0, 1]
        # ax3 = axs[0, 2]
        # ax4 = axs[1, 0]
        ax5 = axs[1, 1]

        # Plot 1: Yaw Control
        ax1.plot(timestamps, np.degrees(yaw_setpoints), 'b-', label='Yaw Setpoint', linewidth=2)
        ax1.plot(timestamps, np.degrees(yaw_control), 'orange', label='Yaw Control', linewidth=1)
        # ax1.plot(timestamps, np.degrees(yaw_misaligns), 'r-', label='Yaw Misalignment', linewidth=1)
        ax1.plot(timestamps, np.degrees(euler_yaw), 'g-', label='Actual Yaw (Euler)', linewidth=1)
        # ax1.plot(timestamps, np.degrees(rollmixed_yaw), 'orange', label='RollMixed Yaw', linewidth=1)
        # ax1.plot(timestamps, np.degrees(yaw_corrected), 'm--', label='Corrected Yaw (RollMixed - Misalign)', linewidth=1)
        ax1.set_xlabel('Time (s)')
        ax1.set_ylabel('Yaw Angle (degrees)')
        ax1.set_title('Yaw Control')
        ax1.legend()
        ax1.grid(True, alpha=0.3)

        # Plot 2: Pitch Control
        ax2.plot(timestamps, np.degrees(pitch_setpoints), 'b-', label='Pitch Setpoint', linewidth=2)
        ax2.plot(timestamps, np.degrees(pitch_control), 'orange', label='Pitch Control', linewidth=1)
        #ax2.plot(timestamps, np.degrees(pitch_misaligns), 'r-', label='Pitch Misalignment', linewidth=1)
        ax2.plot(timestamps, np.degrees(euler_pitch), 'g-', label='Actual Pitch (Euler)', linewidth=1)
        # ax2.plot(timestamps, np.degrees(rollmixed_pitch), 'orange', label='RollMixed Pitch', linewidth=1)
        # ax2.plot(timestamps, np.degrees(pitch_corrected), 'm--', label='Corrected Pitch (RollMixed - Misalign)', linewidth=1)
        ax2.set_xlabel('Time (s)')
        ax2.set_ylabel('Pitch Angle (degrees)')
        ax2.set_title('Pitch Control')
        ax2.legend()
        ax2.grid(True, alpha=0.3)

        # Plot 3: Misalignments
        # ax3.plot(timestamps, np.degrees(yaw_misaligns), 'r-', label='Yaw Misalignment', linewidth=2)
        # ax3.plot(timestamps, np.degrees(pitch_misaligns), 'orange', label='Pitch Misalignment', linewidth=2)
        # ax3.set_xlabel('Time (s)')
        # ax3.set_ylabel('Misalignment (degrees)')
        # ax3.set_title('Gimbal Misalignments')
        # ax3.legend()
        # ax3.grid(True, alpha=0.3)

        # Plot 4: Control Errors
        # yaw_error = np.degrees(yaw_setpoints - rollmixed_yaw)
        # pitch_error = np.degrees(pitch_setpoints - rollmixed_pitch)
        # ax4.plot(timestamps, yaw_error, 'b-', label='Yaw Error (Setpoint - RollMixed)', linewidth=2)
        # ax4.plot(timestamps, pitch_error, 'g-', label='Pitch Error (Setpoint - RollMixed)', linewidth=2)
        # ax4.set_xlabel('Time (s)')
        # ax4.set_ylabel('Control Error (degrees)')
        # ax4.set_title('Control Errors (Setpoint - RollMixed)')
        # ax4.legend()
        # ax4.grid(True, alpha=0.3)

        # Plot 5: Y/Z Position vs Projected Position
        # Remove zeros for each series and keep only matching timestamps
        def filter_nonzero(ts, y, z, y_proj, z_proj):
            filtered_ts1 = []
            filtered_ts2 = []
            filtered_y = []
            filtered_z = []
            filtered_y_proj = []
            filtered_z_proj = []
            for i in range(len(ts)):
                # Only keep if all values are nonzero
                if y[i] != 0.0 or z[i] != 0.0:
                    filtered_ts1.append(ts[i])
                    filtered_y.append(y[i])
                    filtered_z.append(z[i])
                if y_proj[i] != 0.0 or z_proj[i] != 0.0:
                    filtered_ts2.append(ts[i])
                    filtered_y_proj.append(y_proj[i])
                    filtered_z_proj.append(z_proj[i])
                    print('a')

            return np.array(filtered_ts1), np.array(filtered_ts2), np.array(filtered_y), np.array(filtered_z), np.array(filtered_y_proj), np.array(filtered_z_proj)

        f_ts1, f_ts2, f_y, f_z, f_y_proj, f_z_proj = filter_nonzero(timestamps, pos_y, pos_z, proj_y, proj_z)

        ax5.plot(f_ts1, f_y, 'b-', label='Y Position', linewidth=2)
        ax5.plot(f_ts2, f_y_proj, 'b--', label='Y Projected', linewidth=2)
        ax5.plot(f_ts1, f_z, 'g-', label='Z Position', linewidth=2)
        ax5.plot(f_ts2, f_z_proj, 'g--', label='Z Projected', linewidth=2)
        ax5.set_xlabel('Time (s)')
        ax5.set_ylabel('Position (m)')
        ax5.set_title('Y/Z Position vs Projected')
        ax5.legend()
        ax5.grid(True, alpha=0.3)

        # Hide unused subplot (axs[1,2])
        # axs[1,2].axis('off')

        plt.tight_layout(rect=[0, 0.03, 1, 0.95])

        # Embed plot in tkinter window
        canvas = FigureCanvasTkAgg(fig, self.plot_window)
        canvas.draw()
        canvas.get_tk_widget().pack(fill=tk.BOTH, expand=True)
        
        # Add interactive cursors with hover tooltips to all plots
        mplcursors.cursor(ax1, hover=True)
        mplcursors.cursor(ax2, hover=True)  
        mplcursors.cursor(ax5, hover=True)

        # Add current position indicator
        current_time = timestamps[self.current_index] if self.current_index < len(timestamps) else 0
        for ax in [ax1, ax2, ax5]:
            ax.axvline(x=current_time, color='red', linestyle=':', alpha=0.7, label='Current Position')
    
    def reset_temp_vehicle_state(self):
        """Reset temporary vehicle state for plotting"""
        return {
            'timeSinceLaunch': 0.0,
            'eulerAngles': EulerAngles(0.0, 0.0, 0.0),
            'setpoints': {'yaw': 0.0, 'pitch': 0.0},
            'misaligns': {'yaw': 0.0, 'pitch': 0.0},
            'rollMixed': {'yaw': 0.0, 'pitch': 0.0},
            'commands': {'yaw': 0.0, 'pitch': 0.0, 'roll': 0.0},
            'lastUpdated': {
                'STATE_TELEMETRY': -1,
                'LANDER': -1
            }
        }
    
    def update_temp_vehicle_state(self, temp_state, packet, packet_index):
        """Update temporary vehicle state for plotting"""
        packet_type = packet.get('type', 'UNKNOWN')
        
        if packet_type in temp_state['lastUpdated']:
            temp_state['lastUpdated'][packet_type] = packet_index
        
        if packet_type in ['STATE_TELEMETRY', 'STATE_TELEMETRY_V11']:
            temp_state['timeSinceLaunch'] = packet['timeSinceLaunch']
            if 'eulerAngles' in packet:
                temp_state['eulerAngles'] = packet['eulerAngles']
        
        elif packet_type in ['LANDER', 'GPS_V11']:
            if 'setpoints' in packet:
                temp_state['setpoints'] = packet['setpoints'].copy()
            if 'misaligns' in packet:
                temp_state['misaligns'] = packet['misaligns'].copy()
            if 'rollMixed' in packet:
                temp_state['rollMixed'] = packet['rollMixed'].copy()
            if 'commands' in packet:
                temp_state['commands'] = packet['commands'].copy()
                
        # For v1.1 GPS packets that include lander data
        if packet_type == 'GPS_V11':
            if 'setpoints' in packet:
                temp_state['setpoints'] = packet['setpoints'].copy()
            if 'misaligns' in packet:
                temp_state['misaligns'] = packet['misaligns'].copy()
            if 'rollMixed' in packet:
                temp_state['rollMixed'] = packet['rollMixed'].copy()
            if 'commands' in packet:
                temp_state['commands'] = packet['commands'].copy()

def main():
    root = tk.Tk()
    app = TelemetryViewer(root)
    root.mainloop()

if __name__ == "__main__":
    main()
