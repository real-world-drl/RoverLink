# SPDX-License-Identifier: Apache-2.0
# Copyright 2026 Peter Bohm

"""Wire-format constants and struct layouts for the UGV firmware.

Hand-mirrored from main/ugv_packets.h. Drift between this file and the
firmware will be caught by the length-byte check in the UART framing layer
(the firmware logs `rx_bad_len` and drops the frame).
"""

from struct import calcsize

# --- UART framing ----------------------------------------------------------

UART_SYNC = 0xA5
# Frame on the wire:
#   [SYNC] [type:1] [len:1] [payload:len] [crc8:1]   (crc over type+len+payload)
# CRC8 poly 0x07, init 0x00.

# --- Packet type IDs (must match ugv_pkt_type_t in firmware) --------------

PKT_CMD_VEL     = 0x01
PKT_CMD_PID     = 0x02
PKT_CMD_DISPLAY = 0x03
PKT_TEL_WHEEL   = 0x10
PKT_TEL_IMU     = 0x11
PKT_TEL_BATT    = 0x12
PKT_STATUS      = 0x20

# --- Struct formats (must match the packed structs in firmware) ----------

# ugv_cmd_vel_t: uint64 host_ts_us, float linear_x, float angular_z
FMT_CMD_VEL     = "<Qff"
# ugv_cmd_pid_t: float kp, ki, kd, output_clamp, min_drive
# (min_drive is the closed-loop stiction floor in PWM: <0 leaves the
#  build-time CONFIG_UGV_MIN_DRIVE_PWM unchanged, >=0 overrides it live.
#  This was the v1 `deadband` slot, repurposed.)
FMT_CMD_PID     = "<fffff"
# ugv_cmd_display_t: uint64 host_ts_us, float x, y, yaw, pitch, roll
FMT_CMD_DISPLAY = "<Qfffff"
# ugv_wheel_telem_t: uint64 ts, uint32 seq, int32 left_ticks, int32 right_ticks,
#                    floats: left_vel, right_vel, left_setpoint, right_setpoint
FMT_TEL_WHEEL   = "<QIiiffff"
# ugv_imu_telem_t: uint64 ts, uint32 seq,
#                  3 floats accel, 3 floats gyro, 3 floats mag, float temp_c,
#                  uint8 mag_fresh, 3 bytes pad — 10 floats total.
FMT_TEL_IMU     = "<QI" + "f"*10 + "Bxxx"
# ugv_battery_telem_t: uint64 ts, float voltage_v, float current_a
FMT_TEL_BATT    = "<Qff"

# type -> (fmt, size). The bot only sends telemetry/status outbound, and
# only accepts the cmd_* types inbound — sizes guarded both ways by the
# firmware's _Static_asserts.
SIZE_BY_TYPE = {
    PKT_CMD_VEL:     calcsize(FMT_CMD_VEL),       # 16
    PKT_CMD_PID:     calcsize(FMT_CMD_PID),       # 20
    PKT_CMD_DISPLAY: calcsize(FMT_CMD_DISPLAY),   # 28
    PKT_TEL_WHEEL:   calcsize(FMT_TEL_WHEEL),     # 36
    PKT_TEL_IMU:     calcsize(FMT_TEL_IMU),       # 56
    PKT_TEL_BATT:    calcsize(FMT_TEL_BATT),      # 16
    PKT_STATUS:      None,                        # variable length string
}

NAME_BY_TYPE = {
    PKT_CMD_VEL:     "cmd_vel",
    PKT_CMD_PID:     "cmd_pid",
    PKT_CMD_DISPLAY: "cmd_display",
    PKT_TEL_WHEEL:   "tel_wheel",
    PKT_TEL_IMU:     "tel_imu",
    PKT_TEL_BATT:    "tel_battery",
    PKT_STATUS:      "status",
}

# --- CRC8 (poly 0x07, init 0x00) ------------------------------------------

def crc8(data: bytes) -> int:
    crc = 0
    for byte in data:
        crc ^= byte
        for _ in range(8):
            crc = ((crc << 1) ^ 0x07) & 0xFF if (crc & 0x80) else (crc << 1) & 0xFF
    return crc


def frame(pkt_type: int, payload: bytes) -> bytes:
    """Wrap a packed-struct payload in the UART framing."""
    header = bytes([pkt_type, len(payload)])
    body = header + payload
    return bytes([UART_SYNC]) + body + bytes([crc8(body)])
