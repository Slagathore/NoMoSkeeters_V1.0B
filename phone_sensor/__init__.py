"""Phone-as-sensor — PC side of the NoMoSkeeters Sensor Protocol v1.

The phone is a dumb-on-purpose smart camera. The PC orchestrates: switches
lenses, sets exposure/focus, starts/stops streaming. The phone executes and
ships frames. See PHONE_SENSOR_BOOTSTRAP.md for the architectural commitment;
this package is the wire side of that contract.

Layout::

    phone_sensor/
        protocol.py        — message schemas + frame packet wire format
        client.py          — PhoneSensorClient: TCP commands, heartbeat,
                              event dispatch, reconnect
        frame_decoder.py   — UDP frame receiver + raw_yuv / h264 decoder
        calibration.py     — per-camera profile loader (Approach B)
"""
from phone_sensor.protocol import (
    PROTOCOL_VERSION, PhoneCapabilities, PhoneCameraSpec, StreamMode,
    pack_command, parse_message, pack_frame_packet, parse_frame_packet,
    FramePacket,
)
from phone_sensor.client import PhoneSensorClient
from phone_sensor.frame_decoder import PhoneFrameDecoder

__all__ = [
    "PROTOCOL_VERSION", "PhoneCapabilities", "PhoneCameraSpec", "StreamMode",
    "pack_command", "parse_message", "pack_frame_packet", "parse_frame_packet",
    "FramePacket", "PhoneSensorClient", "PhoneFrameDecoder",
]
