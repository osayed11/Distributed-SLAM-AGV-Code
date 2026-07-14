#!/usr/bin/env python3
"""Protocol-correct wrapper for the pinned python-natnet client."""

import time

from natnet import NatNetClient as _NatNetClient


class NatNetClient(_NatNetClient):
    """Fix connection and keepalive handling in python-natnet 0.2.0.

    The upstream client uses a legacy short connection request, declares an
    empty keepalive as a one-byte payload, and sends one on every socket poll.
    This wrapper matches OptiTrack PythonClient 4.4's versioned connection and
    empty-request framing, with the SDK's 10-second keepalive interval.
    """

    # OptiTrack's NatNet 4.4 PythonClient uses a two-second command-socket
    # timeout and sends a keepalive after every timeout in unicast mode.
    KEEPALIVE_PERIOD_SEC = 2.0
    _ZERO_PAYLOAD_COMMANDS = {
        _NatNetClient.NAT_REQUEST_MODELDEF,
        _NatNetClient.NAT_REQUEST_FRAMEOFDATA,
        _NatNetClient.NAT_KEEPALIVE,
    }

    def __init__(self, *args, **kwargs):
        super().__init__(*args, **kwargs)
        self._next_keepalive_at = 0.0

    def send_request(self, command: int, command_str: str = ""):
        if command == self.NAT_CONNECT:
            return self._send_versioned_connect()
        if command not in self._ZERO_PAYLOAD_COMMANDS:
            return super().send_request(command, command_str)

        command_socket = getattr(self, "_NatNetClient__command_socket")
        server_ip = getattr(self, "_NatNetClient__server_ip_address")
        command_port = getattr(self, "_NatNetClient__command_port")
        data = command.to_bytes(2, byteorder="little")
        data += (0).to_bytes(2, byteorder="little")
        # Match OptiTrack's PythonClient 4.4 wire format for empty requests.
        data += b"\0"
        return command_socket.sendto(data, (server_ip, command_port))

    def _send_versioned_connect(self):
        command_socket = getattr(self, "_NatNetClient__command_socket")
        server_ip = getattr(self, "_NatNetClient__server_ip_address")
        command_port = getattr(self, "_NatNetClient__command_port")

        payload = bytearray(270)
        payload[0:4] = b"Ping"
        # python-natnet 0.2.0 parses through NatNet 4.1. NatNet 4.2 adds a
        # rigid-body-description rotation offset that this pinned parser lacks.
        payload[265:269] = bytes((4, 1, 0, 0))
        data = self.NAT_CONNECT.to_bytes(2, byteorder="little")
        data += (len(payload) + 1).to_bytes(2, byteorder="little")
        data += payload
        data += b"\0"
        return command_socket.sendto(data, (server_ip, command_port))

    def connect(self, timeout: float = 5.0):
        super().connect(timeout=timeout)
        self._next_keepalive_at = time.monotonic() + self.KEEPALIVE_PERIOD_SEC

    def update_sync(self):
        if self.running_asynchronously:
            raise RuntimeError("Cannot update synchronously while running asynchronously.")

        process_socket = getattr(self, "_NatNetClient__process_socket")
        data_socket = getattr(self, "_NatNetClient__data_socket")
        command_socket = getattr(self, "_NatNetClient__command_socket")
        use_multicast = getattr(self, "_NatNetClient__use_multicast")

        while process_socket(data_socket):
            pass

        now = time.monotonic()
        if not use_multicast and now >= self._next_keepalive_at:
            self.send_request(self.NAT_KEEPALIVE)
            self._next_keepalive_at = now + self.KEEPALIVE_PERIOD_SEC

        while process_socket(command_socket):
            pass

    def shutdown(self):
        if self.connected:
            try:
                # This is the explicit disconnect command used by the SDK sample.
                self.send_command("Disconnect")
            except OSError:
                pass
        super().shutdown()
