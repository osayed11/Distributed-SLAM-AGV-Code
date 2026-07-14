#!/usr/bin/env python3
"""Adapter from OptiTrack PythonClient 4.4 to the ORKAR bridge API."""

import os
import queue
import struct
import sys
import threading
import time
from pathlib import Path
from types import SimpleNamespace


def _load_sdk_client():
    sdk_dir = Path(
        os.environ.get(
            "NATNET_SDK_PYTHON_PATH",
            "/opt/orkar/natnet-sdk-4.4/PythonClient",
        )
    )
    client_file = sdk_dir / "NatNetClient.py"
    if not client_file.is_file():
        raise ImportError(
            "OptiTrack NatNet SDK PythonClient 4.4 is missing at "
            f"{client_file}"
        )
    sys.path.insert(0, str(sdk_dir))
    from NatNetClient import NatNetClient as SdkNatNetClient

    # The official 4.4 direct-depacketizer sample prints every received frame
    # unconditionally. The ROS node owns health/status logging, so suppress the
    # sample's console printer to avoid blocking its receive thread on stdout.
    sys.modules[SdkNatNetClient.__module__].print = lambda *args, **kwargs: None

    return SdkNatNetClient


_SdkNatNetClient = _load_sdk_client()


def _unpack_minimal_model_definitions(data, packet_size, major, minor):
    """Read rigid-body names/IDs using NatNet 4.1+ dataset lengths.

    The official 4.4 sample fully decodes marker metadata that this bridge does
    not use. Motive can occasionally return a shortened marker section, while
    the dataset header and rigid-body identity fields remain complete.
    """
    if major < 4 or (major == 4 and minor < 1):
        raise ValueError("bounded model-definition datasets require NatNet 4.1+")

    payload = bytes(data[:packet_size])
    if len(payload) < 4:
        raise ValueError("model-definition packet is truncated")

    dataset_count = int.from_bytes(payload[:4], "little", signed=True)
    offset = 4
    rigid_bodies = []
    for _ in range(max(dataset_count, 0)):
        if offset + 8 > len(payload):
            break
        data_type = int.from_bytes(payload[offset:offset + 4], "little", signed=True)
        size_in_bytes = int.from_bytes(
            payload[offset + 4:offset + 8], "little", signed=True
        )
        offset += 8
        if size_in_bytes < 0:
            break
        dataset_end = min(offset + size_in_bytes, len(payload))
        if data_type == 1:
            name_end = payload.find(b"\0", offset, dataset_end)
            id_offset = name_end + 1
            if name_end >= offset and id_offset + 4 <= dataset_end:
                rigid_bodies.append(
                    SimpleNamespace(
                        id_num=int.from_bytes(
                            payload[id_offset:id_offset + 4], "little", signed=True
                        ),
                        sz_name=payload[offset:name_end],
                        rb_marker_list=(),
                    )
                )
        offset = dataset_end

    return offset, SimpleNamespace(
        rigid_body_list=rigid_bodies,
        get_as_string=lambda: "",
    )


def _as_text(value):
    if isinstance(value, bytes):
        return value.split(b"\0", 1)[0].decode("utf-8", errors="replace")
    return str(value)


def _daemon_thread(*args, **kwargs):
    kwargs.setdefault("daemon", True)
    return threading.Thread(*args, **kwargs)


class _Event:
    def __init__(self):
        self.handlers = []

    def emit(self, value):
        for handler in tuple(self.handlers):
            handler(value)


class NatNetClient:
    """Expose the small client surface used by the ROS 2 publisher."""

    def __init__(self, server_ip_address, local_ip_address, use_multicast=False):
        self.server_ip_address = server_ip_address
        self.local_ip_address = local_ip_address
        self.use_multicast = use_multicast
        self.on_data_description_received_event = _Event()
        self.on_data_frame_received_event = _Event()
        self._description_events = queue.Queue(maxsize=8)
        self._frame_lock = threading.Lock()
        self._latest_frame = None
        _SdkNatNetClient.run.__globals__["Thread"] = _daemon_thread
        self._client = _SdkNatNetClient()
        self._client.set_print_level(0)
        self._client.set_server_address(server_ip_address)
        self._client.set_client_address(local_ip_address)
        self._client.set_use_multicast(use_multicast)
        self._client.new_frame_with_data_listener = self._on_sdk_frame
        self._install_description_hook()

    def _install_description_hook(self):
        method_name = "_NatNetClient__unpack_data_descriptions"
        original = getattr(self._client, method_name)
        descriptions_type = original.__globals__["DataDescriptions"].DataDescriptions

        def unpack_and_emit(data, packet_size, major, minor):
            if major <= 0:
                return packet_size, descriptions_type()
            try:
                result = original(data, packet_size, major, minor)
            except (IndexError, UnicodeDecodeError, ValueError, struct.error):
                result = _unpack_minimal_model_definitions(
                    data, packet_size, major, minor
                )
            if not isinstance(result, tuple):
                return int(result), descriptions_type()
            offset, descriptions = result
            rigid_bodies = [
                SimpleNamespace(
                    id_num=int(item.id_num),
                    name=_as_text(item.sz_name),
                    markers=tuple(item.rb_marker_list),
                )
                for item in descriptions.rigid_body_list
            ]
            self._put_description(SimpleNamespace(rigid_bodies=rigid_bodies))
            return offset, descriptions

        setattr(self._client, method_name, unpack_and_emit)

    def _on_sdk_frame(self, data):
        mocap_data = data.get("mocap_data")
        rigid_body_data = None if mocap_data is None else mocap_data.rigid_body_data
        if rigid_body_data is None:
            return
        rigid_bodies = [
            SimpleNamespace(
                id_num=int(item.id_num),
                pos=tuple(item.pos),
                rot=tuple(item.rot),
                tracking_valid=bool(item.tracking_valid),
                marker_error=float(item.error),
            )
            for item in rigid_body_data.rigid_body_list
        ]
        with self._frame_lock:
            self._latest_frame = SimpleNamespace(rigid_bodies=rigid_bodies)

    def _put_description(self, description):
        try:
            self._description_events.put_nowait(description)
        except queue.Full:
            try:
                self._description_events.get_nowait()
            except queue.Empty:
                pass
            self._description_events.put_nowait(description)

    def connect(self, timeout=5.0):
        if not self._client.run("d"):
            raise RuntimeError("OptiTrack NatNet SDK client failed to start")
        self._client.data_socket.settimeout(1.0)
        deadline = time.monotonic() + timeout
        while not self._client.connected():
            if time.monotonic() >= deadline:
                self.shutdown()
                raise TimeoutError("OptiTrack NatNet SDK connection timed out")
            time.sleep(0.05)

    def request_modeldef(self):
        self._client.send_request(
            self._client.command_socket,
            self._client.NAT_REQUEST_MODELDEF,
            "",
            (self.server_ip_address, self._client.command_port),
        )

    def update_sync(self):
        while True:
            try:
                description = self._description_events.get_nowait()
            except queue.Empty:
                break
            self.on_data_description_received_event.emit(description)

        with self._frame_lock:
            frame = self._latest_frame
            self._latest_frame = None
        if frame is not None:
            self.on_data_frame_received_event.emit(frame)

    def shutdown(self):
        self._client.stop_threads = True
        for sock in (self._client.command_socket, self._client.data_socket):
            if sock is not None:
                try:
                    sock.close()
                except OSError:
                    pass
        for thread in (self._client.command_thread, self._client.data_thread):
            if thread is not None and thread.is_alive():
                thread.join(timeout=0.5)
