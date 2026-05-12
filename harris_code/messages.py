"""
ZMQ message helpers — self-contained replacement for real_robot.transport.messages.

All payloads are msgpack-encoded dicts with a "t" tag identifying the message type.
The topic byte string (first multipart frame) matches the "t" tag.
"""
import time
import msgpack


def unpack(raw: bytes) -> dict:
    return msgpack.unpackb(raw, raw=False)


def pose_msg(robot_id: int, x: float, y: float, theta: float,
             source: str = "odom") -> bytes:
    """Robot or mocap pose update. `source` is 'mocap' or 'odom' so the GUI
    can prefer mocap when both are flowing."""
    return msgpack.packb({
        "t": "pose", "id": int(robot_id), "ts": time.time(),
        "x": float(x), "y": float(y), "theta": float(theta),
        "source": source,
    })


def goal_msg(x: float, y: float, theta: float, tol: float,
             robot_id: int = -1) -> bytes:
    """Goal update from the GUI. robot_id=-1 broadcasts to all robots; a
    specific id targets one robot when we extend to multi-robot."""
    return msgpack.packb({
        "t": "goal", "ts": time.time(), "id": int(robot_id),
        "x": float(x), "y": float(y), "theta": float(theta),
        "tol": float(tol),
    })


def cmd_msg(robot_id: int, vx: float, vy: float, omega: float = 0.0) -> bytes:
    """Body-frame velocity command. Only kept for symmetry / future use;
    the new architecture runs the controller on the robot so the laptop
    does not publish cmd messages."""
    return msgpack.packb({
        "t": "cmd", "ts": time.time(), "id": int(robot_id),
        "vx": float(vx), "vy": float(vy), "omega": float(omega),
    })


def estop_msg() -> bytes:
    """Hard stop — robots zero velocity and exit."""
    return msgpack.packb({"t": "estop", "ts": time.time()})


def ctrl_stop_msg() -> bytes:
    """Soft stop — robots pause the controller, clear goal, await next goal."""
    return msgpack.packb({"t": "ctrl_stop", "ts": time.time()})
