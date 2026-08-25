"""Low-level TCP standard-console driver for an Attocube ANC300 controller."""

from __future__ import annotations

import math
import re
import socket
import threading
from typing import Any


ANC300_MIN_OFFSET_V = 0.0
ANC300_MAX_OFFSET_V = 150.0


class ANC300Error(Exception):
    """Base class for ANC300 driver failures."""


class ANC300ConnectionError(ANC300Error):
    """The TCP connection or a command exchange could not be completed."""


class ANC300ProtocolError(ANC300Error):
    """The controller returned data that is not a valid console response."""


class ANC300CommandError(ANC300Error):
    """The controller completed a command with an ``ERROR`` response."""


class ANC300Controller:
    """Single-connection ANC300 Ethernet standard-console controller.

    Each command is sent and fully read while holding ``_lock``.  This is
    important because standard-console replies are not tagged with a command
    identifier, so concurrent callers cannot safely share the stream otherwise.
    """

    def __init__(
        self,
        host: str,
        port: int = 7230,
        password: str = "123456",
        timeout_s: float = 3.0,
    ) -> None:
        if not isinstance(host, str) or not host.strip():
            raise ValueError("host must be a non-empty string")
        if timeout_s <= 0:
            raise ValueError("timeout_s must be positive")
        self.host = host
        self.port = port
        self.password = password
        self.timeout_s = float(timeout_s)
        self._socket: socket.socket | None = None
        self._receive_buffer = b""
        self._lock = threading.RLock()
        self._snapshot: dict[str, Any] | None = None
        self._offset_mode_token: str | None = None

    @property
    def connected(self) -> bool:
        return self._socket is not None

    def connect(self, axes=(1, 2)) -> dict:
        """Open, authenticate, initialise the console, and return a read-only snapshot."""
        axes = tuple(axes)
        for axis in axes:
            self._validate_axis(axis)
        with self._lock:
            if self.connected:
                raise ANC300ConnectionError("controller is already connected")
            self._receive_buffer = b""
            self._offset_mode_token = None
            try:
                sock = socket.create_connection((self.host, self.port), self.timeout_s)
                sock.settimeout(self.timeout_s)
                self._socket = sock
                self._login_and_disable_echo()
                snapshot = self.get_device_snapshot(axes)
                snapshot["connected"] = True
                self._snapshot = snapshot
                return snapshot
            except ANC300Error:
                self._close_unlocked()
                raise
            except (OSError, ValueError) as exc:
                self._close_unlocked()
                raise ANC300ConnectionError("could not connect to ANC300") from exc

    def disconnect(self) -> None:
        """Close the TCP transport without sending any controller command."""
        with self._lock:
            self._close_unlocked()

    def query(self, command: str) -> list[str]:
        """Send one complete standard-console command transaction."""
        if not isinstance(command, str) or not command or "\r" in command or "\n" in command:
            raise ValueError("command must be a non-empty single line")
        with self._lock:
            if not self.connected:
                raise ANC300ConnectionError("ANC300 is not connected")
            return self._transaction(command)

    def get_version(self) -> str:
        value = self._single_response_line("ver", "version")
        if not re.search(r"\bversion\b", value, flags=re.IGNORECASE):
            self._protocol_failure("malformed version response")
        return value

    def get_controller_serial(self) -> str:
        return self._label_value(
            "getcser", "controller serial", {"controller serial number"}
        )

    def get_axis_serial(self, axis: int) -> str:
        self._validate_axis(axis)
        return self._label_value(
            "getser %d" % axis, "axis serial", {"axis serial number"}
        )

    def get_mode(self, axis: int) -> str:
        self._validate_axis(axis)
        return self._label_value("getm %d" % axis, "mode", {"mode"})

    def get_offset(self, axis: int) -> float:
        self._validate_axis(axis)
        return self._voltage_value("geta %d" % axis, "offset voltage")

    def get_output(self, axis: int) -> float:
        self._validate_axis(axis)
        return self._voltage_value("geto %d" % axis, "output voltage")

    def get_ac_input(self, axis: int) -> bool:
        self._validate_axis(axis)
        return self._input_state("getaci %d" % axis, "AC-IN", {"acin"})

    def get_dc_input(self, axis: int) -> bool:
        self._validate_axis(axis)
        return self._input_state("getdci %d" % axis, "DC-IN", {"dcin"})

    def get_filter(self, axis: int) -> str:
        self._validate_axis(axis)
        return self._label_value("getfil %d" % axis, "filter", {"filter"})

    def get_axis_snapshot(self, axis: int) -> dict:
        self._validate_axis(axis)
        return {
            "serial": self.get_axis_serial(axis),
            "mode": self.get_mode(axis),
            "offset": self.get_offset(axis),
            "filter": self.get_filter(axis),
        }

    def get_device_snapshot(self, axes=(1, 2)) -> dict:
        axes = tuple(axes)
        for axis in axes:
            self._validate_axis(axis)
        return {
            "version": self.get_version(),
            "controller_serial": self.get_controller_serial(),
            "axes": {axis: self.get_axis_snapshot(axis) for axis in axes},
        }

    def set_mode(self, axis: int, mode: str) -> list[str]:
        self._validate_write_axis(axis)
        if not isinstance(mode, str) or not mode.strip() or "\r" in mode or "\n" in mode:
            raise ValueError("mode must be a non-empty single line")
        mode = mode.strip()
        if mode.lower() not in {"off", "offs"}:
            return self.query("setm %d %s" % (axis, mode))
        return self._set_offset_mode(axis)

    def set_offset(self, axis: int, voltage: float) -> list[str]:
        self._validate_write_axis(axis)
        if isinstance(voltage, bool):
            raise ValueError("voltage must be finite")
        try:
            value = float(voltage)
        except (TypeError, ValueError) as exc:
            raise ValueError("voltage must be numeric") from exc
        if not math.isfinite(value):
            raise ValueError("voltage must be finite")
        if not ANC300_MIN_OFFSET_V <= value <= ANC300_MAX_OFFSET_V:
            raise ValueError("voltage must be within 0 through 150 V")
        return self.query("seta %d %s" % (axis, format(value, "g")))

    def set_ac_input(self, axis: int, enabled: bool) -> list[str]:
        self._validate_write_axis(axis)
        if not isinstance(enabled, bool):
            raise ValueError("enabled must be a boolean")
        return self.query("setaci %d %s" % (axis, "on" if enabled else "off"))

    def set_dc_input(self, axis: int, enabled: bool) -> list[str]:
        self._validate_write_axis(axis)
        if not isinstance(enabled, bool):
            raise ValueError("enabled must be a boolean")
        return self.query("setdci %d %s" % (axis, "on" if enabled else "off"))

    def stop(self, axis: int) -> list[str]:
        self._validate_write_axis(axis)
        return self.query("stop %d" % axis)

    def _set_offset_mode(self, axis: int) -> list[str]:
        token = self._offset_mode_token
        if token is not None:
            return self.query("setm %d %s" % (axis, token))
        try:
            response = self.query("setm %d off" % axis)
        except ANC300CommandError:
            response = self.query("setm %d offs" % axis)
            self._offset_mode_token = "offs"
            return response
        self._offset_mode_token = "off"
        return response

    def _single_response_line(self, command: str, name: str) -> str:
        lines = self.query(command)
        if len(lines) != 1 or not lines[0].strip():
            self._protocol_failure("malformed %s response" % name)
        return lines[0].strip()

    def _label_value(self, command: str, name: str, expected_labels: set[str]) -> str:
        line = self._single_response_line(command, name)
        if "=" not in line:
            self._protocol_failure("malformed %s response" % name)
        label, value = line.split("=", 1)
        normalized_label = self._normalize_label(label)
        normalized_expected = {self._normalize_label(item) for item in expected_labels}
        if normalized_label not in normalized_expected:
            self._protocol_failure("unexpected %s response label" % name)
        value = value.strip()
        if not value:
            self._protocol_failure("malformed %s response" % name)
        return value

    def _voltage_value(self, command: str, name: str) -> float:
        text = self._label_value(command, name, {"voltage"})
        match = re.fullmatch(
            r"([-+]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][-+]?\d+)?)\s*V?",
            text,
            flags=re.IGNORECASE,
        )
        if not match:
            self._protocol_failure("malformed %s response" % name)
        value = float(match.group(1))
        if not math.isfinite(value):
            self._protocol_failure("malformed %s response" % name)
        return value

    def _input_state(self, command: str, name: str, expected_labels: set[str]) -> bool:
        value = self._label_value(command, name, expected_labels).strip().lower()
        if value == "on":
            return True
        if value == "off":
            return False
        self._protocol_failure("malformed %s response" % name)

    @staticmethod
    def _normalize_label(label: str) -> str:
        return re.sub(r"[^a-z0-9]+", "", label.strip().lower())

    def _protocol_failure(self, message: str):
        with self._lock:
            self._close_unlocked()
        raise ANC300ProtocolError(message)

    def _transaction(self, command: str) -> list[str]:
        """Execute a command while the caller holds ``_lock``."""
        assert self._socket is not None
        try:
            wire_command = command.encode("ascii") + b"\r\n"
        except UnicodeEncodeError as exc:
            raise ANC300ProtocolError("commands must be ASCII") from exc
        try:
            self._socket.sendall(wire_command)
            return self._read_response(command)
        except (ANC300ConnectionError, ANC300ProtocolError):
            self._close_unlocked()
            raise
        except socket.timeout as exc:
            self._close_unlocked()
            raise ANC300ConnectionError("timeout waiting for response to %r" % command) from exc
        except OSError as exc:
            self._close_unlocked()
            raise ANC300ConnectionError("connection failed during %r" % command) from exc

    def _login_and_disable_echo(self) -> None:
        """Send the console login pair before reading its single response."""
        assert self._socket is not None
        try:
            login = self.password.encode("ascii") + b"\r\n"
        except UnicodeEncodeError as exc:
            raise ANC300ProtocolError("commands must be ASCII") from exc
        try:
            self._socket.sendall(login + b"echo off\r\n")
            self._read_response("login and echo off")
        except ANC300CommandError as exc:
            self._close_unlocked()
            raise ANC300ConnectionError("ANC300 authentication failed") from exc
        except ANC300ConnectionError:
            self._close_unlocked()
            raise
        except socket.timeout as exc:
            self._close_unlocked()
            raise ANC300ConnectionError("timeout during ANC300 login") from exc
        except OSError as exc:
            self._close_unlocked()
            raise ANC300ConnectionError("connection failed during ANC300 login") from exc

    def _read_response(self, command: str) -> list[str]:
        lines: list[str] = []
        while True:
            terminator = self._take_line()
            if terminator is not None:
                line = self._decode_line(terminator)
                status = line.strip()
                if status == "OK":
                    return lines
                if status == "ERROR":
                    detail = "; ".join(lines)
                    message = "ANC300 command %r returned ERROR" % command
                    if detail:
                        message += ": " + detail
                    raise ANC300CommandError(message)
                lines.append(line)
                continue
            assert self._socket is not None
            try:
                packet = self._socket.recv(4096)
            except socket.timeout as exc:
                raise ANC300ConnectionError("timeout waiting for response to %r" % command) from exc
            except OSError as exc:
                raise ANC300ConnectionError("connection failed during %r" % command) from exc
            if not packet:
                if self._receive_buffer or lines:
                    raise ANC300ProtocolError("response to %r ended without OK or ERROR" % command)
                raise ANC300ConnectionError("peer closed while waiting for response to %r" % command)
            self._receive_buffer += packet

    def _take_line(self) -> bytes | None:
        marker = self._receive_buffer.find(b"\r\n")
        if marker < 0:
            return None
        line = self._receive_buffer[:marker]
        self._receive_buffer = self._receive_buffer[marker + 2 :]
        return line

    @staticmethod
    def _decode_line(line: bytes) -> str:
        try:
            return line.decode("ascii")
        except UnicodeDecodeError as exc:
            raise ANC300ProtocolError("response contains non-ASCII data") from exc

    @staticmethod
    def _validate_axis(axis: int) -> None:
        if isinstance(axis, bool) or not isinstance(axis, int) or not 1 <= axis <= 7:
            raise ValueError("axis must be an integer from 1 through 7")

    @classmethod
    def _validate_write_axis(cls, axis: int) -> None:
        cls._validate_axis(axis)
        if axis == 3:
            raise ValueError("axis 3 is read-only for this application")

    def _close_unlocked(self) -> None:
        sock, self._socket = self._socket, None
        self._receive_buffer = b""
        self._offset_mode_token = None
        self._snapshot = None
        if sock is not None:
            try:
                sock.shutdown(socket.SHUT_RDWR)
            except OSError:
                pass
            try:
                sock.close()
            except OSError:
                pass
