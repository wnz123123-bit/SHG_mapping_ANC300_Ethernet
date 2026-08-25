"""Socket-level contract tests for the ANC300 standard-console driver."""

from __future__ import annotations

import socket
import select
import threading
import unittest

from anc300_controller import (
    ANC300CommandError,
    ANC300ConnectionError,
    ANC300Controller,
    ANC300ProtocolError,
)


class HeldFragmentedResponse:
    """Reply whose terminator is withheld while the server watches for a write."""

    def __init__(self, fragment=b"first reply\r\n", terminator=b"OK\r\n", hold_s=0.15):
        self.fragment = fragment
        self.terminator = terminator
        self.hold_s = hold_s
        self.fragment_sent = threading.Event()
        self.interleaved_write = threading.Event()


class CloseAfterPayload:
    """Reply bytes followed immediately by an orderly peer close."""

    def __init__(self, payload):
        self.payload = payload


class FakeANC300Server:
    """A tiny real TCP peer that returns scripted standard-console replies."""

    def __init__(self, replies=None):
        self.replies = replies or {}
        self.commands = []
        self._commands_lock = threading.Lock()
        self._stop = threading.Event()
        self._ready = threading.Event()
        self._listener = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
        self._listener.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
        self._listener.bind(("127.0.0.1", 0))
        self._listener.listen(4)
        self.port = self._listener.getsockname()[1]
        self._thread = threading.Thread(target=self._serve, daemon=True)

    def __enter__(self):
        self._thread.start()
        self._ready.wait(1)
        return self

    def __exit__(self, *_):
        self._stop.set()
        try:
            self._listener.close()
        except OSError:
            pass
        self._thread.join(1)

    def received(self):
        with self._commands_lock:
            return list(self.commands)

    def _serve(self):
        self._ready.set()
        self._listener.settimeout(0.05)
        while not self._stop.is_set():
            try:
                conn, _ = self._listener.accept()
            except (TimeoutError, OSError):
                continue
            with conn:
                conn.settimeout(0.05)
                self._handle_connection(conn)

    def _handle_connection(self, conn):
        buffer = b""
        while not self._stop.is_set():
            try:
                data = conn.recv(4096)
            except TimeoutError:
                continue
            if not data:
                return
            buffer += data
            while b"\r\n" in buffer:
                wire_command, buffer = buffer.split(b"\r\n", 1)
                command = wire_command.decode("ascii")
                with self._commands_lock:
                    self.commands.append(command)
                reply = self.replies.get(command, b"OK\r\n")
                if callable(reply):
                    reply = reply(command, self)
                if reply is None:
                    continue
                if reply == "CLOSE":
                    return
                if isinstance(reply, CloseAfterPayload):
                    if reply.payload:
                        conn.sendall(reply.payload)
                    return
                if isinstance(reply, HeldFragmentedResponse):
                    conn.sendall(reply.fragment)
                    reply.fragment_sent.set()
                    readable, _, _ = select.select([conn], [], [], reply.hold_s)
                    if readable:
                        late_packet = conn.recv(4096)
                        if late_packet:
                            buffer += late_packet
                            if b"\r\n" in late_packet:
                                reply.interleaved_write.set()
                    conn.sendall(reply.terminator)
                    continue
                if isinstance(reply, (bytes, bytearray)):
                    reply = [reply]
                for packet in reply:
                    conn.sendall(packet)


def normal_replies(password="123456", mode="off", offset="2.345000 V"):
    return {
        password: None,
        "echo off": b"login accepted\r\nOK\r\n",
        "ver": b"ANC300 version 3.4\r\nOK\r\n",
        "getcser": b"controller serial number = C123\r\nOK\r\n",
        "getser 1": b"axis serial number = AX1\r\nOK\r\n",
        "getm 1": ("mode = %s\r\nOK\r\n" % mode).encode("ascii"),
        "geta 1": ("voltage = %s\r\nOK\r\n" % offset).encode("ascii"),
        "getfil 1": b"filter = 16 Hz\r\nOK\r\n",
        "getser 2": b"axis serial number = AX2\r\nOK\r\n",
        "getm 2": b"mode = stp\r\nOK\r\n",
        "geta 2": b"voltage = 0.125000 V\r\nOK\r\n",
        "getfil 2": b"filter = 1 Hz\r\nOK\r\n",
        "geto 1": b"voltage = 2.345000 V\r\nOK\r\n",
        "getaci 1": b"AC-IN = off\r\nOK\r\n",
        "getdci 1": b"DC-IN = off\r\nOK\r\n",
    }


class ANC300ControllerTests(unittest.TestCase):
    def controller(self, server, **kwargs):
        return ANC300Controller("127.0.0.1", port=server.port, timeout_s=0.2, **kwargs)

    def test_connect_logs_in_disables_echo_and_reads_without_output_writes(self):
        """A connect mutation that writes an axis command must fail this test."""
        with FakeANC300Server(normal_replies()) as server:
            controller = self.controller(server)
            snapshot = controller.connect((1, 2))
            controller.disconnect()
        self.assertTrue(snapshot["connected"])
        self.assertEqual(snapshot["version"], "ANC300 version 3.4")
        self.assertEqual(snapshot["controller_serial"], "C123")
        self.assertEqual(snapshot["axes"][1]["serial"], "AX1")
        self.assertEqual(snapshot["axes"][1]["mode"], "off")
        self.assertEqual(snapshot["axes"][1]["offset"], 2.345)
        self.assertEqual(snapshot["axes"][1]["filter"], "16 Hz")
        self.assertEqual(snapshot["axes"][2]["offset"], 0.125)
        commands = server.received()
        self.assertEqual(commands[:2], ["123456", "echo off"])
        self.assertFalse(any(command.startswith(("setm", "seta", "stop")) for command in commands))

    def test_connect_sends_echo_before_waiting_for_the_login_response(self):
        """Waiting for a password-only reply must fail this console-compatible handshake."""
        replies = normal_replies()
        replies["123456"] = None
        replies["echo off"] = b"login accepted\r\nOK\r\n"
        with FakeANC300Server(replies) as server:
            controller = self.controller(server)
            snapshot = controller.connect((1,))
            controller.disconnect()
        self.assertTrue(snapshot["connected"])
        self.assertEqual(server.received()[:2], ["123456", "echo off"])

    def test_fragmented_multiline_responses_are_reassembled(self):
        """Dropping buffered partial bytes must fail this test."""
        replies = normal_replies()
        replies["ver"] = [b"ANC300 ver", b"sion 3.4\r\nO", b"K\r\n"]
        with FakeANC300Server(replies) as server:
            controller = self.controller(server)
            controller.connect((1,))
            self.assertEqual(controller.get_version(), "ANC300 version 3.4")
            controller.disconnect()

    def test_controller_error_raises_command_error_with_response_lines(self):
        """Ignoring a controller ERROR terminator must fail this test."""
        replies = normal_replies()
        replies["bad command"] = b"filter unavailable\r\nERROR\r\n"
        with FakeANC300Server(replies) as server:
            controller = self.controller(server)
            controller.connect((1,))
            with self.assertRaises(ANC300CommandError) as caught:
                controller.query("bad command")
            self.assertIn("bad command", str(caught.exception))
            self.assertTrue(controller.connected)
            controller.disconnect()

    def test_dead_peer_protocol_faults_close_and_clear_the_transport(self):
        """Keeping a desynchronized socket after partial or malformed data must fail."""
        for command, response in (
            ("partial", CloseAfterPayload(b"unterminated response")),
            ("malformed", b"\xff\r\nOK\r\n"),
        ):
            with self.subTest(command=command):
                replies = normal_replies()
                replies[command] = response
                with FakeANC300Server(replies) as server:
                    controller = self.controller(server)
                    controller.connect((1,))
                    with self.assertRaises(ANC300ProtocolError):
                        controller.query(command)
                    self.assertFalse(controller.connected)

    def test_local_non_ascii_command_is_rejected_without_closing_or_transmission(self):
        """A local encoding rejection must leave an otherwise healthy session connected."""
        with FakeANC300Server(normal_replies()) as server:
            controller = self.controller(server)
            controller.connect((1,))
            before = server.received()
            with self.assertRaises(ANC300ProtocolError):
                controller.query("非ASCII")
            self.assertTrue(controller.connected)
            self.assertEqual(server.received(), before)
            controller.disconnect()

    def test_timeout_and_abrupt_close_raise_connection_errors(self):
        """Treating a silent or closed peer as a valid response must fail this test."""
        replies = normal_replies()
        replies["silent"] = None
        replies["closed"] = "CLOSE"
        with FakeANC300Server(replies) as server:
            controller = self.controller(server)
            controller.connect((1,))
            with self.assertRaises(ANC300ConnectionError):
                controller.query("silent")
            controller.disconnect()
        with FakeANC300Server(replies) as server:
            controller = self.controller(server)
            controller.connect((1,))
            with self.assertRaises(ANC300ConnectionError):
                controller.query("closed")

    def test_incorrect_password_leaves_controller_disconnected(self):
        """Leaving a socket live after a failed handshake must fail this test."""
        replies = normal_replies()
        replies["bad"] = None
        replies["echo off"] = b"authentication failed\r\nERROR\r\n"
        with FakeANC300Server(replies) as server:
            controller = self.controller(server, password="bad")
            with self.assertRaises(ANC300ConnectionError):
                controller.connect((1,))
            self.assertFalse(controller.connected)
            self.assertEqual(server.received(), ["bad", "echo off"])

    def test_getters_parse_mode_voltage_filter_serial_and_version(self):
        """Returning raw protocol labels instead of values must fail this test."""
        with FakeANC300Server(normal_replies()) as server:
            controller = self.controller(server)
            controller.connect((1,))
            self.assertEqual(controller.get_mode(1), "off")
            self.assertEqual(controller.get_offset(1), 2.345)
            self.assertEqual(controller.get_filter(1), "16 Hz")
            self.assertEqual(controller.get_axis_serial(1), "AX1")
            self.assertEqual(controller.get_controller_serial(), "C123")
            self.assertEqual(controller.get_version(), "ANC300 version 3.4")
            controller.disconnect()

    def test_labeled_getters_reject_wrong_labels_and_extra_lines_and_close_transport(self):
        """Accepting a stale/wrong labeled line or ignoring an extra line must fail."""
        cases = (
            ("getm 1", b"filter = off\r\nOK\r\n", "get_mode"),
            ("getfil 1", b"mode = 16 Hz\r\nOK\r\n", "get_filter"),
            ("getser 1", b"controller serial number = AX1\r\nOK\r\n", "get_axis_serial"),
            ("geta 1", b"voltage = 2 V\r\nunexpected\r\nOK\r\n", "get_offset"),
            ("ver", b"ANC300 version 3.4\r\nunexpected\r\nOK\r\n", "get_version"),
        )
        for command, response, method_name in cases:
            with self.subTest(command=command):
                replies = normal_replies()
                with FakeANC300Server(replies) as server:
                    controller = self.controller(server)
                    controller.connect((1,))
                    server.replies[command] = response
                    with self.assertRaises(ANC300ProtocolError):
                        getattr(controller, method_name)(1) if method_name != "get_version" else controller.get_version()
                    self.assertFalse(controller.connected)

    def test_output_and_input_state_apis_parse_only_explicit_labeled_values(self):
        """Missing geto/getaci/getdci support or permissive input-state parsing must fail."""
        replies = normal_replies()
        with FakeANC300Server(replies) as server:
            controller = self.controller(server)
            controller.connect((1,))
            self.assertEqual(controller.get_output(1), 2.345)
            self.assertFalse(controller.get_ac_input(1))
            self.assertFalse(controller.get_dc_input(1))
            controller.set_ac_input(1, False)
            controller.set_dc_input(1, False)
            self.assertIn("setaci 1 off", server.received())
            self.assertIn("setdci 1 off", server.received())
            server.replies["getaci 1"] = b"AC-IN = maybe\r\nOK\r\n"
            with self.assertRaises(ANC300ProtocolError):
                controller.get_ac_input(1)
            self.assertFalse(controller.connected)

    def test_set_offset_enforces_hard_physical_bounds_without_transmission(self):
        """Sending a locally invalid voltage outside 0-150 V must fail this boundary test."""
        with FakeANC300Server(normal_replies()) as server:
            controller = self.controller(server)
            controller.connect((1,))
            before = server.received()
            for value in (-1e-12, 150.000000000001, float("nan"), float("inf")):
                with self.subTest(value=value), self.assertRaises(ValueError):
                    controller.set_offset(1, value)
            self.assertEqual(server.received(), before)
            controller.set_offset(1, 0.0)
            controller.set_offset(1, 150.0)
            self.assertEqual(server.received()[-2:], ["seta 1 0", "seta 1 150"])
            controller.disconnect()

    def test_snapshot_command_error_keeps_command_error_type_and_closes_transport(self):
        """Collapsing an authenticated snapshot ERROR into a connection error must fail."""
        replies = normal_replies()
        replies["geta 2"] = b"offset unavailable\r\nERROR\r\n"
        with FakeANC300Server(replies) as server:
            controller = self.controller(server)
            with self.assertRaises(ANC300CommandError):
                controller.connect((1, 2))
            self.assertFalse(controller.connected)

    def test_malformed_voltage_and_invalid_response_encoding_raise_protocol_error(self):
        """Accepting malformed protocol content must fail this test."""
        replies = normal_replies()
        replies["geta 1"] = b"voltage = uncertain\r\nOK\r\n"
        with FakeANC300Server(replies) as server:
            controller = self.controller(server)
            with self.assertRaises(ANC300ProtocolError):
                controller.connect((1,))
            self.assertFalse(controller.connected)
        replies = normal_replies()
        replies["ver"] = b"\xff\r\nOK\r\n"
        with FakeANC300Server(replies) as server:
            controller = self.controller(server)
            with self.assertRaises(ANC300ProtocolError):
                controller.connect((1,))
            self.assertFalse(controller.connected)

    def test_offset_mode_uses_off_and_caches_offs_fallback_only_after_error(self):
        """Retrying a non-offset command or forgetting the accepted token must fail this test."""
        replies = normal_replies()
        replies["setm 1 off"] = b"unknown mode\r\nERROR\r\n"
        replies["setm 1 offs"] = b"OK\r\n"
        replies["seta 1 1.5"] = b"OK\r\n"
        replies["stop 1"] = b"OK\r\n"
        with FakeANC300Server(replies) as server:
            controller = self.controller(server)
            controller.connect((1,))
            controller.set_mode(1, "off")
            controller.set_mode(1, "off")
            controller.set_offset(1, 1.5)
            controller.stop(1)
            controller.disconnect()
        self.assertEqual(server.received().count("setm 1 off"), 1)
        self.assertEqual(server.received().count("setm 1 offs"), 2)
        self.assertIn("seta 1 1.5", server.received())
        self.assertIn("stop 1", server.received())

    def test_axis_validation_rejects_out_of_range_axis_without_sending_it(self):
        """Permitting an out-of-range axis command must fail this test."""
        with FakeANC300Server(normal_replies()) as server:
            controller = self.controller(server)
            controller.connect((1,))
            with self.assertRaises(ValueError):
                controller.get_mode(8)
            controller.disconnect()
        self.assertNotIn("getm 8", server.received())

    def test_axis_three_write_commands_are_rejected_without_transmission(self):
        """Allowing any output-changing command on protected axis 3 must fail this test."""
        with FakeANC300Server(normal_replies()) as server:
            controller = self.controller(server)
            controller.connect((1,))
            with self.assertRaises(ValueError):
                controller.set_mode(3, "off")
            with self.assertRaises(ValueError):
                controller.set_offset(3, 1.5)
            with self.assertRaises(ValueError):
                controller.stop(3)
            controller.disconnect()
        commands = server.received()
        self.assertFalse(any(command.startswith(("setm 3", "seta 3", "stop 3")) for command in commands))

    def test_queries_from_threads_are_not_interleaved(self):
        """Releasing the transaction lock before the terminator must fail this test."""
        replies = normal_replies()
        held_response = HeldFragmentedResponse()
        replies["first"] = held_response
        replies["second"] = b"second reply\r\nOK\r\n"
        with FakeANC300Server(replies) as server:
            controller = self.controller(server)
            controller.connect((1,))
            results = []

            def run(command):
                results.append(controller.query(command))

            first = threading.Thread(target=run, args=("first",))
            first.start()
            self.assertTrue(held_response.fragment_sent.wait(0.5))
            second = threading.Thread(target=run, args=("second",))
            second.start()
            first.join(1)
            second.join(1)
            controller.disconnect()
        commands = server.received()
        self.assertEqual(len(results), 2)
        self.assertEqual(sorted(results), [["first reply"], ["second reply"]])
        self.assertFalse(held_response.interleaved_write.is_set())
        self.assertEqual(commands[-2:], ["first", "second"])

    def test_constructor_rejects_empty_host_and_nonpositive_timeout_before_connecting(self):
        """Opening a connection for invalid constructor parameters must fail this test."""
        with self.assertRaises(ValueError):
            ANC300Controller("", timeout_s=1)
        with self.assertRaises(ValueError):
            ANC300Controller("127.0.0.1", timeout_s=0)


if __name__ == "__main__":
    unittest.main(verbosity=2)
