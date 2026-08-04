import serial
import time


class DACController:
    def __init__(self, port: str, baud: int = 115200, timeout: float = 2.0):
        self.ser = serial.Serial(port, baud, timeout=timeout)
        time.sleep(2)

        # Scan for READY rather than demanding it on the first line. Opening the
        # port toggles DTR and resets the board, but data the board sent BEFORE
        # that reset is still buffered by the OS and arrives first -- so if the
        # previous session left it streaming, line 1 is a stale half-sample and
        # a healthy board looks dead. READY is emitted after the reset, so it is
        # always at the end of that backlog. Same bounded-scan shape as
        # set_voltage(), for the same reason: this wire carries two conversations.
        for _ in range(200):
            if self.ser.readline().decode(errors="replace").strip() == "READY":
                break
        else:
            raise RuntimeError(
                f"Arduino never sent READY on {port} (scanned 200 lines). "
                "Wrong port, board not flashed, or wrong baud.")

        # Land in a known state: the reset should have cleared `streaming`, but
        # say so explicitly and drop anything still in flight.
        self.ser.write(b"STOP\n")
        time.sleep(0.2)
        self.ser.reset_input_buffer()
        print(f"Connected on {port}")

    def set_voltage(self, channel: int, voltage: float) -> str:
        if not (0 <= channel <= 7):
            raise ValueError("Channel must be 0–7")
        if not (0.0 <= voltage <= 2.5):
            raise ValueError("Voltage must be 0.0–2.5 V")
        cmd = f"SET {channel} {voltage:.4f}\n"
        self.ser.write(cmd.encode())

        # Skip any stray streaming data lines until we see the real reply.
        for _ in range(50):  # bounded, so we can't hang forever
            response = self.ser.readline().decode(errors="replace").strip()
            if response.startswith("OK") or response.startswith("ERR"):
                break
        else:
            raise RuntimeError(f"No OK/ERR reply seen for SET {channel} {voltage:.4f} "
                            f"(serial may be desynced)")

        if response.startswith("ERR"):
            print(f"    !! DAC REJECTED: ch={channel} v={voltage:.4f} -> {response}")
        else:
            print(f"    {response}")
        return response

    def start_stream(self) -> str:
        self.ser.reset_input_buffer()
        self.ser.write(b"STREAM\n")
        response = self.ser.readline().decode().strip()
        print(f"    {response}")
        return response

    def stop_stream(self) -> str:
        self.ser.write(b"STOP\n")
        response = self.ser.readline().decode().strip()
        print(f"    {response}")
        return response

    def close(self):
        self.ser.close()
        print("Connection closed.")
