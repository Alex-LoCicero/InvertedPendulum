import serial

class SerialListener:
    def __init__(self, port, baudrate=9600, timeout=1):
        self.port = port
        self.baudrate = baudrate
        self.timeout = timeout
        self.serial_connection = None

    def connect(self):
        try:
            self.serial_connection = serial.Serial(self.port, self.baudrate, timeout=self.timeout)
            print(f"Connected to {self.port} at {self.baudrate} baud.")
        except serial.SerialException as e:
            print(f"Error connecting to {self.port}: {e}")

    def read_data(self) -> str:
        if self.serial_connection.in_waiting > 0:
            try:
                raw_data = self.serial_connection.readline()
                return raw_data.decode('utf-8', errors='ignore').strip()
            except UnicodeDecodeError as e:
                print(f"Decoding error: {e}")
                return None

    def available(self) -> bool:
        return self.serial_connection.in_waiting > 0

    def close(self):
        if self.serial_connection:
            self.serial_connection.close()
            print("Serial connection closed.")