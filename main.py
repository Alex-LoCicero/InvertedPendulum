from utils.serial_listener import SerialListener
from utils.data_parser import DataParser
from utils.live_plotter import LivePlotter
import sys

def main():
    port = "COM3"  # Replace with your Arduino's serial port
    baudrate = 9600

    listener = SerialListener(port, baudrate)
    parser = DataParser()
    plotter = LivePlotter()

    listener.connect()

    try:
        while True:
            if listener.available():
                raw_data = listener.read_data()
                parsed_data = parser.parse(raw_data)
                print(f"Parsed data: {parsed_data}")
                if parsed_data:
                    plotter.update(parsed_data)
                    
    except KeyboardInterrupt:
        print("Stopping...")
        
    finally:
        plotter.show()
        listener.close()

if __name__ == "__main__":
    main()