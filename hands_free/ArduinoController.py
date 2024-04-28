import serial
from serial.tools import list_ports

class ArduinoController:
    instance = None
    def __init__(self, port = None, baud_rate = 9600, timeout = 10):
        if self.instance == None:
            self.port = port if port != None else self.getSerialPort()
            if self.port:
                print(f"Found port: {self.port}")
                self.connection = serial.Serial(port=self.port, baudrate=baud_rate, timeout=timeout)
            self.instance = self
        
    

    def write(self, command):
        self.connection.write(str.encode(command, "utf-8"))

    def read(self):
        try:
            data = self.connection.readline().decode("utf-8").strip()
            return data
        except UnicodeDecodeError:
            print(f"Error decoding data as UTF-8. Trying raw bytes.")
            data = self.connection.readline()  # Read as raw bytes
            # Implement additional handling or interpretation of raw data here
            # (e.g., check for specific byte patterns, define a custom decoding logic)
            return data
        except Exception as error:
            print(f"Error reading from serial: {error}")
            return None

    def disconnect(self):
        if self.connection.is_open:
            self.connection.close()
        return


    def getSerialPort(self):
        ports = list_ports.comports()
        for port in ports:
            if port.device != "/dev/ttyAMA0" and not "USB" in port.device:
                return port.device
        return None
