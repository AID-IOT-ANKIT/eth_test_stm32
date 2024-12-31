# import socket
# import struct
# from time import sleep

# # import 

# from pymodbus.client.tcp import ModbusTcpClient as pyTcp
# from pymodbus.payload import BinaryPayloadDecoder, BinaryPayloadBuilder

# def pretty_hex(data: bytes, label: str = "Data"):
#     """Print bytes in a readable hex format."""
#     print(f"{label} (length: {len(data)}):")
#     print(" ".join(f"{byte:02X}" for byte in data))
#     print("-" * 50)

# def create_modbus_request(angle: float):
#     # MBAP Header
#     transaction_id = b'\x00\x01'  # Transaction ID (arbitrary)
#     protocol_id = b'\x00\x00'     # Protocol ID (always 0 for Modbus TCP)
#     unit_id = b'\x01'             # Unit Identifier (default is 1)

#     # PDU (Protocol Data Unit)
#     function_code = b'\x10'       # Function code (Write Multiple Registers)
#     starting_address = b'\x00\x64'  # Starting address (register 100)
#     quantity_of_registers = b'\x00\x02'  # Quantity of registers (2 registers for 32-bit float)
#     byte_count = b'\x04'          # Byte count (4 bytes for two 16-bit registers)
    
#     # Convert 30.2 to IEEE 754 single precision (4 bytes)
#     ieee_float = struct.pack('>f', angle)  # '>f' means big-endian float
    
#     # Unpack the IEEE 754 float to get the two 16-bit register values
#     register_value_high, register_value_low = struct.unpack('>HH', ieee_float)
    
#     # PDU with the two registers representing 30.2
#     pdu = function_code + starting_address + quantity_of_registers + byte_count
#     pdu += struct.pack('>H', register_value_high)  # First register (high)
#     pdu += struct.pack('>H', register_value_low)   # Second register (low)

#     # Length (PDU length + Unit ID)
#     length = len(pdu) + 1
#     length_bytes = length.to_bytes(2, byteorder='big')

#     # Combine MBAP Header
#     mbap_header = transaction_id + protocol_id + length_bytes + unit_id

#     # Full Modbus TCP Request
#     request = mbap_header + pdu
#     return request

# def send_modbus_request(angle: float):
#     request = create_modbus_request(angle=angle)

#     pretty_hex(bytes.fromhex(request.hex()), "Request");
#     # Connect to the Modbus TCP server
#     with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client:
#         client.connect(("192.168.1.184", 502)) # STM32 modbus server IP
#         client.sendall(request)
#         response = client.recv(1024)
#         pretty_hex(bytes.fromhex(response.hex()), "Response")

# # Test angles
# angles = [-180.5, -90.25, -45.5, 0.0, 45.5, 90.25, 180.5]

# for angle in angles:
#     print("Angle:", angle);
#     send_modbus_request(angle=angle)
#     sleep(1);

from pymodbus.client import ModbusTcpClient
from pymodbus.constants import Endian
import logging
from time import sleep

# Enable logging for debugging
logging.basicConfig(level=logging.INFO)
logger = logging.getLogger("Modbus TCP")

# Server details
SERVER_IP = "192.168.1.184"  # STM32 Modbus TCP IP
SERVER_PORT = 502            # Default Modbus TCP port

# Register details
REGISTER_START = 0x22  # Starting register address
SLAVE_ID = 1          # Unit ID of the Modbus device


def write_float_to_register(client: ModbusTcpClient, address, value):
    payload = client.convert_to_registers(value=value, data_type=client.DATATYPE.FLOAT32)

    logger.info(f"Angle = {value}")

    response = client.write_registers(address, payload, slave=SLAVE_ID)
    if response.isError():
        logger.error(f"Error writing to register {address}: {response}")
    else:
        logger.info(f"Successfully wrote {value} to register {address}")


def read_float_from_register(client: ModbusTcpClient, address):
    response = client.read_holding_registers(REGISTER_START, count=2, slave=1)

    if response.isError():
        logger.error(f"Error reading from register {address}: {response}")
        return None
    else:
        value = client.convert_from_registers(registers=response.registers, data_type=client.DATATYPE.FLOAT32)
        return value
    

def rotate_to_angle(client: ModbusTcpClient, value: float):
    if not client.connect():
        logger.error(f"Failed to connect to Modbus server at {SERVER_IP}:{SERVER_PORT}")
    else:
        try:
            write_float_to_register(client, REGISTER_START, value=value)
        finally:
            client.close()

def read_test(client: ModbusTcpClient, expectedValue: float):
    if not client.connect():
        logger.error(f"Failed to connect to Modbus server at {SERVER_IP}:{SERVER_PORT}")
    else:
        try:
            value = read_float_from_register(client=client,address=REGISTER_START)
            logger.info(f"Expected= {expectedValue} Got= {value}")
            assert expectedValue == value
            
        finally:
            client.close()

# Test angles
angles = [-180.5, -90.25, -45.5, 0.0, 45.5, 90.25, 180.5]

if __name__ == "__main__":
    client = ModbusTcpClient(SERVER_IP, port=SERVER_PORT)
    logger.info("Rotation Test")
    for angle in angles:
        rotate_to_angle(client=client,value=angle)
        sleep(1)
    
    # check for last value
    print()
    logger.info("Read Test")
    read_test(client=client,expectedValue=angles[-1])
    

