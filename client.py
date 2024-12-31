import socket
import struct
from time import sleep

def pretty_hex(data: bytes, label: str = "Data"):
    """Print bytes in a readable hex format."""
    print(f"{label} (length: {len(data)}):")
    print(" ".join(f"{byte:02X}" for byte in data))
    print("-" * 50)

def create_modbus_request(angle: float):
    # MBAP Header
    transaction_id = b'\x00\x01'  # Transaction ID (arbitrary)
    protocol_id = b'\x00\x00'     # Protocol ID (always 0 for Modbus TCP)
    unit_id = b'\x01'             # Unit Identifier (default is 1)

    # PDU (Protocol Data Unit)
    function_code = b'\x10'       # Function code (Write Multiple Registers)
    starting_address = b'\x00\x64'  # Starting address (register 100)
    quantity_of_registers = b'\x00\x02'  # Quantity of registers (2 registers for 32-bit float)
    byte_count = b'\x04'          # Byte count (4 bytes for two 16-bit registers)
    
    # Convert 30.2 to IEEE 754 single precision (4 bytes)
    ieee_float = struct.pack('>f', angle)  # '>f' means big-endian float
    
    # Unpack the IEEE 754 float to get the two 16-bit register values
    register_value_high, register_value_low = struct.unpack('>HH', ieee_float)
    
    # PDU with the two registers representing 30.2
    pdu = function_code + starting_address + quantity_of_registers + byte_count
    pdu += struct.pack('>H', register_value_high)  # First register (high)
    pdu += struct.pack('>H', register_value_low)   # Second register (low)

    # Length (PDU length + Unit ID)
    length = len(pdu) + 1
    length_bytes = length.to_bytes(2, byteorder='big')

    # Combine MBAP Header
    mbap_header = transaction_id + protocol_id + length_bytes + unit_id

    # Full Modbus TCP Request
    request = mbap_header + pdu
    return request

def send_modbus_request(angle: float):
    request = create_modbus_request(angle=angle)

    pretty_hex(bytes.fromhex(request.hex()), "Request");
    # Connect to the Modbus TCP server
    with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as client:
        client.connect(("192.168.1.184", 502)) # STM32 modbus server IP
        client.sendall(request)
        response = client.recv(1024)
        pretty_hex(bytes.fromhex(response.hex()), "Response")

# Test angles
angles = [-180.5, -90.25, -45.5, 0.0, 45.5, 90.25, 180.5]

for angle in angles:
    print("Angle:", angle);
    send_modbus_request(angle=angle)
    sleep(1);

