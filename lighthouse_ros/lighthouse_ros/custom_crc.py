import zlib
from typing import Union

class CRC32Context:
    """
    CRC32 context to hold the state of the checksum calculation.
    """
    def __init__(self):
        self.remainder = 0xFFFFFFFF

    def reset(self):
        """Reset the CRC context to its initial state."""
        self.remainder = 0xFFFFFFFF

def crc32_context_init(context: CRC32Context):
    """
    Initialize the CRC32 context.

    :param context: Context to initialize
    """
    context.reset()

def crc32_update(context: CRC32Context, data: Union[bytes, bytearray]):
    """
    Update checksum with new data.

    :param context: An initialized context
    :param data: Data buffer to add to the checksum
    """
    for byte in data:
        context.remainder = _crc_byte(byte, context.remainder)

def crc32_out(context: CRC32Context) -> int:
    """
    Generate and return the CRC32 checksum.

    :param context: An initialized context
    :return: The CRC32 checksum
    """
    return context.remainder ^ 0xFFFFFFFF

def crc32_calculate_buffer(buffer: Union[bytes, bytearray]) -> int:
    """
    Calculate the checksum of a buffer in one call.

    :param buffer: Data buffer to checksum
    :return: The CRC32 checksum
    """
    context = CRC32Context()
    crc32_context_init(context)
    crc32_update(context, buffer)
    return crc32_out(context)

# Precomputed CRC table (lazy initialization)
_crc_table = None

def _init_crc_table():
    """
    Initialize the CRC table if not already initialized.
    """
    global _crc_table
    if _crc_table is None:
        _crc_table = []
        for dividend in range(256):
            remainder = dividend
            for _ in range(8):
                if remainder & 1:
                    remainder = (remainder >> 1) ^ 0xEDB88320
                else:
                    remainder >>= 1
            _crc_table.append(remainder)

def _crc_byte(byte: int, remainder: int) -> int:
    """
    Calculate the CRC32 for a single byte.

    :param byte: The byte to process
    :param remainder: The current CRC remainder
    :return: Updated CRC remainder
    """
    global _crc_table
    _init_crc_table()

    index = (remainder ^ byte) & 0xFF
    return _crc_table[index] ^ (remainder >> 8)
