#!/usr/bin/env python3

# Example usage:
#
# ./calculate_crc32.py 10kB.txt
#
# This will print the CRC32 value of the expected HTTP response (incl. HTTP
# response headers) of the 10kB.txt file, when served using ./testserver.py.
# This can be used to confirm that the `smoltcp_httpclient` example binary
# received the correct data served by the `./serve_testdata.py` script.

import zlib
import sys

with open(sys.argv[1], 'rb') as f:
    file_data = f.read()
    file_length = len(file_data)
    data = (b"HTTP/1.0 200 OK\r\n"
            b"Content-type: text/plain\r\n"
            b"Content-Length: " + bytes(str(file_length), "utf-8") + b"\r\n\r\n" +
            file_data)

    print(zlib.crc32(data))
