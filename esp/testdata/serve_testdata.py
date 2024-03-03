#!/usr/bin/env python3

# Run `./serve_testdata.py` within the `testdata/` directory to serve the
# test data files over HTTP, such that the HTTP response headers are always
# deterministically the same, thereby allowing us to calculate an expected
# CRC32 hash over the whole HTTP response, incl. the response headers. See
# ./calculate_crc32.py.

import http.server
import socketserver

PORT = 8080

class Handler(http.server.SimpleHTTPRequestHandler):
  def send_header(self, header, value):
    # Skip these headers to ensure the response is always deterministically the
    # same.
    if header in ('Date', 'Server', 'Last-Modified'):
      return

    super().send_header(header, value)

with socketserver.TCPServer(("", PORT), Handler) as httpd:
  print("serving at port", PORT)
  httpd.serve_forever()
