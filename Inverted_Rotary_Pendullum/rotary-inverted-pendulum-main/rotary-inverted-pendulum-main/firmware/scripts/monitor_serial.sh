#!/bin/bash
# Simple serial monitor script
# Usage: ./monitor_serial.sh <port> <baud_rate> [duration_seconds]
#
# Backed by pyserial rather than stty: macOS's stty rejects the non-standard
# rates this project actually uses (500000 for RLControl and the test sketches, 2000000
# for LowLevelServer) with "tcsetattr: Invalid argument", and then cat happily
# prints garbage at whatever rate the port was left in.

set -euo pipefail

PORT=${1:-/dev/cu.usbserial-10}
BAUD=${2:-115200}
DURATION=${3:-10}

echo "Monitoring $PORT at $BAUD baud..."
echo "Waiting for Arduino to reset..."
echo "---"

python3 - "$PORT" "$BAUD" "$DURATION" <<'PY'
import sys, time
try:
    import serial
except ImportError:
    sys.exit("pyserial not installed: pip install pyserial")

port, baud, duration = sys.argv[1], int(sys.argv[2]), float(sys.argv[3])
try:
    ser = serial.Serial(port, baud, timeout=0.5)
except serial.SerialException as e:
    sys.exit(f"could not open {port}: {e}")

# Toggle DTR to force a clean reset, then read from the boot banner onwards.
ser.setDTR(False)
time.sleep(0.1)
ser.reset_input_buffer()
ser.setDTR(True)

end = time.monotonic() + duration
try:
    while time.monotonic() < end:
        line = ser.readline()
        if line:
            print(line.decode(errors="replace").rstrip())
            sys.stdout.flush()
except KeyboardInterrupt:
    pass
finally:
    ser.close()
PY

echo "---"
echo "Monitoring stopped."
