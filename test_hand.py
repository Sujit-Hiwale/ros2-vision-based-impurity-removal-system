import serial
import time

PORT = "/dev/ttyUSB0"
BAUD = 115200

ser = serial.Serial(PORT, BAUD, timeout=1)
time.sleep(2)

def send(cmd):
    ser.write((cmd + "\n").encode())
    ser.flush()
    print("Sent:", cmd)

# optional initialization
send("StopStepper")
send("SetSpeed 1000")
send("SetAcceleration 500")
send("Forward")
send("StartStepper")

time.sleep(2)

# move robot
for x in range(-60, 60, 20):
    cmd = f"IK [{x},0,-40]"
    send(cmd)
    time.sleep(1)

ser.close()