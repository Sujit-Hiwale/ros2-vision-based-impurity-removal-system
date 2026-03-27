import serial
import time

PORT = "/dev/ttyUSB0"
BAUD = 115200


def send(ser, cmd):
    ser.write((cmd + "\n").encode())
    ser.flush()
    print("Sent:", cmd)
    time.sleep(0.3)   # allow firmware to process


def main():

    ser = serial.Serial(PORT, BAUD, timeout=1)
    time.sleep(2)

    print("Connected")

    # IMPORTANT: select electric gripper
    send(ser, "SetGripperType 1")

    time.sleep(1)

    try:
        while True:

            print("Closing gripper")
            send(ser, "grip")

            time.sleep(1)

            print("Opening gripper")
            send(ser, "release")

            time.sleep(1)

    except KeyboardInterrupt:
        print("Stopping")

    ser.close()


if __name__ == "__main__":
    main()