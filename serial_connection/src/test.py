import serial

ser = serial.Serial('/dev/ttyACM0', 9600)

if __name__ == "__main__":
    while(True):
        string = ser.readline().decode('utf-8')
        print(string)