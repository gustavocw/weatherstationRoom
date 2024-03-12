from digi.xbee.devices import XBeeDevice
from time import sleep

# PORT XBEE
PORT = '/dev/ttyUSB0'
# BAUD RATE XBEE
BAUD_RATE = 115200

# PRINCIPAL FUNCTION TO SEND AND FORMAT THE MESSAGES
def data_receive_callback(xbee_message):
    data = xbee_message.data.decode('latin1').strip("#")
    print(f"Received data from XBee: {data}")  # print data to terminal
    sleep(1)

    # READ AND SHOW THE BATTERY VOLTAGE VALUES
    if "FTTech Wind Speed:" in data:
        speed = data.split("Wind Speed:")[1].strip()
        print(f"WindSpeed: {speed}")
        
    elif "FTTech Wind Direction:" in data:
        direction = data.split("Wind Direction:")[1].strip()
        print(f"WindDirection: {direction}")
        
    elif "FTTech Vibration:" in data:
        vibrations = data.split("Vibration:")[1].split(';')
        x_vib, y_vib, z_vib = vibrations
        print(f"VibrationX: {x_vib.strip()}; VibrationY: {y_vib.strip()}; VibrationZ: {z_vib.strip()}")

# THE MAIN FUNCTION
def run():
    # CONNECT XBEE
    device = XBeeDevice(PORT, BAUD_RATE)
    device.open()
    device.add_data_received_callback(data_receive_callback)

    try:
        while True:
            sleep(1)
    except KeyboardInterrupt:
        print("Received exit command. Closing...")
    finally:
        if device.is_open():
            device.close()

if __name__ == '__main__':
    run()
