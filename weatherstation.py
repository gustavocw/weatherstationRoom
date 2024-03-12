from digi.xbee.devices import XBeeDevice
import psycopg2
from time import sleep

# PORT XBEE
PORT = '/dev/ttyUSB0'
# BAUD RATE XBEE
BAUD_RATE = 115200

# PostgreSQL connection parameters
DB_NAME = "weatherstation"
DB_USER = "postgres"
DB_PASS = "U8I9O0P-"
DB_HOST = "localhost"
DB_PORT = "5432"

# Store the last received values
last_received_values = {
    'WindSpeed': None,
    'WindDirection': None,
    'Vibration': [None, None, None]
}

def connect_db():
    try:
        conn = psycopg2.connect(database=DB_NAME, user=DB_USER, password=DB_PASS, host=DB_HOST, port=DB_PORT)
        return conn
    except Exception as error:
        print(f"Error connecting to PostgreSQL: {error}")
        return None

def insert_data_to_db(conn):
    try:
        cursor = conn.cursor()
        windspeed = last_received_values['WindSpeed'] or 0
        winddirection = last_received_values['WindDirection'] or 0
        x_vib, y_vib, z_vib = (vib or 0 for vib in last_received_values['Vibration'])

        print(f"Inserting into DB: windspeed={windspeed}, winddirection={winddirection}, vibrations={x_vib}, {y_vib}, {z_vib}")

        cursor.execute("""
            INSERT INTO weather_data(type, windspeed, winddirection, vibration_x, vibration_y, vibration_z)
            VALUES(%s, %s, %s, %s, %s, %s)
        """, ('WeatherData', windspeed, winddirection, x_vib, y_vib, z_vib))
        
        conn.commit()
    except Exception as error:
        print(f"Error inserting data to PostgreSQL: {error}")


def data_receive_callback(xbee_message):
    global non_zero_vibration_received
    
    data = xbee_message.data.decode('latin1').strip("#")
    conn = connect_db()
    if not conn:
        return

    print(f"Received data from XBee: {data}")  # print data to terminal

    patterns = {
        "FTTech Wind Speed:": ("WindSpeed", lambda d: float(d)),
        "FTTech Wind Direction:": ("WindDirection", lambda d: float(d)),
        "FTTech Vibration:": ("Vibration", lambda d: list(map(float, d.split(';'))))
    }

    data_updated = False
    for pattern, (data_type, func) in patterns.items():
        if pattern in data:
            value = func(data.split(pattern)[1].strip())
            if data_type == 'Vibration':
                if any(v != 0 for v in value) or non_zero_vibration_received:
                    non_zero_vibration_received = True
                    if value != last_received_values[data_type]:
                        last_received_values[data_type] = value
                        data_updated = True
            else:
                if value != last_received_values[data_type]:
                    last_received_values[data_type] = value
                    data_updated = True

    if data_updated:
        insert_data_to_db(conn)

    conn.close()

def run():
    device = XBeeDevice(PORT, BAUD_RATE)
    try:
        device.open()
        device.add_data_received_callback(data_receive_callback)

        while True:
            sleep(5)

    except KeyboardInterrupt:
        print("Received exit command. Closing...")
    except Exception as e:
        print(f"Error: {e}")
    finally:
        if device.is_open():
            device.close()

if __name__ == '__main__':
    run()
