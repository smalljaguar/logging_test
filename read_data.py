import struct
import csv

bno055_format = (
    "18f"   # 3 floats each for orientation, angular_velocity, linear_acceleration
            # magnetometer, accelerometer, gravity (6*3=18 floats in total)
    "4B"    # 4 unsigned bytes for calibration
    "b"     # 1 signed byte for board_temperature
    "3x"    # 3 bytes padding for 4-byte alignment
)

dps310_format = "2f" # 2 floats for temperature and pressure

bmp280_format = "3f" # 3 floats for temperature, pressure, and altitude

flight_data_format = (
    "<L"    # Unsigned long for timestamp
    + bno055_format
    + dps310_format
    + bmp280_format
)

def format_data(parsed_data):
# Organize the parsed data for clarity
    return {
        "timestamp": parsed_data[0],
        "bno055": {
            "orientation": parsed_data[1:4],
            "angular_velocity": parsed_data[4:7],
            "linear_acceleration": parsed_data[7:10],
            "magnetometer": parsed_data[10:13],
            "accelerometer": parsed_data[13:16],
            "gravity": parsed_data[16:19],
            "calibration": parsed_data[19:23],
            "board_temperature": parsed_data[23],
        },
        "dps310": {
            "temperature": parsed_data[24],
            "pressure": parsed_data[25],
        },
        "bmp280": {
            "temperature": parsed_data[26],
            "pressure": parsed_data[27],
            "altitude": parsed_data[28],
        },
    }


def read_raw(file_path):
    flight_data_size = struct.calcsize(flight_data_format)
    flight_data = []
    with open(file_path, 'rb') as f:
        chunk = f.read(flight_data_size)
        while chunk:
            if len(chunk) != flight_data_size:
                raise ValueError("Incomplete data chunk")
            parsed_data = struct.unpack(flight_data_format, chunk)
            formatted_data = format_data(parsed_data)
            flight_data.append(formatted_data)

    return flight_data

def cleanup_row(row):
    # Convert strings to floats, pack x,y,z values into tuples
    return {
        "timestamp": float(row["Timestamp"]),
        "bno055": {
            "orientation": (float(row["BNO055_Orientation_X"]), float(row["BNO055_Orientation_Y"]), float(row["BNO055_Orientation_Z"])),
            "angular_velocity": (float(row["BNO055_AngularVelocity_X"]), float(row["BNO055_AngularVelocity_Y"]), float(row["BNO055_AngularVelocity_Z"])),
            "linear_acceleration": (float(row["BNO055_LinearAccel_X"]), float(row["BNO055_LinearAccel_Y"]), float(row["BNO055_LinearAccel_Z"])),
            "magnetometer": (float(row["BNO055_Magnetometer_X"]), float(row["BNO055_Magnetometer_Y"]), float(row["BNO055_Magnetometer_Z"])),
            "accelerometer": (float(row["BNO055_Accelerometer_X"]), float(row["BNO055_Accelerometer_Y"]), float(row["BNO055_Accelerometer_Z"])),
            "gravity": (float(row["BNO055_Gravity_X"]), float(row["BNO055_Gravity_Y"]), float(row["BNO055_Gravity_Z"])),
            "calibration": (int(row["BNO055_Calib_System"]), int(row["BNO055_Calib_Gyro"]), int(row["BNO055_Calib_Accel"]), int(row["BNO055_Calib_Mag"])),
            "board_temperature": float(row["BNO055_BoardTemp"]),
        },
        "dps310": {
            "temperature": float(row["DPS310_Temperature"]),
            "pressure": float(row["DPS310_Pressure"]),
        },
        "bmp280": {
            "temperature": float(row["BMP280_Temperature"]),
            "pressure": float(row["BMP280_Pressure"]),
            "altitude": float(row["BMP280_Altitude"]),
        },
    }

def read_csv(file_path):
    # header for reference:
    """
        "Timestamp," +
        "BNO055_Orientation_X,BNO055_Orientation_Y,BNO055_Orientation_Z," +
        "BNO055_AngularVelocity_X,BNO055_AngularVelocity_Y,BNO055_AngularVelocity_Z," +
        "BNO055_LinearAccel_X,BNO055_LinearAccel_Y,BNO055_LinearAccel_Z," +
        "BNO055_Magnetometer_X,BNO055_Magnetometer_Y,BNO055_Magnetometer_Z," +
        "BNO055_Accelerometer_X,BNO055_Accelerometer_Y,BNO055_Accelerometer_Z," +
        "BNO055_Gravity_X,BNO055_Gravity_Y,BNO055_Gravity_Z," +
        "BNO055_Calib_System,BNO055_Calib_Gyro,BNO055_Calib_Accel,BNO055_Calib_Mag," +
        "BNO055_BoardTemp," +
        "DPS310_Temperature,DPS310_Pressure," +
        "BMP280_Temperature,BMP280_Pressure,BMP280_Altitude";
    """
    
    csv_reader = csv.DictReader(open(file_path))
    return [cleanup_row(row) for row in csv_reader]

def read_json(file_path):
    import json
    with open(file_path) as f:
        return json.load(f)