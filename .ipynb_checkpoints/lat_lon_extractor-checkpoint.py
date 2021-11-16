import rosbag
import pandas as pd

# from sensor_msgs.msg import (
#    Range,
#    FluidPressure,
#    NavSatFix,
#    Temperature,
#    RelativeHumidity,
# )
# from geometry_msgs.msg import QuaternionStamped
# from cola2_msgs.msg import Setpoints

from tf.transformations import euler_from_quaternion
import json
from pathlib import Path
import argparse
import math
import datetime

from plotly.subplots import make_subplots
import plotly.graph_objects as go

import numpy as np
import os
import csv

topics = [
    "/turbot/actuators/thrusters_info",
    "/turbot/adis_imu/depth",
    "/turbot/navigator/imu",
    "/turbot/adis_imu/fluid_pressure",
    "/turbot/controller/thruster_setpoints",
    "/turbot/emus_bms/battery_state",
    "/turbot/imagenex_echosounder_down/profile_range_raw",
    "/turbot/imagenex_echosounder_down/profile_rx_raw",
    "/turbot/modem_raw",
    "/turbot/modem_delayed",
    "/turbot/navigator/gps_raw",
    "/turbot/navigator/imu_raw",
    "/turbot/teledyne_explorer_dvl/altitude",
    "/turbot/teledyne_explorer_dvl/data",
    "/turbot/teledyne_explorer_dvl/dvl",
    "/xiroi/usbllong",
    "/turbot/navigator/ekf_map/odometry",
    "/turbot/navigator/ekf_odom/odometry",
    "/turbot/navigator/navigation",
]

class Category:
    POSITION = "position"
    ORIENTATION = "orientation"
    VELOCITY = "velocity"
    DEPTH = "depth"
    ALTITUDE = "altitude"
    USBL = "usbl"
    TIDE = "tide"
    THRUSTER = "thruster"
    TEMPERATURE = "temperature"
    HUMIDITY = "humidity"

class Timestamp:
    def __init__(self):
        self.epoch_timestamp = None


class Position(Timestamp):
    def __init__(self):
        Timestamp.__init__(self)
        self.latitude = None
        self.longitude = None


def stamp_to_epoch(stamp):
    s = float(stamp.secs)
    ns = float(stamp.nsecs)
    return s + ns * 1e-9


def gps_to_json(msg):
    x = Position()
    x.epoch_timestamp = stamp_to_epoch(msg.header.stamp)
    x.latitude = msg.latitude
    x.longitude = msg.longitude
    json_output = {
        "epoch_timestamp": x.epoch_timestamp,
        "class": "measurement",
        "sensor": "driftcam",
        "data": [
            {
                "latitude": x.latitude,
                "latitude_std": 0,
            },
            {
                "longitude": x.longitude,
                "longitude_std": 0,
            },
        ],
    }
    return json_output, x


def parse_bagfile(bagfile, force_overwrite):
    # Check bagfile exists
    bagfile_fn = Path(bagfile)
    if not bagfile_fn.exists():
        print("The bagfile does not exist. Please check the path provided.")

    # Check if output file exists:
    json_fn = bagfile_fn.with_suffix(".json")

    if json_fn.exists() and not force_overwrite:
        print(
            "The file "
            + str(json_fn.name)
            + " already exists at the destination folder. Use the force flag (-F) to overwrite the file if you wish."
        )
        print("Exitting...")
        return

    position_list = []

    json_output = []
    print("Opening bagfile. This can take a while...")
    bag = rosbag.Bag(bagfile)
    for topic, msg, t in bag.read_messages(topics=topics):
        json_data = None
        if "/turbot/navigator/gps_raw" in topic:
            json_data, x = gps_to_json(msg)
            position_list.append(x)

        if json_data is not None:
            json_output.append(json_data)

    bag.close()

    print("Bag parsed! Now working on the plots...")

    export_to_csv(position_list,bagfile)

def export_to_csv(position_list,bagfile):
        print("WRITING_CSV")
        header=["Longitude","latitude"]
        bagfile_fn = Path(bagfile)
        if not bagfile_fn.exists():
            print("The bagfile does not exist. Please check the path provided.")

        bagfile_path=os.path.split(str(bagfile_fn))[0]    

        csv_file=os.path.join(str(bagfile_path)+'lat_lon.csv')
        print("bagfile_fn is :",bagfile_fn)
        # if not (os.path.exists(csv_file)):
        #     os.mkdir(csv_file)
        print("witing data: in file: ",csv_file)

        with open(csv_file, 'a+') as file:
            writer = csv.writer(file, delimiter=';')
            writer.writerow(header)
            for data in position_list:
                writer.writerow([data.longitude,data.latitude])
        file.close()

if __name__ == "__main__":
    parser = argparse.ArgumentParser(
        description="Parses a bagfile to json format compatible with auv_nav"
    )
    parser.add_argument(
        "bagfile",
        help="Bagfile to process.",
    )
    parser.add_argument(
        "-F",
        "--Force",
        dest="force",
        action="store_true",
        help="Force file overwite",
    )

    args = parser.parse_args()
    parse_bagfile(args.bagfile, args.force)