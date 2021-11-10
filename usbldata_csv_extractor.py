#!/usr/bin/env python3

import rosbag
import pandas as pd
from tf.transformations import euler_from_quaternion
import json
from pathlib import Path
import argparse
import math
import datetime
import os 
from plotly.subplots import make_subplots
import plotly.graph_objects as go

import numpy as np

topics=["/xiroi/usbllong",
        "/turbot/navigator/navigation",
        "/xiroi/navigator/navigation",
        "/turbot/modem_delayed"]

class Timestamp:
    def __init__(self):
        self.epoch_timestamp = None

class Nav_status(Timestamp):
    def __init__(self):
        Timestamp.__init__(self)
        self.origin_lat=None
        self.origin_lon=None
        
        self.lat=None
        self.lon=None

        self.position_north = None
        self.position_east = None
        self.position_depth = None

        self.altitude = None

        self.orientation_roll = None
        self.orientation_pitch = None
        self.orientation_yaw = None


class USBLlong(Timestamp):
    def __init__(self):
        Timestamp.__init__(self)
        self.X=None
        self.Y=None
        self.Z=None

        self.N=None
        self.E=None
        self.D=None

        self.roll=None
        self.pitch=None
        self.yaw=None

class Modem(Timestamp):
    def __init__(self):
        Timestamp.__init__(self)
        self.position_x=None
        self.position_y=None
        self.position_z=None
        self.orientation_x=None
        self.orientation_y=None
        self.orientation_z=None
        self.orientation_w=None

def stamp_to_epoch(stamp):
    s = float(stamp.secs)
    ns = float(stamp.nsecs)
    return s + ns * 1e-9

def get_modem_info(msg):
    x=Modem()
    x.epoch_timestamp = stamp_to_epoch(msg.header.stamp)

    x.position_x=msg.pose.pose.position.x
    x.position_y=msg.pose.pose.position.y
    x.position_z=msg.pose.pose.position.z

    x.orientation_x=msg.pose.pose.orientation.x
    x.orientation_y=msg.pose.pose.orientation.y
    x.orientation_z=msg.pose.pose.orientation.z
    x.orientation_w=msg.pose.pose.orientation.w
    return x

def get_Nav_status(msg):
    x = Nav_status()
    x.epoch_timestamp = stamp_to_epoch(msg.header.stamp)

    x.origin_lat=msg.origin.latitude
    x.origin_lon=msg.origin.longitude
    
    x.position_north = msg.position.north
    x.position_east = msg.position.east
    x.position_depth = msg.position.depth

    x.altitude=msg.altitude
     
    x.orientation_roll = msg.orientation.roll
    x.orientation_pitch = msg.orientation.pitch
    x.orientation_yaw = msg.orientation.yaw

    return x

def get_UBL_lon(msg):
    x = USBLlong()
    x.epoch_timestamp = stamp_to_epoch(msg.header.stamp)

    x.X=msg.X
    x.Y=msg.Y
    x.Z=msg.Z

    x.N=msg.N
    x.E=msg.E
    x.D=msg.D

    x.roll=msg.roll
    x.pitch=msg.pitch
    x.yaw=msg.yaw
   
    return x

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

    nav_status_tu = []
    USBllongs=[]
    modem_delayed=[]
    nav_status_xi = []
    
    print("Opening bagfile. This can take a while...")
    bag = rosbag.Bag(bagfile)

    for topic, msg, t in bag.read_messages(topics=topics):
        json_data = None

        if "/xiroi/usbllong" in topic:
            x = get_UBL_lon(msg)
            USBllongs.append(x)

        elif "/turbot/modem_delayed" in topic:
            x = get_modem_info(msg)
            modem_delayed.append(x)

        elif "/turbot/navigator/navigation" in topic:
            x = get_Nav_status(msg)
            nav_status_tu.append(x)
        
        elif "/xiroi/navigator/navigation" in topic:
            x = get_Nav_status(msg)
            nav_status_xi.append(x)

        if json_data is not None:
            json_output.append(json_data)

    bag.close()

    print("Bag parsed! Now working on the plots...")



#BAG PARSED CONVERT TO CSV
    #USBLlong--------------------------------
    USBLlong_t = [datetime.datetime.utcfromtimestamp(x.epoch_timestamp)for x in USBllongs]
    usbllong_X= [x.X for x in USBllongs]
    usbllong_Y= [x.Y for x in USBllongs]
    usbllong_Z= [x.Z for x in USBllongs]
    usbllong_N= [x.N for x in USBllongs]
    usbllong_E= [x.E for x in USBllongs]
    usbllong_roll= [x.roll for x in USBllongs]
    usbllong_pitch= [x.pitch for x in USBllongs]
    usbllong_yaw= [x.yaw for x in USBllongs]

    # data_list=[USBLlong_t,usbllong_X,usbllong_Y,usbllong_Z,usbllong_N,usbllong_E,usbllong_roll,usbllong_pitch,usbllong_yaw]
    # header=["USBLlong_t","usbllong_X","usbllong_Y","usbllong_Z","usbllong_N","usbllong_roll","usbllong_pitch","usbllong_yaw"]
    data_dict={"a_USBLlong_t":USBLlong_t,"usbllong_X":usbllong_X,"usbllong_Y":usbllong_Y,"usbllong_Z":usbllong_Z,
    "usbllong_N":usbllong_N,"usbllong_roll":usbllong_roll,"usbllong_pitch":usbllong_pitch,"usbllong_yaw":usbllong_yaw}
    export_to_csv(data_dict,"/USBLlon.csv",bagfile)

    #Modem delayed-----------------------------------------------------
    modem_delayed_t = [datetime.datetime.utcfromtimestamp(x.epoch_timestamp)for x in modem_delayed]

    modem_delayed_X= [x.position_x for x in modem_delayed]
    modem_delayed_Y= [x.position_y for x in modem_delayed]
    modem_delayed_Z= [x.position_z for x in modem_delayed]

    modem_delayed_ori_X= [x.orientation_x for x in modem_delayed]
    modem_delayed_ori_Y= [x.orientation_y for x in modem_delayed]
    modem_delayed_ori_Z= [x.orientation_z for x in modem_delayed]
    modem_delayed_ori_W= [x.orientation_w for x in modem_delayed]

    # data_list=[modem_delayed_t,modem_delayed_X,modem_delayed_Y,modem_delayed_Z,modem_delayed_ori_X,modem_delayed_ori_Y,modem_delayed_ori_Z,modem_delayed_ori_W]
    # header=["modem_delayed_t","modem_delayed_X","modem_delayed_Y","modem_delayed_Z","modem_delayed_ori_X","modem_delayed_ori_Y","modem_delayed_ori_Z","modem_delayed_ori_W"]

    data_dict={"a_modem_delayed_t":modem_delayed_t,"modem_delayed_X":modem_delayed_X,"modem_delayed_Y": modem_delayed_Y, "modem_delayed_Z":modem_delayed_Z,
    "modem_delayed_ori_X":modem_delayed_ori_X,"modem_delayed_ori_Y":modem_delayed_ori_Y,"modem_delayed_ori_Z":modem_delayed_ori_Z,
    "modem_delayed_ori_W":modem_delayed_ori_W}
    
    export_to_csv(data_dict,"/modem_delayed.csv",bagfile)

    #Nav_status turbot-------------------------------------------
    nav_status_t = [datetime.datetime.utcfromtimestamp(x.epoch_timestamp)for x in nav_status_tu]

    nav_status_origin_lat= [x.origin_lat for x in nav_status_tu]
    nav_status_origin_lon= [x.origin_lon for x in nav_status_tu]

    nav_status_N= [x.position_north for x in nav_status_tu]
    nav_status_E= [x.position_east for x in nav_status_tu]
    nav_status_D= [x.position_depth for x in nav_status_tu]

    nav_status_alt= [x.altitude for x in nav_status_tu]

    nav_status_roll= [x.orientation_roll for x in nav_status_tu]
    nav_status_pitch= [x.orientation_pitch for x in nav_status_tu]
    nav_status_yaw= [x.orientation_yaw for x in nav_status_tu]

    # data_list=[nav_status_t,nav_status_origin_lat,nav_status_origin_lon,nav_status_N,nav_status_E,nav_status_D,nav_status_alt,nav_status_roll,nav_status_pitch,nav_status_yaw]
    # header=["nav_status_t","nav_status_origin_lat","nav_status_origin_lon","nav_status_N","nav_status_E","nav_status_D","nav_status_alt","nav_status_roll","nav_status_pitch","nav_status_yaw"]
    
    data_dict={"a_nav_status_t":nav_status_t,"nav_status_origin_lat":nav_status_origin_lat,"nav_status_origin_lon":nav_status_origin_lon,"nav_status_N":nav_status_N,
    "nav_status_E":nav_status_E,"nav_status_D":nav_status_D,"nav_status_alt":nav_status_alt,"nav_status_roll":nav_status_roll,"nav_status_pitch":nav_status_pitch,
    "nav_status_yaw":nav_status_yaw}
    export_to_csv(data_dict,"/nav_status_tu.csv",bagfile)

    #Nav_status xiroi-------------------------------------------
    nav_status_t = [datetime.datetime.utcfromtimestamp(x.epoch_timestamp)for x in nav_status_xi]

    nav_status_origin_lat= [x.origin_lat for x in nav_status_xi]
    nav_status_origin_lon= [x.origin_lon for x in nav_status_xi]

    nav_status_N= [x.position_north for x in nav_status_xi]
    nav_status_E= [x.position_east for x in nav_status_xi]
    nav_status_D= [x.position_depth for x in nav_status_xi]
    nav_status_alt=[x.altitude for x in nav_status_xi]

    nav_status_roll= [x.orientation_roll for x in nav_status_xi]
    nav_status_pitch= [x.orientation_pitch for x in nav_status_xi]
    nav_status_yaw= [x.orientation_yaw for x in nav_status_xi]

    # data_list=[nav_status_t,nav_status_origin_lat,nav_status_origin_lon,nav_status_N,nav_status_E,nav_status_D,nav_status_alt,nav_status_roll,nav_status_pitch,nav_status_yaw]
    # header=["nav_status_t","nav_status_origin_lat","nav_status_origin_lon","nav_status_N","nav_status_E","nav_status_D","nav_status_alt","nav_status_roll","nav_status_pitch","nav_status_yaw"]
        
    data_dict={"a_nav_status_xi":nav_status_t,"nav_status_origin_lat":nav_status_origin_lat,"nav_status_origin_lon":nav_status_origin_lon,"nav_status_N":nav_status_N,
    "nav_status_E":nav_status_E,"nav_status_D":nav_status_D,"nav_status_alt":nav_status_alt,
    "nav_status_roll":nav_status_roll,"nav_status_pitch":nav_status_pitch,"nav_status_yaw":nav_status_yaw}

    export_to_csv(data_dict,"/nav_status_xi.csv",bagfile)


def export_to_csv(data_dict,output_dir_name,bagfile):
        print("WRITING_CSV")

        bagfile_fn = Path(bagfile)
        if not bagfile_fn.exists():
            print("The bagfile does not exist. Please check the path provided.")
        bagfile_path=os.path.split(str(bagfile_fn))[0]    
        csv_file=os.path.join(str(bagfile_path)+output_dir_name)

        print("bagfile_fn is :",bagfile_fn)
        # if not (os.path.exists(csv_file)):
        #     os.mkdir(csv_file)
        print("witing data: in file: ",csv_file)

        df = pd.DataFrame(data_dict)
        df.to_csv(csv_file, index=True)
        # with open(csv_file, 'a+') as file:
        #     writer = csv.writer(file, delimiter=';')
        #     writer.writerow(header)
        #     for data in data_list:
        #         writer.writerow(data)
        # file.close()


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

        