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
    "/turbot/navigator/gps_raw",
    "/turbot/navigator/imu_raw",
    "/turbot/teledyne_explorer_dvl/altitude",
    "/turbot/teledyne_explorer_dvl/data",
    "/turbot/teledyne_explorer_dvl/dvl",
    "/xiroi/usbllong",
    "/turbot/navigator/ekf_map/odometry",
    "/turbot/navigator/ekf_odom/odometry",
    "/turbot/navigator/navigation",
    "/turbot/controller/body_velocity_req ",
    "/turbot/contrler/merged_body_velocity_req",
    "/turbot/controller/merged_world_waypoint_req"
]

# topics = [
#     "/xiroi/ekf_map/odometry",
#     "/xiroi/navigator/navigation",
#     "/xiroi/imu/rpy/filtered",
#     "/xiroi/imu/rpy/raw",
#     "/xiroi/sensors/gps",
#     "/xiroi/sensors/gps_raw",
#     "/xiroi/sensors/imu_raw",
#     "/xiroi/sensors/imu_raw_filtered_calibrated"]



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
    VEL_REQ="body_velocity_request"


class Timestamp:
    def __init__(self):
        self.epoch_timestamp = None


class Position(Timestamp):
    def __init__(self):
        Timestamp.__init__(self)
        self.latitude = None
        self.longitude = None


class Setpoints(Timestamp):
    def __init__(self):
        Timestamp.__init__(self)
        self.setpoint0 = None
        self.setpoint1 = None
        self.setpoint2 = None


class Depth(Timestamp):
    def __init__(self):
        Timestamp.__init__(self)
        self.depth = None


class Altitude(Timestamp):
    def __init__(self):
        Timestamp.__init__(self)
        self.altitude = None


class Humidity(Timestamp):
    def __init__(self):
        Timestamp.__init__(self)
        self.humidity = None


class Temperature(Timestamp):
    def __init__(self):
        Timestamp.__init__(self)
        self.temperature = None


class Orientation(Timestamp):
    def __init__(self):
        Timestamp.__init__(self)
        self.roll = None
        self.pitch = None
        self.yaw = None


class BodyVelocity(Timestamp):
    def __init__(self):
        Timestamp.__init__(self)
        self.x_velocity = None
        self.y_velocity = None
        self.z_velocity = None
        self.x_velocity_std = None
        self.y_velocity_std = None
        self.z_velocity_std = None

class WaypointReq(Timestamp):
    def __init__(self):
        Timestamp.__init__(self)
        self.position_north = None
        self.position_east = None
        self.position_depth = None
        self.altitude = None
        self.orientation_roll = None
        self.orientation_pitch = None
        self.orientation_yaw = None


class VelReq(Timestamp):
    def __init__(self):
        Timestamp.__init__(self)
        self.twist_linear_x = None
        self.twist_linear_y = None
        self.twist_linear_z = None
        self.twist_angular_x = None
        self.twist_angular_y = None
        self.twist_angular_z = None
      
class DvlData(Timestamp):
    def __init__(self):
        Timestamp.__init__(self)
        self.wi_x_axis_mms = None
        self.wi_y_axis_mms = None
        self.wi_z_axis_mms = None
        self.wi_error_mms = None
        self.wi_status = None
        self.bi_x_axis_mms = None
        self.bi_y_axis_mms = None
        self.bi_z_axis_mms = None
        self.bi_error_mms = None
        self.bi_status = None
        self.bd_range = None

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
        # self.orientation_x=None
        # self.orientation_y=None
        # self.orientation_z=None
        # self.orientation_w=None


class Frame:
    BODY = "body"
    INERTIAL = "intertial"
    BASE_LINK="base_link"


def stamp_to_epoch(stamp):
    s = float(stamp.secs)
    ns = float(stamp.nsecs)
    return s + ns * 1e-9


def setpoints_to_json(msg):
    x = Setpoints()
    x.epoch_timestamp = stamp_to_epoch(msg.header.stamp)
    x.setpoint0 = float(msg.setpoints[0])
    x.setpoint1 = float(msg.setpoints[1])
    x.setpoint2 = float(msg.setpoints[2])
    json_output = {
        "epoch_timestamp": x.epoch_timestamp,
        "class": "measurement",
        "sensor": "driftcam",
        "frame": Frame.BODY,
        "category": Category.THRUSTER,
        "data": [
            {
                "setpoint0": x.setpoint0,
                "setpoint1": x.setpoint1,
                "setpoint2": x.setpoint2,
            }
        ],
    }
    return json_output, x


def pressure_to_json(msg):
    x = Depth()
    x.epoch_timestamp = stamp_to_epoch(msg.header.stamp)
    x.depth = msg.pose.pose.position.z
    json_output = {
        "epoch_timestamp": x.epoch_timestamp,
        "class": "measurement",
        "sensor": "driftcam",
        "frame": Frame.INERTIAL,
        "category": Category.DEPTH,
        "data": [
            {
                "depth": x.depth,
                "depth_std": 0,
            }
        ],
    }
    return json_output, x


def altitude_to_json(msg):
    x = Altitude()
    x.epoch_timestamp = stamp_to_epoch(msg.header.stamp)
    x.altitude = msg.range
    if x.altitude < 0:
        x.altitude = -1.0
    json_output = {
        "epoch_timestamp": x.epoch_timestamp,
        "class": "measurement",
        "sensor": "driftcam",
        "frame": Frame.BODY,
        "category": Category.ALTITUDE,
        "data": [
            {
                "altitude": x.altitude,
                "altitude_std": 0,
            }
        ],
    }
    return json_output, x


def gps_to_json(msg):
    x = Position()
    x.epoch_timestamp = stamp_to_epoch(msg.header.stamp)
    x.latitude = msg.latitude
    x.longitude = msg.longitude
    json_output = {
        "epoch_timestamp": x.epoch_timestamp,
        "class": "measurement",
        "sensor": "driftcam",
        "frame": Frame.INERTIAL,
        "category": Category.POSITION,
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


def orientation_to_json(msg):
    q = [
        msg.orientation.x,
        msg.orientation.y,
        msg.orientation.z,
        msg.orientation.w,
    ]
    r, p, y = euler_from_quaternion(q)
    x = Orientation()
    x.epoch_timestamp = stamp_to_epoch(msg.header.stamp)
    x.roll = r
    x.pitch = p
    x.yaw = y
    json_output = {
        "epoch_timestamp": x.epoch_timestamp,
        "class": "measurement",
        "sensor": "driftcam",
        "frame": Frame.BODY,
        "category": Category.ORIENTATION,
        "data": [
            {
                "heading": y,
                "heading_std": 0,
            },
            {
                "roll": r,
                "roll_std": 0,
            },
            {
                "pitch": p,
                "pitch_std": 0,
            },
        ],
    }
    return json_output, x


def temperature_to_json(msg):
    x = Temperature()
    x.epoch_timestamp = stamp_to_epoch(msg.header.stamp)
    x.temperature = msg.temperature
    json_output = {
        "epoch_timestamp": x.epoch_timestamp,
        "class": "measurement",
        "sensor": "driftcam",
        "frame": Frame.BODY,
        "category": Category.TEMPERATURE,
        "data": [
            {
                "temperature": x.temperature,
                "temperature_std": 0,
            }
        ],
    }
    return json_output, x


def Vel_req_to_json(msg):
    x = VelReq()

    x.twist_linear_x = msg.twist.linear.x
    x.twist_linear_y = msg.twist.linear.y
    x.twist_linear_z = msg.twist.linear.z

    x.twist_angular_x = msg.twist.angular.x
    x.twist_angular_y = msg.twist.angular.y
    x.twist_angular_z = msg.twist.angular.z
    return x

def modem_to_json(msg):
    x=Modem()
    x.position_x=msg.pose.pose.position.x
    x.position_y=msg.pose.pose.position.y
    x.position_z=msg.pose.pose.position.z
    return x


def Waypoint_req_to_json(msg):
    x = WaypointReq()
    x.epoch_timestamp = stamp_to_epoch(msg.header.stamp)
    x.position_north = msg.position.north
    x.position_east = msg.position.east
    x.position_down = msg.position.depth

    x.altitude=msg.altitude
     
    x.orientation_roll = msg.orientation.roll
    x.orientation_pitch = msg.orientation.pitch
    x.orientation_yaw = msg.orientation.yaw

    return x


def UBL_long_to_json(msg):
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
    # print("usbl msg is ",x.X)
    return x


def world_waypoint_to_json(msg):
    x = WaypointReq()
    x.x_velocity = msg.velocity.x
    x.x_velocity_std = msg.velocity_covariance[0]
    x.y_velocity = msg.velocity.y
    x.y_velocity_std = msg.velocity_covariance[4]
    x.z_velocity = msg.velocity.z
    x.z_velocity_std = msg.velocity_covariance[8]
    json_output = {
        "epoch_timestamp": stamp_to_epoch(msg.header.stamp),
        "class": "measurement",
        "sensor": "turbot",
        "frame": Frame.BODY,
        "category": Category.VELOCITY,
        "data": [
            {
                "x_velocity": msg.velocity.x,
                "x_velocity_std": msg.velocity_covariance[0],
            },
            {
                "y_velocity": msg.velocity.y,
                "y_velocity_std": msg.velocity_covariance[4],
            },
            {
                "z_velocity": msg.velocity.z,
                "z_velocity_std": msg.velocity_covariance[8],
            },
        ],
    }
    return json_output, x


def humidity_to_json(msg):
    x = Humidity()
    x.epoch_timestamp = stamp_to_epoch(msg.header.stamp)
    x.relative_humidity = msg.relative_humidity
    json_output = {
        "epoch_timestamp": x.epoch_timestamp,
        "class": "measurement",
        "sensor": "driftcam",
        "frame": Frame.BODY,
        "category": Category.HUMIDITY,
        "data": [
            {
                "humidity": x.relative_humidity,
                "humidity_std": 0,
            }
        ],
    }
    return json_output, x


def pressure_to_depth(p):
    fluid_density = 1024
    gravity = 9.80665
    air_pressure = 101325
    depth = (p - air_pressure) / fluid_density / gravity
    return depth


def dvl_data_to_json(msg):
    x = DvlData()
    x.epoch_timestamp = stamp_to_epoch(msg.header.stamp)
    x.wi_x_axis_mms = msg.wi_x_axis_mms
    x.wi_y_axis_mms = msg.wi_y_axis_mms
    x.wi_z_axis_mms = msg.wi_z_axis_mms
    x.wi_error_mms = msg.wi_error_mms
    if msg.wi_status == "A":
        x.wi_status = 1
    else:
        x.wi_status = 0
    if msg.bi_status == "A":
        x.bi_status = 1
    else:
        x.bi_status = 0
    x.bi_x_axis_mms = msg.bi_x_axis_mms
    x.bi_y_axis_mms = msg.bi_y_axis_mms
    x.bi_z_axis_mms = msg.bi_z_axis_mms
    x.bi_error_mms = msg.bi_error_mms
    x.bi_status = [1 if msg.bi_status == "A" else 0]
    x.bd_range = msg.bd_range
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

    orientation_list = []
    position_list = []
    depth_list = []
    altitude_list = []
    setpoints_list = []
    velocity_list = []
    dvl_data_list = []
    waypoint_req_list=[]
    body_velocity_req=[]
    USBllongs=[]
    modem_delayed=[]
    modem_delayed_ac=[]


    json_output = []
    print("Opening bagfile. This can take a while...")
    bag = rosbag.Bag(bagfile)

    # for topic, msg, t in bag.read_messages(topics=topics):
    #     print(topic)

    # stop
    for topic, msg, t in bag.read_messages(topics=topics):
        json_data = None

        if "/turbot/navigator/imu" in topic:
            json_data, x = orientation_to_json(msg)
            orientation_list.append(x)
            pass
        elif "/turbot/actuators/thrusters_info" in topic:
            pass
        elif "/turbot/adis_imu/depth" in topic:
            json_data, x = pressure_to_json(msg)
            depth_list.append(x)
        elif "/turbot/adis_imu/fluid_pressure" in topic:
            pass
        elif "/turbot/controller/thruster_setpoints" in topic:
            json_data, x = setpoints_to_json(msg)
            setpoints_list.append(x)
        elif "/turbot/emus_bms/battery_state" in topic:
            pass
        elif "/turbot/imagenex_echosounder_down/profile_range_raw" in topic:
            pass
        elif "/turbot/imagenex_echosounder_down/profile_rx_raw" in topic:
            pass
        elif "/turbot/modem_raw" in topic:
            pass
        
        elif "/turbot/controller/merged_body_velocity_req" in topic:
            print("HELLO_2!!")
            x = Vel_req_to_json(msg)
            body_velocity_req.append(x)

        elif "/turbot/controller/body_velocity_req" in topic:
            print("HELLO_23!!")
            x = Vel_req_to_json(msg)
            body_velocity_req.append(x)

        elif "/turbot/navigator/gps_raw" in topic:
            json_data, x = gps_to_json(msg)
            position_list.append(x)
        elif "/turbot/navigator/imu_raw" in topic:
            pass
        elif "/turbot/teledyne_explorer_dvl/altitude" in topic:
            # print("HELLO 3333333333333333")
            json_data, x = altitude_to_json(msg)
            altitude_list.append(x)
            pass
        elif "/turbot/teledyne_explorer_dvl/data" in topic:
            x = dvl_data_to_json(msg)
            dvl_data_list.append(x)

        # elif "/turbot/teledyne_explorer_dvl/dvl" in topic:
        #     json_data, x = dvl_to_json(msg)
        #     velocity_list.append(x)

        elif "/turbot/controller/merged_world_waypoint_req" in topic:
            # print("HELLO_1!!")
            x = Waypoint_req_to_json(msg)
            waypoint_req_list.append(x)
        
        elif "/xiroi/usbllong" in topic:
            x = UBL_long_to_json(msg)
            USBllongs.append(x)

        elif "/turbot/modem_delayed" in topic:
            x = modem_to_json(msg)
            modem_delayed.append(x)

        elif "/turbot/modem_delayed_acoustic" in topic:
            x = modem_to_json(msg)
            modem_delayed_ac.append(x)

        elif "/turbot/navigator/ekf_map/odometry" in topic:
            pass
        elif "/turbot/navigator/ekf_odom/odometry" in topic:
            pass
        elif "/turbot/navigator/navigation" in topic:
            pass

        if json_data is not None:
            json_output.append(json_data)

    bag.close()

    print("Bag parsed! Now working on the plots...")

    with json_fn.open("w") as outfile:
        data = json.dumps(json_output, ensure_ascii=False, indent=4)
        outfile.write(unicode(data))

    # Do the plotting
    ori_t = [
        datetime.datetime.utcfromtimestamp(x.epoch_timestamp)
        for x in orientation_list
    ]
    ori_roll = [x.roll for x in orientation_list]
    ori_pitch = [x.pitch for x in orientation_list]
    ori_yaw = [x.yaw for x in orientation_list]

    setpoints_t = [
        datetime.datetime.utcfromtimestamp(x.epoch_timestamp)
        for x in setpoints_list
    ]
    setpoints_0 = [x.setpoint0 for x in setpoints_list]
    setpoints_1 = [x.setpoint1 for x in setpoints_list]
    setpoints_2 = [x.setpoint2 for x in setpoints_list]

    depth_t = [
        datetime.datetime.utcfromtimestamp(x.epoch_timestamp)
        for x in depth_list
    ]
    depth_depth = [x.depth for x in depth_list]

    altitude_t = [
        datetime.datetime.utcfromtimestamp(x.epoch_timestamp)
        for x in altitude_list
    ]
    altitude_altitude = [x.altitude for x in altitude_list]

    location_lat = [x.latitude for x in position_list]
    location_lon = [x.longitude for x in position_list]

    body_vel_req_t = [
        datetime.datetime.utcfromtimestamp(x.epoch_timestamp)
        for x in body_velocity_req
    ]
    body_vel_lin_x = [x.twist_linear_x for x in body_velocity_req]
    body_vel_lin_y = [x.twist_linear_y for x in body_velocity_req]
    body_vel_lin_z = [x.twist_linear_z for x in body_velocity_req]

    body_vel_angular_x = [x.twist_angular_x for x in body_velocity_req]
    body_vel_angular_y = [x.twist_angular_y for x in body_velocity_req]
    body_vel_angular_z = [x.twist_angular_z for x in body_velocity_req]

    Waypoint_req_t = [
        datetime.datetime.utcfromtimestamp(x.epoch_timestamp)
        for x in waypoint_req_list
    ]

    wpt_north = [x.position_north for x in waypoint_req_list]
    wpt_east = [x.position_east for x in waypoint_req_list]
    wpt_down = [x.position_down for x in waypoint_req_list]
    
    wpt_altitude = [x.altitude for x in waypoint_req_list]

    wpt_roll = [x.orientation_roll for x in waypoint_req_list]
    wpt_pitch = [x.orientation_pitch for x in waypoint_req_list]
    wpt_yaw = [x.orientation_yaw for x in waypoint_req_list]


    USBLlong_t = [
        datetime.datetime.utcfromtimestamp(x.epoch_timestamp)
        for x in USBllongs
    ]

    usbllong_X= [x.X for x in USBllongs]
    usbllong_Y= [x.Y for x in USBllongs]
    usbllong_Z= [x.Z for x in USBllongs]
    usbllong_N= [x.N for x in USBllongs]
    usbllong_E= [x.E for x in USBllongs]


    modem_delayed_t = [
        datetime.datetime.utcfromtimestamp(x.epoch_timestamp)
        for x in modem_delayed
    ]

    usbllong_X= [x.X for x in USBllongs]
    usbllong_Y= [x.Y for x in USBllongs]
    usbllong_Z= [x.Z for x in USBllongs]
    usbllong_N= [x.N for x in USBllongs]
    usbllong_E= [x.E for x in USBllongs]


    print(usbllong_X)
    print("USBLlong!",usbllong_X)
        
    # Create traces
    fig = make_subplots(
        rows=3,
        cols=2,
        subplot_titles=(
            "Orientation",
            "Setpoints",
            "Depth and altitude",
            "Velocity_req",
            "Waypoint_req"
        ),
        shared_xaxes=True,
    )
    fig.add_trace(
        go.Scatter(x=ori_t, y=ori_roll, name="roll"),
        row=1,
        col=1,
    )
    fig.add_trace(
        go.Scatter(x=ori_t, y=ori_pitch, name="pitch"),
        row=1,
        col=1,
    )
    fig.add_trace(
        go.Scatter(x=ori_t, y=ori_yaw, name="yaw"),
        row=1,
        col=1,
    )
    fig.add_trace(
        go.Scatter(x=setpoints_t, y=setpoints_0, name="setpoint0"),
        row=1,
        col=2,
    )
    fig.add_trace(
        go.Scatter(x=setpoints_t, y=setpoints_1, name="setpoint1"),
        row=1,
        col=2,
    )
    fig.add_trace(
        go.Scatter(x=setpoints_t, y=setpoints_2, name="setpoint2W"),
        row=1,
        col=2,
    )

    fig.add_trace(
        go.Scatter(x=setpoints_t, y=wpt_yaw, name="wpt_yaw"),
        row=1,
        col=2,
    )

    fig.add_trace(
        go.Scatter(x=depth_t, y=depth_depth, name="depth"),
        row=2,
        col=1,
    )
    fig.add_trace(
        go.Scatter(x=altitude_t, y=altitude_altitude, name="altitude"),
        row=2,
        col=1,
    )

    #new
    fig.add_trace(
        go.Scatter(x=body_vel_req_t, y=body_vel_lin_x, name="body_vel_lin_x"),
        row=2,
        col=2,
    )
    fig.add_trace(
        go.Scatter(x=body_vel_req_t, y=body_vel_lin_y, name="body_vel_lin_y"),
        row=2,
        col=2,
    )
    fig.add_trace(
        go.Scatter(x=body_vel_req_t, y=body_vel_lin_z, name="body_vel_lin_z"),
        row=2,
        col=2,
    )

    fig.add_trace(
        go.Scatter(x=body_vel_req_t, y=body_vel_angular_x, name="body_vel_angular_x"),
        row=2,
        col=2,
    )
    fig.add_trace(
        go.Scatter(x=body_vel_req_t, y=body_vel_angular_y, name="body_vel_angular_y"),
        row=2,
        col=2,
    )
    fig.add_trace(
        go.Scatter(x=body_vel_req_t, y=body_vel_angular_z, name="body_vel_angular_z"),
        row=2,
        col=2,
    )

    #Worldwaypoint

    fig.add_trace(
        go.Scatter(x=Waypoint_req_t, y=wpt_north, name="wpt_north"),
        row=3,
        col=1,
    )
    fig.add_trace(
        go.Scatter(x=Waypoint_req_t, y=wpt_east, name="wpt_east"),
        row=3,
        col=1,
    )
    fig.add_trace(
        go.Scatter(x=Waypoint_req_t, y=wpt_down, name="wpt_down"),
        row=3,
        col=1,
    )

    fig.add_trace(
        go.Scatter(x=Waypoint_req_t, y=wpt_altitude, name="wpt_altitude"),
        row=3,
        col=1,
    )
    fig.add_trace(
        go.Scatter(x=Waypoint_req_t, y=wpt_roll, name="wpt_roll"),
        row=3,
        col=1,
    )
    fig.add_trace(
        go.Scatter(x=Waypoint_req_t, y=wpt_pitch, name="wpt_pitch"),
        row=3,
        col=1,
    )
    fig.add_trace(
        go.Scatter(x=Waypoint_req_t, y=wpt_yaw, name="wpt_yaw"),
        row=3,
        col=1,
    )

    #Body_vel_req
    # fig.show()
    
    html_fn = bagfile_fn.parent / (bagfile_fn.stem + "general_info.html")
    fig.write_html(str(html_fn))
    print("Plot HTML file save at " + str(html_fn))
    print("HOLA")
    #----------------------------------------------------------
    fig = make_subplots(
        rows=2,
        cols=2,
        subplot_titles=(
            "usbllong_XYZ",
            "usbllong_NE",
        ),
        shared_xaxes=True,
    )
    
    print("USBL")

    USBLlong_t = [
        datetime.datetime.utcfromtimestamp(x.epoch_timestamp)
        for x in USBllongs
    ]

    usbllong_X= [x.X for x in USBllongs]
    usbllong_Y= [x.Y for x in USBllongs]
    usbllong_Z= [x.Z for x in USBllongs]
    usbllong_N= [x.N for x in USBllongs]
    usbllong_E= [x.E for x in USBllongs]

    fig.add_trace(
        go.Scatter(x=USBLlong_t, y=usbllong_X, name="usbllong_X"),
        row=1,
        col=1,
    )
    fig.add_trace(
        go.Scatter(x=USBLlong_t, y=usbllong_Y, name="usbllong_Y"),
        row=1,
        col=1,
    )
    fig.add_trace(
        go.Scatter(x=USBLlong_t, y=usbllong_Z, name="usbllong_Z"),
        row=1,
        col=1,
    )

    fig.add_trace(
        go.Scatter(x=USBLlong_t, y=usbllong_N, name="usbllong_N"),
        row=2,
        col=1,
    )
    fig.add_trace(
        go.Scatter(x=USBLlong_t, y=usbllong_E, name="usbllong_E"),
        row=2,
        col=1,
    )

    html_fn = bagfile_fn.parent / (bagfile_fn.stem + "_UBL.html")
    fig.write_html(str(html_fn))
    print("Plot HTML file save at " + str(html_fn))




#-----------------------------------------------
    fig = make_subplots(
        rows=2,
        cols=2,
        subplot_titles=(
            "Water",
            "Bottom",
            "Error",
            "Status",
        ),
        shared_xaxes=True,
    )

    dvl_t = [
        datetime.datetime.utcfromtimestamp(x.epoch_timestamp)
        for x in dvl_data_list
    ]
    wi_x_axis_mms = [x.wi_x_axis_mms for x in dvl_data_list]
    wi_y_axis_mms = [x.wi_y_axis_mms for x in dvl_data_list]
    wi_z_axis_mms = [x.wi_z_axis_mms for x in dvl_data_list]

    bi_x_axis_mms = [x.bi_x_axis_mms for x in dvl_data_list]
    bi_y_axis_mms = [x.bi_y_axis_mms for x in dvl_data_list]
    bi_z_axis_mms = [x.bi_z_axis_mms for x in dvl_data_list]

    wi_error_mms = [x.wi_error_mms for x in dvl_data_list]
    bi_error_mms = [x.bi_error_mms for x in dvl_data_list]

    wi_status = [x.wi_status for x in dvl_data_list]
    bi_status = [x.bi_status for x in dvl_data_list]

    fig.add_trace(
        go.Scatter(x=dvl_t, y=wi_x_axis_mms, name="x_mms"),
        row=1,
        col=1,
    )
    fig.add_trace(
        go.Scatter(x=dvl_t, y=wi_y_axis_mms, name="y_mms"),
        row=1,
        col=1,
    )
    fig.add_trace(
        go.Scatter(x=dvl_t, y=wi_z_axis_mms, name="z_mms"),
        row=1,
        col=1,
    )

    fig.add_trace(
        go.Scatter(x=dvl_t, y=bi_x_axis_mms, name="x_mms"),
        row=1,
        col=2,
    )
    fig.add_trace(
        go.Scatter(x=dvl_t, y=bi_y_axis_mms, name="y_mms"),
        row=1,
        col=2,
    )
    fig.add_trace(
        go.Scatter(x=dvl_t, y=bi_z_axis_mms, name="z_mms"),
        row=1,
        col=2,
    )
    fig.add_trace(
        go.Scatter(x=dvl_t, y=wi_error_mms, name="wi"),
        row=2,
        col=1,
    )
    fig.add_trace(
        go.Scatter(x=dvl_t, y=bi_error_mms, name="bi"),
        row=2,
        col=1,
    )
    fig.add_trace(
        go.Scatter(x=dvl_t, y=wi_status, name="wi"),
        row=2,
        col=2,
    )
    fig.add_trace(
        go.Scatter(x=dvl_t, y=bi_status, name="bi"),
        row=2,
        col=2,
    )

    html_fn = bagfile_fn.parent / (bagfile_fn.stem + "_dvl.html")
    fig.write_html(str(html_fn))
    print("Plot HTML file save at " + str(html_fn))


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
