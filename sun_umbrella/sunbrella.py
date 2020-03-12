import suncalcPy.suncalc as sc
from datetime import datetime
import math



def calculate_pose(altitude, azimuth):
    # This function assumes your umbrella has been callibrated to have its start point facing directly north
    # Returns two variables which can be passed via serial to the umbrella controller


    

def send_pose(rot, ang):

    # TODO: init serial communication with pyserial
    #       Send desired angular poses to umbrella

def monitor_feedback():
    # Read serial port for information regarding the ending angle




def calculate_max_coverage(rot, ang):


def calculate_static_coverage(rot, ang):

def set_coverage_location(r, theta):
    # Give the location where shade will be centered in eulerian coordinates (r, theta)

if __name__ == "__main__":
    pos = sc.getPosition(datetime.now(), 37.86948, -122.25929)
    altitude = pos["altitude"]
    azimuth = pos["azimuth"]



print("altitude: ")
    print(pos["altitude"] * 180)
    print("\n azimuth")
    print(pos["azimuth"] * 180)
    
    print(pos)
