from __future__ import print_function

from dronekit import connect, VehicleMode, LocationGlobalRelative, LocationGlobal, Command
import time
import math
from pymavlink import mavutil
import argparse  

def arm_and_takeoff(aTargetAltitude):

    print("\nSet Vehicle.mode = GUIDED (currently: %s)" % vehicle.mode.name) 
    vehicle.mode = VehicleMode("GUIDED")
    
    while not vehicle.mode.name=='GUIDED':  #Wait until mode has changed
        print(" Waiting for mode change ...")
        time.sleep(1)

    """
    Arms vehicle and fly to aTargetAltitude.
    """

    print("Basic pre-arm checks")
    # Don't let the user try to arm until autopilot is ready
    while not vehicle.is_armable:
        print(" Waiting for vehicle to initialise...")
        time.sleep(1)

    while not vehicle.armed:      
        print(" Waiting for arming...")
        time.sleep(1)

    print("Taking off!")
    vehicle.simple_takeoff(aTargetAltitude) # Take off to target altitude

    # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command 
    #  after Vehicle.simple_takeoff will execute immediately).
    while True:
        print(" Altitude: ", vehicle.location.global_relative_frame.alt)      
        if vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95: #Trigger just below target alt.
            print("Reached target altitude")
            break
        time.sleep(1)

##Simple go to the required gps location command
def go_to_required_location(targetlocation):
    vehicle.simple_goto(targetlocation)
    while True:
        if targetlocation.distance_to(vehicle.location.global_relative_frame) <= 0.5:
            print("Reached Target Location")
            break
        time.sleep(1)

##If we want to increase or decrese the height of the drone depending on the on object detected on the same latitude and longitude
def increase_decrease_height_same_latitude_and_longitude(choice):
    current_altitude=vehicle.location.global_relative_frame.alt
    ##If choice=1 increase the height by 1m and if choice=0 decrease the height by 2m
    if choice==1:
        new_alt=current_altitude+2
        vehicle.simple_goto(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,new_alt)
        while True:
            current_altitude = vehicle.location.global_relative_frame.alt
            if current_altitude >= new_alt * 0.95:
                print("Reached target altitude")
                break
            time.sleep(1)
    elif choice==2:
        new_alt=current_altitude-1
        vehicle.simple_goto(vehicle.location.global_relative_frame.lat,vehicle.location.global_relative_frame.lon,new_alt)
        while True:
            current_altitude = vehicle.location.global_relative_frame.alt
            if current_altitude <= new_alt * 1.05:
                print("Reached target altitude")
                break
            time.sleep(1)


parser = argparse.ArgumentParser(description='Demonstrates basic mission operations.')
parser.add_argument('--connect',help="vehicle connection target string. If not specified, SITL automatically started and used.")
args = parser.parse_args()
go_to_location=args.gps_location.split(" ")
connection_string = args.connect
sitl = None

#Start SITL if no connection string specified

if not connection_string:
    import dronekit_sitl
    sitl = dronekit_sitl.start_default()
    connection_string = sitl.connection_string()

print('Connecting to vehicle on: %s' % connection_string)
vehicle = connect(connection_string, wait_ready=True)

#vehicle home_location, mode, and armed attributes (the only settable attributes)
#GETTING HOME LOCATION OF THE DRONE

while not vehicle.home_location:
    cmds = vehicle.commands
    cmds.download()
    cmds.wait_ready()
    if not vehicle.home_location:
        print(" Waiting for home location ...")

# We have a home location, so print it!        

print("\n Home location: %s" % vehicle.home_location)
homelocation=[vehicle.home_location.lat,vehicle.home_location.lon,vehicle.home_location.alt]
# From Copter 3.3 you will be able to take off using a mission item. Plane must take off using a mission item (currently).

#TAKING OFF
arm_and_takeoff(10)

#MOVING TO THE REQUIRED LOCATION
target_location=LocationGlobalRelative(go_to_location[0],go_to_location[1],go_to_location[2])
go_to_required_location(target_location)

##Detection of object in the area

object=True
##If object not present in the given target coordinates
##Increase Height by 2m
if object==False:
    increase_decrease_height_same_latitude_and_longitude(1)
##If object present in the frame,decrease the height by 1m after moving on top of the object after finding the gps coordinates of the object
elif object ==True:
    increase_decrease_height_same_latitude_and_longitude(2)
