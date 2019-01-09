from dronekit import connect,Vehicle,VehicleMode,mavutil
import time
import math


class ColorTracker():
    def __init__(self,connect_string):
        self.search_yaw_speed = 5
        self.connect_string = connect_string
        self.vehicle=connect(self.connect_string,wait_ready=True)
        self.total_yaw_angle = 0
        self.target_yaw=self.vehicle.attitude.yaw

    def condition_yaw(self,heading):
        # create the CONDITION_YAW command
        msg = self.vehicle.message_factory.mission_item_encode(0, 0,  # target system, target component
                                                               0,  # sequence
                                                               mavutil.mavlink.MAV_FRAME_GLOBAL_RELATIVE_ALT,  # frame
                                                               mavutil.mavlink.MAV_CMD_CONDITION_YAW,  # command
                                                               2,  # current - set to 2 to make it a guided command
                                                               0,  # auto continue
                                                               heading, 0, 0, 0, 0, 0, 0)  # param 1 ~ 7
        # send command to vehicle
        self.vehicle.send_mavlink(msg)
        self.vehicle.flush()

    def arm_and_takeoff(self,aTargetAltitude):
        """
        Arms vehicle and fly to aTargetAltitude.
        """

        print "Basic pre-arm checks"
        # Don't try to arm until autopilot is ready
        while not self.vehicle.is_armable:
            print " Waiting for vehicle to initialise..."
            time.sleep(1)

        print "Arming motors"
        # Copter should arm in GUIDED mode
        self.vehicle.mode = VehicleMode("GUIDED")
        self.vehicle.armed = True

        # Confirm vehicle armed before attempting to take off
        while not self.vehicle.armed:
            print " Waiting for arming..."
            time.sleep(1)

        print "Taking off!"
        self.vehicle.simple_takeoff(aTargetAltitude)

        # Wait until the vehicle reaches a safe height before processing the goto (otherwise the command
        #  after Vehicle.simple_takeoff will execute immediately).
        while True:
            print " Altitude: ", self.vehicle.location.global_relative_frame.alt
            if self.vehicle.location.global_relative_frame.alt>=aTargetAltitude*0.95:
                print "Reached target altitude"
                break
            time.sleep(1)
        #self.condition_yaw(-40)
        time.sleep(1)
        #print "yaw:%s" % self.vehicle.attitude.yaw
        #print "yaw-angle:%s" % math.degrees(self.vehicle.attitude.yaw)
        #print "roll:%s" % self.vehicle.attitude.roll
        #print "pitch:%s" % self.vehicle.attitude.pitch

    def search(self):

        self.target_yaw=self.target_yaw - math.radians(self.search_yaw_speed)
        self.total_yaw_angle=self.total_yaw_angle+self.search_yaw_speed
        print math.degrees(self.target_yaw)
        self.condition_yaw(math.degrees(self.target_yaw))
        print "vehicle yaw:%s" % math.degrees(self.vehicle.attitude.yaw)

    def run(self):
        self.arm_and_takeoff(5)
        while self.total_yaw_angle<=360:
            if math.fabs(self.wrap_PI(self.vehicle.attitude.yaw - self.target_yaw)) < math.radians(
                    self.search_yaw_speed * 2.0):
                self.search()
                time.sleep(0.05)

    def wrap_PI(self,angle):
        if (angle > math.pi):
            return (angle - (math.pi * 2.0))
        if (angle < -math.pi):
            return (angle + (math.pi * 2.0))
        return angle


if __name__ == "__main__":
    connect_string = "udp:127.0.0.1:14550"
    colortracker = ColorTracker(connect_string)
    colortracker.run()

