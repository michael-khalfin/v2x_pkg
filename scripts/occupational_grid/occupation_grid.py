#!/usr/bin/env python3

import os
os.chdir("/home/reu-actor/actor_ws/src/v2x_pkg/scripts/occupational_grid/")

import rospy
from grid import Grid
from occupant import Occupant
import cv2 as cv
from tpn_pkg.msg import Position
import timeit
from std_msgs.msg import Int32
from yolov8ros_pkg.msg import BBoxes
from geometry_msgs.msg import Twist

def c1_cb(msg):
    id = "actor1"
    ll = msg.lat, msg.lon
    tag = "v"
    heading = msg.heading
    velocity = msg.velocity
    ang_z = msg.angZ
    actor1 = Occupant(id, tag, ll, heading, velocity, ang_z)
    grid.VEHICLES = [x for x in grid.VEHICLES if x.id != id]
    grid.add_vehicle(actor1)
    display()
    
def c2_cb(msg):
    id = "actor2"
    ll = msg.lat, msg.lon
    tag = "v"
    heading = msg.heading
    velocity = msg.velocity
    ang_z = msg.angZ
    actor2 = Occupant(id, tag, ll, heading, velocity, ang_z)
    grid.VEHICLES = [x for x in grid.VEHICLES if x.id != id]
    grid.add_vehicle(actor2)

def cone_cb1(msg):
    if not msg.boxes or msg.boxes[0].confidence <= 0.5:
        return
    
    id = "cone" + str(len(grid.OBSTACLES))
    tag = "o"
    car = grid.get_vehicle_by_id("actor1")
    if not car: return
    ll = car.ll
    obs = Occupant(id, tag, ll)
    grid.VEHICLES = [x for x in grid.VEHICLES if x.id != id]
    grid.add_obstacle(obs)

def cone_cb2(msg):
    if not msg.boxes or msg.boxes[0].confidence <= 0.5:
        return
    
    id = "cone0"
    tag = "o"
    car = grid.get_vehicle_by_id("actor2")
    if not car: return
    ll = car.ll
    obs = Occupant(id, tag, ll)
    grid.VEHICLES = [x for x in grid.VEHICLES if x.id != id]
    grid.add_obstacle(obs)

def display():
    elapsed_time = timeit.timeit(grid.flip, number=1) * 1000
    print(f"Grid flipped in [{elapsed_time} ms]")

    for event in grid.EVENTS:
        if event[2] < 1:
            t_msg = Twist()
            print(t_msg)
    
    cv.imshow("Occupancy grid", grid.image)
    cv.imshow("More Info", grid.info)

    cv.waitKey(1)
    
map_img = cv.imread("map.png")
map_img = cv.resize(map_img, (800,800))
ll_top_left = (42.47247003293518, -83.25026723111947)
pix_top_left = (0,0)
shape_ll = (-0.00028997467238, 0.0004425054541251)

if __name__ == "__main__":
    rospy.init_node("occupation_grid", anonymous=True)

    actor1 = Occupant("actor1", "v", (0,0), 270, 4, -.2)
    actor2 = Occupant("actor2", "v", (0,0), 0, 4, .2)
    cone0 = Occupant("cone0", "o", (0,0))

    grid = Grid(ll_top_left,
            pix_top_left,
            map_img.shape[:2],
            shape_ll,
            map_img)
    
    grid.add_vehicle(actor1)
    grid.add_vehicle(actor2)
    grid.add_obstacle(cone0)


    chatter1_sub = rospy.Subscriber("/actor1/tpn_node/gps_chatter", Position, c1_cb, queue_size=1)
    chatter2_sub = rospy.Subscriber("/actor2/tpn_node/gps_chatter", Position, c2_cb, queue_size=1)
    cone_sub1 = rospy.Subscriber("/actor1/predictions", BBoxes, cone_cb1, queue_size=1)
    cone_sub2 = rospy.Subscriber("/actor2/predictions", BBoxes, cone_cb2, queue_size=1)

    while not rospy.is_shutdown():
        rospy.spin()