import os
os.chdir("../../../home/reu-actor/actor_ws/src/v2x_pkg/scripts/occupational_grid")

import cv2 as cv
from grid import Grid
from occupant import Occupant
import timeit
import os

# I wrote just about all of this in one day so I'm going to go through
# and comment everything I can so I'm not lost when I see this again

# Initialize parameters for Grid()
map_img = cv.imread("map.png")
map_img = cv.resize(map_img, (800,800))
ll_top_left = (42.47247003293518, -83.25026723111947)
pix_top_left = (0,0)
shape_ll = (-0.00028997467238, 0.0004425054541251)

# Grid __init__() takes the following in order:
# lat/lon of the top left corner
# xy coordinates of the top left (should be (0,0))
# width / height of the map in pixels
# width / height of the map in lat / lon
# Image of the map

grid = Grid(ll_top_left,
            pix_top_left,
            map_img.shape[:2],
            shape_ll,
            map_img)

# Just naming and moving the windows
cv.namedWindow("More Info")
cv.namedWindow("Occ Grid")
cv.moveWindow("More Info", 0,0)
cv.moveWindow("Occ Grid", 500,0)

def dispaly_grid(vehicles=[], obstacles=[]):


    
    # Occupant __init__() takes the following in order:
    # id of the vehicle or obstacle
    # tag (v for vehicles, o for obstacles)
    # (lat / lon)
    # heading
    # velocity
    # angular z
    #v1lat, v1lon = 42.47232207832096, -83.2498880760587
    #veh1 = Occupant('Actor 2', 'v', (42.47232207832096, -83.2498880760587), 90, 4, 0.2)
    #veh2 = Occupant('Actor 1', 'v', (42.47227801451705, -83.24987130956126), 0, 6)
    
    # Use add_vehicle to add a vehicle to the grid
    # or add_obstacle to add an obstacle
    # both can be removed with grid.remove_<vehicle / obstacle>
    #grid.add_vehicle(veh1)
    #grid.add_vehicle(veh2)

    # Loop for testing the grid

    # Flip updates the whole grid
    # This lets you just add, remove, and update occupants then flip
    elapsed_time = timeit.timeit(grid.flip, number=1) * 1000
    print(f"Grid flipped in [{elapsed_time} ms]")

    cv.imshow("More Info", grid.info)
    cv.imshow("Occ Grid", grid.image)
