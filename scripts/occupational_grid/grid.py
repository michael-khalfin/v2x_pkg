# import sys
# sys.path.append("./grid_functions")
import os
from grid_functions import GridFunctions
from large_square import LargeSquare
import numpy as np

# Grid surrounds the entire map, and contains smaller squares

class Grid(GridFunctions):

    def __init__(self, ll_top_left, xy_top_left, shape_xy, shape_ll, image):

        # Data from outside of our control is capitalized
        self.VEHICLES = []
        self.OBSTACLES = []
        self.EVENTS = []

        # Different coordinates in different units
        # xy is always in pixels, ll is lat / lon
        # Important to note that lat / lon is in that order, (lat,lon)
        # Lattitude is up/down, longitude is left/right
        # It can be kind of counterintuitive so if you 
        # work with ll be very careful
        self.ll = ll_top_left
        self.xy = xy_top_left
        self.shape_xy = shape_xy
        self.shape_ll = shape_ll
        
        # Using ll to get the shape of the course in meters
        # This isn't absolutely necessary, but it lets us control
        # the shape fo the grids in meters
        # also lets us calculate time to impact
        lat1, lon1 = ll_top_left
        lat2, lon2 = ll_top_left[0] + shape_ll[0], ll_top_left[1] + shape_ll[1]
        meters_x = self.equirectangular_approximation(lat1, lon1, lat1, lon2)
        meters_y = self.equirectangular_approximation(lat1, lon1, lat2, lon2)
        self.shape_meters = (meters_x, meters_y)

        # Define important features for sub grid
        self.sub_meters = 5, 5
        self.n_row = int(meters_y // self.sub_meters[0])
        self.n_col = int(meters_x // self.sub_meters[1])

        # Define important features for drawing
        # Important to remember that if you want
        # to mess with self.image and not change the map
        # make a copy and mess with that one
        self.color = (0,0,0)
        self.thickness = 5
        self.image = image

        # Define important features for map
        self.map = {}
        self.sub_type = LargeSquare
        
        # Run functions to create the rest of the grid
        self.generate_sub_grid()
        self.display_more()
        self.draw()
        # Copy image with grid lines
        self.image_copy = np.copy(image)

        self.yaw_sensitivity = 2
        self.velocity_sensitivity = 0.75

    # Draws everything
    # This is only ever called from __init__()
    # This is done to save time not drawing the
    # same thing when a grid hasn't updated
    def draw(self):
        self.image = super().draw(self.image)

    # Returns distance between 2 ll points in meters
    def equirectangular_approximation(self, lat1, lon1, lat2, lon2):
        return super().equirectangular_approximation(lat1, lon1, lat2, lon2)

    # Turns lat / lon to xy coordinates
    def ll2xy(self, ll):
        return super().ll2xy(ll)

    # Generates all subgrids and subgrids of subgrids
    def generate_sub_grid(self):
        super().generate_sub_grid()

    # Bool, returns True if xy is withing square bounds
    def does_bound_xy(self, xy):
        return super().does_bound_xy(xy)

    # Converts lat / lon to a pair of keys
    # The first key is to this squares map
    # The second key is to the sub grids map
    def ll2keys(self, ll):
        return super().ll2keys(ll)
    
    # Adds a vehicle to the grid
    def add_vehicle(self, vic):
        self.VEHICLES.append(vic)

    def clear_vehicles(self):
        self.VEHICLES.clear()

    # Removes a vehicle from the grid
    # You can query by id or object
    def remove_vehicle(self, vic):
        if type(vic) is type(""):
            self.VEHICLES = [x for x in self.VEHICLES if x.id != vic]
        else:
            self.VEHICLES = [x for x in self.VEHICLES if x != vic]

    # Adds an obstacle to the grid
    def add_obstacle(self, obstacle):
        self.VEHICLES.append(obstacle)

    # Removes an obstacle from the grid
    # You can also query by id or object
    def remove_obstacle(self, obstacle):
        if type(obstacle) is type(""):
            self.VEHICLES = [x for x in self.VEHICLES if x.id != obstacle]
        else:
            self.VEHICLES = [x for x in self.VEHICLES if x != obstacle]

    # Deoccupies, then occupies squares,
    # Detects collisions, and creates event messages
    # This function is bloated, and needs to be split up
    def update_squares(self):
        super().update_vehicle_squares()

    # Creates the second window that holds information
    # about the grid in realtime
    def display_more(self):
        super().display_more()

    # Updates both windows
    def flip(self):
        self.generate_sub_grid()
        self.update_squares()
        self.display_more()

    # Converts an x,y coordinate to a meter,meter coordinate
    # This has only really been useful in collision detection so far
    def xy2meter(self, xy1, xy2):
        return super().xy2meter(xy1, xy2)
    
    # Just what it says, calculates and returns
    # the time of an anticipatied collision in seconds
    def time_to_impact(self, xy1, v1, xy2, v2, square):
        return super().time_to_impact(xy1, v1, xy2, v2, square)
    
    # Used to lighten update_squares a bit
    # querys for a vehicle by its id
    def get_vehicle_by_id(self, id):
        return super().get_vehicle_by_id(id)
    
    def update_vehicle(self, vehicle="", **kwargs):
        kwargs_keys = kwargs.keys()
        if 'll' in kwargs_keys:
            lat_lon = kwargs['ll']
        if 'heading' in kwargs_keys:
            head = kwargs['heading']
        if 'ang_z' in kwargs_keys:
            ang = kwargs['ang_z']
        if 'velocity' in kwargs_keys:
            vel = kwargs['velocity']
        return super().update_vehicle(vehicle, ll=lat_lon, heading=head, ang_z=ang, velocity=vel)
    
    def handle_collision(self, path):
        print("Handle Collision invoked...")
        return super().handle_collision(path)