import cv2 as cv
import math
import numpy as np

# GridFunctions holds almost all of the functions the grids and sub grids use
# This is done to lighten up the code on the other classes, and have one function
# definition, as opposed to writing the same function for all three classes

class GridFunctions:

    # Draw simply creates the rectangles based 
    # upon the specifications of the instance
    # When called from a higher order grid
    # (i.e. LargeSquare) all of its subgrids
    # will also be drawn.
    def draw(self, image):

        for key in self.map.keys():
            self.map[key].draw(image)

        x, y = self.xy
        l, w = self.shape_xy
        rect = (int(x), int(y), int(l), int(w))
        image = cv.rectangle(image, rect, self.color, self.thickness)


        return image
    
    # This function returns a distance between
    # two lat / lon points
    def equirectangular_approximation(self, lat1, lon1, lat2, lon2):

        # Convert degrees to radians
        lat1 = math.radians(lat1)
        lon1 = math.radians(lon1)
        lat2 = math.radians(lat2)
        lon2 = math.radians(lon2)

        # Approximation formula
        x = (lon2 - lon1) * math.cos((lat1 + lat2) / 2)
        y = lat2 - lat1
        radius_of_earth = 6371  # Radius of the Earth in kilometers
        distance_kilometers = math.sqrt(x**2 + y**2) * radius_of_earth
        distance_meters = distance_kilometers * 1000

        return distance_meters

    # Iteratively add to instances self.map dictionary
    def generate_sub_grid(self):
        
        x, y = self.xy

        x_step = self.shape_xy[0] / self.n_col
        y_step = self.shape_xy[1] / self.n_row

        for row in range(self.n_row):
            for col in range(self.n_col):
                
                sub_x = x + x_step * col
                sub_y = y + y_step * row

                key = str(row) + str(col)

                self.map[key] = self.sub_type(
                                (sub_x,sub_y),
                                (x_step, y_step),
                                self.sub_meters
                            )
        
    # This function can ONLY be used by Grid
    # Creates an image to be displayed giving
    # realtime text updates
    def display_more(self):
        font = cv.FONT_HERSHEY_SIMPLEX
        color = (0,0,0)
        scale = .5
        xy = (0,50)
        more_info = np.ones((800,200)) * 50

        cv.putText(more_info, "Tracking Vehicles:", xy, font, scale, color)
        it = 1
        for vic in self.VEHICLES:
            cv.putText(more_info, vic.id, (xy[0], xy[1] + 20 * it), font, scale, color)
            it += 1

        xy = (0,300)
        cv.putText(more_info, "Events:", xy, font, scale, color)
        it = 1
        for event in self.EVENTS:
            txt = f'{event[0]} hits {event[1]}'
            cv.putText(more_info, txt, (xy[0], xy[1] + 20 * it), font, scale, color)
            it += 1
            txt = f'Time to Impact: {event[2]:.2f} s.'
            cv.putText(more_info, txt, (xy[0], xy[1] + 20 * it), font, scale, color)
            it += 1
            if "cone" not in event[0] and "cone" not in event[1]:
                txt = f'Actions: stop {event[3]}'
                cv.putText(more_info, txt, (xy[0], xy[1] + 20 * it), font, scale, color)
            else:
                if "cone" in event[0]: event_to_halt = event[1]
                else: event_to_halt = event[0]
                txt = f'Actions: stop {event_to_halt}'
                cv.putText(more_info, txt, (xy[0], xy[1] + 20 * it), font, scale, color)

            it += 1

        self.info = more_info

    # This function can ONLY be used by Grid
    # Updates vehicle object
    def update_vehicle(self, vehicle="", **kwargs):
        kwarg_keys = kwargs.keys()
        vehicle = self.get_vehicle_by_id(vehicle)
        if not vehicle:
            print("Vehicle to update is not connected.")
            return
        
        if "ll" in kwarg_keys:
            vehicle.ll = kwargs["ll"]
        if "heading" in kwarg_keys:
            vehicle.heading = kwargs["heading"]
        if "ang_z" in kwarg_keys:
            vehicle.ang_z = kwargs["ang_z"]
        if "velocity" in kwarg_keys:
            vehicle.velocity = kwargs["velocity"]

    # This function converts lat, long to x, y
    # This function is a deck of cards, so please
    # do not touch it unless you are absolutely
    # sure what you are doing
    def ll2xy(self, ll):
        x, y = self.shape_xy
        shape_lat, shape_lon = self.shape_ll
        x_lon_ratio = x / shape_lon
        y_lat_ratio = y / shape_lat

        lat, lon = ll
        slat, slon = self.ll
        dlat, dlon = lat - slat, lon - slon

        x = x_lon_ratio * dlon
        y = y_lat_ratio * dlat

        return x, y

    # Bool, returns True if xy is in the bounds of the square
    def does_bound_xy(self, xy):
        x, y = xy
        sxl, syt = self.xy
        dx, dy = self.shape_xy
        sxr, syb = sxl + dx, syt + dy

        if sxl <= x <= sxr:
            if syt <= y <= syb:
                return True

    # This function can ONLY be used by Grid
    # Returns a tuple of keys
    # key1 is for Grid's map
    # key2 is for LargeSquare's map
    def ll2keys(self, ll):
        rtr = False
        rtr2 = False
        xy = self.ll2xy(ll)
        for key in self.map.keys():
            if self.map[key].does_bound_xy(xy):
                rtr = key
                break

        if not rtr: return rtr, rtr2

        for key in self.map[rtr].map.keys():
            if self.map[rtr].map[key].does_bound_xy(xy):
                rtr2 = key
                break
        return rtr, rtr2
    
    # This function can ONLY be used by Grid
    # Also returns a tuple of keys
    # useful for creating a path line in x,y
    # then finding the underlying squares
    def xy2keys(self, xy):
        rtr = False
        rtr2 = False
        for key in self.map.keys():
            if self.map[key].does_bound_xy(xy):
                rtr = key
                break

        for key in self.map[rtr].map.keys():
            if self.map[rtr].map[key].does_bound_xy(xy):
                rtr2 = key
                break
        return rtr, rtr2

    # Converts 2 x,y coordinates to meter, meter coordinates
    # then finds the shortest distance between them
    # this distance is in meters
    # used for predicting time to impact
    def xy2meter(self, xy1, xy2):
        x_meters, y_meters = self.shape_meters
        sx, sy = self.shape_xy
        x_meters_ratio = x_meters / sx
        y_meters_ratio = y_meters / sy

        x, y = xy1
        metx1, mety1 = x_meters_ratio * x, y_meters_ratio * y

        x, y = xy2
        metx2, mety2 = x_meters_ratio * x, y_meters_ratio * y

        a, b = metx2 - metx1, mety2 - mety1
        c = math.sqrt(a**2 + b**2)

        return c
    
    # Predicts time to impact
    def time_to_impact(self, xy1, v1, xy2, v2, square):
        car1_dist2square = self.xy2meter(xy1, square.xy)
        car2_dist2square = self.xy2meter(xy2, square.xy)

        if v1 != 0: car1_time2square = car1_dist2square / v1
        else: car1_time2square = 0
        if v2 != 0: car2_time2square = car2_dist2square / v2
        else: car2_time2square = 0

        if car1_time2square > car2_time2square:
            return car1_time2square
        else:
            return car2_time2square
    
    # Querys vehicles by their id's
    def get_vehicle_by_id(self, id):
        for vic in self.VEHICLES:
            if vic.id == id:
                return vic
        return False
    
    def fresh_image(self):
        self.image = np.copy(self.image_copy)

    # deoccupies all squares, then redraws all squares,
    # updates their colors, 
    # draws newly unoccupied squares,
    # occupies up to date squares,
    # creates vehicle path
    # changes colors,
    # draws occupied squares,
    # detects collisions
    # creates event messages
    def update_vehicle_squares(self):

        # Map is refreshed to empty grids
        # Events is made empty
        # Note, will need to generate subgrid every flip
        self.fresh_image()
        self.EVENTS = []

        for vic in self.VEHICLES:
            # Update squares with vehicles
            vic_ll = vic.ll
            vic_key1, vic_key2 = self.ll2keys(vic_ll)
            if not vic_key1 or not vic_key2: continue

            big_square = self.map[vic_key1]
            big_square.num_vehicles += 1
            big_square.is_occupied = True

            small_square = big_square.map[vic_key2]
            small_square.num_vehicles += 1
            small_square.is_occupied = True
            small_square.occupants.append(vic.id)
            small_square.occ_obj = vic
            collision = small_square.update()

            if collision:
                self.handle_collision(small_square)

            # Update squares with paths
            center_of_square = \
            (
                small_square.xy[0] + small_square.shape_xy[0] / 2,
                small_square.xy[1] + small_square.shape_xy[1] / 2
            )

            xstep, ystep = \
            (
                small_square.shape_xy[0] * self.velocity_sensitivity,
                small_square.shape_xy[1] * self.velocity_sensitivity,
            )

            paths_found = []
            heading = vic.heading - 90
            steering_angle = - vic.ang_z

            for meters in range(int(vic.velocity) * 2):
                heading += steering_angle * meters * self.yaw_sensitivity
                heading += steering_angle * meters * self.yaw_sensitivity
                x_angle = math.radians(heading)
                y_angle = math.radians(heading)

                dx = math.cos(x_angle)
                dy = math.sin(y_angle)
                path_x = center_of_square[0] + xstep * meters * dx
                path_y = center_of_square[1] + ystep * meters * dy
                path_key1, path_key2 = self.xy2keys((path_x, path_y))
                if path_key1 != vic_key1 or path_key2 != vic_key2:
                    if not path_key1 or not path_key2:
                        continue
                    combo = path_key1 + path_key2
                    if combo in paths_found:
                        continue
                    paths_found.append(combo)
                    path_square = self.map[path_key1].map[path_key2]
                    path_square.num_paths += 1
                    path_square.occupants.append(vic.id)
                    collision = path_square.update()
                    if collision:
                        self.handle_collision(path_square)
                    path_square.draw(self.image)
            
            big_square.draw(self.image)

    def handle_collision(self, path_square):
        event = path_square.occupants

        vic1 = self.get_vehicle_by_id(event[0])
        vic2 = self.get_vehicle_by_id(event[1])
        vic1xy = self.ll2xy(vic1.ll)
        vic2xy = self.ll2xy(vic2.ll)

        time2impact = self.time_to_impact(
            vic1xy,
            vic1.velocity,
            vic2xy,
            vic2.velocity,
            path_square
        )
        event.append(time2impact)
        if vic1.velocity > vic2.velocity \
            and vic2.velocity != 0:
            event.append(vic2.id)
        else:
            event.append(vic1.id)
        self.EVENTS.append(event)
        