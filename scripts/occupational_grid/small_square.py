import sys
sys.path.append("./grid_functions")

from grid_functions import GridFunctions

# If you read the comments in grid.py or large_square.py
# These will be similar

class SmallSquare(GridFunctions):

    def __init__(self, xy, shape_xy, shape_meters):

        # Occupant info
        # This has more info than LargeSquare because
        # it is where collisions will be detected
        self.num_vehicles = 0
        self.num_obstacles = 0
        self.num_paths = 0
        self.collision = False
        self.is_occupied = False
        self.occupants = []
        self.occ_obj = None

        # Location / Shape info
        self.xy = xy
        self.shape_xy = shape_xy
        self.shape_meters = shape_meters

        # Drawing info
        self.color = (100,100,100)
        self.thickness = 1

        # self.map so self.draw doesn't get angry
        self.map = {}

    # Update the vehicle
    # As I mentioned in grid.py, we are not drawing
    # every square every time the map is updated.
    # We are only drawing what is changing, that is
    # why this function exists.
    # override is not in use right now, it is there
    # for when the square detects a collision, but 
    # calculations prove that the paths intersect
    # safely. This may never actually happend at
    # the speeds we are travelling on the course.
    def update(self, override=False):

        if len(self.occupants) > 1:
            self.collision = True
            self.color = (0,0,255) 
            self.thickness = -1
            return self.collision
        
        if self.num_vehicles == 0:
            self.color = (100,100,100)
            self.thickness = 1
        elif self.num_vehicles == 1:
            self.color = (0,200,0)
            self.thickness = -1
        
        if self.occ_obj != None and self.occ_obj.tag == "o":
            self.color = (0,165,255)
            self.thickness = -1
        
        if self.num_paths == 1:
            self.color = (0, 250, 255)
            self.thickness = -1

        if self.color == (0,0,255): self.collision = True

        return self.collision