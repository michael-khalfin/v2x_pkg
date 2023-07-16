import sys
sys.path.append("./grid_functions")

from grid_functions import GridFunctions
from small_square import SmallSquare

# If you read the comments in grid.py these are going to be similar

class LargeSquare(GridFunctions):

    def __init__(self, xy, shape_xy, shape_meters):

        # Occupancy info
        self.num_vehicles = 0
        self.num_obstacles = 0
        self.num_paths = 0
        self.is_occupied = False

        # Drawing info
        self.color = (255,0,0)
        self.thickness = 2
        
        # Location / shape info
        self.xy = xy
        self.shape_xy = shape_xy
        self.shape_meters = shape_meters

        # Sub grid info
        self.n_col = self.shape_meters[0]
        self.n_row = self.shape_meters[1]
        self.sub_meters = (1,1)
        self.map = {}
        self.sub_type = SmallSquare

        # Generate all of the sub grids for this square
        self.generate_sub_grid()

    # Change color based upon occupancy, 
    # then draw self and sub grids
    def draw(self, image):
        if self.num_obstacles > 0:
            self.color = (100,0,0)
        elif self.num_vehicles > 0:
            self.color = (0,255,0)
        return super().draw(image)
    
    # Generates all sub grids for this square
    def generate_sub_grid(self):
        super().generate_sub_grid()