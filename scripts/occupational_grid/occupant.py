class Occupant:
    def __init__(self, id, tag, ll, heading=0, velocity=0, ang_z=0):
        self.id = id
        self.ll = ll
        self.heading = heading
        self.velocity = velocity
        self.ang_z = ang_z
        self.tag = tag