class ControlLegRobot:
    def __init__(self,ID_leg,position,rangePosition):
        self.ID_leg         = ID_leg
        self.position       = {"knee":position[0],"hip":position[1]}
        self.rangePosition  = {"knee":rangePosition[0],"hip":rangePosition[1]}
        self.force          = 0
    def __str__(self):
        return f"ID : {self.ID_leg} > Knee : {self.position['knee']} and Hip : {self.position['hip']}"