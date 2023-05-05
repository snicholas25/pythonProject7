from autoroutine import AutoRoutine
from wpimath.controller import PIDController


class DriveStraight(AutoRoutine):

    def __init__(self, drivetrain, goal_in_meters):
        self.drivetrain = drivetrain
        self.goal = goal_in_meters
        self.pid_controller_straight=PIDController(20, 0, 0)
        self.pid_controller_dist = PIDController(20, 0, 0)
        self.pid_controller_straight.setSetpoint(0)
        self.pid_controller_dist.setSetpoint(goal_in_meters)
        self.pid_controller_straight.setTolerance(.05)
        self.pid_controller_dist.setTolerance(.05)


        #self.kp = -20

    def run(self):

        difference = self.drivetrain.getLeftDistanceMeter() - self.drivetrain.getRightDistanceMeter()
        rotate=self.pid_controller_straight.calculate(difference)
        forward = self.pid_controller_dist.calculate(self.drivetrain.averageDistanceMeter())
        if self.pid_controller_straight.atSetpoint():
            rotate=0
        if self.pid_controller_dist.atSetpoint():
            self.drivetrain.arcadeDrive(0, 0)
        else:
            #rotate = difference * self.kp
            # rotate=0
            print(
                f"Fwd: {forward}, Rot: {rotate}  distance:{self.drivetrain.averageDistanceMeter()} difference:{difference}")
            self.drivetrain.arcadeDrive(rotate, forward)