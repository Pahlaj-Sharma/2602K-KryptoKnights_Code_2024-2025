''' ---------------------------------------------------------------------------- 
                                                                              
 	Copyright © Pahlaj Sharma 2024-2025, All rights reserved.                  
    Module:       main.py                                                      
 	Author:       Pahlaj Sharma                                                
 	Created:      4/3/2024, 4:17:56 PM                                         
 	Description:  2602K KryptoKnights Competition Code
    GitHub: https://github.com/Pahlaj-Sharma/KryptoKnights_RobotCode_2602K                               
                                                                              
 ------------------------------------------------------------------------------- '''

from vex import *  # Import Modules
brain = Brain()  # Define Brain
mainController = Controller(PRIMARY)  # Define Controller

# Middle Right Top Drivetrain Motor Normal 6:1 
frontRight = Motor(Ports.PORT19, GearSetting.RATIO_6_1, False)
# Middle Right Bottom Drivetrain Motor Reverse 6:1
middleRight = Motor(Ports.PORT3, GearSetting.RATIO_6_1, False)
# Back Right Drivetrain Motor Normal 6:1
backRight = Motor(Ports.PORT7, GearSetting.RATIO_6_1, False)
# Middle Top Left Drivetrain Motor Reverse 6:1
frontLeft = Motor(Ports.PORT1, GearSetting.RATIO_6_1, True)
# Middle Bottom Left Drivetrain Motor Normal 6:1
middleLeft = Motor(Ports.PORT4, GearSetting.RATIO_6_1, True)
# Back Left Drivetrain Motor Reverse 6:1
backLeft = Motor(Ports.PORT6, GearSetting.RATIO_6_1, True)
# Intake
intake = Motor(Ports.PORT8, GearSetting.RATIO_6_1, False)
# Arm Motor Group
armGroup = Motor(Ports.PORT20, GearSetting.RATIO_18_1, True)
# Inertial Sensor - Gyro
inertial = Inertial(Ports.PORT17)
# Three Wire Expander
wireExpander = Triport(Ports.PORT9)
# Limit Switch Autonomous Selecter
autonomousSelector = Limit(wireExpander.h)
# Parallel Tracking Wheel Odometry
odomParallel = Rotation(Ports.PORT5)
# Mogo Pneumatic
mogo = DigitalOut(wireExpander.f)
# Ring Color Detector
ringSorter = Optical(Ports.PORT2)
# Arm Rotation Sensor
armSensor = Rotation(Ports.PORT21)
# Doinger
doinger = DigitalOut(wireExpander.g)
# Team Picker
teamPicker = Limit(wireExpander.e)
# Intake Picker
intakePickerPnuematic = DigitalOut(wireExpander.c)
intakePickerPnuematic.set(True)
# Odom Distance
'''
front = Distance(Ports.PORT15) # 4.6in 127mm: 
right = Distance(Ports.PORT14) # 5.8in 147.32mm
left = Distance(Ports.PORT16) # 5.8in 147.32mm
back = Distance(Ports.PORT8) # 6in 152.4mm
'''

# Clear Screen #
wait(230, MSEC)
print("\033[2J")
brain.screen.clear_screen()
mainController.screen.clear_screen()

# Define Global Variables and Constants #
# Define PID Constants #

FORWARD_KP: float = 1.3
FORWARD_KI: float = 0.1
FORWARD_KD: float = 0.5
TURN_KP: float = 3.5
TURN_KI: float = 0.05
TURN_KD: float = 5
MM_PER_TICK = 0.4767 #0.47 # Wheel Diameter(MM) * PI / Ticks(360) How much the robot travels after one rotation of the rotation sensor
# Define Enabled Motors #
frontRightMotor: bool = True
middleRightMotor: bool = True
backRightMotor: bool = True
frontLeftMotor: bool = True
middleLeftMotor: bool = True
backLeftMotor: bool = True
# Define button booleans
intakeSpin: bool = False
mogoClamp: bool = False
armPicked: bool = False
doingerBool: bool = False
intakePicker: bool = False
team: int = 1 # 0: Red; 1: Blue
tempTeam: int = team # For better arm
odomParallel.reset_position()
autonselect = 1


class Drive_Train_Control:
    # DriveTrain Functions #

    def __init__(self, rm, lm, wd, gr) -> None:
        self.rm = rm  # Left Motors
        self.lm = lm  # Right Motors
        self.wd = wd  # Wheel Diameter
        self.gr = gr  # Gear Ratio

    def spin(self, directionRightMotors, directionLeftMotors, speedLeftMotors: float, speedRightMotors: float, unit) -> None:
        # Drive Motors #
        frontRight.spin(directionRightMotors, speedRightMotors, unit)
        middleRight.spin(directionRightMotors, speedRightMotors, unit)
        backRight.spin(directionRightMotors, speedRightMotors, unit)
        frontLeft.spin(directionLeftMotors, speedLeftMotors, unit)
        middleLeft.spin(directionLeftMotors, speedLeftMotors, unit)
        backLeft.spin(directionLeftMotors, speedLeftMotors, unit)

    def stop(self) -> None:
        # Stop Motors #
        frontRight.set_velocity(0, RPM)
        middleRight.set_velocity(0, RPM)
        backRight.set_velocity(0, RPM)
        frontLeft.set_velocity(0, RPM)
        middleLeft.set_velocity(0, RPM)
        backLeft.set_velocity(0, RPM)

    def stopping(self, typeRightMotors = COAST, typeLeftMotors = COAST) -> None:
        # Set Stopping of Motors #
        frontRight.set_stopping(typeRightMotors)
        middleRight.set_stopping(typeRightMotors)
        backRight.set_stopping(typeRightMotors)
        frontLeft.set_stopping(typeLeftMotors)
        middleLeft.set_stopping(typeLeftMotors)
        backLeft.set_stopping(typeLeftMotors)

    def velocity(self, speedRightMotors: float, speedLeftMotors: float, unit) -> None:
        # Set Motor Velocity #
        frontRight.set_velocity(speedRightMotors, unit)
        middleRight.set_velocity(speedRightMotors, unit)
        backRight.set_velocity(speedRightMotors, unit)
        frontLeft.set_velocity(speedLeftMotors, unit)
        middleLeft.set_velocity(speedLeftMotors, unit)
        backLeft.set_velocity(speedLeftMotors, unit)
    
    def get_currenttemperature(self) -> float:
        # Returns Average Temperature of the Drivetrain
        return round(func.average(frontRight.temperature(), middleRight.temperature(), backRight.temperature(), frontLeft.temperature(), middleLeft.temperature(), backLeft.temperature()), 1)
        
    # PID Function Forward #
    def forward(self, distanceInches: float, speedScale: float = 1, robotHeading: float = 0, timeout: int = 3000, KP: float = FORWARD_KP, KI: float = FORWARD_KI, KD: float = FORWARD_KD) -> None:
        """
            This function controls the robot to move forward a specified distance using a PID controller.

            Args:
                self: The current instance of the robot class.
                distanceInches: The target distance to travel in inches.
                speedScale: A factor to adjust the robot's speed (default: 1).
                robotHeading: The desired robot heading (orientation) in degrees (default: 0).
                timeout: The maximum time allowed for the movement in milliseconds (default: 3000).
                KP: Proportional gain for the PID controller (default: FORWARD_KP).
                KI: Integral gain for the PID controller (default: FORWARD_KI).
                KD: Derivative gain for the PID controller (default: FORWARD_KD).

            Returns:
                None
        """

        wait(50, MSEC)  # Short delay
        odomParallel.reset_position()  # Reset odometry position
        integral = 0  # Initialize integral term
        mmForward = distanceInches * 25.4  # Convert distance to millimeters
        lastGyroPos = inertial.rotation(DEGREES)  # Record initial robot heading
        error = mmForward - round(odomParallel.position(DEGREES) * MM_PER_TICK, 2)  # Calculate initial error
        startIntegral = error * 0.15  # Limit for integral term
        previousError = 0  # Initialize previous error
        brain.timer.clear()  # Clear the timer
        # PID control loop
        while math.fabs(error) >= 5 and brain.timer.time(MSEC) <= timeout:
            error = mmForward - round(odomParallel.position(DEGREES) * MM_PER_TICK, 2)  # Update error
            currentHeading = inertial.rotation(DEGREES) - lastGyroPos  # Calculate current heading change
            integral += (error if math.fabs(error) < startIntegral else 0)  # Update integral with anti-windup
            integral = func.limit(integral, -100, 100)  # Limit integral term
            derivative = error - previousError  # Calculate derivative
            previousError = error  # Update previous error
            
            # Calculate motor power with PID and speed scaling
            motorPower = func.limit(math.fabs(round((KP * error + KI * integral + KD * derivative) * speedScale)), 30, 400) * (1 if error > 0 else -1) 

            # Calculate heading correction
            driftCorrection = (robotHeading - currentHeading) * TURN_KP

            # Set motor speeds with PID output and heading correction
            dt.spin(FORWARD, FORWARD, motorPower + driftCorrection, motorPower - driftCorrection, RPM) 
            wait(25, MSEC)  # Short delay within the loop
            
        dt.stop() # Stop motors after loop completes or timeout
            
    # PID Function Turn #
    def turn(self, angleDegrees: float, speedScale: float = 1, timeout: int = 3000, KP: float = TURN_KP, KI: float = TURN_KI, KD: float = TURN_KD) -> None:
        """
            This function controls the robot to turn a specified angle using a PID controller.

            Args:
                self: The current instance of the robot class.
                angleDegrees: The target angle to turn in degrees.
                speedScale: A factor to adjust the robot's turning speed (default: 1).
                timeout: The maximum time allowed for the turn in milliseconds (default: 3000).
                KP: Proportional gain for the PID controller (default: TURN_KP).
                KI: Integral gain for the PID controller (default: TURN_KI).
                KD: Derivative gain for the PID controller (default: TURN_KD).

            Returns:
                None
        """
        
        wait(50, MSEC)  # Short delay
        integral = 0  # Initialize integral term
        # Calculate initial error with angle wrapping for accurate error calculation
        error = (((angleDegrees - inertial.heading(DEGREES)) + 180) % 360) - 180 
        startIntegral = error * 0.15  # Limit for integral term
        previousError = 0  # Initialize previous error
        brain.timer.clear()  # Clear the timer
        # PID control loop
        while math.fabs(error) >= 0.5 and brain.timer.time(MSEC) <= timeout:
            # Calculate error with angle wrapping
            error = (((angleDegrees - inertial.heading(DEGREES)) + 180) % 360) - 180 
            integral += (error if math.fabs(error) < startIntegral else 0)  # Update integral with anti-windup
            integral = func.limit(integral, -100, 100)  # Limit integral term
            derivative = error - previousError  # Calculate derivative
            previousError = error  # Update previous error

            # Calculate motor power with PID and speed scaling
            motorPower = func.limit(math.fabs(round((KP * error + KI * integral + KD * derivative) * speedScale)), 30, 400) * (1 if error > 0 else -1) 

            # Set motor speeds for turning (opposite directions)
            dt.spin(REVERSE, FORWARD, motorPower, motorPower, RPM) 
            wait(25, MSEC)  # Short delay within the loop
            
        dt.stop() # Stop motors after loop completes or timeout
        
    # PID Function Curve #
    def curve(self, distanceInches: float, distanceBefore_Turn: float, angleDegrees: float, correctionStrength: float, speedScale: float = 1, timeout: int = 3000, FKP: float = FORWARD_KP, FKI: float = FORWARD_KI, FKD: float = FORWARD_KD, TKP: float = TURN_KP, TKI: float = TURN_KI, TKD: float = TURN_KD) -> None:
        """
            This function controls the robot to drive forward a certain distance and then curve at a specified angle using PID controllers for both driving and turning.

            Args:
                self: The current instance of the robot class.
                distanceInches: The total distance to travel in inches.
                distanceBefore_Turn: The distance to travel before starting the turn in inches.
                angleDegrees: The target angle to turn in degrees.
                correctionStrength: A factor to adjust the strength of the turning correction.
                speedScale: A factor to adjust the robot's speed (default: 1).
                timeout: The maximum time allowed for the movement in milliseconds (default: 3000).
                FKP, FKI, FKD: Proportional, Integral, and Derivative gains for the driving PID controller (default: FORWARD_KP, FORWARD_KI, FORWARD_KD).
                TKP, TKI, TKD: Proportional, Integral, and Derivative gains for the turning PID controller (default: TURN_KP, TURN_KI, TURN_KD).

            Returns:
                None
        """

        wait(50, MSEC)  # Short delay
        odomParallel.reset_position()  # Reset Tracking Position
        integral_Drive = 0  # Initialize integral term for driving
        integral_Angle = 0  # Initialize integral term for turning
        mmForward = distanceInches * 25.4  # Convert distance to millimeters
        error_Drive = mmForward - round(odomParallel.position(DEGREES) * MM_PER_TICK, 2)  # Calculate initial driving error
        error_Angle = (((angleDegrees - inertial.heading(DEGREES)) + 180) % 360) - 180  # Calculate initial turning error (with angle wrapping)
        startIntegralDrive = error_Drive * 0.15  # Limit for driving integral term
        startIntegralAngle = error_Angle * 0.15  # Limit for turning integral term
        previousError_Drive = 0  # Initialize previous driving error
        previousError_Angle = 0  # Initialize previous turning error
        brain.timer.clear()  # Clear the timer
        # PID control loop
        while (math.fabs(error_Drive) >= 5 and math.fabs(error_Angle) >= 0.5) and brain.timer.time(MSEC) <= timeout:
            # Update driving error
            error_Drive = mmForward - round(odomParallel.position(DEGREES) * MM_PER_TICK, 2) 
            # Update driving integral with anti-windup
            integral_Drive += (error_Drive if math.fabs(error_Drive) < startIntegralDrive else 0) 
            integral_Drive = func.limit(integral_Drive, -100, 100)  # Limit driving integral
            derivative_Drive = error_Drive - previousError_Drive  # Calculate driving derivative
            previousError_Drive = error_Drive  # Update previous driving error

            # Calculate driving motor power with PID and speed scaling
            motorPower_Drive = func.limit(round(math.fabs(FKP * error_Drive + FKI * integral_Drive + FKD * derivative_Drive) * speedScale), 30, 400) * (1 if error_Drive > 0 else -1)

            # Update turning error with angle wrapping
            error_Angle = (((angleDegrees - inertial.heading(DEGREES)) + 180) % 360) - 180 
            # Update turning integral with anti-windup
            integral_Angle += (error_Angle if math.fabs(error_Angle) < startIntegralAngle else 0) 
            integral_Angle = func.limit(integral_Angle, -100, 100)  # Limit turning integral
            derivative_Angle = error_Angle - previousError_Angle  # Calculate turning derivative
            previousError_Angle = error_Angle  # Update previous turning error

            # Calculate turning motor power with PID, speed scaling, and correction strength
            # Activate turning correction only after driving a certain distance or when turning error is significant
            angleOutput = 1 if ((math.fabs(round(odomParallel.position(DEGREES) * MM_PER_TICK, 2)) * 0.04 >= distanceBefore_Turn) or not abs(error_Angle) < 0.5) else 0 
            motorPower_Angle = func.limit(round(math.fabs(TKP * error_Angle + TKI * integral_Angle + TKD * derivative_Angle) * correctionStrength * angleOutput * speedScale), 30, 400) * (1 if error_Angle > 0 else -1)

            # Calculate individual motor powers for differential drive
            rightMotorPower = motorPower_Drive - motorPower_Angle
            leftMotorPower = motorPower_Drive + motorPower_Angle

            # Set motor speeds
            dt.spin(FORWARD, FORWARD, leftMotorPower, rightMotorPower, RPM) 
            wait(25, MSEC)  # Short delay within the loop
            
        dt.stop() # Stop motors after loop completes or timeout

class Miscellaneous_Functions:
    # Miscellaneous Functions #

    def __init__(self) -> None:
        self.self = self
        
    def robotinformation(self) -> None:
        global autonOptions, autonselect, team
        # Prints Battery, Drivetrain Temperature, and Disconnected Motors #
        motorNames = ["frontRight", "middleRight", "backRight", "frontLeft", "middleLeft", "backLeft"]
        autonOptions = {
        1: "Skills",
        2: "Audience Side AWP",
        3: "Audience Side Two Ring",
        4: "Far Side 5 Ring AWP",
        5: "Far Side Alliance Stake",
        6: "Audience Side Alliance Stake Elims",
        7: "Audience Side Elims",
        8: "Far Side 5 Ring Elims",
        9: "Far Side Alliance Stake Elims" 
    }
        while True:
            mainController.screen.set_cursor(1, 1)
            mainController.screen.print("  ", brain.battery.capacity(), "%    ", dt.get_currenttemperature(), "  ", round(intake.temperature(), 1), sep='')
            mainController.screen.set_cursor(2, 1)
            mainController.screen.print("RED   " if team == 0 else "BLUE   ", autonOptions.get(autonselect))
            mainController.screen.set_cursor(3, 1)
            if dt.get_currenttemperature() > 65:
                mainController.rumble("-.-.")
            DriveTrainMotors_Installed = [frontRight.installed(), middleRight.installed(), backRight.installed(), frontLeft.installed(), middleLeft.installed(), backLeft.installed()]
            if not all(DriveTrainMotors_Installed):
                for i, list in enumerate(list):
                    if not list:
                        disconnectedMotor = motorNames[i]
                        oppositeMotor = motorNames[i].replace("Left", "Right") if "Left" in motorNames[i] else motorNames[i].replace("Right", "Left")
                        mainController.screen.print(disconnectedMotor.capitalize(), " DISCONNECTED, DISABLING ", oppositeMotor.capitalize(), sep='')
                        globals()[oppositeMotor + "Motor"] = False
            wait(5000, MSEC)
            mainController.screen.clear_screen()
            
    def odometry(self, OffsetX = 0, OffsetY = 0, offSetHeading = 0) -> None:
        # Updates Odometry Pos #
        global currentXPos, currentYPos, parallelTrackerPos, inertialRotation, inertialHeading
        currentXPos = OffsetX
        currentYPos = OffsetY
        lastParallelTrackerPos = 0
        inertial.set_heading(((offSetHeading - inertial.heading() + 180) % 360) - 180)
        inertial.set_rotation(offSetHeading)
        MM_TO_INCHES = 0.04
        while True:
            parallelTrackerPos = odomParallel.position() * MM_PER_TICK
            inertialHeading = round(inertial.heading(DEGREES), 2)
            inertialRotation = round(inertial.rotation(DEGREES), 2)
            currentXPos += round((((parallelTrackerPos - lastParallelTrackerPos) * round(math.cos(math.radians(inertialRotation)), 2)) * MM_TO_INCHES) + OffsetX, 2)
            currentYPos += round((((parallelTrackerPos - lastParallelTrackerPos) * round(math.sin(math.radians(inertialRotation)), 2)) * MM_TO_INCHES) + OffsetY, 2)
            lastParallelTrackerPos = parallelTrackerPos
            wait(25, MSEC)
            
    def colorSort(self) -> None:
        global colorSort
        ringSorter.gesture_disable()
        ringSorter.set_light(LedStateType.ON)
        ringSorter.set_light_power(20, PERCENT)
        colorSort = True
        # Color Sort Logic
        # 0: Red; 1: Blue
        #650 degree is one revolution
        while True:
            if intake.velocity(RPM) > 40 and (not mainController.buttonL2.pressing()) and (not colorSort == False):
                if (ringSorter.color() == Color.BLUE) and team == 0:
                    wait(125, MSEC)
                    intake.stop()
                    wait(100, MSEC)
                    intake.spin(FORWARD)
                elif (ringSorter.color() == Color.RED) and team == 1:
                    wait(125, MSEC)
                    intake.stop()
                    wait(100, MSEC)
                    intake.spin(FORWARD)
            wait(50, MSEC)
            
    def average(self, *values) -> float:
        # Returns the Average of a List of Numbers #
        return round(sum(values)/len(values), 2)
    
    def limit(self, value, min, max) -> int:
        # Limits a Number between a Minimum and a Maximum #
        return (value if min <= value <= max else min if value < min else max)
    
    def calculate_distance(self, x1, y1, x2, y2) -> float:
        # Calculates the Distance between two Points #
        return round(math.sqrt((x2 - x1) ** 2 + (y2 - y1) ** 2), 2)
    
    def calculate_angle(self, x1, y1, x2, y2) -> float:
        # Calculates Angle between two Points #
        return (round(math.degrees(math.atan2(y2 - y1, x2 - x1)), 1) - inertialHeading) % 360
            
    def armSpinTo(self, angle, scale: float = 1, settleError: float = 1) -> None:
        """
            This function controls the arm to spin to a target angle using a PID controller.

            Args:
                self: The current instance of the robot class.
                angle: The target angle in degrees.
                scale: A factor to adjust the arm's speed (default: 1).
                settleError: The maximum allowable error before the arm is considered settled (default: 1 degree).

            Returns:
                None
        """
        # Initialize variables
        integral = 0  # Initialize integral term
        previousError = 0  # Initialize previous error
        if armSensor.position(DEGREES) > 300:  # Handle potential sensor wrap-around
            armSensor.set_position(0, DEGREES) 
        error = angle - armSensor.position(DEGREES)  # Calculate initial error
        brain.timer.clear()  # Clear the timer
        # PID control loop
        while math.fabs(error) >= settleError and brain.timer.time(MSEC) <= 1500: 
            # Calculate error with angle wrapping
            error = (((angle - armSensor.position(DEGREES)) + 180) % 360) - 180 
            integral += error  # Update integral term
            integral = func.limit(integral, -100, 100)  # Limit integral term
            derivative = error - previousError  # Calculate derivative

            # Integral reset on sign change to prevent windup
            if (error > 0 and previousError < 0) or (error < 0 and previousError > 0):
                integral = 0 
                
            previousError = error  # Update previous error
            # Calculate arm velocity using PID and speed scaling
            armVelocity = (error * 4 + integral * 0.001 + derivative * 0.3) * scale 

            # Set arm velocity and spin direction
            armGroup.set_velocity(armVelocity, RPM)
            armGroup.spin(FORWARD) 
            wait(25, MSEC)  # Short delay within the loop

        armGroup.stop()  # Stop the arm after reaching the target or timeout

        
def when_started1() -> None:
    global dt, func, autonselect, robotInformationThread, team, tempTeam, runOdometry, colorSorterThread, autonOptions
    # Declare DriveTrain Settings - Right Motors, Left Motors, Wheel Diameter, Gear Ratio
    dt = Drive_Train_Control((frontRight, middleRight, backRight), (
        frontLeft, middleLeft, backLeft), 2.75, 1.33)
    func = Miscellaneous_Functions() # Declares callable for Miscellaneous Functions
    robotInformationThread = Thread(func.robotinformation) # Starts Thread for Robot Information
   #colorSorterThread = Thread(func.colorSort) # Starts Thread for Color Sorting
    dt.stopping() # Set Stopping to Coast
    inertial.calibrate() # Calibrate Inertial Sensor
    while inertial.is_calibrating():
        sleep(50)
    autonOptions = {
        1: "Skills",
        2: "Audience Side AWP",
        3: "Audience Side Two Ring",
        4: "Far Side 5 Ring AWP",
        5: "Far Side Alliance Stake",
        6: "Audience Side Elims",
        7: "Far Side 5 Ring Elims",
        8: "Far Side Alliance Stake Elims"
    }
    brain.screen.print_at("RED   " if team == 0 else "BLUE   ", autonOptions.get(autonselect), x=30, y=30)
    mainController.screen.set_cursor(2, 1)
    mainController.screen.print("RED   " if team == 0 else "BLUE   ", autonOptions.get(autonselect))
    # Autonomous Selector Logic/Team Selector #
    while not (frontLeft.velocity(RPM) > 20):
        if autonomousSelector.pressing() or teamPicker.pressing():
            brain.screen.clear_screen()
            mainController.screen.clear_row(2)
            if autonomousSelector.pressing():
                autonselect = (autonselect + 1) % 9
                wait(150, MSEC)
            if teamPicker.pressing():
                team = (team + 1) % 2
                tempTeam = team
                wait(150, MSEC)
            brain.screen.print_at("RED   " if team == 0 else "BLUE   ", autonOptions.get(autonselect), x=30, y=30)
            mainController.screen.set_cursor(2, 1)
            mainController.screen.print("RED   " if team == 0 else "BLUE   ", autonOptions.get(autonselect))
            wait(100, MSEC)
        wait(50, MSEC)


def onauton_autonomous_0() -> None:
    global autonselect, team, colorSort
    dt.stopping(BRAKE, BRAKE)
    # Runs when Competition Switch is on Autonomous #
    # Use Path.jerry.io, and make sure to add 72 to the x and y coords for this code (0, 0) is the red audience side positive
    if autonselect == 1:
        # Skills
        inertial.set_rotation(90)
        inertial.set_heading(90)
        wait(50, MSEC)
        team = 0 # Set to red for skills
        intake.set_velocity(400, RPM)
        wait(50, MSEC)
        intake.spin(FORWARD)
        wait(250, MSEC)
        intake.set_velocity(200, RPM)
        intake.spin(REVERSE)
        wait(50, MSEC)
        dt.forward(13.5)
        dt.turn(0)
        dt.forward(-24, 0.45)
        mogo.set(True)
        intake.set_velocity(400, RPM)
        intake.spin(FORWARD)
        dt.turn(90, 0.95)
        dt.forward(17)
        dt.turn(136)
        dt.forward(48)
        dt.turn(180)
        wait(100, MSEC)
        dt.forward(-6.5)
        dt.turn(270, 0.9)
        wait(50, MSEC)
        intake.set_velocity(550, RPM)
        dt.forward(60, 0.4)
        intake.set_velocity(400, RPM)
        dt.turn(135)
        dt.forward(9)
        dt.turn(90)
        intake.stop()
        dt.curve(-15, 5, 45, 1.5, 1, 1000)
        mogo.set(False)
        dt.forward(11)
        dt.turn(180, 0.9)
        dt.forward(-75, 0.45)
        #
        mogo.set(True)
        intake.set_velocity(400, RPM)
        intake.spin(FORWARD)
        dt.turn(90, 0.95)
        dt.forward(17)
        dt.turn(44)
        dt.forward(48)
        dt.turn(0)
        wait(100, MSEC)
        dt.forward(-6.5)
        dt.turn(270, 0.9)
        wait(50, MSEC)
        intake.set_velocity(550, RPM)
        dt.forward(60, 0.4)
        intake.set_velocity(400, RPM)
        dt.turn(45)
        dt.forward(5)
        dt.turn(90)
        dt.curve(-15, 5, 135, 27, 1, 1000)
        mogo.set(False)
        intake.spin(REVERSE, 200, RPM)
        dt.forward(9)
        intake.spin(FORWARD, 200, RPM)
        dt.turn(90)
        dt.forward(70)
        dt.turn(140)
        dt.forward(63)
        dt.turn(180)
        dt.curve(-40, -10, 225, 0.7)
        dt.curve(10, 5, 180, 0.7)
        dt.spin(FORWARD, FORWARD, 350, 400, RPM)
        wait(2, SECONDS)
        dt.stop()
        dt.curve(20, 5, 135, 0.7)
        
    elif autonselect == 2:
        # Audience Side AWP
        if team == 1: # Blue
            inertial.set_rotation(0, DEGREES)
            inertial.set_heading(0, DEGREES)
            wait(50, MSEC)
            armGroup.set_stopping(HOLD)
            dt.turn(25, 0.9)
            func.armSpinTo(148, 1.5, 1.5)
            armGroup.set_stopping(COAST)
            armspin = Thread(func.armSpinTo, (5, 1.5, 7))
            dt.turn(320, 1.1)
            intakePickerPnuematic.set(False)
            intake.set_velocity(400, RPM)
            intake.spin(FORWARD)
            dt.forward(17)
            armspin.stop()
            intakePickerPnuematic.set(True)
            wait(200, MSEC)
            intake.stop()
            dt.turn(40)
            dt.curve(-28, 3, 55, 0.8, 0.8)
            dt.forward(-8, 0.6)
            mogo.set(True)
            intake.spin(FORWARD)
            dt.turn(180)
            dt.forward(27)
            wait(1200, MSEC)
            dt.turn(0)
            dt.curve(27.5, 5, 315, 0.8)
            func.armSpinTo(148, 1)
            intake.stop()
        else: # Red
            inertial.set_rotation(0, DEGREES)
            inertial.set_heading(0, DEGREES)
            wait(50, MSEC)
            armGroup.set_stopping(HOLD)
            dt.turn(335, 0.9)
            func.armSpinTo(148, 1.5, 1.5)
            armGroup.set_stopping(COAST)
            armspin = Thread(func.armSpinTo, (5, 1.5, 7))
            dt.turn(40, 1.1)
            intakePickerPnuematic.set(False)
            intake.set_velocity(400, RPM)
            intake.spin(FORWARD)
            dt.forward(17)
            armspin.stop()
            intakePickerPnuematic.set(True)
            wait(200, MSEC)
            intake.stop()
            dt.turn(320)
            dt.curve(-28, 3, 305, 0.8, 0.8)
            dt.forward(-8, 0.6)
            mogo.set(True)
            intake.spin(FORWARD)
            dt.turn(180)
            dt.forward(27)
            wait(1200, MSEC)
            dt.turn(0)
            dt.curve(27.5, 5, 45, 0.8)
            func.armSpinTo(148, 1)
            intake.stop()
        
    elif autonselect == 3:
        # Audience Side Two Ring
        if team == 1:
            inertial.set_rotation(90)
            inertial.set_heading(90)
            wait(50, MSEC)
            dt.curve(-21, 10, 50, 0.8)
            dt.forward(-8, 0.5)
            mogo.set(True)
            dt.turn(180)
            intake.set_velocity(400, RPM)
            intake.spin(FORWARD)
            dt.forward(27)
            wait(1200, MSEC)
            intake.set_velocity(400, RPM)
            intake.spin(FORWARD)
            dt.turn(0)
            dt.curve(27.5, 5, 315, 0.8)
            func.armSpinTo(148, 1)
            intake.stop()
        else:
            inertial.set_rotation(270)
            inertial.set_heading(270)
            wait(50, MSEC)
            dt.curve(-21, 10, 310, 0.8)
            dt.forward(-8, 0.5)
            mogo.set(True)
            dt.turn(180)
            intake.set_velocity(400, RPM)
            intake.spin(FORWARD)
            dt.forward(27)
            wait(1200, MSEC)
            intake.set_velocity(400, RPM)
            intake.spin(FORWARD)
            dt.turn(0)
            dt.curve(27.5, 5, 45, 0.8)
            func.armSpinTo(148, 1)
            intake.stop()
        
    elif autonselect == 4:
        # Far Side 5 Ring AWP
        if team == 1: # Blue
            inertial.set_rotation(90)
            inertial.set_heading(90)
            wait(50, MSEC)
            dt.curve(-21, 10, 130, 0.8)
            dt.forward(-8, 0.5)
            mogo.set(True)
            intake.set_velocity(400, RPM)
            intake.spin(FORWARD)
            intakePickerPnuematic.set(False)
            dt.curve(32, 5, 135, 0.7)
            intakePickerPnuematic.set(True)
            dt.forward(-5)
            wait(400, MSEC)
            dt.turn(337)
            dt.forward(40, 1.1)
            dt.forward(-10)
            dt.turn(277)
            dt.forward(19, 0.8)
            wait(100, MSEC)
            dt.curve(-15, 2, 250, 1.8)
            dt.turn(290)
            dt.forward(16)
            wait(200, MSEC)
            dt.forward(-7)
            dt.turn(180)
            dt.forward(20)
            func.armSpinTo(148, 1)
            intake.stop()
        else: # Red
            inertial.set_rotation(270)
            inertial.set_heading(270)
            wait(50, MSEC)
            dt.curve(-21, 10, 230, 0.8)
            dt.forward(-8, 0.5)
            mogo.set(True)
            intake.set_velocity(400, RPM)
            intake.spin(FORWARD)
            intakePickerPnuematic.set(False)
            dt.curve(32, 5, 225, 0.7)
            intakePickerPnuematic.set(True)
            dt.forward(-5)
            wait(400, MSEC)
            dt.turn(23)
            dt.forward(40, 1.1)
            dt.forward(-10)
            dt.turn(83)
            dt.forward(19, 0.8)
            wait(100, MSEC)
            dt.curve(-15, 2, 110, 1.8)
            dt.turn(70)
            dt.forward(16)
            wait(200, MSEC)
            dt.forward(-7)
            dt.turn(180)
            dt.forward(20)
            func.armSpinTo(148, 1)
            intake.stop()
        
    elif autonselect == 5:
        # Far Side Alliance Stake
        if team == 1: # Blue
            inertial.set_rotation(180, DEGREES)
            inertial.set_heading(180, DEGREES)
            wait(50, MSEC)
            armGroup.set_stopping(HOLD)
            dt.turn(155, 0.9)
            func.armSpinTo(148, 1.5, 1.5)
            armGroup.set_stopping(COAST)
            armspin = Thread(func.armSpinTo, (5, 1.5, 7))
            dt.turn(220, 1.1)
            intakePickerPnuematic.set(False)
            intake.set_velocity(400, RPM)
            intake.spin(FORWARD)
            dt.forward(17)
            armspin.stop()
            intakePickerPnuematic.set(True)
            wait(200, MSEC)
            intake.stop()
            dt.turn(140)
            dt.curve(-28, 3, 125, 0.8, 0.7)
            dt.forward(-8, 0.7)
            mogo.set(True)
            intake.set_velocity(400, RPM)
            intake.spin(FORWARD)
            dt.turn(0)
            dt.forward(26, 1.2)
            wait(50, MSEC)
            dt.forward(-10)
            dt.turn(276, 0.95)
            intake.set_velocity(500, RPM)
            intake.spin(FORWARD)
            dt.forward(16)
            wait(100, MSEC)
            dt.forward(-5.5)
            dt.turn(322)
            dt.forward(8)
            wait(100, MSEC)
            dt.forward(-7)
            dt.turn(185, 0.95)
            dt.forward(24, 1.1)
            intake.stop()
        else: # Red
            inertial.set_rotation(180, DEGREES)
            inertial.set_heading(180, DEGREES)
            wait(50, MSEC)
            armGroup.set_stopping(HOLD)
            dt.turn(205, 0.9)
            func.armSpinTo(148, 1.5, 1.5)
            armGroup.set_stopping(COAST)
            armspin = Thread(func.armSpinTo, (5, 1.5, 7))
            dt.turn(140, 1.1)
            intakePickerPnuematic.set(False)
            intake.set_velocity(400, RPM)
            intake.spin(FORWARD)
            dt.forward(17)
            armspin.stop()
            intakePickerPnuematic.set(True)
            wait(200, MSEC)
            intake.stop()
            dt.turn(220)
            dt.curve(-28, 3, 235, 0.8, 0.7)
            dt.forward(-8, 0.7)
            mogo.set(True)
            intake.set_velocity(400, RPM)
            intake.spin(FORWARD)
            dt.turn(0)
            dt.forward(26, 1.2)
            wait(50, MSEC)
            dt.forward(-10)
            dt.turn(84, 0.95)
            intake.set_velocity(500, RPM)
            intake.spin(FORWARD)
            dt.forward(16)
            wait(100, MSEC)
            dt.forward(-5.5)
            dt.turn(38)
            dt.forward(8)
            wait(100, MSEC)
            dt.forward(-7)
            dt.turn(175, 0.95)
            dt.forward(24, 1.1)
            intake.stop()
              
    elif autonselect == 6:
        # Audience Side Elims (Rush)
        if team == 1: # Blue
            inertial.set_heading(266)
            inertial.set_rotation(-94)
            dt.forward(41)
            doinger.set(True)
            wait(150, MSEC)
            dt.forward(-10)
            wait(50, MSEC)
            doinger.set(False)
            wait(100, MSEC)
            dt.forward(-16)
            dt.turn(145)
            dt.curve(-13, 4, 130, 0.8, 0.6)
            dt.forward(-5, 0.7)
            mogo.set(True)
            wait(50, MSEC)
            dt.turn(180)
            colorSort = False
            intake.set_velocity(500, RPM)
            intake.spin(FORWARD)
            dt.forward(28)
            wait(250, MSEC)
            dt.forward(-18)
            dt.turn(280)
            colorSort = True
            dt.curve(-25, 10, 272, 0.8)
            intake.stop()
            mogo.set(False)
            dt.curve(28, 10, 275, 1)
            intake.stop()
            doinger.set(True)
            wait(100, MSEC)
            dt.curve(-24, 21, 180, 1.1)
            intake.stop()
            doinger.set(False)
            dt.turn(180)
        else: # Red
            inertial.set_heading(94)
            inertial.set_rotation(-94)
            dt.forward(42)
            doinger.set(True)
            wait(180, MSEC)
            dt.curve(-60, 3, 250, 0.2)
            doinger.set(False)
            dt.turn(255)
            dt.curve(-35, 28, 270, 0.8, 0.6)
            dt.forward(-5, 0.7)
            mogo.set(True)
            wait(100, MSEC)
            dt.turn(180)
            wait(50, MSEC)
            intake.set_velocity(500, RPM)
            intake.spin(FORWARD)
            dt.forward(30)
            wait(200, MSEC)
            dt.forward(-5)
            dt.turn(260)
            intake.stop()
            dt.forward(17)
            '''
            doinger.set(True)
            dt.forward(35)
            dt.turn(305, 1.1)
            '''
            
    elif autonselect == 7:
        # Three Goal AWP
        if team == 1: # Blue
            ...
        else: # Red
            inertial.set_rotation(180, DEGREES)
            inertial.set_heading(180, DEGREES)
            wait(50, MSEC)
            armGroup.set_stopping(HOLD)
            dt.turn(205, 0.9)
            func.armSpinTo(146, 1.5, 1.5)
            armGroup.set_stopping(COAST)
            armspin = Thread(func.armSpinTo, (5, 1.5, 7))
            dt.turn(140, 1.1)
            intakePickerPnuematic.set(False)
            intake.set_velocity(500, RPM)
            intake.spin(FORWARD)
            dt.forward(17)
            armspin.stop()
            intakePickerPnuematic.set(True)
            wait(200, MSEC)
            intake.stop()
            dt.turn(220)
            dt.curve(-28, 3, 235, 0.8, 0.7)
            dt.forward(-8, 0.7)
            mogo.set(True)
            intake.set_velocity(400, RPM)
            intake.spin(FORWARD)
            dt.turn(5)
            dt.forward(26, 1.2)
            wait(50, MSEC)
            dt.turn(200, 1.1)
            dt.forward(65)
            intake.stop()
            mogo.set(False)
            dt.turn(300)
            dt.forward(-25, 0.5)
            mogo.set(True)
               
    elif autonselect == 8:
        #
        if team == 1: # Blue
            ...
        else: # Red
            ...
        

def ondriver_drivercontrol_0() -> None:
    # Runs when Competition Switch is on Driver Control #
    dt.stopping()
    deadband = 5
    intakePickerPnuematic.set(True)
    while True:
        controleraxis1pos = mainController.axis1.position() * 6
        controleraxis3pos = mainController.axis3.position() * 6
        if math.fabs(controleraxis1pos) + math.fabs(controleraxis3pos) > deadband:
            if math.fabs(mainController.axis3.position()) >= -100 and math.fabs(mainController.axis3.position()) <= 100:
                # Forward
                dt.spin(FORWARD, FORWARD, func.limit(controleraxis3pos + controleraxis1pos, -570, 570), func.limit(controleraxis3pos - controleraxis1pos, -570, 570), RPM)
            elif math.fabs(mainController.axis1.position()) >= -100 and math.fabs(mainController.axis1.position()) <= 100:
                # Turn
                dt.spin(FORWARD, FORWARD, func.limit((controleraxis3pos + controleraxis1pos) / 1.4, -570, 570), func.limit((controleraxis3pos - controleraxis1pos) / 1.4, -570, 570), PERCENT)
        else:
            dt.velocity(0, 0, PERCENT)
        wait(25, MSEC)
        
        
def onevent_controller_1buttonY_pressed_0():
    # Start and stop intake
    global intakeSpin
    intake.set_stopping(COAST)
    if intakeSpin:
        intakeSpin = False
        intake.stop()
    else:
        intakeSpin = True
        intake.set_velocity(400, RPM)
        intake.spin(FORWARD)


def onevent_controller_1buttonRight_pressed_0():
    # Up and Down Mogo
    global mogoClamp
    if mogoClamp:
        mogoClamp = False
        mogo.set(False)
    else:
        mogoClamp = True
        mogo.set(True)
        
        
def onevent_controller_1buttonR1_pressed_0():
    # Arm Control
    global armPicked, team
    if not armPicked and not armGroup.is_spinning():
        armGroup.set_stopping(HOLD)
        team = (team + 1) % 2
        func.armSpinTo(18, 1, 0.5)
        intake.set_velocity(400, RPM)
        intake.spin(FORWARD)
        armPicked = True
    else:
        if not armGroup.is_spinning():
            intake.set_velocity(380, RPM)
            intake.spin(REVERSE)
            wait(150, MSEC)
            intake.stop()
            wait(200, MSEC)
            armGroup.set_stopping(HOLD)
            func.armSpinTo(148, 1.5, 2)
            armPicked = False
            team = tempTeam
        

def onevent_controller_1buttonR2_pressed_0():
    # Arm Control
    global armPicked, team
    armGroup.set_stopping(COAST)
    team = tempTeam
    func.armSpinTo(3, 3, 5)
    armPicked = False
    
    
def onevent_controller_1buttonL1_pressed_0():
    global doingerBool
    if doingerBool:
        doingerBool = False
        doinger.set(False)
    else:
        doingerBool = True
        doinger.set(True)
        
        
def onevent_controller_1buttonA_pressed_0():
    # Up and Down Intake
    global intakePicker
    if intakePicker:
        intakePicker = False
        intakePickerPnuematic.set(True)
    else:
        intakePicker = True
        intakePickerPnuematic.set(False)
        

def onevent_controller_1buttonLeft_pressed_0():
    intake.set_velocity(250, RPM)
    intake.spin(REVERSE)
    while mainController.buttonLeft.pressing():
        wait(25, MSEC)
    while not mainController.buttonLeft.pressing():
        wait(25, MSEC)
    intake.stop()


def onevent_controller_1buttonUp_pressed_0():
    armGroup.set_velocity(150, RPM)
    armGroup.set_stopping(HOLD)
    armGroup.spin(FORWARD)
    while mainController.buttonUp.pressing():
        wait(25, MSEC)
    armGroup.stop()
    
    
def onevent_controller_1buttonDown_pressed_0():
    armGroup.set_velocity(150, RPM)
    armGroup.set_stopping(HOLD)
    armGroup.spin(REVERSE)
    while mainController.buttonDown.pressing():
        wait(25, MSEC)
    armGroup.stop()


def vexcode_auton_function() -> None:
    auton_task_0 = Thread(onauton_autonomous_0)
    while (competition.is_autonomous() and competition.is_enabled()):
        wait(10, MSEC)
    auton_task_0.stop()


def vexcode_driver_function() -> None:
    driver_control_task_0 = Thread(ondriver_drivercontrol_0)
    while (competition.is_driver_control() and competition.is_enabled()):
        wait(10, MSEC)
    driver_control_task_0.stop()
    robotInformationThread.stop()
    #colorSorterThread.stop()


competition = Competition(vexcode_driver_function, vexcode_auton_function)
mainController.buttonY.pressed(onevent_controller_1buttonY_pressed_0)
mainController.buttonRight.pressed(onevent_controller_1buttonRight_pressed_0)
mainController.buttonR1.pressed(onevent_controller_1buttonR1_pressed_0)
mainController.buttonR2.pressed(onevent_controller_1buttonR2_pressed_0)
mainController.buttonL1.pressed(onevent_controller_1buttonL1_pressed_0)
mainController.buttonLeft.pressed(onevent_controller_1buttonLeft_pressed_0)
mainController.buttonA.pressed(onevent_controller_1buttonA_pressed_0)
mainController.buttonUp.pressed(onevent_controller_1buttonUp_pressed_0)
mainController.buttonDown.pressed(onevent_controller_1buttonDown_pressed_0)
when_started1()
