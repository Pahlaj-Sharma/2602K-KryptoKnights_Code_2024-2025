''' ---------------------------------------------------------------------------- 
                                                                              
 	Copyright © Pahlaj Sharma 2024-2025, All rights reserved.                  
    Module:       main.py                                                      
 	Author:       Pahlaj Sharma                                                
 	Created:      4/3/2024, 4:17:56 PM                                         
 	Description:  2602K KryptoKnights Competition Code
    GitHub: https://github.com/Pahlaj-Sharma/KryptoKnights_RobotCode_2602K                               
                                                                              
 ------------------------------------------------------------------------------- '''

''' ----------------------------------------------------------------------------
    -- Notes --
    
    - All values (except PID) are rounded to the nearest hundredth                              
                                                                              
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
# Three Wire Expander
wireExpander = Triport(Ports.PORT9)
# Mogo Pneumatic
mogo = DigitalOut(wireExpander.f)
# Ring Color Detector
ringSorter = Optical(Ports.PORT2)
# Arm Rotation Sensor
armSensor = Rotation(Ports.PORT21)
# Doinger
doinger = DigitalOut(wireExpander.g)
# Intake Picker
intakePickerPnuematic = DigitalOut(wireExpander.c)
intakePickerPnuematic.set(True)

# Clear Screen #
wait(230, MSEC)
print("\033[2J")
brain.screen.clear_screen()
mainController.screen.clear_screen()
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
team: int = 0 # 0: Red; 1: Blue
tempTeam: int = team # For better arm


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
    
    
class Miscellaneous_Functions:
    # Miscellaneous Functions #

    def __init__(self) -> None:
        self.self = self
        
    def robotinformation(self) -> None:
        # Prints Battery, Drivetrain Temperature, and Disconnected Motors #
        motorNames = ["frontRight", "middleRight", "backRight", "frontLeft", "middleLeft", "backLeft"]
        while True:
            mainController.screen.set_cursor(1, 1)
            mainController.screen.print("  ", brain.battery.capacity(), "%    ", frontLeft.temperature(), "  ", round(intake.temperature(), 1), sep='')
            if competition.is_enabled():
                mainController.screen.set_cursor(3, 1)
                if frontLeft.temperature() > 65:
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
            
    def colorSort(self) -> None:
        ringSorter.gesture_disable()
        ringSorter.set_light(LedStateType.ON)
        ringSorter.set_light_power(25, PERCENT)
        # Color Sort Logic
        # 0: Red; 1: Blue
        #650 degree is one revolution
        while True:
            if intake.velocity(RPM) > 20 and (not mainController.buttonL2.pressing()):
                if (ringSorter.color() == Color.BLUE) and team == 0:
                    wait(90, MSEC)
                    intake.stop()
                    wait(200, MSEC)
                    intake.spin(FORWARD)
                elif (ringSorter.color() == Color.RED) and team == 1:
                    wait(90, MSEC)
                    intake.stop()
                    wait(200, MSEC)
                    intake.spin(FORWARD)
                wait(50, MSEC)
            
    def limit(self, value, min, max) -> int:
        # Limits a Number between a Minimum and a Maximum #
        return (value if min <= value <= max else min if value < min else max)
    
    def armSpinTo(self, angle, scale: float = 1, settleError = 0.5) -> None:
        integral = 0
        previousError = 0
        if armSensor.position(DEGREES) > 300:
            armSensor.set_position(0, DEGREES)
        error = angle - armSensor.position(DEGREES)
        brain.timer.clear()
        while math.fabs(error) >= settleError and brain.timer.time(MSEC) <= 1500:
            error = (((angle - armSensor.position(DEGREES)) + 180) % 360) - 180
            integral += error
            integral = func.limit(integral, -100, 100)
            derivative = error - previousError
            if (error > 0 and previousError < 0) or (error < 0 and previousError > 0):
                integral = 0
            previousError = error
            armGroup.set_velocity((error * 4 + integral * 0.001 + derivative * 0.3) * scale, RPM)
            armGroup.spin(FORWARD)
            wait(25, MSEC)
        armGroup.stop()
     
        
def when_started1() -> None:
    global dt, func, robotInformationThread, team, tempTeam, colorSorterThread
    # Declare DriveTrain Settings - Right Motors, Left Motors, Wheel Diameter, Gear Ratio
    dt = Drive_Train_Control((frontRight, middleRight, backRight), (
        frontLeft, middleLeft, backLeft), 2.75, 1.33)
    func = Miscellaneous_Functions() # Declares callable for Miscellaneous Functions
    robotInformationThread = Thread(func.robotinformation) # Starts Thread for Robot Information
    colorSorterThread = Thread(func.colorSort) # Starts Thread for Color Sorting
    dt.stopping() # Set Stopping to Coast
    brain.screen.print("Driving Skills")


def ondriver_drivercontrol_0() -> None:
    # Runs when Competition Switch is on Driver Control #
    dt.stopping()
    Deadband = 5
    intakePickerPnuematic.set(True)
    intake.set_velocity(380, RPM)
    intake.spin(FORWARD)
    wait(300, MSEC)
    intake.set_velocity(200, RPM)
    intake.spin(REVERSE)
    wait(100, MSEC)
    while True:
        controleraxis1pos = mainController.axis1.position()
        controleraxis3pos = mainController.axis3.position()
        if math.fabs(controleraxis1pos) + math.fabs(controleraxis3pos) > Deadband:
            if math.fabs(mainController.axis3.position()) >= -100 and math.fabs(mainController.axis3.position()) <= 100:
                # Forward
                dt.spin(FORWARD, FORWARD, func.limit(controleraxis3pos + controleraxis1pos, -85, 85), func.limit(controleraxis3pos - controleraxis1pos, -85, 85), PERCENT)
            elif math.fabs(mainController.axis1.position()) >= -100 and math.fabs(mainController.axis1.position()) <= 100:
                # Turn
                dt.spin(FORWARD, FORWARD, func.limit((controleraxis3pos + controleraxis1pos) / 2, -85, 85), func.limit((controleraxis3pos - controleraxis1pos) / 2, -85, 85), PERCENT)
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
        intake.set_velocity(420, RPM)
        intake.spin(FORWARD)


def onevent_controller_1buttonRight_pressed_0():
    # Up and Down Mogo
    global mogoClamp
    if mogoClamp:
        intake.spin(REVERSE, 400, RPM)
        mogoClamp = False
        mogo.set(False)
        wait(200, MSEC)
        intake.spin(FORWARD, 400, RPM)
    else:
        mogoClamp = True
        mogo.set(True)
        
        
def onevent_controller_1buttonR1_pressed_0():
    # Arm Control
    global armPicked, team, forceDown
    if not armPicked and not armGroup.is_spinning():
        armGroup.set_stopping(HOLD)
        team = (team + 1) % 2
        func.armSpinTo(18)
        intake.set_velocity(380, RPM)
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
            func.armSpinTo(148)
            armPicked = False
            team = tempTeam
        

def onevent_controller_1buttonR2_pressed_0():
    # Arm Control
    global armPicked, forceDown, team
    armGroup.set_stopping(COAST)
    team = tempTeam
    func.armSpinTo(4)
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
    while (competition.is_autonomous() and competition.is_enabled()):
        wait(10, MSEC)


def vexcode_driver_function() -> None:
    driver_control_task_0 = Thread(ondriver_drivercontrol_0)
    while (competition.is_driver_control() and competition.is_enabled()):
        wait(10, MSEC)
    driver_control_task_0.stop()
    robotInformationThread.stop()
    colorSorterThread.stop()


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
