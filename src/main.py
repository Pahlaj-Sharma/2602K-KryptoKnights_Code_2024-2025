''' ---------------------------------------------------------------------------- 
                                                                              
 	Copyright © Pahlaj Sharma 2024-2025, All rights reserved.                  
    Module:       main.py                                                      
 	Author:       Pahlaj Sharma                                                
 	Created:      4/3/2024, 4:17:56 PM                                         
 	Description:  2602K KryptoKnights Competition Code
    Github Repository: https://github.com/Pahlaj-Sharma/KryptoKnights_RobotCode_2602K                               
                                                                              
 ---------------------------------------------------------------------------- '''

''' --------------------------------------------------------------------------------------------------------
    -- Developer Notes --
    
    - This code has not yet been tested on a robot, this is prewritten for when the robot is actually built
    - All numbers, variables, constants (except PID values) are rounded to the nearest hundreth                              
                                                                              
 ----------------------------------------------------------------------------------------------------------- '''


from vex import *  # Import Modules
brain = Brain()  # Define Brain
main_controller = Controller(PRIMARY)  # Define Controller

# Middle Right Top Drivetrain Motor Normal 6:1
Middle_Top_Right = Motor(Ports.PORT1, GearSetting.RATIO_6_1, False)
# Middle Right Bottom Drivetrain Motor Reverse 6:1
Middle_Bottom_Right = Motor(Ports.PORT2, GearSetting.RATIO_6_1, True)
# Back Right Drivetrain Motor Normal 6:1
Back_Right = Motor(Ports.PORT3, GearSetting.RATIO_6_1, False)
# Middle Top Left Drivetrain Motor Reverse 6:1
Middle_Top_Left = Motor(Ports.PORT4, GearSetting.RATIO_6_1, True)
# Middle Bottom Left Drivetrain Motor Normal 6:1
Middle_Bottom_Left = Motor(Ports.PORT5, GearSetting.RATIO_6_1, False)
# Back Left Drivetrain Motor Reverse 6:1
Back_Left = Motor(Ports.PORT6, GearSetting.RATIO_6_1, True)
inertial = Inertial(Ports.PORT7)  # Inertial
# Limit Switch Autonomous Selecter
Autonomous_Selector = Limit(brain.three_wire_port.a)
# Perpendicular Tracking Wheel Odometry
Odom_Perpendicular = Rotation(Ports.PORT8)
Odom_Parallel_1 = Rotation(Ports.PORT9)  # Parallel Tracking Wheel Odometry 1
Odom_Parallel_2 = Rotation(Ports.PORT14)  # Paralled Tracking Wheel Odometry 2
# Distance Sensor Odometry Correction FRONT
Odom_Front = Distance(Ports.PORT10)
Odom_Back = Distance(Ports.PORT11)  # Distance Sensor Odometry Correction BACK
Odom_Left = Distance(Ports.PORT12)  # Distance Sensor Odometry Correction LEFT
# Distance Sensor Odometry Correction RIGHT
Odom_Right = Distance(Ports.PORT13)

# Clear Screen and Reset Functions #
wait(230, MSEC)
print("\033[2J")
brain.screen.clear_screen()
main_controller.screen.clear_screen()

# Define Global Variables and Constants #
# Define PID Constants #
FORWARD_KP: float = 1.0
FORWARD_KI: float = 0.5
FORWARD_KD: float = 0.1
TURN_KP: float = 1.0
TURN_KI: float = 0.5
TURN_KD: float = 0.1
ROBOT_WIDTH_MM: float = 355.6  # Define Robot Width in MM
# Define Enabled Motors #
Middle_Top_Right_motor: bool = True
Middle_Bottom_Right_motor: bool = True
Back_Right_motor: bool = True
Middle_Top_Left_motor: bool = True
Middle_Bottom_Left_motor: bool = True
Back_Left_motor: bool = True


class Drive_Train_Control:
    # DriveTrain Functions #

    def __init__(self, rm, lm, wd, gr) -> None:
        self.rm = rm  # Left Motors
        self.lm = lm  # Right Motors
        self.wd = wd  # Wheel Diameter
        self.gr = gr  # Gear Ratio

    def drive(self, Direction_Right_Motors, Direction_Left_Motors, Speed_Left_Motors: float, Speed_Right_Motors: float, Unit) -> None:
        # Drive Motors #
        Middle_Top_Right.spin(Direction_Right_Motors, Speed_Right_Motors * Middle_Top_Right_motor, Unit)
        Middle_Bottom_Right.spin(Direction_Right_Motors, Speed_Right_Motors * Middle_Bottom_Right_motor, Unit)
        Back_Right.spin(Direction_Right_Motors, Speed_Right_Motors * Back_Right_motor, Unit)
        Middle_Top_Left.spin(Direction_Left_Motors, Speed_Left_Motors * Middle_Top_Left_motor, Unit)
        Middle_Bottom_Left.spin(Direction_Left_Motors, Speed_Left_Motors * Middle_Bottom_Left_motor, Unit)
        Back_Left.spin(Direction_Left_Motors, Speed_Left_Motors * Back_Left_motor, Unit)

    def stop(self) -> None:
        # Stop Motors #
        Middle_Top_Right.set_velocity(0, RPM)
        Middle_Bottom_Right.set_velocity(0, RPM)
        Back_Right.set_velocity(0, RPM)
        Middle_Top_Left.set_velocity(0, RPM)
        Middle_Bottom_Left.set_velocity(0, RPM)
        Back_Left.set_velocity(0, RPM)

    def stopping(self, Type_Left_Motors=COAST, Type_Right_Motors=COAST):
        # Set Stopping of Motors #
        Middle_Top_Right.set_stopping(Type_Right_Motors)
        Middle_Bottom_Right.set_stopping(Type_Right_Motors)
        Back_Right.set_stopping(Type_Right_Motors)
        Middle_Top_Left.set_stopping(Type_Left_Motors)
        Middle_Bottom_Left.set_stopping(Type_Left_Motors)
        Back_Left.set_stopping(Type_Left_Motors)

    def velocity(self, Speed_Left_Motors: float, Speed_Right_Motors: float, Unit):
        # Set Motor Velocity #
        Middle_Top_Right.set_velocity(Speed_Right_Motors * Middle_Top_Right_motor, Unit)
        Middle_Bottom_Right.set_velocity(Speed_Right_Motors * Middle_Bottom_Right_motor, Unit)
        Back_Right.set_velocity(Speed_Right_Motors * Back_Right_motor, Unit)
        Middle_Top_Left.set_velocity(Speed_Left_Motors * Middle_Top_Left_motor, Unit)
        Middle_Bottom_Left.set_velocity(Speed_Left_Motors * Middle_Bottom_Left_motor, Unit)
        Back_Left.set_velocity(Speed_Left_Motors * Back_Left_motor, Unit)
        
    def spin(self, Direction_Left_Motors, Direction_Right_Motors) -> None:
        # Spin Motors in a Specific Direction
        Middle_Top_Right.spin(Direction_Right_Motors)
        Middle_Bottom_Right.spin(Direction_Right_Motors)
        Back_Right.spin(Direction_Right_Motors)
        Middle_Top_Left.spin(Direction_Left_Motors)
        Middle_Bottom_Left.spin(Direction_Left_Motors)
        Back_Left.spin(Direction_Left_Motors)
    
    def get_current_temperature(self) -> float:
        # Returns Average Temperature of the Drivetrain
        return round(func.average(Middle_Top_Right.temperature(), Middle_Bottom_Left.temperature(), Back_Right.temperature(), Middle_Top_Left.temperature(), Middle_Bottom_Left.temperature(), Back_Left.temperature()), 1)
        
    # PID Function Forward #
    def forward(self, Distance_Inches: float, Speed_Scale: float = 1, Robot_Heading: float = 0, KP: float = FORWARD_KP, KI: float = FORWARD_KI, KD: float = FORWARD_KD) -> None:
        Integral = 0
        MM_Forward = Distance_Inches * 25.4
        Last_Position_Parallel_Tracker = Parallel_Tracker_Position
        Last_Gyro_Position = inertial.rotation() if calibrated else 0
        Error = 6
        while not abs(Error) < 5:
            Current_Heading = inertial.rotation() - Last_Gyro_Position if calibrated else 0
            Error = MM_Forward - (Parallel_Tracker_Position - Last_Position_Parallel_Tracker)
            Integral += Error
            Derivative = Error - Previous_Error
            Previous_Error = Error
            Motor_Power = func.limit(round((KP * Error + KI * Integral + KD * Derivative) * Speed_Scale), -450, 450)
            Drift_Correction = ((Robot_Heading - Current_Heading) * KP) if calibrated else 0
            Right_Motor_Power = Motor_Power + Drift_Correction
            Left_Motor_Power = Motor_Power - Drift_Correction
            dt.drive(FORWARD, FORWARD, Left_Motor_Power, Right_Motor_Power, RPM)
        dt.stop()
            
        
    # PID Function Turn #
    def turn(self, Angle_Degrees: float, Speed_Scale: float = 1, KP: float = TURN_KP, KI: float = TURN_KI, KD: float = TURN_KD, Raw_Angle: bool = False) -> None:
        Integral = 0
        Error = 2
        while not abs(Error) < 1:
            Rotation_IMU = inertial.heading() if calibrated else Perpendicular_Tracker_Position
            Error = ((Angle_Degrees - Rotation_IMU + 180) % 360) - 180 if not Raw_Angle else Angle_Degrees - Rotation_IMU
            Integral += Error
            Derivative = Error - Previous_Error
            Previous_Error = Error
            Motor_Power = func.limit(round(KP * Error + KI * Integral + KD * Derivative), -450, 450)
            dt.drive(FORWARD, REVERSE, Motor_Power, Motor_Power, RPM)
        dt.stop()
        
    # PID Function Curve #
    def curve(self, Distance_Inches, Distance_Before_Turn: float, Angle_Degrees: float, Correction_Strength: float, Speed_Scale: float = 1, FKP: float = FORWARD_KP, FKI: float = FORWARD_KI, FKD: float = FORWARD_KD, TKP: float = TURN_KP, TKI: float = TURN_KI, TKD: float = TURN_KD, Raw_Angle: bool = False) -> None:
        Integral_Drive = 0
        Integral_Angle = 0
        MM_Forward = Distance_Inches * 25.4
        Last_Position_Paralled_Tracker = Parallel_Tracker_Position
        Error_Drive = 6
        Error_Angle = 2
        while not (abs(Error_Drive) < 5 and abs(Error_Angle) < 1):
            Rotation_IMU = inertial.heading() if calibrated else Perpendicular_Tracker_Position
            Error_Drive = MM_Forward - (Parallel_Tracker_Position - Last_Position_Paralled_Tracker)
            Integral_Drive += Error_Drive
            Derivative_Drive = Error_Drive - Previous_Error_Drive
            Previous_Error_Drive = Error_Drive
            Motor_Power_Drive = func.limit(round(FKP * Error_Drive + FKI * Integral_Drive + FKD * Derivative_Drive), -450, 450)
            Error_Angle = ((Angle_Degrees - Rotation_IMU + 180) % 360) - 180 if not Raw_Angle else Angle_Degrees - Rotation_IMU
            Integral_Angle += Error_Angle
            Derivative_Angle = Error_Angle - Previous_Error_Angle
            Previous_Error_Angle = Error_Angle
            Angle_Output = 1 if abs(Parallel_Tracker_Position - Last_Position_Paralled_Tracker) * 0.04 >= Distance_Before_Turn else 0
            Motor_Power_Angle = func.limit(round(TKP * Error_Angle + TKI * Integral_Angle + TKD * Derivative_Angle) * Correction_Strength * Angle_Output, -450, 450)
            Angle_Output = 0 if abs(Error_Angle) < 1 else 1
            Right_Motor_Power = Motor_Power_Drive - Motor_Power_Angle
            Left_Motor_Power = Motor_Power_Drive + Motor_Power_Angle
            dt.drive(FORWARD, FORWARD, Left_Motor_Power, Right_Motor_Power, RPM)
        dt.stop()
        
    # PID Function Drive To #
    def drive_to_coordinate(self, X, Y, Heading = None) -> None:
        ...
        
class Miscellaneous_Functions:
    # Miscellaneous Functions #

    def __init__(self) -> None:
        self.self = self
        
    def robot_information(self) -> None:
        # Prints Battery, Drivetrain Temperature, and Disconnected Motors #
        Motor_Names = ["Middle_Top_Right", "Middle_Bottom_Right", "Back_Right", "Middle_Top_Left", "Middle_Bottom_Left", "Back_Left"]
        while True:
            main_controller.screen.set_cursor(1, 1)
            main_controller.screen.print(f"Battery: {brain.battery.capacity()}%    DtTemp: {dt.get_current_temperature()}°F")
            if competition.is_enabled():
                main_controller.screen.set_cursor(3, 1)
                if dt.get_current_temperature() > 65:
                    main_controller.screen.print("DRIVETRAIN TEMPERATURE HIGH! COOL DOWN NOW!")
                    main_controller.rumble("-.-.")
                elif brain.battery.capacity() < 70:
                    main_controller.screen.print("BATTERY IS USUABLE, ~70%, TRY TO REPLACE" if brain.battery.capacity() >= 50 else "BATTERY IS LESS THAN 50%, CHANGE NOW!")
            DriveTrain_Motors_Installed = [Middle_Top_Right.installed(), Middle_Bottom_Right.installed(), Back_Right.installed(), Middle_Top_Left.installed(), Middle_Bottom_Left.installed(), Back_Left.installed()]
            if not all(DriveTrain_Motors_Installed):
                for i, list in enumerate(list):
                    if not list:
                        Disconnected_Motor = Motor_Names[i]
                        Opposite_Motor = Motor_Names[i].replace("Left", "Right") if "Left" in Motor_Names[i] else Motor_Names[i].replace("Right", "Left")
                        main_controller.screen.print(f"{Disconnected_Motor.capitalize()} DISCONNECTED, DISABLIN {Opposite_Motor.capitalize()}")
                        globals()[Opposite_Motor + "_motor"] = False
            wait(5000, MSEC)
            main_controller.screen.clear_screen()
            
    def odometry(self, OffsetX = 0, OffsetY = 0, OffsetHeading = 0):
        # Updates Odometry Position #
        global Current_X_Position, Current_Y_Position, Parallel_Tracker_Position, Perpendicular_Tracker_Position
        Current_X_Position = OffsetX
        Current_Y_Position = OffsetY
        inertial.set_heading(((OffsetHeading - inertial.heading() + 180) % 360) - 180)
        inertial.set_rotation(OffsetHeading)
        MM_PER_TICK = 0.443 # Wheel Diameter(MM) * PI / Ticks(360) How much the robot travels after one rotation of the rotation sensor
        MM_TO_INCHES = 0.04
        while True:
            Parallel_Tracker_Position = func.average(Odom_Parallel_1.position(), Odom_Parallel_2.position()) * MM_PER_TICK
            Perpendicular_Tracker_Position = (math.degrees(Odom_Perpendicular.position() * MM_PER_TICK)) % 360 # Wrap around 360 degrees
            Calculate_Theta = (func.average(inertial.rotation(), Perpendicular_Tracker_Position / ROBOT_WIDTH_MM)) if calibrated else Perpendicular_Tracker_Position / ROBOT_WIDTH_MM
            Current_X_Position = round(((Parallel_Tracker_Position * math.cos(Calculate_Theta)) * MM_TO_INCHES), 2)
            Current_Y_Position = round(((Parallel_Tracker_Position * math.sin(Calculate_Theta)) * MM_TO_INCHES), 2)
            
    def average(self, *values) -> float:
        # Returns the Average of a List of Numbers #
        return round(sum(values)/len(values), 2)
    
    def limit(self, value, min, max) -> int:
        # Limits a Number between a Minimum and a Maximum #
        return (value if min <= value <= max else min if value < min else max)
    
    def calculate_distance(self, x1, y1, x2, y2) -> float:
        # Calculates the Distance between two Points #
        return round(math.dist([x1, y1], [x2, y2]), 2)
    
    def calculate_angle(self, x1, y1, x2, y2) -> float:
        # Calculates Angle between two Points #
        return (round(math.degrees(math.atan2(y2 - y1, x2 - x1)), 1) + inertial.heading()) % 360
    
    def calibrate_inertial(self):
        # Calibrates Inertial Sensor #
        global calibrated
        calibrated: bool = False
        for attempt in range(1, 6):
            inertial.calibrate()
            while inertial.is_calibrating():
                sleep(50)
            if abs(inertial.rotation()) < 1.5:
                calibrated = True
                break
            brain.screen.set_cursor(4, 1)
            brain.screen.print("Gyro Calibration Successful" if calibrated else "Gyro Calibration Unsuccessful")
            
            
class Autonomous_Routes:
    # Autonomous Routes #
    
    def run_auton_1(self) -> None:
        # Skills #
        global Run_Odometry
        Run_Odometry = Thread(func.odometry, (0, 0, 0))
        raise AssertionError(f"No script for {Autonomous_Selection}")
    
    
    def run_auton_2(self) -> None:
        # Solo AWP Right #
        raise AssertionError(f"No script for {Autonomous_Selection}")


    def run_auton_3(self) -> None:
        raise AssertionError(f"No script for {Autonomous_Selection}")


    def run_auton_4(self) -> None:
        raise AssertionError(f"No script for {Autonomous_Selection}")


    def run_auton_5(self) -> None:
        raise AssertionError(f"No script for {Autonomous_Selection}")


    def run_auton_6(self) -> None:
        raise AssertionError(f"No script for {Autonomous_Selection}")
    
    
    def test_auton(self) -> None:
        # Testing Autonomous Code #
        ...
        
        
    def tune_pid(self) -> None:
        # PID Tuning #
        ...
        
        
def when_started1() -> None:
    global dt, func, Autonomous_Selection, Robot_Information_Thread
    # Declare DriveTrain Settings - Right Motors, Left Motors, Wheel Diameter, Gear Ratio
    dt = Drive_Train_Control((Middle_Top_Right, Middle_Bottom_Right, Back_Right), (
        Middle_Top_Left, Middle_Bottom_Left, Back_Left), 3.25, 1.33)
    func = Miscellaneous_Functions() # Declares callable for Miscellaneous Functions
    Robot_Information_Thread = Thread(func.robot_information) #Starts Thread for Robot Information
    dt.stopping() # Set Stopping to Coast
    func.calibrate_inertial() # Calibrate Inertial Sensor
    Autonomous_Selection = 1 # Defaults Selection to 1
    brain.screen.set_pen_width(15)
    # Prints Current Autonomous Route #
    actions = {
        1: (lambda: (brain.screen.set_cursor(2, 1), brain.screen.print(f"Auton: {currentAuton}"), main_controller.screen.set_cursor(3, 1), main_controller.screen.print(f"Auton: {currentAuton}"))),
        2: (lambda: (brain.screen.set_cursor(2, 1), brain.screen.print(f"Auton: {currentAuton}"), main_controller.screen.set_cursor(3, 1), main_controller.screen.print(f"Auton: {currentAuton}"))),
        3: (lambda: (brain.screen.set_cursor(2, 1), brain.screen.print(f"Auton: {currentAuton}"), main_controller.screen.set_cursor(3, 1), main_controller.screen.print(f"Auton: {currentAuton}"))),
        4: (lambda: (brain.screen.set_cursor(2, 1), brain.screen.print(f"Auton: {currentAuton}"), main_controller.screen.set_cursor(3, 1), main_controller.screen.print(f"Auton: {currentAuton}"))),
        5: (lambda: (brain.screen.set_cursor(2, 1), brain.screen.print(f"Auton: {currentAuton}"), main_controller.screen.set_cursor(3, 1), main_controller.screen.print(f"Auton: {currentAuton}"))),
        6: (lambda: (brain.screen.set_cursor(2, 1), brain.screen.print(f"Auton: {currentAuton}"), main_controller.screen.set_cursor(3, 1), main_controller.screen.print(f"Auton: {currentAuton}"))),
    }
    autons = {
        1: "",
        2: "",
        3: "",
        4: "",
        5: "",
        6: "",
    }
    # Autonomous Selector Logic #
    while not competition.is_autonomous() or not competition.is_driver_control():
        currentAuton = autons.get(autonselect, 1)
        if Autonomous_Selector.pressing():
            autonselect = (autonselect + 1) % 7
            actions.get(autonselect, lambda: None)
            wait(200, MSEC)


def onauton_autonomous_0() -> None:
    # Runs when Competition Switch is on Autonomous #
    Autonomous_Functions = [None, Autonomous_Routes.run_auton_1, Autonomous_Routes.run_auton_2, Autonomous_Routes.run_auton_3, Autonomous_Routes.run_auton_4, Autonomous_Routes.run_auton_5, Autonomous_Routes.run_auton_6]
    try:
        Autonomous_Functions[Autonomous_Selection]()
    except IndexError:
        Autonomous_Functions[1]


def ondriver_drivercontrol_0() -> None:
    # Runs when Competition Switch is on Driver Control #
    dt.stopping()
    Deadband = 5
    while True:
        controleraxis1pos = main_controller.axis1.position()
        controleraxis3pos = main_controller.axis3.position()
        if math.fabs(controleraxis1pos) + math.fabs(controleraxis3pos) > Deadband:
            dt.velocity(controleraxis3pos + controleraxis1pos, controleraxis3pos - controleraxis1pos, PERCENT)
        else:
            dt.velocity(0, 0, PERCENT)
        dt.spin(FORWARD, FORWARD)
        wait(10, MSEC)


def vexcode_auton_function() -> None:
    auton_task_0 = Thread(onauton_autonomous_0)
    while (competition.is_autonomous() and competition.is_enabled()):
        wait(10, MSEC)
    auton_task_0.stop()
    Run_Odometry.stop()


def vexcode_driver_function() -> None:
    driver_control_task_0 = Thread(ondriver_drivercontrol_0)
    while (competition.is_driver_control() and competition.is_enabled()):
        wait(10, MSEC)
    driver_control_task_0.stop()
    Robot_Information_Thread.stop()


competition = Competition(vexcode_driver_function, vexcode_auton_function)
when_started1()
