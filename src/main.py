''' ---------------------------------------------------------------------------- 
                                                                              
 	Copyright © Pahlaj Sharma 2024-2025, All rights reserved.                  
    Module:       main.py                                                      
 	Author:       Pahlaj Sharma                                                
 	Created:      4/3/2024, 4:17:56 PM                                         
 	Description:  2602K KryptoKnights Competition Code                               
                                                                              
 ---------------------------------------------------------------------------- '''


from vex import *

brain = Brain()
controller_1 = Controller(PRIMARY)

FrontRight = Motor(Ports.PORT1, GearSetting.RATIO_6_1, True)
MiddleRight = Motor(Ports.PORT2, GearSetting.RATIO_6_1, True)
BackRight = Motor(Ports.PORT3, GearSetting.RATIO_6_1, True)
FrontLeft = Motor(Ports.PORT4, GearSetting.RATIO_6_1, False)
MiddleLeft = Motor(Ports.PORT5, GearSetting.RATIO_6_1, False)
BackLeft = Motor(Ports.PORT6, GearSetting.RATIO_6_1, False)
gyro = Inertial(Ports.PORT7)
AutonSelector = Limit(brain.three_wire_port.a)
XTracker = Rotation(Ports.PORT8)
YTracker1 = Rotation(Ports.PORT9)
YTracker2 = Rotation(Ports.PORT14)
Front = Distance(Ports.PORT10)
Back = Distance(Ports.PORT11)
Left = Distance(Ports.PORT12)
Right = Distance(Ports.PORT13)

wait(230, MSEC)
print("\033[2J")
brain.screen.clear_screen()
controller_1.screen.clear_screen()

FORWARDKP = 1
FORWARDKI = 0.5
FORWARDKD = 0.1
TURNKP = 1
TURNKI = 0.5
TURNKD = 0.1
ROBOT_WIDTH_MM = 355.6 #make sure to put real robot width (this is 14inches)
FrontRightmotor = 1
MiddleRightmotor = 1
BackRightmotor = 1
FrontLeftmotor = 1
MiddleLeftmotor = 1
BackLeftmotor = 1
record_path_start = [False, 50, 0, 0, True] #1. On/Off 2. time_interval 3/4. offsetx/y 5. save to file

class DriveTrainControl:
    #DriveTrain Functions#
    
    def __init__(self, lm, rm, wd, gr) -> None:
        self.lm = lm #left motors
        self.rm = rm #right motors
        self.wd = wd #wheel diameter
        self.gr = gr #gear ratio

    def drive(self, directionR, directionL, speedL: int, speedR: int, unit) -> None:
        FrontLeft.spin(directionL, speedL * FrontLeftmotor, unit)
        MiddleLeft.spin(directionL, speedL * MiddleLeftmotor, unit)
        BackLeft.spin(directionL, speedL * BackLeftmotor, unit)
        FrontRight.spin(directionR, speedR * FrontRightmotor, unit)
        MiddleRight.spin(directionR, speedR * MiddleRightmotor, unit)
        BackRight.spin(directionR, speedR * BackRightmotor, unit)
        wait(20, MSEC)

    def stop(self) -> None:
        FrontRight.set_velocity(0, RPM)
        MiddleRight.set_velocity(0, RPM)
        BackRight.set_velocity(0, RPM)
        FrontLeft.set_velocity(0, RPM)
        MiddleLeft.set_velocity(0, RPM)
        BackLeft.set_velocity(0, RPM)

    def stopping(self, typeL=COAST, typeR = COAST) -> None:
        FrontRight.set_stopping(typeR)
        MiddleRight.set_stopping(typeR)
        BackRight.set_stopping(typeR)
        FrontLeft.set_stopping(typeL)
        MiddleLeft.set_stopping(typeL)
        BackLeft.set_stopping(typeL)

    def velocity(self, velocityL, velocityR, unit) -> None:
        FrontRight.set_velocity(velocityR * FrontRightmotor, unit)
        MiddleRight.set_velocity(velocityR * MiddleRightmotor, unit)
        BackRight.set_velocity(velocityR * BackRightmotor, unit)
        FrontLeft.set_velocity(velocityL * FrontLeftmotor, unit)
        MiddleLeft.set_velocity(velocityL * MiddleLeftmotor, unit)
        BackLeft.set_velocity(velocityL * BackLeftmotor, unit)
        
    def spin(self, directionL, directionR) -> None:
        FrontRight.spin(directionR)
        MiddleRight.spin(directionR)
        BackRight.spin(directionR)
        FrontLeft.spin(directionL)
        MiddleLeft.spin(directionL)
        BackLeft.spin(directionL)
        
    def Forward(self, distance, scale = 1, heading = 0, FkP=FORWARDKP, FkI=FORWARDKI, FkD=FORWARDKD) -> None:
        integral = trackertotalvalue = 0
        mmforward = distance * 25.4
        lastposytracker = func.avg(YTracker1.position(), YTracker2.position())
        lastimupos = gyro.rotation() if calibrated else XTracker.position() * 0.443
        error = 6
        while not abs(error) < 5:
            trackertotalvalue += func.avg(YTracker1.position(), YTracker2.position())
            curr_heading = gyro.rotation() - lastimupos
            error = mmforward - (trackertotalvalue - lastposytracker)
            integral += error
            derivative = error - preverror
            preverror = error
            motorpower = func.limit((FkP * error + FkI * integral + FkD * derivative) * scale, -500, 500)
            correctfordrift = ((heading - curr_heading) * TURNKP)
            rightpower = round(motorpower + correctfordrift)
            leftpower = round(motorpower - correctfordrift)
            dt.drive(FORWARD, FORWARD, leftpower, rightpower, RPM)
        dt.stop()
        
    def Turn(self, degrees, scale = 1, TkP=TURNKP, TkI=TURNKI, TkD=TURNKD, raw=False) -> None:
        integral = 0
        error = 2
        while not abs(error) < 1:
            imu = gyro.heading() if calibrated else (degrees - (XTracker.position() * 0.443 + 180)) % 360
            error = ((degrees - imu + 180) % 360) - 180 if raw else degrees - imu
            integral += error
            derivative = (error - preverror)
            preverror = error
            motorpower = round(func.limit((TkP * error + TkI * integral + TkD * derivative * scale), -500, 500))
            dt.drive(FORWARD, REVERSE, motorpower, motorpower, RPM)
        dt.stop()
        
    def Curve(self, distance, distance_before_turn, degrees, correction_strength, scale = 1, raw=False) -> None:
        integralDrive = integralAngle = currentdistance = 0
        mmforward = distance * 25.4
        initialposition = func.avg(YTracker1.position(), YTracker2.position())
        errorDrive = errorAngle = 6
        while not (abs(errorDrive) < 5 and abs(errorAngle) < 1):
            currentdistance += func.avg(YTracker1.position(), YTracker2.position())
            errorDrive = mmforward - (currentdistance - initialposition)
            integralDrive += errorDrive
            derivativeDrive = errorDrive - preverrorDrive
            preverrorDrive = errorDrive
            motorpowerDrive = func.limit((FORWARDKP * errorDrive + FORWARDKI * integralDrive + FORWARDKD * derivativeDrive) * scale, -500, 500)
            imu = gyro.heading() if calibrated else (degrees - (XTracker.position() * 0.443 + 180)) % 360
            errorAngle = ((degrees - imu + 180) % 360) - 180 if raw else degrees - imu
            integralAngle += errorAngle
            derivativeAngle = errorAngle - preverrorAngle
            preverrorAngle = errorAngle
            angleOutput = 1 if abs((currentdistance - initialposition)) * 0.039 >= distance_before_turn else 0
            motorpowerAngle = func.limit((((TURNKP * errorAngle + TURNKI * integralAngle) + TURNKD * derivativeAngle) * correction_strength * angleOutput) * scale, -500, 500)
            if abs(errorAngle) < 1:
                angleOutput = 0
            rightpower = round(motorpowerDrive - motorpowerAngle)
            leftpower = round(motorpowerDrive + motorpowerAngle)
            dt.drive(FORWARD, FORWARD, leftpower, rightpower, RPM)
        dt.stop()
        
    def getpos(self) -> tuple:
        return currentxpos, currentypos
    
    def gettemp(self) -> float:
        return round(func.avg(FrontLeft.temperature(), MiddleLeft.temperature(), BackLeft.temperature(), FrontRight.temperature(), MiddleRight.temperature(), BackRight.temperature()) * 1.8 + 32, 1)
    
    def driveto(self, x, y, heading = None) -> None:
        dt.Turn(func.calc_angle(currentxpos, currentypos, x, y) if heading is None else ((heading - gyro.heading() + 180) % 360) - 180)
        dt.Forward(func.calc_distance(currentxpos, currentypos, x, y))
        
    def curveto(self, *points, strength=None, disBeforeCurve = 0) -> None:
        x, y = points[-1]
        if strength is None:
            func.expREG(points)
            dt.Curve(func.length_of_curve(points[-1]), disBeforeCurve, func.calc_angle(currentxpos, currentypos, x, y), b)
        else:
            dt.Curve(func.length_of_curve(points[-1]), disBeforeCurve, func.calc_angle(currentxpos, currentypos, x, y), strength)
    
    def record_path(self, time_interval = 50, offsetX = 0, offsetY = 0) -> None:
        global runodom, recorded_path
        recorded_path = []
        while True:
            runodom = Thread(func.odometry, (offsetX, offsetY))
            recorded_path.append((currentxpos, currentypos, gyro.heading()))
            wait(time_interval, MSEC)
   
class Funcs:
    #Miscellaneous Functions#
    
    def __init__(self) -> None:
        self.self = self
    
    def robotinfo_controller(self) -> None:
        global FrontLeftmotor, FrontRightmotor, MiddleLeftmotor, MiddleRightmotor, BackLeftmotor, BackRightmotor
        m_names = ["FrontLeft", "MiddleLeft", "BackLeft", "FrontRight", "MiddleRight", "BackRight"]
        while True:
            controller_1.screen.set_cursor(1, 1)
            controller_1.screen.print(f"Battery: {brain.battery.capacity()}%    DtTemp: {dt.gettemp()}°F")
            if competition.is_enabled():
                controller_1.screen.set_cursor(3, 1)
                if dt.gettemp() > 65:
                    controller_1.screen.print("DRIVETRAIN TEMPERATURE HIGH! COOL DOWN NOW!")
                    controller_1.rumble("-.-.")
                elif brain.battery.capacity() < 70:
                    controller_1.screen.print("BATTERY IS USUABLE, ~70%, TRY TO REPLACE" if brain.battery.capacity() >= 50 else "BATTERY IS LESS THAN 50%, CHANGE NOW!")
            dt_motor_plugged_list = [FrontLeft.installed(), FrontRight.installed(), MiddleLeft.installed(), MiddleRight.installed(), BackLeft.installed(), BackRight.installed()]
            if not all(dt_motor_plugged_list):
                for i, list in enumerate(list):
                    if not list:
                        discon_motor = m_names[i]
                        opp_motor = m_names[i].replace('Left', 'Right') if "Left" in m_names[i] else m_names[i].replace('Right', 'Left')
                        controller_1.screen.print(f"{discon_motor.capitalize()} DISCONECTED, DISABLING {opp_motor.capitalize()}")
                        globals()[opp_motor + "motor"] = 0
            wait(5000, MSEC)
            controller_1.screen.clear_screen()
            
    def odometry(self, offsetx = 0, offsety = 0, headingR = 0) -> None:
        global currentxpos, currentypos
        currentxpos = offsetx
        currentypos = offsety
        gyro.set_heading(((headingR - gyro.heading() + 180) % 360) - 180)
        gyro.set_rotation(headingR)
        MM_PER_TICK = 0.443  #wheel diameter(MM) * pi / ticks(360)  how much the robot travels after 1 degree of encoder revolution
        MMTOIN = 0.039
        while True:
            ytrackerpos = func.avg(YTracker1.position(), YTracker2.position()) * MM_PER_TICK
            xtrackerpos = XTracker.position() * MM_PER_TICK
            calcincossin = func.avg(gyro.rotation(), math.degrees(abs(xtrackerpos) / ROBOT_WIDTH_MM)) if calibrated else math.degrees(abs(xtrackerpos)) / ROBOT_WIDTH_MM
            currentxpos = ((ytrackerpos * math.cos(calcincossin)) * MMTOIN)
            currentypos = ((ytrackerpos * math.sin(calcincossin)) * MMTOIN)
            wait(50, MSEC)
            
    def expREG(self, *points) -> None:
        global a, b
        try:
            x_data: list = []
            y_data: list = []
            x_data, y_data = list(map(list, zip(*points)))
            x_data.insert(0, currentxpos)
            y_data.insert(0, currentypos)
            n = len(x_data)
            sum_x = sum(x_data)
            sum_y = sum([math.log(y) for y in y_data])
            sum_xx = sum([x**2 for x in x_data])
            sum_xy = sum([x * math.log(y) for x, y in zip(x_data, y_data)])
            b = (n * sum_xy - sum_x * sum_y) / (n * sum_xx - sum_x**2)
            a = 2.718 ** ((sum_y - b * sum_x) / n)
            b = 2.718 ** b
        except ValueError:
            controller_1.screen.set_cursor(3, 1)
            controller_1.screen.print("Invalid domain, running default")
            x, y = points[-1]
            dt.Turn(func.calc_angle(currentxpos, currentypos, x, y))
            dt.Forward(func.calc_distance(currentxpos, currentypos, x, y))

    def length_of_curve(self, last_point) -> float:
        min_x = round(currentxpos)
        max_x = last_point[0]
        approx_length = 0
        past_x = currentxpos
        past_y = currentypos
        direction = 1 if min_x < max_x else -1
        step = 5 * direction
        for current_x in range(min_x, max_x + direction, step):
            current_y = a * (b ** current_x)
            approx_length += math.dist((current_x, current_y), (past_x, past_y))
            past_x = current_x
            past_y = current_y
        return approx_length * direction

    def limit(self, value, min, max) -> int:
        return (value if min <= value <= max else min if value < min else max)
    
    def avg(self, *values) -> float:
       return sum(values)/len(values)
   
    def calc_distance(self, x1, y1, x2, y2) -> float:
        return math.dist([x1, y1], [x2, y2])
    
    def calc_angle(self, x1, y1, x2, y2) -> float:
        return (round(math.degrees(math.atan2(y2 - y1, x2 - x1)), 1) + gyro.heading()) % 360
    
    def calibrate_IMU(self) -> bool:
        global calibrated
        calibrated: bool = False
        for attempt in range(1, 6):
            gyro.calibrate()
            while gyro.is_calibrating():
                sleep(50)
            if abs(gyro.rotation()) < 1:
                calibrated = True
                break
        brain.screen.set_cursor(4, 1)
        brain.screen.print("Gyro Calibration Successful" if calibrated else "Gyro Calibration Unsuccessful, using tracking wheels")
        return calibrated

class Autons:
    #Autonomous Routes# 
    
    def __init__(self) -> None:
        self.self = self
        
        
    def run_auton_1(self) -> None:
        #Skills#
        global runodom
        runodom = Thread(func.odometry, (10, 0, 45)) # add offset pos and offset rotation
        raise AssertionError(f"No script for {autonselect}")
    
    
    def run_auton_2(self) -> None:
        #Solo AWP Right
        global runodom
        runodom = Thread(func.odometry, (88, 120, -112))
        dt.driveto(144, 98)
        dt.Turn(236)
        dt.curveto((116, 83), (78, 78))
        dt.Turn(345)
        dt.curveto((88, 105), (120, 120))
        raise AssertionError(f"No script for {autonselect}")


    def run_auton_3(self) -> None:
        raise AssertionError(f"No script for {autonselect}")


    def run_auton_4(self) -> None:
        raise AssertionError(f"No script for {autonselect}")


    def run_auton_5(self) -> None:
        raise AssertionError(f"No script for {autonselect}")


    def run_auton_6(self) -> None:
        raise AssertionError(f"No script for {autonselect}")
    
    
    def test_auton(self) -> None:
        #test code goes here
        ...
        
        
    def tune_pid(self) -> None:
        #pid tune code goes here
        '''
        CwholeN = PotentiometerV2(brain.three_wire_port.e)
        CtenthN = PotentiometerV2(brain.three_wire_port.f)
        ChundN = PotentiometerV2(brain.three_wire_port.g)
        wholeN = 0
        tenthN = 0
        hundN = 0
        '''

def when_started1() -> None:
    global autonselect, robotinfo, dt, func, path_record
    brain.screen.clear_screen()
    controller_1.screen.clear_screen()
    func = Funcs()
    robotinfo = Thread(func.robotinfo_controller)
    #define drivetrain here
    dt = DriveTrainControl((FrontLeft, MiddleLeft, BackLeft), (FrontRight, MiddleRight, BackRight), 3.25, 1.33)
    dt.stopping()
    func.calibrate_IMU()
    autonselect = 1
    if record_path_start[0]:
        path_record = Thread(dt.record_path, (record_path_start[1], record_path_start[2], record_path_start[3], record_path_start[4]))
    brain.screen.set_pen_width(15)
    actions = {
        1: (lambda: (brain.screen.set_cursor(2, 1), brain.screen.print(f"Auton: {currentAuton}"), controller_1.screen.set_cursor(3, 1), controller_1.screen.print(f"Auton: {currentAuton}"))),
        2: (lambda: (brain.screen.set_cursor(2, 1), brain.screen.print(f"Auton: {currentAuton}"), controller_1.screen.set_cursor(3, 1), controller_1.screen.print(f"Auton: {currentAuton}"))),
        3: (lambda: (brain.screen.set_cursor(2, 1), brain.screen.print(f"Auton: {currentAuton}"), controller_1.screen.set_cursor(3, 1), controller_1.screen.print(f"Auton: {currentAuton}"))),
        4: (lambda: (brain.screen.set_cursor(2, 1), brain.screen.print(f"Auton: {currentAuton}"), controller_1.screen.set_cursor(3, 1), controller_1.screen.print(f"Auton: {currentAuton}"))),
        5: (lambda: (brain.screen.set_cursor(2, 1), brain.screen.print(f"Auton: {currentAuton}"), controller_1.screen.set_cursor(3, 1), controller_1.screen.print(f"Auton: {currentAuton}"))),
        6: (lambda: (brain.screen.set_cursor(2, 1), brain.screen.print(f"Auton: {currentAuton}"), controller_1.screen.set_cursor(3, 1), controller_1.screen.print(f"Auton: {currentAuton}"))),
    }
    autons = {
        1: "",
        2: "",
        3: "",
        4: "",
        5: "",
        6: "",
    }
    while not competition.is_autonomous() or not competition.is_driver_control():
        currentAuton = autons.get(autonselect, 1)
        if AutonSelector.pressing():
            autonselect = (autonselect + 1) % 7
            actions.get(autonselect, lambda: None)
            wait(200, MSEC)
    brain.screen.draw_image_from_file("logo.png", 0, 0)
    
def onauton_autonomous_0() -> None:
  auton_functions = [None, Autons.run_auton_1, Autons.run_auton_2, Autons.run_auton_3, Autons.run_auton_4, Autons.run_auton_5, Autons.run_auton_6]
  try:
    auton_functions[autonselect]()
  except IndexError:
    auton_functions[1]

def ondriver_drivercontrol_0() -> None:
    dt.stopping(COAST)
    Deadband = 5
    while True:
        controleraxis1pos = controller_1.axis1.position()
        controleraxis3pos = controller_1.axis3.position()
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
    runodom.stop()
    
def vexcode_driver_function() -> None:
    driver_control_task_0 = Thread(ondriver_drivercontrol_0)
    while (competition.is_driver_control() and competition.is_enabled()):
        wait(10, MSEC)
    driver_control_task_0.stop()
    robotinfo.stop()
    path_record.stop()
    if record_path_start[0] and brain.sdcard.is_inserted():
        x = list([path for path in set(recorded_path)])
        brain.sdcard.savefile("path", x)
    
competition = Competition(vexcode_driver_function, vexcode_auton_function)
when_started1()
