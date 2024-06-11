def pure_pursuit(self, pathInput:list, from_path_gen=False):
        path = []
        LOOKAHEADDISTANCE = 1
        if from_path_gen is True:
            for tuple1 in pathInput:
                new_tuple = tuple(round(element + 144, 2) for element in tuple1)
                path.append(new_tuple)
        else:
            pathInput = path
        for i in range(len(path)):
            look_ahead_x, look_ahead_y = func.look_ahead_point(path, 1)
            dx = look_ahead_x - currentxpos
            dy = look_ahead_y - currentypos
            theta = math.atan2(dy, dx)
            heading_error = round(theta - gyro.rotation())
            dt.drive(FORWARD, FORWARD, heading_error, heading_error * -1, RPM)
            
            
            
            
def look_ahead_point(self, path:list, look_ahead_distance):
        for i in range(len(path)):
            x_target, y_target = path[0]
            dx = x_target - currentxpos
            dy = y_target - currentypos
            if dx * math.cos(gyro.rotation()) + dy * math.sin(gyro.rotation()) < 0:
                continue
            distance = math.sqrt(dx**2 + dy**2)
            if distance <= look_ahead_distance:
                return x_target, y_target
        return path[-1]