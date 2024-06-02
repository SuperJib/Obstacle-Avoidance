from controller import Robot
import numpy as np
import skfuzzy as fuzz
import csv
from skfuzzy import control as ctrl

class FuzzyController:
    def __init__(self):
        # Define time step
        self.TIME_STEP = int(robot.getBasicTimeStep())
        # Set motor speed
        self.Max = 3.14
        
        # Input variables
        self.delta_error = ctrl.Antecedent(np.arange(-100, 101, 1), 'delta_error')
        self.delta_speed = ctrl.Antecedent(np.arange(-3.14, 3.15, 0.01), 'delta_speed')
        
        # Output variables
        self.left_speed = ctrl.Consequent(np.arange(-1, 1.01, 0.01), 'left_speed')
        self.right_speed = ctrl.Consequent(np.arange(-1, 1.01, 0.01), 'right_speed')
        
        # Fuzzy membership functions for input delta_error
        self.delta_error['negative'] = fuzz.trapmf(self.delta_error.universe, [-100, -100, -50, 0])
        self.delta_error['zero'] = fuzz.trapmf(self.delta_error.universe, [-50, 0, 0, 50])
        self.delta_error['positive'] = fuzz.trapmf(self.delta_error.universe, [0, 50, 100, 100])

        # Fuzzy membership functions for input delta_speed
        self.delta_speed['negative'] = fuzz.trapmf(self.delta_speed.universe, [-3.14, -3.14, -1.57, 0])
        self.delta_speed['zero'] = fuzz.trapmf(self.delta_speed.universe, [-1.57, 0, 0, 1.57])
        self.delta_speed['positive'] = fuzz.trapmf(self.delta_speed.universe, [0, 1.57, 3.14, 3.14])

        # Fuzzy membership functions for output left_speed and right_speed
        self.left_speed['lambatL'] = fuzz.trapmf(self.left_speed.universe, [-1, -1, -0.5, 0])
        self.left_speed['sedangL'] = fuzz.trapmf(self.left_speed.universe, [-0.5, 0, 0, 0.5])
        self.left_speed['cepatL'] = fuzz.trapmf(self.left_speed.universe, [0, 0.5, 1, 1])

        self.right_speed['lambatR'] = fuzz.trapmf(self.right_speed.universe, [-1, -1, -0.5, 0])
        self.right_speed['sedangR'] = fuzz.trapmf(self.right_speed.universe, [-0.5, 0, 0, 0.5])
        self.right_speed['cepatR'] = fuzz.trapmf(self.right_speed.universe, [0, 0.5, 1, 1])

        # Fuzzy rules
        rule1 = ctrl.Rule(self.delta_error['negative'] & self.delta_speed['negative'], [self.left_speed['lambatL'], self.right_speed['cepatR']])
        rule2 = ctrl.Rule(self.delta_error['negative'] & self.delta_speed['zero'], [self.left_speed['lambatL'], self.right_speed['sedangR']])
        rule3 = ctrl.Rule(self.delta_error['negative'] & self.delta_speed['positive'], [self.left_speed['lambatL'], self.right_speed['lambatR']])
        rule4 = ctrl.Rule(self.delta_error['zero'] & self.delta_speed['negative'], [self.left_speed['sedangL'], self.right_speed['cepatR']])
        rule5 = ctrl.Rule(self.delta_error['zero'] & self.delta_speed['zero'], [self.left_speed['sedangL'], self.right_speed['sedangR']])
        rule6 = ctrl.Rule(self.delta_error['zero'] & self.delta_speed['positive'], [self.left_speed['sedangL'], self.right_speed['lambatR']])
        rule7 = ctrl.Rule(self.delta_error['positive'] & self.delta_speed['negative'], [self.left_speed['cepatL'], self.right_speed['cepatR']])
        rule8 = ctrl.Rule(self.delta_error['positive'] & self.delta_speed['zero'], [self.left_speed['cepatL'], self.right_speed['sedangR']])
        rule9 = ctrl.Rule(self.delta_error['positive'] & self.delta_speed['positive'], [self.left_speed['cepatL'], self.right_speed['lambatR']])

        # Compile rules into a fuzzy controller
        self.speed_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9])
        self.speeding = ctrl.ControlSystemSimulation(self.speed_ctrl)
        
        # Initialize motors
        self.left_motor = robot.getDevice('left wheel motor')
        self.right_motor = robot.getDevice('right wheel motor')

        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))

        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)
    
   def calculate_delta_speed_fuzzy(self, error):
       if error < -50:
            delta_speed_val = -3.14  # Max backward speed
       elif error > 50:
            delta_speed_val = 3.14   # Max forward speed
       else:
            delta_speed_val = 0      # No change in speed
       return delta_speed_val
    
    def motor_action(self, base_speed, delta_speed):
    # Apply motor actions based on fuzzy control
        left_motor_speed = base_speed + delta_speed
        right_motor_speed = base_speed - delta_speed
    
    # Limit speeds to allowable range [-3.14, 3.14]
        left_motor_speed = max(min(left_motor_speed, 3.14), -3.14)
        right_motor_speed = max(min(right_motor_speed, 3.14), -3.14)
    
    # Set motor velocities
        self.left_motor.setVelocity(left_motor_speed)
        self.right_motor.setVelocity(right_motor_speed)
    
if __name__ == "__main__":
    robot = Robot()
    controller = FuzzyController()
    data = []

    # Get devices
    ps = []
    ps_names = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
    for name in ps_names:
        sensor = robot.getDevice(name)
        sensor.enable(controller.TIME_STEP)
        ps.append(sensor)

    while robot.step(controller.TIME_STEP) != -1:
        ps_values = [sensor.getValue() for sensor in ps]

        left_distance_input = min(ps_values[5], ps_values[6], ps_values[7])
        right_distance_input = min(ps_values[0], ps_values[1], ps_values[2])

        if 0 <= left_distance_input <= 100 and 0 <= right_distance_input <= 100:
            delta_error_input = left_distance_input - right_distance_input
            delta_speed_input = controller.calculate_delta_speed_fuzzy(delta_error_input)

            controller.motor_action(1.0, delta_speed_input)  # Assuming base_speed as 1.0 for demonstration

            # Record data
            data.append([delta_error_input, delta_speed_input, left_speed_val * controller.Max, right_speed_val * controller.Max])

        print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
        print("Nilai Sensor Jarak:", [round(val, 2) for val in ps_values])
        print("Delta Error:", delta_error_input)
        print("Delta Speed:", delta_speed_input)
        print("Kecepatan Motor Kiri:", left_speed_val * controller.Max)
        print("Kecepatan Motor Kanan:", right_speed_val * controller.Max)
        print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")

    # Save data to CSV file
    with open('robot_performance_data.csv', 'w', newline='') as file:
        writer = csv.writer(file)
        writer.writerow(["Delta Error", "Delta Speed", "Left Speed", "Right Speed"])
        writer.writerows(data)
