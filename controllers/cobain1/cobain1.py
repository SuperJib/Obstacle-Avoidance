import numpy as np
import skfuzzy as fuzz
import csv
from skfuzzy import control as ctrl
from controller import Robot, Camera
import cv2

class CustomCamera:
    def __init__(self, camera):
        self.camera = camera
        self.width = int(self.camera.getWidth())
        self.height = int(self.camera.getHeight())
        self.image_data = np.zeros((self.height, self.width, 3), dtype=np.uint8)

    def getImage(self):
        res = self.camera.getImageArray()
        if res is not None:
            self.image_data = np.array(res, dtype=np.uint8)
        return self.image_data
        
    def displayImage(self):
        cv2.imshow('Camera Image', self.image_data)
        cv2.waitKey(1)

class FuzzyController:
    def __init__(self):
        self.robot = Robot()

        # Initialize data storage
        self.data = []

        # Definisi waktu langkah dasar
        self.TIME_STEP = int(self.robot.getBasicTimeStep())
        # Set kecepatan motor
        self.basespeed = 3.14

        # Get devices
        self.ps = []
        self.ps_names = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
        for name in self.ps_names:
            sensor = self.robot.getDevice(name)
            sensor.enable(self.TIME_STEP)
            self.ps.append(sensor)

        # Inisialisasi motor
        self.left_motor = self.robot.getDevice('left wheel motor')
        self.right_motor = self.robot.getDevice('right wheel motor')

        self.left_motor.setPosition(float('inf'))
        self.right_motor.setPosition(float('inf'))

        self.left_motor.setVelocity(0.0)
        self.right_motor.setVelocity(0.0)

        # Inisialisasi kamera
        camera_device = self.robot.getDevice('camera')
        camera_device.enable(self.TIME_STEP)
        self.camera = CustomCamera(camera_device)

        # Variabel input
        self.left_distance = ctrl.Antecedent(np.arange(0, 101, 1), 'left_distance')
        self.right_distance = ctrl.Antecedent(np.arange(0, 101, 1), 'right_distance')

        # Variabel output
        self.left_speed = ctrl.Consequent(np.arange(-1, 1.01, 0.01), 'left_speed')
        self.right_speed = ctrl.Consequent(np.arange(-1, 1.01, 0.01), 'right_speed')

        # Fuzzy membership functions untuk input front_distance, left_distance
        self.left_distance['dekatL'] = fuzz.trapmf(self.left_distance.universe, [0, 0, 25, 50])
        self.left_distance['sedenganL'] = fuzz.trapmf(self.left_distance.universe, [25, 35, 65, 75])
        self.left_distance['jauhL'] = fuzz.trapmf(self.left_distance.universe, [50, 75, 100, 100])

        self.right_distance['dekatR'] = fuzz.trapmf(self.right_distance.universe, [0, 0, 25, 50])
        self.right_distance['sedenganR'] = fuzz.trapmf(self.right_distance.universe, [25, 35, 65, 75])
        self.right_distance['jauhR'] = fuzz.trapmf(self.right_distance.universe, [50, 75, 100, 100])

        # Fuzzy membership functions untuk output left_speed dan right_speed
        self.left_speed['lambatL'] = fuzz.trapmf(self.left_speed.universe, [-1, -1, -0.5, 0])
        self.left_speed['sedangL'] = fuzz.trapmf(self.left_speed.universe, [-0.5, 0, 0, 0.5])
        self.left_speed['cepatL'] = fuzz.trapmf(self.left_speed.universe, [0, 0.5, 1, 1])

        self.right_speed['lambatR'] = fuzz.trapmf(self.right_speed.universe, [-1, -1, -0.5, 0])
        self.right_speed['sedangR'] = fuzz.trapmf(self.right_speed.universe, [-0.5, 0, 0, 0.5])
        self.right_speed['cepatR'] = fuzz.trapmf(self.right_speed.universe, [0, 0.5, 1, 1])

        # Aturan fuzzy
        self.rule1 = ctrl.Rule(self.left_distance['dekatL'] & self.right_distance['dekatR'], [self.left_speed['lambatL'], self.right_speed['lambatR']])
        self.rule2 = ctrl.Rule(self.left_distance['sedenganL'] & self.right_distance['dekatR'], [self.left_speed['sedangL'], self.right_speed['lambatR']])
        self.rule3 = ctrl.Rule(self.left_distance['jauhL'] & self.right_distance['dekatR'], [self.left_speed['cepatL'], self.right_speed['lambatR']])
        self.rule4 = ctrl.Rule(self.left_distance['dekatL'] & self.right_distance['sedenganR'], [self.left_speed['lambatL'], self.right_speed['lambatR']])
        self.rule5 = ctrl.Rule(self.left_distance['dekatL'] & self.right_distance['jauhR'], [self.left_speed['lambatL'], self.right_speed['cepatR']])
        self.rule6 = ctrl.Rule(self.left_distance['sedenganL'] & self.right_distance['sedenganR'], [self.left_speed['sedangL'], self.right_speed['sedangR']])
        self.rule7 = ctrl.Rule(self.left_distance['sedenganL'] & self.right_distance['jauhR'], [self.left_speed['lambatL'], self.right_speed['cepatR']])
        self.rule8 = ctrl.Rule(self.left_distance['jauhL'] & self.right_distance['sedenganR'], [self.left_speed['cepatL'], self.right_speed['lambatR']])
        self.rule9 = ctrl.Rule(self.left_distance['jauhL'] & self.right_distance['jauhR'], [self.left_speed['cepatL'], self.right_speed['cepatR']])

        # Kompilasi aturan menjadi kontroler fuzzy
        self.speed_ctrl = ctrl.ControlSystem([self.rule1, self.rule2, self.rule3, self.rule4, self.rule5, self.rule6, self.rule7, self.rule8, self.rule9])
        self.speeding = ctrl.ControlSystemSimulation(self.speed_ctrl)

    def run(self):
        while self.robot.step(self.TIME_STEP) != -1:
            # Get camera image data
            camera_image = self.camera.getImage()

            # Display the camera image
            self.camera.displayImage()

            # Define the red color range (BGR format)
            lower_red = np.array([0, 0, 100])  # Lower bound of red color range
            upper_red = np.array([50, 50, 140])  # Upper bound of red color range

            # Create a mask to extract red pixels
            mask = cv2.inRange(camera_image, lower_red, upper_red)

            # Count the number of red pixels
            red_pixels = cv2.countNonZero(mask)

            # Print or process the number of red pixels here
            print("Number of red pixels:", red_pixels)

            if red_pixels > 0:
                # If red color is detected, stop the robot
                self.left_motor.setVelocity(0)
                self.right_motor.setVelocity(0)
                print("Red color detected, stopping the robot.")
                continue  # Skip the rest of the loop

            # Get sensor data
            ps_values = [sensor.getValue() for sensor in self.ps]

            left_distance_input = min(ps_values[4], ps_values[5], ps_values[6], ps_values[7])
            right_distance_input = min(ps_values[0], ps_values[1], ps_values[2], ps_values[3])

            if 0 <= left_distance_input <= 100 and 0 <= right_distance_input <= 100:
                self.speeding.input['left_distance'] = left_distance_input
                self.speeding.input['right_distance'] = right_distance_input

                self.speeding.compute()

                left_speed_val = self.speeding.output['left_speed']
                right_speed_val = self.speeding.output['right_speed']
            else:
                left_speed_val = 1
                right_speed_val = 1

            if max(ps_values[0], ps_values[1], ps_values[2], ps_values[3]) > 100:
                left_speed_val = -1
                right_speed_val = 1
            elif max(ps_values[4], ps_values[5], ps_values[6], ps_values[7]) > 100:
                left_speed_val = 1
                right_speed_val = -1
            else:
                left_speed_val = 1
                right_speed_val = 1

            self.left_motor.setVelocity(left_speed_val * self.basespeed)
            self.right_motor.setVelocity(right_speed_val * self.basespeed)

            # Record data
            self.data.append([left_distance_input, right_distance_input, left_speed_val * self.basespeed, right_speed_val * self.basespeed])

            print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
            print("Nilai Sensor Jarak:", [round(val, 2) for val in ps_values])
            print("Kecepatan Motor Kiri:", left_speed_val * self.basespeed)
            print("Kecepatan Motor Kanan:", right_speed_val * self.basespeed)
            print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")

    def save_data(self, filename='robot_performance_data.csv'):
        with open(filename, 'w', newline='') as file:
            writer = csv.writer(file)
            writer.writerow(["Left Distance", "Right Distance", "Left Speed", "Right Speed"])
            writer.writerows(self.data)

if __name__ == "__main__":
    controller = FuzzyController()
    controller.run()
    controller.save_data()
