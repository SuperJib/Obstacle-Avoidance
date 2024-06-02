from controller import Robot
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

robot = Robot()

# Definisi waktu langkah dasar
TIME_STEP = int(robot.getBasicTimeStep())
# Set kecepatan motor

Max = 6.28

# variabel input
left_distance = ctrl.Antecedent(np.arange(0, 101, 1), 'left_distance')
right_distance = ctrl.Antecedent(np.arange(0, 101, 1), 'right_distance')

# variabel output
left_speed = ctrl.Consequent(np.arange(-3, 3.01, 0.01), 'left_speed')
right_speed = ctrl.Consequent(np.arange(-3, 3.01, 0.01), 'right_speed')

# Fuzzy membership functions untuk input front_distance, left_distance
left_distance['dekatL'] = fuzz.trimf(left_distance.universe, [0, 0, 50])
left_distance['jauhL'] = fuzz.trimf(left_distance.universe, [0, 50, 100])

right_distance['dekatR'] = fuzz.trimf(right_distance.universe, [0, 0, 50])
right_distance['jauhR'] = fuzz.trimf(right_distance.universe, [0, 50, 100])

# Fuzzy membership functions untuk output left_speed dan right_speed
left_speed['lambatL'] = fuzz.trimf(left_speed.universe, [-3, -1.5, 0])
left_speed['sedangL'] = fuzz.trimf(left_speed.universe, [-1.5, 0, 1.5])
left_speed['cepatL'] = fuzz.trimf(left_speed.universe, [0, 1.5, 3])

right_speed['lambatR'] = fuzz.trimf(right_speed.universe, [-3, -1.5, 0])
right_speed['sedangR'] = fuzz.trimf(right_speed.universe, [-1.5, 0, 1.5])
right_speed['cepatR'] = fuzz.trimf(right_speed.universe, [0, 1.5, 3])

# Aturan fuzzy
rule1 = ctrl.Rule(left_distance['dekatL'] & right_distance['dekatR'], [left_speed['lambatL'], right_speed['lambatR']])
rule2 = ctrl.Rule(left_distance['jauhL'] & right_distance['dekatR'], [left_speed['cepatL'], right_speed['lambatR']])
rule3 = ctrl.Rule(left_distance['dekatL'] & right_distance['jauhR'], [left_speed['lambatL'], right_speed['cepatR']])
rule4 = ctrl.Rule(left_distance['jauhL'] & right_distance['jauhR'], [left_speed['cepatL'], right_speed['cepatR']])

# Kompilasi aturan menjadi kontroler fuzzy
speed_ctrl = ctrl.ControlSystem([rule1, rule2, rule3, rule4])
speeding = ctrl.ControlSystemSimulation(speed_ctrl)

# Get devices
ps = []
ps_names = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']
for name in ps_names:
    sensor = robot.getDevice(name)
    sensor.enable(TIME_STEP)
    ps.append(sensor)

left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

while robot.step(TIME_STEP) != -1:
    ps_values = [sensor.getValue() for sensor in ps]

    # Input ke dalam kontroler fuzzy
    speeding.input['left_distance'] = min(ps_values[5], ps_values[6], ps_values[7])  # Sensor 5, 6 dan 7 untuk mendeteksi rintangan di kiri
    speeding.input['right_distance'] = min(ps_values[0], ps_values[1], ps_values[2])  # Sensor 0, 1 dan 2 untuk mendeteksi rintangan di kanan

    # Perhitungan output berdasarkan aturan fuzzy
    speeding.compute()

    # Set kecepatan motor berdasarkan output dari kontroler fuzzy
    left_speed_val = speeding.output['left_speed']
    right_speed_val = speeding.output['right_speed']

    if ps_values[0] > 80:  # Jika ada rintangan di sebelah kanan dengan jarak lebih dari 80
        left_speed_val = -1  # Berjalan belok kiri
        right_speed_val = 1
    elif ps_values[7] > 80:  # Jika ada rintangan di sebelah kiri dengan jarak lebih dari 80
        left_speed_val = 1  # Berjalan belok kanan
        right_speed_val = -1
    else:  # Jika tidak ada rintangan di sekitar
        left_speed_val = 1
        right_speed_val = 1

    # Set kecepatan motor berdasarkan output dari kontroler fuzzy
    left_motor.setVelocity(left_speed_val * Max)
    right_motor.setVelocity(right_speed_val * Max)

    print("Nilai Sensor Jarak:", ps_values)
    print("Kecepatan Motor Kiri:", left_speed_val * Max)
    print("Kecepatan Motor Kanan:", right_speed_val * Max)
