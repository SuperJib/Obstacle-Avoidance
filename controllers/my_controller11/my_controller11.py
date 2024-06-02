from controller import Robot
import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl

robot = Robot()

# Definisi waktu langkah dasar
TIME_STEP = int(robot.getBasicTimeStep())
# Set kecepatan motor
Max = 6.28

# Implementasi fuzzy logic controller (FLC)
left_distance = ctrl.Antecedent(np.arange(0, 101, 1), 'left_distance')
right_distance = ctrl.Antecedent(np.arange(0, 101, 1), 'right_distance')

left_speed = ctrl.Consequent(np.arange(-1, 1.01, 0.01), 'left_speed')
right_speed = ctrl.Consequent(np.arange(-1, 1.01, 0.01), 'right_speed')

# Fuzzy membership functions untuk input left_distance dan right_distance
left_distance['close'] = fuzz.trimf(left_distance.universe, [0, 0, 50])
left_distance['far'] = fuzz.trimf(left_distance.universe, [0, 50, 100])

right_distance['close'] = fuzz.trimf(right_distance.universe, [0, 0, 50])
right_distance['far'] = fuzz.trimf(right_distance.universe, [0, 50, 100])

# Fuzzy membership functions untuk output left_speed dan right_speed
left_speed['slow'] = fuzz.trimf(left_speed.universe, [-1, -1, 0])
left_speed['medium'] = fuzz.trimf(left_speed.universe, [-0.5, 0, 0.5])
left_speed['fast'] = fuzz.trimf(left_speed.universe, [0, 1, 1])

right_speed['slow'] = fuzz.trimf(right_speed.universe, [-1, -1, 0])
right_speed['medium'] = fuzz.trimf(right_speed.universe, [-0.5, 0, 0.5])
right_speed['fast'] = fuzz.trimf(right_speed.universe, [0, 1, 1])

# Aturan fuzzy
rule1 = ctrl.Rule(left_distance['close'] | right_distance['close'], (left_speed['fast'], right_speed['slow']))
rule2 = ctrl.Rule(left_distance['far'] & right_distance['far'], (left_speed['medium'], right_speed['medium']))
rule3 = ctrl.Rule(left_distance['close'] & right_distance['far'], (left_speed['slow'], right_speed['fast']))
rule4 = ctrl.Rule(left_distance['far'] & right_distance['close'], (left_speed['fast'], right_speed['slow']))

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
    speeding.input['left_distance'] = min(ps_values[5:8])
    speeding.input['right_distance'] = min(ps_values[0:3])

    # Perhitungan output berdasarkan aturan fuzzy
    speeding.compute()

    # Set kecepatan motor berdasarkan output dari kontroler fuzzy
    left_speed = speeding.output['left_speed']
    right_speed = speeding.output['right_speed']

    # Untuk belok ke kanan
    if left_speed >= 0:  # Motor kiri maju
        left_motor.setVelocity(left_speed + Max)

    # Untuk belok ke kiri
    elif right_speed >= 0:  # Motor kanan maju
        right_motor.setVelocity(right_speed + Max)


    print("Nilai Sensor Jarak:", ps_values)
    print("Kecepatan Motor Kiri:", left_speed)
    print("Kecepatan Motor Kanan:", right_speed)