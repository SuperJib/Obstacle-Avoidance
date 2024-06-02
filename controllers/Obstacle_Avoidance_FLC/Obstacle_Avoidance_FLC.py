import numpy as np
import skfuzzy as fuzz
from skfuzzy import control as ctrl
from controller import Robot, DistanceSensor, Motor

# Initialize robot and sensors
robot = Robot()
TIME_STEP = 50
MAX_SPEED = 6.28

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

# Define fuzzy input variables
SI = ctrl.Antecedent(np.arange(0, 101, 1), 'SI')
Sf = ctrl.Antecedent(np.arange(0, 101, 1), 'Sf')
Sr = ctrl.Antecedent(np.arange(0, 101, 1), 'Sr')

# Define fuzzy output variables
target_direction = ctrl.Consequent(np.arange(-1, 1.01, 0.01), 'target_direction')
left_speed = ctrl.Consequent(np.arange(-80, 101, 1), 'left_speed')
right_speed = ctrl.Consequent(np.arange(-80, 101, 1), 'right_speed')

# Define membership functions for input variables
SI['near'] = fuzz.trimf(SI.universe, [0, 25, 50])
SI['far'] = fuzz.trimf(SI.universe, [25, 50, 100])
Sf['near'] = fuzz.trimf(Sf.universe, [0, 25, 50])
Sf['far'] = fuzz.trimf(Sf.universe, [25, 50, 100])
Sr['near'] = fuzz.trimf(Sr.universe, [0, 25, 50])
Sr['far'] = fuzz.trimf(Sr.universe, [25, 50, 100])

# Define membership functions for output variables
target_direction['Neg'] = fuzz.trimf(target_direction.universe, [-1, -1, 0])
target_direction['Z'] = fuzz.trimf(target_direction.universe, [-0.5, 0, 0.5])
target_direction['Pos'] = fuzz.trimf(target_direction.universe, [0, 1, 1])

left_speed['Neg'] = fuzz.trimf(left_speed.universe, [-100, -100, 0])
left_speed['Z'] = fuzz.trimf(left_speed.universe, [-50, 0, 50])
left_speed['Pos'] = fuzz.trimf(left_speed.universe, [0, 100, 100])

right_speed['Neg'] = fuzz.trimf(right_speed.universe, [-100, -100, 0])
right_speed['Z'] = fuzz.trimf(right_speed.universe, [-50, 0, 50])
right_speed['Pos'] = fuzz.trimf(right_speed.universe, [0, 100, 100])

# Define fuzzy rules based on the provided rules
rule1 = ctrl.Rule(SI['far'] & Sf['far'] & Sr['near'], 
                  [target_direction['Neg'], left_speed['Neg'], right_speed['Pos']])
rule2 = ctrl.Rule(SI['far'] & Sf['far'] & Sr['far'], 
                  [target_direction['Z'], left_speed['Z'], right_speed['Z']])
rule3 = ctrl.Rule(SI['near'] & Sf['far'] & Sr['far'], 
                  [target_direction['Pos'], left_speed['Pos'], right_speed['Neg']])
rule4 = ctrl.Rule(SI['far'] & Sf['near'] & Sr['far'], 
                  [target_direction['Neg'], left_speed['Neg'], right_speed['Pos']])
rule5 = ctrl.Rule(SI['far'] & Sf['near'] & Sr['far'], 
                  [target_direction['Pos'], left_speed['Pos'], right_speed['Neg']])
rule6 = ctrl.Rule(SI['near'] & Sf['far'] & Sr['far'], 
                  [target_direction['Z'], left_speed['Pos'], right_speed['Pos']])
rule7 = ctrl.Rule(SI['near'] & Sf['far'] & Sr['near'], 
                  [target_direction['Z'], left_speed['Pos'], right_speed['Pos']])
rule8 = ctrl.Rule(SI['near'] & Sf['near'] & Sr['far'], 
                  [target_direction['Z'], left_speed['Pos'], right_speed['Neg']])
rule9 = ctrl.Rule(SI['far'] & Sf['near'] & Sr['near'], 
                  [target_direction['Z'], left_speed['Neg'], right_speed['Pos']])

# Create control system
system = ctrl.ControlSystem([rule1, rule2, rule3, rule4, rule5, rule6, rule7, rule8, rule9])
controller = ctrl.ControlSystemSimulation(system)

while robot.step(TIME_STEP) != -1:
    # Read sensor values
    ps_values = [sensor.getValue() for sensor in ps]

    # Provide input values to the fuzzy controller
    controller.input['SI'] = ps_values[0]
    controller.input['Sf'] = ps_values[2]
    controller.input['Sr'] = ps_values[7]

    # Compute the fuzzy controller
    controller.compute()

    # Get the output values
    target_direction_value = controller.output['target_direction']
    left_speed_value = controller.output['left_speed']
    right_speed_value = controller.output['right_speed']

    # Set motor velocities
    left_motor.setVelocity(MAX_SPEED * left_speed_value)
    right_motor.setVelocity(MAX_SPEED * right_speed_value)
