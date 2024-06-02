from controller import Robot, DistanceSensor, Motor
import csv

# Initialize the Robot
robot = Robot()

TIME_STEP = int(robot.getBasicTimeStep())
MAX_SPEED = 6.28

# Initialize distance sensors
ps = []
ps_names = ['ps0', 'ps1', 'ps2', 'ps3', 'ps4', 'ps5', 'ps6', 'ps7']

for name in ps_names:
    sensor = robot.getDevice(name)
    sensor.enable(TIME_STEP)
    ps.append(sensor)

# Initialize motors
left_motor = robot.getDevice('left wheel motor')
right_motor = robot.getDevice('right wheel motor')

left_motor.setPosition(float('inf'))
right_motor.setPosition(float('inf'))

left_motor.setVelocity(0.0)
right_motor.setVelocity(0.0)

data = []

# Main control loop
while robot.step(TIME_STEP) != -1:
    ps_values = [sensor.getValue() for sensor in ps]

    right_obstacle = ps_values[0] > 80.0 or ps_values[1] > 80.0 or ps_values[2] > 80.0
    left_obstacle = ps_values[5] > 80.0 or ps_values[6] > 80.0 or ps_values[7] > 80.0

    left_speed = 0.5 * MAX_SPEED
    right_speed = 0.5 * MAX_SPEED

    if left_obstacle:
        left_speed = 0.5 * MAX_SPEED
        right_speed = -0.5 * MAX_SPEED
    elif right_obstacle:
        left_speed = -0.5 * MAX_SPEED
        right_speed = 0.5 * MAX_SPEED

    left_motor.setVelocity(left_speed)
    right_motor.setVelocity(right_speed)
    
    # Record data
    data.append([ps_values[5], ps_values[0], left_speed, right_speed])
    
    print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")
    print("Nilai Sensor Jarak:", [round(val, 2) for val in ps_values])
    print("Kecepatan Motor Kiri:", left_speed)
    print("Kecepatan Motor Kanan:", right_speed)
    print("++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++++")

# Save data to CSV file
with open('data_speed.csv', 'w', newline='') as file:
    writer = csv.writer(file)
    writer.writerow(["Left Distance", "Right Distance", "Left Speed", "Right Speed"])
    writer.writerows(data)