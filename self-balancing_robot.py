import math
import hub
import time

class PID:
    def __init__(self, kp, ki, kd):
        self._kp = kp
        self._ki = ki
        self._kd = kd
        self.reset()

    def process(self, setpoint, process_value, dt):
        error = setpoint - process_value
        self.integral += error * dt
        derivative = (error - self.error_prev) / dt
        self.error_prev = error
        return self._kp * error + self._ki * self.integral + self._kd * derivative

    def reset(self):
        self.integral = 0
        self.error_prev = 0

def clamp(value, min_value, max_value):
    return min(max(round(value), min_value), max_value)

def motor_pwm(main, correction = 0):
    hub.port.B.pwm(clamp(main - correction, -127, 127))
    hub.port.F.pwm(-clamp(main + correction, -127, 127))

HPF = 0.995
LPF = 1-HPF
RAD2DEG = 180/math.pi

neutral_pitch = 1.6
speed_setpoint = 0 
yaw_setpoint = 0

pitch_PID = PID(kp = 13.5, ki = 155, kd = 0.19)
speed_PID = PID(kp = 0.04, ki = 0.03, kd = 0.00055)
yaw_PID = PID(kp = 0.9, ki = 2, kd = 0.01)

state_active = False
lastTime = time.ticks_us()
pitch = (hub.motion.yaw_pitch_roll()[2]-90)

while (True):
    dt = time.ticks_diff(time.ticks_us(), lastTime) * 0.000001
    lastTime = time.ticks_us()

    accel = hub.motion.accelerometer()
    gyroPitch = pitch + hub.motion.gyroscope()[1] * dt
    accelPitch = -math.atan2(accel[2], math.sqrt(accel[0]*accel[0] + accel[1]*accel[1])) * RAD2DEG

    pitch = HPF * gyroPitch + LPF * accelPitch
    speed = (hub.port.F.device.get()[0] - hub.port.B.device.get()[0]) / 2
    yaw = -(hub.port.B.device.get()[1] + hub.port.F.device.get()[1])

    if (state_active):
        if (abs(pitch)<45):
            speed_feedback = speed_PID.process(speed_setpoint, speed, dt)
            pitch_feedback = pitch_PID.process(speed_feedback + neutral_pitch, pitch, dt)
            yaw_feedback = yaw_PID.process(yaw_setpoint, yaw, dt)
            motor_pwm(pitch_feedback, yaw_feedback)
        else:
            state_active = False
            motor_pwm(0)
            pitch_PID.reset()
            speed_PID.reset()
            yaw_PID.reset()
    else:
        if (abs(pitch)<3):
            state_active = True
