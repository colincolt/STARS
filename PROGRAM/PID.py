import time

class PID:
    # PID Controller
    def __init__(self, P=0.3, I=0.0, D=0.0):

        self.Kp = P
        self.Ki = I
        self.Kd = D

        self.sample_time = 0.1
        self.current_time = time.time()
        self.last_time = self.current_time

        self.clear()

    def clear(self):
        # Clears PID computations and coefficients
        self.SetPoint = 0.0

        self.PTerm = 0.0
        self.ITerm = 0.0
        self.DTerm = 0.0
        self.last_error = 0.0

        # Windup Guard
        self.int_error = 0.0
        self.windup_guard = 200.0

        self.output = 0.0

    def update(self, pixel_displacement):
        pixel_displacement_scaledown = (pixel_displacement / 10) #-90     # Assuming max pixelDisp ~2000 (~ 0 - 200)

        error = self.SetPoint + pixel_displacement_scaledown

        self.current_time = time.time()
        delta_time = self.current_time - self.last_time

        if self.last_error is not None:
            delta_error = error - self.last_error
        else:   # << If this is the first measurement
            delta_error = 0

        if (delta_time >= self.sample_time):
            self.PTerm = self.Kp * error        # ~ 0 - 200
            self.ITerm += error * delta_time    # ~ 2* 0.1 = 0.2, 200 *0.1 = 20

            if (self.ITerm < -self.windup_guard):
                self.ITerm = -self.windup_guard
            elif (self.ITerm > self.windup_guard):
                self.ITerm = self.windup_guard

            self.DTerm = 0.0
            if delta_time > 0:
                self.DTerm = delta_error / (delta_time)   # 2/0.1 = 20, 200/0.1 = 2000

            # Remember last time and last error for next calculation
            self.last_time = self.current_time
            self.last_error = error
            self.output = self.PTerm + (self.Ki * self.ITerm) + (self.Kd * self.DTerm) # MAX: FIRST_MEASURE: P:(0-200) + I:0-20 each loop(max:200) + D:0 = 220, SECOND_MEASURE: 150 + 35 -25 = 165 ... -> ~400 max

    def setKp(self, proportional_gain):
        self.Kp = proportional_gain

    def setKi(self, integral_gain):
        self.Ki = integral_gain

    def setKd(self, derivative_gain):
        self.Kd = derivative_gain

    def setWindup(self, windup):
        self.windup_guard = windup

    def setSampleTime(self, sample_time):
        # PID that should be updated at a regular interval.
        # Based on a pre-determined sampe time, the PID decides if it should compute or return immediately.
        self.sample_time = sample_time



