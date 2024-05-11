from simple_pid import PID as PIDBase
import time


class PID(PIDBase):
    """A simple PID wrapper that automatically supplies a time function so deltattime is calculated automatically.
    """
    def __init__(self, kp, ki, kd, setpoint=0, sample_time=0.01, output_limits=(None, None), auto_mode=True):
        super().__init__(kp, ki, kd, setpoint=setpoint, sample_time=sample_time, output_limits=output_limits, auto_mode=auto_mode, time_fn=time.time)

    def __call__(self, input, dt):
        return self(input, dt)