
class Controller1D():
    """
    This class computes the commanded thrusts (N) to be applied to the quadrotor plant.

    You are to implement the "compute_commands" method.
    """
    def __init__(self, cfparams, pid_gains):
        """
        Inputs:
        - cfparams (CrazyflieParams dataclass):     model parameter class for the crazyflie
        - pid_gains (PIDGains dataclass):           pid gain class
        """
        self.params = cfparams

        # control gains
        self.kp_z = pid_gains.kp
        self.ki_z = pid_gains.ki
        self.kd_z = pid_gains.kd

        #error def
        self.pos_err = 0
        self.vel_err = 0
        self.int_err = 0
        self.d_pos_err = 0



    def compute_commands(self, setpoint, state):
        """
        Inputs:
        - setpoint (State dataclass):   the desired control setpoint
        - state (State dataclass):      the current state of the system

        Returns:
        - U (float): total upward thrust
        """
        
        self.d_pos_err = (setpoint.z_pos-state.z_pos) - self.pos_err
        self.pos_err = setpoint.z_pos-state.z_pos
        if (self.pos_err<.2) and (self.pos_err>-.2):
            self.int_err = self.int_err + self.pos_err
        self.vel_err = setpoint.z_vel-state.z_vel

        U = self.kp_z*(self.pos_err)+self.kd_z*(self.d_pos_err) + self.ki_z*(self.int_err)

        # print(state)

        # your code here

        return U
