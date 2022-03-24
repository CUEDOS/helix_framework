class AgentTelemetry:
    def __init__ (self):
        self.arm_status = False
        self.geodetic = [0, 0, 0]
        self.heading = 0.0
        self.position_ned = [0, 0, 0]
        self.velocity_ned = [0, 0, 0]
