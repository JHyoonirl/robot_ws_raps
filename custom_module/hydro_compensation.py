import numpy as np
import math

class HydroCompensation:
    def __init__(self):
        self.drag_coefficient = 1.46  # Drag coefficient
        self.added_mass_coefficient = 0.64  # Added mass coefficient

        self.I_drag = 1.58  # Drag moment of inertia
        self.I_added_mass = 0.1

    def calculate_drag_moment(self, velocity):
        velocity_rad = math.radians(velocity)  # Convert velocity to radians
        drag_moment = 0.5 * self.drag_coefficient * self.I_drag * abs(velocity_rad) * velocity_rad
        return drag_moment

    def calculate_added_mass_moment(self, acceleration):
        acceleration_rad = math.radians(acceleration)  # Convert acceleration to radians
        added_mass_moment = self.added_mass_coefficient * self.I_added_mass * acceleration_rad
        return added_mass_moment

    def calculate_total_moment(self, velocity, acceleration):
        drag_moment = self.calculate_drag_moment(velocity)
        added_mass_moment = self.calculate_added_mass_moment(acceleration)

        total_moment = drag_moment + added_mass_moment
        return total_moment

