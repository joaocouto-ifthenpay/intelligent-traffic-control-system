from spade.agent import Agent
from spade.behaviour import CyclicBehaviour
from spade.template import Template
from spade.message import Message
import asyncio
import spade

from collections import deque
from typing import Deque, Optional, Tuple

from scipy.spatial import distance

from traffic_signal import TrafficSignal
from vehicle import Vehicle


class Road(Agent):
    def __init__(self, start: Tuple[int, int], end: Tuple[int, int], index: int, jid, password):
        super().__init__(jid, password)
        print(jid)
        print(password)
        self.start = start
        self.end = end
        self.index = index

        self.vehicles: Deque[Vehicle] = deque()

        self.length: float = distance.euclidean(self.start, self.end)
        self.angle_sin: float = (self.end[1] - self.start[1]) / self.length
        self.angle_cos: float = (self.end[0] - self.start[0]) / self.length

        self.has_traffic_signal: bool = False
        self.traffic_signal: Optional[TrafficSignal] = None
        self.traffic_signal_group: Optional[int] = None

    async def setup(self):
        # Define a behavior to perceive and interact with the environment
        class EnvironmentInteraction(CyclicBehaviour):
            async def run(self):
                print("EnvironmentInteraction behavior is running")
                # Perceive environment data - you can use ACL messages or other means
                #aircraft_position = self.get_aircraft_position()
                #weather_data = self.get_weather_data()
                #runway_status = self.get_runway_status()

                # Make decisions based on perceptions and update the environment
                # Example: Check for conflicts and send instructions to aircraft

            #def get_aircraft_position(self):
                # Implement logic to retrieve aircraft positions from the environment
            #    pass

            #def get_weather_data(self):
                # Implement logic to retrieve weather data from the environment
            #    pass

            #def get_runway_status(self):
                # Implement logic to retrieve runway status from the environment
            #    pass

        # Add the behavior to the agent
        #self.add_behaviour(EnvironmentInteraction())

    def set_traffic_signal(self, signal: TrafficSignal, group: int):
        self.has_traffic_signal = True
        self.traffic_signal = signal
        self.traffic_signal_group = group

    def __str__(self):
        return f'Road {self.index}'

    @property
    def traffic_signal_state(self):
        """ Returns the traffic signal state if the road has a traffic signal, else True"""
        if self.has_traffic_signal:
            i = self.traffic_signal_group
            return self.traffic_signal.current_cycle[i]
        return True

    def update(self, dt, sim_t):
        n = len(self.vehicles)
        if n > 0:
            lead: Vehicle = self.vehicles[0]

            # Check for traffic signal
            if self.traffic_signal_state:
                # If traffic signal is green (or doesn't exist), let vehicles pass
                self.notify_vehicle_to_start(sim_t, lead)

            elif self.has_traffic_signal:
                # The traffic signal is red (existence checked to access its stop_distance)
                lead_can_stop_safely = lead.x <= self.length - self.traffic_signal.stop_distance / 1.5
                # This check is to ensure that we don't stop vehicles that are too close to the traffic
                # signal when it turns to yellow. In such a case, the vehicle should pass as quickly as possible,
                # without being even slowed down
                if lead_can_stop_safely:
                    lead.slow(self.traffic_signal.slow_factor)  # slow vehicles in slow zone
                    lead_in_stop_zone = self.length - self.traffic_signal.stop_distance <= lead.x
                    if lead_in_stop_zone:
                        lead.stop(sim_t)

            # Update first vehicle
            lead.update(None, dt, self)
            # Update other vehicles
            for i in range(1, n):
                lead = self.vehicles[i - 1]
                self.vehicles[i].update(lead, dt, self)

    def notify_vehicle_to_start(self, sim_t, lead):
        #print('Go!')
        lead.unstop(sim_t)
        for vehicle in self.vehicles:
            vehicle.unslow()