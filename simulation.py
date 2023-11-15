from itertools import chain
from typing import List, Dict, Tuple, Set, Optional

from scipy.spatial import distance

from traffic_controller import TrafficController
from traffic_signal import TrafficSignal
from vehicle_generator import VehicleGenerator
from window import Window


class Simulation:
    def __init__(self, max_gen: int = None):
        self.t = 0.0  # Time
        self.dt = 1 / 60  # Time step
        self.traffic_controllers: List[TrafficController] = []
        self.generators: List[VehicleGenerator] = []
        self.traffic_signals: List[TrafficSignal] = []

        self.collision_detected: bool = False
        self.n_vehicles_generated: int = 0
        self.n_vehicles_on_map: int = 0

        self._gui: Optional[Window] = None

        self._non_empty_roads: Set[int] = set()
        # To calculate the number of vehicles in the junction, use:
        # n_vehicles_on_map - _inbound_roads vehicles - _outbound_roads vehicles
        self._inbound_roads: Set[int] = set()
        self._outbound_roads: Set[int] = set()

        self._intersections: Dict[int, Set[int]] = {}  # {TrafficController index: [intersecting roads' indexes]}
        self.max_gen: Optional[int] = max_gen  # Vehicle generation limit
        self._waiting_times_sum: float = 0  # for vehicles that completed the journey

    def add_intersections(self, intersections_dict: Dict[int, Set[int]]) -> None:
        self._intersections.update(intersections_dict)

    def add_traffic_controller(self, start: Tuple[int, int], end: Tuple[int, int]) -> None:
        traffic_controller = TrafficController(
            start, end, index=len(self.traffic_controllers), jid="atc_agent@localhost", pwd="password"
        )
        #await traffic_controller.start(auto_register=True)
        self.traffic_controllers.append(traffic_controller)

    def add_traffic_controllers(self, traffic_controllers: List[Tuple[int, int]]) -> None:
        for traffic_controller in traffic_controllers:
            self.add_traffic_controller(*traffic_controller)

    def add_generator(self, vehicle_rate, paths: List[List]) -> None:
        inbound_roads: List[TrafficController] = [self.traffic_controllers[roads[0]] for weight, roads in paths]
        inbound_dict: Dict[int: TrafficController] = {
            traffic_controller.index: traffic_controller for traffic_controller in inbound_roads
        }
        vehicle_generator = VehicleGenerator(vehicle_rate, paths, inbound_dict)
        self.generators.append(vehicle_generator)

        for (weight, roads) in paths:
            self._inbound_roads.add(roads[0])
            self._outbound_roads.add(roads[-1])

    def add_traffic_signal(self, traffic_controllers: List[List[int]], cycle: List[Tuple],
                           slow_distance: float, slow_factor: float, stop_distance: float) -> None:
        traffic_controllers: List[List[TrafficController]] = \
            [[self.traffic_controllers[i] for i in traffic_controller_group] for traffic_controller_group in traffic_controllers]
        traffic_signal = TrafficSignal(traffic_controllers, cycle, slow_distance, slow_factor, stop_distance)
        self.traffic_signals.append(traffic_signal)

    @property
    def gui_closed(self) -> bool:
        """ Returns an indicator whether the GUI was closed """
        return self._gui and self._gui.closed

    @property
    def non_empty_roads(self) -> Set[int]:
        """ Returns a set of non-empty road indexes """
        return self._non_empty_roads

    @property
    def completed(self) -> bool:
        """
        Whether a terminal state (as defined under the MDP of the task) is reached.
        """
        if self.max_gen:
            return self.collision_detected or (self.n_vehicles_generated == self.max_gen
                                               and not self.n_vehicles_on_map)
        return self.collision_detected

    @property
    def intersections(self) -> Dict[int, Set[int]]:
        """
        Reduces the intersections' dict to non-empty roads
        :return: a dictionary of {non-empty road index: [non-empty intersecting roads indexes]}
        """
        output: Dict[int, Set[int]] = {}
        non_empty_roads: Set[int] = self._non_empty_roads
        for road in non_empty_roads:
            if road in self._intersections:
                intersecting_roads = self._intersections[road].intersection(non_empty_roads)
                if intersecting_roads:
                    output[road] = intersecting_roads
        return output

    @property
    def current_average_wait_time(self) -> float:
        """ Returns the average wait time of vehicles
        that completed the journey and aren't on the map """

        on_map_wait_time = 0
        completed_wait_time = 0
        n_completed_journey = self.n_vehicles_generated - self.n_vehicles_on_map
        if n_completed_journey:
            completed_wait_time = round(self._waiting_times_sum / n_completed_journey, 2)
        if self.n_vehicles_on_map:
            total_on_map_wait_time = sum(vehicle.get_wait_time(self.t) for i in self.non_empty_roads
                                         for vehicle in self.traffic_controllers[i].vehicles)
            on_map_wait_time = total_on_map_wait_time / self.n_vehicles_on_map
        return completed_wait_time + on_map_wait_time

    @property
    def inbound_roads(self) -> Set[int]:
        return self._inbound_roads

    @property
    def outbound_roads(self) -> Set[int]:
        return self._outbound_roads

    def init_gui(self) -> None:
        """ Initializes the GUI and updates the display """
        if not self._gui:
            self._gui = Window(self)
        self._gui.update()

    async def run_agents(self, action: Optional[int] = None) -> None:
        """ Executa um passo de simulação para todos os agentes """
        n = 180  # 3 simulation seconds
        if action:
            await self.update_agents()
            await self._detect_collisions()
            await self._check_out_of_bounds_agents()
            await self._update_signals()
            if self.completed or self.gui_closed:
                return
        self._loop(n)

    def run(self, action: Optional[int] = None) -> None:
        """ Performs n simulation updates. Terminates early upon completion or GUI closing
        :param action: an action from a reinforcement learning environment action space
        """
        n = 180  # 3 simulation seconds
        if action:
            self._update_signals()
            self._loop(n)
            if self.collision_detected or self.gui_closed:
                return
            self._update_signals()
            if self.completed or self.gui_closed:
                return
        self._loop(n)

    def update(self) -> None:
        """ Updates the roads, generates vehicles, detect collisions and updates the gui """
        # Update every road
        for i in self._non_empty_roads:
            self.traffic_controllers[i].update(self.dt, self.t)

        # Add vehicles
        for gen in self.generators:
            if self.max_gen and self.n_vehicles_generated == self.max_gen:
                break
            road_index = gen.update(self.t, self.n_vehicles_generated)
            if road_index is not None:
                self.n_vehicles_generated += 1
                self.n_vehicles_on_map += 1
                self._non_empty_roads.add(road_index)

        self._check_out_of_bounds_vehicles()

        self._detect_collisions()

        # Increment time
        self.t += self.dt

        # Update the display
        if self._gui:
            self._gui.update()

    def _loop(self, n: int) -> None:
        """ Performs n simulation updates. Terminates early upon completion or GUI closing"""
        for _ in range(n):
            self.update()
            if self.completed or self.gui_closed:
                return

    def _update_signals(self) -> None:
        """ Updates all the simulation traffic signals and updates the gui, if exists """
        for traffic_signal in self.traffic_signals:
            traffic_signal.update()
        if self._gui:
            self._gui.update()

    def _detect_collisions(self) -> None:
        """ Detects collisions by checking all non-empty intersecting vehicle paths.
        Updates the self.collision_detected attribute """
        radius = 3
        for main_road, intersecting_roads in self.intersections.items():
            vehicles = self.traffic_controllers[main_road].vehicles
            intersecting_vehicles = chain.from_iterable(
                self.traffic_controllers[i].vehicles for i in intersecting_roads)
            for vehicle in vehicles:
                for intersecting in intersecting_vehicles:
                    if distance.euclidean(vehicle.position, intersecting.position) < radius:
                        self.collision_detected = True
                        return

    def _check_out_of_bounds_vehicles(self):
        """ Check roads for out-of-bounds vehicles, updates self.non_empty_roads """
        new_non_empty_roads = set()
        new_empty_roads = set()
        for i in self._non_empty_roads:
            road = self.traffic_controllers[i]
            lead = road.vehicles[0]
            # If first vehicle is out of road bounds
            if lead.x >= road.length:
                # If vehicle has a next road
                if lead.current_road_index + 1 < len(lead.path):
                    # Remove it from its road
                    road.vehicles.popleft()
                    # Reset the position relative to the road
                    lead.x = 0
                    # Add it to the next road
                    lead.current_road_index += 1
                    next_road_index = lead.path[lead.current_road_index]
                    new_non_empty_roads.add(next_road_index)
                    self.traffic_controllers[next_road_index].vehicles.append(lead)
                    # road.vehicles.popleft()
                    if not road.vehicles:
                        new_empty_roads.add(road.index)
                else:
                    # Remove it from its road
                    road.vehicles.popleft()
                    # Remove from non_empty_roads if it has no vehicles
                    if not road.vehicles:
                        new_empty_roads.add(road.index)
                    self.n_vehicles_on_map -= 1
                    # Update the waiting times sum
                    self._waiting_times_sum += lead.get_wait_time(self.t)

        self._non_empty_roads.difference_update(new_empty_roads)
        self._non_empty_roads.update(new_non_empty_roads)
