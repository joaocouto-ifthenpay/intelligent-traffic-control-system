from spade.agent import Agent
from spade.behaviour import CyclicBehaviour
from spade.template import Template
from spade.message import Message
import asyncio
import spade

from simulation_controller import SimulationController

t = 10  # limite de tempo do ciclo


def longest_queue_action(curr_state, prev_state) -> bool:
    """ Devolve um booleano, caso tenha passado tempo suficiente desde a ação anterior """
    switch = False
    traffic_signal = curr_state.traffic_signals[0]
    time_elapsed = curr_state.t - traffic_signal.prev_update_time >= t
    if time_elapsed:
        traffic_signal_state, n_direction_1_vehicles, n_direction_2_vehicles, non_empty_junction = prev_state
        # Se a direção da maioria dos carros tiver sinal vermelho, mudar para verde
        if traffic_signal_state and n_direction_1_vehicles < n_direction_2_vehicles:
            switch = True
        elif not traffic_signal_state and n_direction_1_vehicles > n_direction_2_vehicles:
            switch = True
    if switch:
        # Update the traffic signal update time
        traffic_signal.prev_update_time = curr_state.t
    return switch


action_funcs = {'lqf': longest_queue_action }


def default_cycle(n_episodes: int, action_func_name: str, render):
    print(f"\n -- Sistema Multi-Agente de Controlo de Tráfego -- ")
    simulation_controller: SimulationController = SimulationController()

    total_wait_time, total_collisions = 0, 0
    action_func = action_funcs[action_func_name]

    #spade.run(test_spade())

    for episode in range(1, n_episodes + 1):
        state = simulation_controller.reset(render)
        print()
        score = 0
        collision_detected = 0
        done = False

        while not done:
            action = action_func(simulation_controller.sim, state)
            state, done, truncated = simulation_controller.step(action)
            if truncated:
                exit()
            collision_detected += simulation_controller.sim.collision_detected

        if collision_detected:
            print(f"Episódio {episode} - Acidentes: {int(collision_detected)}")
            total_collisions += 1
        else:
            wait_time = simulation_controller.sim.current_average_wait_time
            total_wait_time += wait_time
            print(f"Episódio {episode} - Tempo de espera: {wait_time:.2f}")

    n_completed = n_episodes - total_collisions
    print(f"\n -- Resultados após {n_episodes} episódios: -- ")
    print(
        f"Tempo médio de espera por episódio: {total_wait_time / n_completed:.2f}")
    print(f"Média de acidentes por episódio: {total_collisions / n_episodes:.2f}")


#async def test_spade():
    # Create and initialize the environment
    #env = Environment()

    #atc_agent = AirTrafficControlAgent("atc_agent@localhost", "password", env)
    #aircraft_agent = AircraftAgent("aircraft@localhost", "password", env)

    #await atc_agent.start(auto_register=True)
    #await aircraft_agent.start(auto_register=True)