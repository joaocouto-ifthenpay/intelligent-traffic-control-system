from spade.agent import Agent
from spade.behaviour import CyclicBehaviour
from spade.template import Template
from spade.message import Message
import asyncio
import spade

from argparse import ArgumentParser
from default_cycles_utils import default_cycle
from env_mas import Environment



#if __name__ == "__main__":
#    spade.run(main())


if __name__ == "__main__":
    parser = ArgumentParser(description="AI Controller")
    methods = ['fc', 'lqf']
    parser.add_argument("-m", "--method", choices=methods, required=True)
    parser.add_argument("-e", "--episodes", metavar='N', type=int, required=True,
                        help="Number of evaluation episodes to run")
    parser.add_argument("-r", "--render", action='store_true',
                        help="Displays the simulation window")
    args = parser.parse_args()

    if args.method in ['fc', 'lqf']:
        default_cycle(n_episodes=args.episodes, action_func_name=args.method, render=args.render)

