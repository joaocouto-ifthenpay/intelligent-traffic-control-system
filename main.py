from spade.agent import Agent
from spade.behaviour import CyclicBehaviour
from spade.template import Template
from spade.message import Message
import asyncio
import spade

from argparse import ArgumentParser
from default_cycles_utils import default_cycle



#if __name__ == "__main__":
#    spade.run(main())


if __name__ == "__main__":
    default_cycle(n_episodes=1, action_func_name='lqf', render=1)