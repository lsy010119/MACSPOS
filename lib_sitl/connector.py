#!/usr/bin/python3
import asyncio

from mavsdk             import System
from mavsdk.offboard    import OffboardError, VelocityNedYaw, PositionNedYaw



class Connector:


    def __init__(self, agents_sitl, macspos, simparams):


        self.agents = agents_sitl

        self.macspos = macspos

        self.simparams = simparams

    
    async def connect(self, agent):

        print(f"### Agent {agent.id} ###")
        print(f"connecting to {agent.udp_port}..",end="\r")

        await agent.system.connect(system_address=agent.udp_port)
            
        print(f"connected to {agent.udp_port}     \n")

        self.simparams.FLAG_connected[agent.id] = True


    async def connect_all(self, telem_event_loop):
        
        for agent in self.agents:

            mavlink_connection = telem_event_loop.create_task(self.connect(agent))

            await mavlink_connection
