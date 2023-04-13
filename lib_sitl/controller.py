#!/usr/bin/python3
import asyncio
import time

from mavsdk.offboard    import OffboardError, VelocityNedYaw, PositionNedYaw

from numpy.linalg   import norm

import matplotlib.pyplot as plt


class Controller:

    def __init__(self, agents_sitl, macspos, simparams ):

        self.agents = agents_sitl
        self.macspos = macspos
        self.simparams = simparams



    async def offboard_start(self, agent):

        print(f"Agent.{agent.id} : Offboard Starting ...")

        await agent.system.offboard.set_position_ned(
            PositionNedYaw(0.0, 0.0, 0.0, 0.0))
        
        await agent.system.offboard.set_velocity_ned(
            VelocityNedYaw(0.0, 0.0, 0.0, 0.0))


        try:
            await agent.system.offboard.start()

            return True

        except OffboardError as error:

            print(f"Agent.{agent.id} : Offboard failed : \
                {error._result.result}")

            return False


    async def control_agent(self, agent):

        while not all(self.simparams.FLAG_connected):

            # print("Telemetry : waiting for northn...",end="\r")

            await asyncio.sleep(0.001)

        print(f"Agent.{agent.id} : control start")

        await agent.system.action.arm()        

        await self.offboard_start(agent)

        pos_init = self.macspos.sharedmemory.agents[agent.id].waypoints[0].loc


        print(f"Agent.{agent.id} : moving to starting point ( {pos_init[1]}, {pos_init[0]} )...")


        await agent.system.offboard.set_position_ned(\
            PositionNedYaw( pos_init[1]-self.simparams.spawn_loc[agent.id][1],\
                            pos_init[0]-self.simparams.spawn_loc[agent.id][0],-10*(agent.id+1),0.0))
        
        await asyncio.sleep(10)

        print(f"Agent.{agent.id} : arrived to starting point")

        self.simparams.FLAG_initialized[agent.id] = True


        while not self.simparams.FLAG_mission_done[agent.id]:
            
            velin = self.macspos.sharedmemory.agents[agent.id].velin

            # print(f"Agent.{agent.id} : velocity command {norm(velin)}")

            await agent.system.offboard.set_velocity_ned(VelocityNedYaw(velin[1],velin[0],0.0,0.0))

            await asyncio.sleep(0.01)
    
        # await agent.system.offboard.set_velocity_ned(VelocityNedYaw(0.0,0.0,-1.0,0.0))
        # await agent.system.offboard.set_position_ned(PositionNedYaw(0.0,0.0,-10.0*(agent.id+1),0.0))

