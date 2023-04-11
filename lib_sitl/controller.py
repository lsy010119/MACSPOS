#!/usr/bin/python3
import asyncio
import time

from mavsdk.offboard    import OffboardError, VelocityNedYaw, PositionNedYaw


class Controller:

    def __init__(self, agents_sitl, macspos, flags ):

        self.agents = agents_sitl
        self.macspos = macspos
        self.flags = flags


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


        while not self.flags.FLAG_all_connected:

            # print("Telemetry : waiting for northn...",end="\r")

            await asyncio.sleep(0.001)

        await agent.system.action.arm()        


        await self.offboard_start(agent)

        print(f"Agent.{agent.id} : control start")

        # await agent.system.offboard.set_velocity_ned(VelocityNedYaw(0.0,0.0,-1.0,0.0))
        await agent.system.offboard.set_position_ned(PositionNedYaw(0.0,0.0,-10.0*(agent.id+1),0.0))

        await asyncio.sleep(10)