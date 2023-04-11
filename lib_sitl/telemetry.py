#!/usr/bin/python3
import asyncio
import time

from mavsdk.offboard    import OffboardError, VelocityNedYaw, PositionNedYaw


class Telemetry:


    def __init__(self, agents_sitl, macspos, flags):


        self.agents_sitl = agents_sitl
        
        self.macspos     = macspos

        self.flags       = flags


    async def telem_posvelNED(self, agent):

        while not self.flags.FLAG_all_connected:

            # print("Telemetry : waiting for northn...",end="\r")

            await asyncio.sleep(0.001)
        

        # async for gps_info in agent.system.telemetry.position():

            # print(f"Agent.{agent.id} : {gps_info}")

            # await asyncio.sleep(1)


        async for pos_ned in agent.system.telemetry.position_velocity_ned():
            
        #     # agent_macspos.pos[0] = pos_ned.position.north_m
        #     # agent_macspos.pos[1] = pos_ned.position.east_m
        #     # agent_macspos.vel[0] = pos_ned.velocity.north_m_s
        #     # agent_macspos.vel[1] = pos_ned.velocity.down_m_s

            print(f"Agent.{agent.id} : {pos_ned.position.north_m,pos_ned.position.east_m,pos_ned.position.down_m}")
            await asyncio.sleep(0.1)
