#!/usr/bin/python3
import asyncio
import time

from mavsdk.offboard    import OffboardError, VelocityNedYaw, PositionNedYaw


class Telemetry:


    def __init__(self, agents_sitl, macspos, simparams):


        self.agents_sitl = agents_sitl
        
        self.macspos     = macspos

        self.simparams   = simparams



    async def telem_posvelNED(self, agent):

        while not all(self.simparams.FLAG_connected):

            # print("Telemetry : waiting for northn...",end="\r")

            await asyncio.sleep(0.001)
    
        # mission_item = MissionItem(home_position[0], home_position[1], home_position[2], 0, True, float('nan'), float('nan'), MissionItem.CameraAction.NONE, float('nan'), float('nan'))
        # mission_plan = MissionPlan([mission_item])
        # await drone.mission.set_return_to_launch_after_mission(True)
        # await drone.mission.upload_mission(mission_plan)


        # await agent.system.telemetry.set_home_position(*zero_point)

        async for pos_ned in agent.system.telemetry.position_velocity_ned():
            
            self.macspos.sharedmemory.agents[agent.id].pos[0] = pos_ned.position.east_m     +   self.simparams.spawn_loc[agent.id][0]
            self.macspos.sharedmemory.agents[agent.id].pos[1] = pos_ned.position.north_m    +   self.simparams.spawn_loc[agent.id][1]
            self.macspos.sharedmemory.agents[agent.id].vel[0] = pos_ned.velocity.east_m_s
            self.macspos.sharedmemory.agents[agent.id].vel[1] = pos_ned.velocity.north_m_s

            # print(f"Agent.{agent.id} : {self.macspos.sharedmemory.agents[agent.id].pos[0],self.macspos.sharedmemory.agents[agent.id].pos[1]}")
            await asyncio.sleep(0.001)

        # async for pos in agent.system.telemetry.position():
            
        #     # print(f"Agent.{agent.id} : {pos.position.north_m,pos_ned.position.east_m,pos_ned.position.down_m}")
        #     print(f"Agent.{agent.id} : {pos}")
        #     await asyncio.sleep(0.1)
