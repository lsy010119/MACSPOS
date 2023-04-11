#!/usr/bin/python3
import asyncio

from mavsdk             import System
from mavsdk.offboard    import OffboardError, VelocityNedYaw, PositionNedYaw



class Connector:


    def __init__(self, agents_sitl, macspos, flags):


        self.agents = agents_sitl

        self.macspos = macspos

        self.flags = flags

    
    async def connect(self, agent):

        print(f"### Agent {agent.id} ###")
        print(f"connecting to {agent.udp_port}..",end="\r")

        await agent.system.connect(system_address=agent.udp_port)
            
        print(f"connected to {agent.udp_port}     \n")

    async def connect_all(self, telem_event_loop):
        
        for agent in self.agents:

            mavlink_connection = telem_event_loop.create_task(self.connect(agent))

            await mavlink_connection

        self.flags.FLAG_all_connected = True





# async def run():
#     # Init the drone
#     print("1")
#     drone = System()
#     print("2")
#     await drone.connect(system_address="udp://:14541")
#     print("3")

#     # Start the tasks
#     asyncio.ensure_future(print_battery(drone))
#     asyncio.ensure_future(print_gps_info(drone))
#     asyncio.ensure_future(print_in_air(drone))
#     asyncio.ensure_future(print_position(drone))

#     while True:
#         await asyncio.sleep(1)


# async def print_battery(drone):
#     async for battery in drone.telemetry.battery():
#         print(f"Battery: {battery.remaining_percent}")


# async def print_gps_info(drone):
#     async for gps_info in drone.telemetry.gps_info():
#         print(f"GPS info: {gps_info}")


# async def print_in_air(drone):
#     async for in_air in drone.telemetry.in_air():
#         print(f"In air: {in_air}")


# async def print_position(drone):
#     async for position in drone.telemetry.position():
#         print(position)


# if __name__ == "__main__":

#     asyncio.run(run())


if __name__ == "__main__":
    # Start the main function
    # agent1 = System(mavsdk_server_address="udp://:14541",sysid=1)
    # agent2 = System(mavsdk_server_address="udp://:14542",sysid=2)
    # agent3 = System(mavsdk_server_address="udp://:14543",sysid=3)

    agent1 = System()
    agent2 = System()
    agent3 = System()
    
    agents = [agent1,agent2,agent3]

    connector = Connector(agents,0)

    telem_event_loop = asyncio.get_event_loop()

    telem_event_loop.run_until_complete(connector.connect_all(telem_event_loop))