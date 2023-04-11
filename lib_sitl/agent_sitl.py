#!/usr/bin/python3
import asyncio

from mavsdk             import System



class AgentSITL:


    def __init__(self, id):

        self.id = id

        self.udp_port    = "udp://:1454" + str( id + 1 )
        self.mavsdk_port = 50040 + (id + 1)

        self.system = System(port=self.mavsdk_port)
