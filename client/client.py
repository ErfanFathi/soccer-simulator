import socket
from cfg import *
from observation_pb2 import Observation
from commands_pb2 import Commands
import sys

class Network:
    """ UDP Socket setup for soccer simulator """ 
    def __init__(self, IP='localhost', COMMAND_PORT=command_port, OBSERVATION_PORT=observation_port):
        """
            IP : simulator ip ['localhost'] \n
            COMMAND_PORT : send command port [20011] \n
            OBSERVATION_PORT : receive observation port [1234]
        """
        self.IP = IP
        self.COMMAND_PORT = COMMAND_PORT
        self.OBSERVATION_PORT = OBSERVATION_PORT

        # soccer simulator receive team status setup
        self.client_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_address = (self.IP, self.OBSERVATION_PORT)
        self.client_socket.bind(self.server_address)
        self.client_socket.settimeout(1)

        # soccer simulator send command setup
        self.server_socket = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        self.server_socket.settimeout(1)

    def receive(self):
        # receive observation from server
        packet, _ = self.client_socket.recvfrom(4096)
        return packet

    def send(self, commands):
        # send command for server
        self.server_socket.sendto(commands, (self.IP, self.COMMAND_PORT))

class Provider:
    """
        Provide class help you too serialize and deserialize
        protobuf packets for send and receive *-
    """
    def __init__(self):
        self.network = Network()

        self.observationPacket = Observation()

    def send(self, commands, NO, replacement):

        """
            commands : A dictionary include of commands like protobuf \n
            replacement : Boolean flag for specify command is replavement command or not ;)
            NO : How many robots command 
        """

        commandsPacket = Commands()

        commandsPacket.reset_frame_number = commands['reset_frame_number']

        if not replacement:
            for i in range(NO):
                command_rob             = commandsPacket.robot_command.add()
                command_rob.id          = commands['robot_id_'+str(i)]       
                command_rob.yellow_team = commands['robot_yellow_team_'+str(i)]
                command_rob.vx          = commands['robot_vx_'+str(i)]
                command_rob.vy          = commands['robot_vy_'+str(i)]   
                command_rob.vw          = commands['robot_vw_'+str(i)]
                command_rob.kickspeedx  = commands['robot_kickspeedx_'+str(i)]
                command_rob.kickspeedz  = commands['robot_kickspeedz_'+str(i)]
                command_rob.spinner     = commands['robot_spinner_'+str(i)]

        if replacement:
            commandsPacket.replacement.ball_replacement.x  = commands['ball_x']
            commandsPacket.replacement.ball_replacement.y  = commands['ball_y']
            commandsPacket.replacement.ball_replacement.vx = commands['ball_vx']
            commandsPacket.replacement.ball_replacement.vy = commands['ball_vy']

            for i in range(NO):
                replacement_rob             = commandsPacket.replacement.robot_replacement.add()
                replacement_rob.id          = commands['robot_id_'+str(i)]
                replacement_rob.yellow_team = commands['robot_yellow_team_'+str(i)] 
                replacement_rob.x           = commands['robot_x_'+str(i)]
                replacement_rob.y           = commands['robot_y_'+str(i)]
                replacement_rob.orientation = commands['robot_orientation_'+str(i)]
                replacement_rob.vx          = commands['robot_vx_'+str(i)]
                replacement_rob.vy          = commands['robot_vy_'+str(i)]
                replacement_rob.vw          = commands['robot_vw_'+str(i)]

        self.network.send(commandsPacket.SerializeToString())

    def receive(self):

        # Return observation in a dictionary 

        packet = self.network.receive()
        self.observationPacket.ParseFromString(packet)

        _observation = {}
        _observation['frame_number'] = self.observationPacket.frame_number

        for i in range(robot_count):
            _observation['robot_id_'+str(i)]          = self.observationPacket.robots[i].id
            _observation['robot_yellow_team_'+str(i)] = self.observationPacket.robots[i].yellow_team
            _observation['robot_x_'+str(i)]           = self.observationPacket.robots[i].x / 1000.0
            _observation['robot_y_'+str(i)]           = self.observationPacket.robots[i].y / 1000.0
            _observation['robot_orientation_'+str(i)] = self.observationPacket.robots[i].orientation
            _observation['robot_vx_'+str(i)]          = self.observationPacket.robots[i].vx
            _observation['robot_vy_'+str(i)]          = self.observationPacket.robots[i].vy
            _observation['robot_vw_'+str(i)]          = self.observationPacket.robots[i].vw
            _observation['robot_touch_ball_'+str(i)]  = self.observationPacket.robots[i].touch_ball
            _observation['robot_flat_kick_'+str(i)]   = self.observationPacket.robots[i].flat_kick
            _observation['robot_chip_kick_'+str(i)]   = self.observationPacket.robots[i].chip_kick
        
        _observation['ball_x'] = self.observationPacket.ball[0].x / 1000.0
        _observation['ball_y'] = self.observationPacket.ball[0].y / 1000.0
        _observation['ball_vx'] = self.observationPacket.ball[0].vx
        _observation['ball_vy'] = self.observationPacket.ball[0].vy

        return _observation
