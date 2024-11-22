# Copyright (c) 2024 Kinova, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.
import argparse

from kortex_api.TCPTransport import TCPTransport
from kortex_api.UDPTransport import UDPTransport
from kortex_api.RouterClient import RouterClient, RouterClientSendOptions
from kortex_api.SessionManager import SessionManager
from kortex_api.autogen.messages import Session_pb2


def parseConnectionArguments(parser=argparse.ArgumentParser()):
    """
    Parse connection arguments.

    Args:
    ----
    parser : argparse.ArgumentParser
        The argument parser to use.

    Returns
    -------
    argparse.Namespace
        The parsed arguments.

    """
    parser.add_argument("--ip", type=str, help="IP address of destination", default="192.168.1.10")
    parser.add_argument("-u", "--username", type=str, help="username to login", default="admin")
    parser.add_argument("-p", "--password", type=str, help="password to login", default="admin")
    return parser.parse_args()


class DeviceConnection:
    """
    Represents a connection to a device.

    Provides methods to create TCP and UDP connections to the device.
    """

    TCP_PORT = 10000
    UDP_PORT = 10001

    @staticmethod
    def createTcpConnection(args):
        """
        Return a RouterClient to create services and send requests to the device.

        Args:
        ----
        args : argparse.Namespace
            The arguments containing connection details.

        Returns
        -------
        DeviceConnection
            A device connection object for TCP.

        """
        return DeviceConnection(
            args.ip, port=DeviceConnection.TCP_PORT, credentials=(args.username, args.password)
        )

    @staticmethod
    def createUdpConnection(args):
        """
        Return a RouterClient to create services and send requests to the device at 1 kHz.

        Args:
        ----
        args : argparse.Namespace
            The arguments containing connection details.

        Returns
        -------
        DeviceConnection
            A device connection object for UDP.

        """
        return DeviceConnection(
            args.ip, port=DeviceConnection.UDP_PORT, credentials=(args.username, args.password)
        )

    def __init__(self, ipAddress, port=TCP_PORT, credentials=("", "")):
        """
        Initialize a new device connection.

        Args:
        ----
        ipAddress : str
            The IP address of the device.
        port : int
            The port to use for the connection (default TCP_PORT).
        credentials : tuple
            A tuple containing the username and password.

        """
        self.ipAddress = ipAddress
        self.port = port
        self.credentials = credentials

        self.sessionManager = None

        # Setup API
        self.transport = TCPTransport() if port == DeviceConnection.TCP_PORT else UDPTransport()
        self.router = RouterClient(self.transport, RouterClient.basicErrorCallback)

    def __enter__(self):
        """
        Connect to the device when entering a 'with' statement.

        Returns
        -------
        RouterClient
            The router client for sending requests.

        """
        self.transport.connect(self.ipAddress, self.port)

        if self.credentials[0] != "":
            session_info = Session_pb2.CreateSessionInfo()
            session_info.username = self.credentials[0]
            session_info.password = self.credentials[1]
            session_info.session_inactivity_timeout = 10000  # milliseconds
            session_info.connection_inactivity_timeout = 2000  # milliseconds

            self.sessionManager = SessionManager(self.router)
            print("Logging as", self.credentials[0], "on device", self.ipAddress)
            self.sessionManager.CreateSession(session_info)

        return self.router

    def __exit__(self, exc_type, exc_value, traceback):
        """
        Disconnect from the device when exiting a 'with' statement.

        Args:
        ----
        exc_type : type
            The type of the exception (if any).
        exc_value : Exception
            The exception instance (if any).
        traceback : traceback
            The traceback object (if any).

        """
        if self.sessionManager is not None:
            router_options = RouterClientSendOptions()
            router_options.timeout_ms = 1000

            self.sessionManager.CloseSession(router_options)

        self.transport.disconnect()
