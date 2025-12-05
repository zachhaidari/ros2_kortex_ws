import sys
import os
import time

from kortex_api.autogen.client_stubs.BaseClientRpc import BaseClient
from kortex_api.autogen.client_stubs.SessionClientRpc import SessionClient
from kortex_api.autogen.messages import Session_pb2
from kortex_api.RouterClient import RouterClient, RouterClientSendOptions
from kortex_api.TCPTransport import TCPTransport

ip = "192.168.1.10"   # default robot IP
username = "admin"
password = "admin"

transport = TCPTransport()
transport.connect(ip, port=10000)
router = RouterClient(transport, RouterClientSendOptions())

session = SessionClient(router)
create_session = Session_pb2.CreateSessionInfo()
create_session.username = username
create_session.password = password
create_session.session_inactivity_timeout = 60000
create_session.connection_inactivity_timeout = 2000
session.CreateSession(create_session)

base = BaseClient(router)
base.ClearFaults()

session.CloseSession()
transport.disconnect()

print("Fault clear command sent.")
