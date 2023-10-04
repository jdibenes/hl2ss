
# Import socket module
import socket            
from pynput import keyboard

from hololen2_connector import Connector  
# Creatq a sockeq object
s = socket.socket()        
 
# Define the port on which you want to connect 
connector = Connector()

connector.init_ipc()
#connector.load_texture('./capture.jpg')
#connector.add_texture(0, position, rotation, scale)
#connector.load_texture('./mumps.png')
#connector.add_texture(connector.texture_keys[-1], position1, rotation, scale)
 
# connect to the server on local computer
s.connect(('10.9.8.106', 8052))
while(True):

    print (s.recv(1024).decode())

s.close()    
    
