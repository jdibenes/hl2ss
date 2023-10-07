
# Import socket module
import socket            
from pynput import keyboard
from page import Book
from hololens2_connector import Connector  
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
stop_event = threading.Event()

enable = True

book = Book('./books/opd/') 
book.get_page(205)
book.pages[205].parsing_page_content()

def on_press(key):
    global enable
    #global trigger
    if (key == keyboard.Key.esc):
        enable = False
    #elif (key == keyboard.Key.space):
    #    trigger = True
    enable = key != keyboard.Key.esc
    return enable


listener = keyboard.Listener(on_press=on_press)
listener.start()

s.connect(('10.9.8.106', 8052))
while(enable):
    # CODE,X,Y
    #print (s.recv(1024).decode())
    code = s.recv(1024).decode()
    print(code)
    # if code[:4] == 'D':
s.close()    
listener.join()   
