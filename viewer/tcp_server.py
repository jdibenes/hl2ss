
# Import socket module
import socket            
from pynput import keyboard
from page import Book
from hololens2_connector import Connector
from search import *
# Creatq a sockeq object
import threading

def convert_position_to_pixel(x, y, height, width):
    return int(x*width), int(y*height)

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
current_page = 205

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

s.connect(('10.9.8.105', 8052))
while(enable):
    # CODE,X,Y
    #print (s.recv(1024).decode())
    code = s.recv(1024).decode()
    print(code)
    #continue
    if code[:5] == 'MODE1':
        
        x, y = code[6:-1].split(',') 
        x = float(x)
        y = float(y)
        page = book.get_page(current_page)
        touched_position = convert_position_to_pixel(x, y, page.height, page.width)
        print(touched_position)
        #book.pages[205].parsing_page_content()
        item_type, touched_content = page.get_content_at_touch_position(touched_position)
        
        if item_type == -1:
            print('no item')

        elif item_type == 'text':
            print(item_type)
            print(touched_content)
            #title, content = dictionary(touched_content)
      
            connector.change_panel_content('Title', touched_content) 
        elif item_type == 'ref_num':
            pass
        elif item_type == 'img':
            pass

s.close()    
listener.join()   