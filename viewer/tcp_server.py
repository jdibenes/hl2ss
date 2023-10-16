
# Import socket module
import socket
import json
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
current_page_left = 206
current_page_right = 207

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
page_left = book.get_page(current_page_left - 1)
page_right  = book.get_page(current_page_right - 1)
connector.set_page_size(1, page_left.width, page_left.height)
connector.set_items(1, page_left.items)
connector.set_page_size(2, page_right.width, page_right.height)
connector.set_items(2, page_right.items)
     
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
        page = book.get_page(current_page_left)
        touched_position = convert_position_to_pixel(x, y, page.height, page.width)
        #print(touched_position)
        #book.pages[205].parsing_page_content()
        item_type, touched_content = page.get_content_at_touch_position(touched_position)
        
        if item_type == -1:
            print('no item')

        elif item_type == 'text':
            print(item_type)
            #print(touched_content)
            #title, content = dictionary(touched_content)
      
            connector.change_panel_content('Title', touched_content) 
        elif item_type == 'ref_num':
            pass
        elif item_type == 'img':
            pass
        
    elif code[:5] == 'MODE2':
        x, y = code[6:-1].split(',') 
        x = float(x)
        y = float(y)
        page = book.get_page(current_page_right)
        touched_position = convert_position_to_pixel(x, y, page.height, page.width)
        #print(touched_position)
        #book.pages[205].parsing_page_content()
        item_type, touched_content = page.get_content_at_touch_position(touched_position)
        
        if item_type == -1:
            print('no item')

        elif item_type == 'text':
            #print(item_type)
            #print(touched_content)
            #title, content = dictionary(touched_content)
      
            connector.change_panel_content('Title', touched_content) 
        elif item_type == 'ref_num':
            pass
        elif item_type == 'img':
            pass

    elif code[:5] == 'MODE3':
        if int(code[5:]) > 0:
            current_page_left = current_page_right + 1 #int(code[5:])
            current_page_right = current_page_right + 2 #int(code[5:])
        else:
            current_page_right = current_page_left - 2
            current_page_left = current_page_left  - 1 #int(code[5:]) 
        page_left = book.get_page(current_page_left - 1)
        page_right  = book.get_page(current_page_right - 1)
        connector.set_page_size(1, page_left.width, page_left.height)
        connector.set_item(1, page_left.items)
        connector.set_page_size(2, page_right.width, page_right.height)
        connector.set_item(2, page_right.items)
        #connector.visualize(1)
        #connector.visualize(2)
        #print(current_page)
    
    elif code[:5] == 'MODE4':
        print("mode 4")

s.close()    
listener.join()   
