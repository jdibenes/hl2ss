
# Import socket module
import socket
import json
from get_pv import get_pv_image
from pynput import keyboard
from page import Book
from hololens2_connector import Connector
from search import *
import webbrowser
import win32clipboard
# Creatq a sockeq object
import threading
import clipboard
import webbrowser
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
def send_to_clipboard(clip_type, data):
    win32clipboard.OpenClipboard()
    win32clipboard.EmptyClipboard()
    win32clipboard.SetClipboardData(clip_type, data)
    win32clipboard.CloseClipboard()
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

s.connect(('10.0.10.118', 8052))
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

    # Mode 1 touch left
    # Mode 2  touch right
    # Mode 3 set item left
    # Mode 3 set item right
    # Mode 4 copy text clipboard left
    # Mode 5 copy text clipboard right
    # Mode 6 open website
    # Mode 7 open dictionary
    # Mode 8 copy image clipboard
    if code.count("MODE") > 1:
        continue
    #continue
    if code[:5] == 'MODE1':
        
        x, y = code[6:-1].split(',') 
        x = float(x)
        y = float(y)
        page = book.get_page(current_page_left - 1)
        touched_position = convert_position_to_pixel(x, y, page.height, page.width)
        item_type, touched_content = page.get_content_at_touch_position(touched_position)
        
        if item_type == -1:
            print('no item')

        elif item_type == 'text':
            print(item_type)
            #print(touched_content)
            #title, content = dictionary(touched_content)
      
            connector.change_panel_content('Title', touched_content) 
        elif item_type == 'ref_num':
            connector.sent_ref_content('Title', touched_content)
        elif item_type == 'img':
            connector.sent_img_content('Title', touched_content)

   
    elif code[:5] == 'MODE2':
        x, y = code[6:-1].split(',') 
        x = float(x)
        #elif item_type == 'img':
        #    connector.sent_img_content('Title', touched_content)
        y = float(y)
        page = book.get_page(current_page_left)
        touched_position = convert_position_to_pixel(x, y, page.height, page.width)
        item_type, touched_content = page.get_content_at_touch_position(touched_position)
        
        if item_type == -1:
            print('no item')

        elif item_type == 'text':
            print(item_type)
            #print(touched_content)
            #title, content = dictionary(touched_content)
      
            connector.change_panel_content('Title', touched_content) 
        elif item_type == 'ref_num':
            connector.sent_ref_content('Title', touched_content)
        elif item_type == 'img':
            connector.sent_img_content('Title', touched_content)

 

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
    
    elif code[:5] == 'MODE4':
        
        x, y = code[6:-1].split(',') 
        x = float(x)
        y = float(y)
        page = book.get_page(current_page_left - 1)
        touched_position = convert_position_to_pixel(x, y, page.height, page.width)
        item_type, touched_content = page.get_content_at_touch_position(touched_position)
        
        if item_type == -1:
            print('no item')

        elif item_type == 'text': 
            clipboard.copy(touched_content)

    elif code[:5] == 'MODE5':
        x, y = code[6:-1].split(',') 
        x = float(x)
        y = float(y)
        page = book.get_page(current_page_right - 1)
        touched_position = convert_position_to_pixel(x, y, page.height, page.width)
        item_type, touched_content = page.get_content_at_touch_position(touched_position)
        
        if item_type == -1:
            print('no item')

        elif item_type == 'text':
            print(touched_content)
            clipboard.copy(touched_content)


    elif code[:5] == 'MODE6':
        pass
    elif code[:5] == 'MODE7':
        pass
    elif code[:5] == 'MODE8':
        data = get_pv_image()
        send_to_clipboard(win32clipboard.CF_DIB, data)

s.close()    
listener.join()   
