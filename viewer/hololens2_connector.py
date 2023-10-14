from pynput import keyboard

import threading
import hl2ss
import hl2ss_lnm
import hl2ss_rus
import configparser


class Connector:
    
    def __init__(self):
        # Settings --------------------------------------------------------------------
        config = configparser.ConfigParser()
        config.read('config.ini')
        # HoloLens address
        self.host = config['DEFAULT']['ip']
        print(self.host)
        # Position in camera space (x, y, z)
        #position = [0,0, 0.5]

        # Rotation in camera space (x, y, z, w) as a quaternion
        #rotation = [0, 0, 0, 1]

        # Scale (x, y, z) in meters
        #scale = [0.05, 0.05, 1]

        # Texture file (must be jpg or png)
        #self.texture_file = 'texture.jpg'
        self.texture = None
#------------------------------------------------------------------------------
        self.texture_keys = [] 
        self.stop_event = None #threading.Event()

    def init_ipc(self):
        self.ipc = hl2ss_lnm.ipc_umq(self.host, hl2ss.IPCPort.UNITY_MESSAGE_QUEUE)
        self.ipc.open()

        command_buffer = hl2ss_rus.command_buffer()
        command_buffer.remove_all()
        self.ipc.push(command_buffer)
        results = self.ipc.pull(command_buffer)

    def end_ipc(self):
        if self.stop_event:
            self.stop_event.wait()
        #self.stop_event.wait() 
        self.ipc.close()

    def load_texture(self, texture_file):
        with open(texture_file, mode='rb') as file:
            self.texture = file.read()

    def add_texture(self, key, position, rotation, scale): 
        
        display_list = hl2ss_rus.command_buffer()
        display_list.begin_display_list() # Begin command sequence
        #display_list.remove_all() # Remove all objects that were created remotely
        display_list.create_primitive(hl2ss_rus.PrimitiveType.Quad) # Create a quad, server will return its id
        display_list.set_target_mode(1) # Set server to use the last created object as target, this avoids waiting for the id of the quad
        display_list.set_world_transform(key, position, rotation, scale) # Set the local transform of the cube
        display_list.set_texture(key, self.texture) # Set the texture of the quad
        display_list.set_active(key, hl2ss_rus.ActiveState.Active) # Make the quad visible
        display_list.set_target_mode(0) # Restore target mode
        display_list.end_display_list() # End command sequence
        self.ipc.push(display_list) # Send commands to server
        results = self.ipc.pull(display_list) # Get results from server
        key = results[1] # Get the quad id, created by the 3rd command in the list
        self.texture_keys.append(key)
        
        print(f'Created quad with id {key}')

    def remove_texture(self, index = None, key = None):
        #stop_event.wait()
        if key:
            index = self.texture_keys.index(key)
        command_buffer = hl2ss_rus.command_buffer()

        command_buffer.remove(self.texture_keys[index]) # Destroy quad
        ipc.push(command_buffer)
        results = ipc.pull(command_buffer)
        del self.texture_keys[index] 

    def remove_all_texture(self):
        for k in self.texture_keys:
            self.remove_texture(key = k)

    def change_panel_content(self, title, content):
        display_list = hl2ss_rus.command_buffer()
        display_list.begin_display_list() # Begin command sequence
        display_list.set_panel_content(title, content)
        #display_list.set_target_mode(0) # Restore target mode
        display_list.end_display_list() # End command sequence
        self.ipc.push(display_list) # Send commands to server
        results = self.ipc.pull(display_list) # Get results from server
        key = results[0] # Get the quad id, created by the 3rd command in the list
        #self.texture_keys.append(key)
        
        print(f'Change the panel content with id {key}')
    
    def set_page_size(self, mode, width, height):
        display_list = hl2ss_rus.command_buffer()
        display_list.begin_display_list()
        display_list.set_mode(mode) 
        display_list.send_page_size(width, height)
        display_list.set_mode(-1)
        display_list.end_display_list()
        self.ipc,push(display_list)
        results = self.ipc.pull(display_list)
        key = results[0]

    def set_item(self, mode, item_list):
        display_list = hl2ss_rus.command_buffer()
        display_list.begin_display_list()
        display_list.set_mode(mode)
        
        for item in item_list:
            display_list.send_item(item.position[0], item.position[1], item.position[2], item.position[3], item.item_type)
        display_list.set_mode(-1) 
        display_list.end_display_list()
        self.ipc,push(display_list)
        results = self.ipc.pull(display_list)
        key = results[0]

        print(f'SetItem')


if __name__ == "__main__":
    
    # Position in camera space (x, y, z)
    position = [0,0, 0.4]
    position1 = [0.4,0.4, 0.5]


    # Rotation in camera space (x, y, z, w) as a quaternion
    rotation = [0, 0, 0, 1]

    # Scale (x, y, z) in meters
    scale = [0.05, 0.05, 1]
    scale1 = [0.5, 0.5, 1]
    stop_event = threading.Event()

    def on_press(key):
        if (key == keyboard.Key.esc): 
            stop_event.set()
            return False
        return True

    listener = keyboard.Listener(on_press=on_press)
    listener.start()


    # Texture file (must be jpg or png)
    #self.texture_file = 'texture.jpg'
    connector = Connector()
    connector.init_ipc()
    connector.load_texture('./capture.jpg')
    connector.add_texture(0, position, rotation, scale)
    connector.load_texture('./mumps.png')
    connector.add_texture(connector.texture_keys[-1], position1, rotation, scale)
    connector.end_ipc()
    
    #stop_event.wait()
    listener.join()
