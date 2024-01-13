
import struct
import hl2ss


# 3D Primitive Types
class PrimitiveType:
    Sphere = 0
    Capsule = 1
    Cylinder = 2
    Cube = 3
    Plane = 4
    Quad = 5


# Server Target Mode
class TargetMode:
    UseID = 0
    UseLast = 1


# Object Active State
class ActiveState:
    Inactive = 0,
    Active = 1


#------------------------------------------------------------------------------
# Commands
#------------------------------------------------------------------------------

class command_buffer(hl2ss.umq_command_buffer):
    def create_primitive(self, type):
        self.add(0, struct.pack('<I', type))

    def set_active(self, key, state):
        self.add(1, struct.pack('<II', key, state))

    def set_world_transform(self, key, position, rotation, scale):
        self.add(2, struct.pack('<Iffffffffff', key, position[0], position[1], position[2], rotation[0], rotation[1], rotation[2], rotation[3], scale[0], scale[1], scale[2]))

    def set_local_transform(self, key, position, rotation, scale):
        self.add(3, struct.pack('<Iffffffffff', key, position[0], position[1], position[2], rotation[0], rotation[1], rotation[2], rotation[3], scale[0], scale[1], scale[2]))

    def set_color(self, key, rgba):
        self.add(4, struct.pack('<Iffff', key, rgba[0], rgba[1], rgba[2], rgba[3]))

    def set_texture(self, key, texture):
        self.add(5, struct.pack('<I', key) + texture)

    def create_text(self): 
        self.add(6, b'')

    def set_text(self, key, font_size, rgba, string):
        self.add(7, struct.pack('<Ifffff', key, font_size, rgba[0], rgba[1], rgba[2], rgba[3]) + string.encode('utf-8'))

    def load_mesh(self, data):
        # struct.pack
        # <If bla bla is format
        self.add(8, data)
    
    def set_panel_content(self,title, content):
        string = '<TITLE>' + title + '<CONTENT>' + content
        self.add(9, string.encode('utf-8'))

    
    def sent_img_content(self,title, content):
        string = '<TITLE>' + title + '<CONTENT>' + content
        self.add(21, string.encode('utf-8'))
    
    def sent_ref_content(self,title, content):
        string = '<TITLE>' + title + '<CONTENT>' + content
        self.add(22, string.encode('utf-8'))
 

    def highlight_reference(self, xmin, ymin, xmax, ymax):
        self.add(10, struct.pack('<IIII', xmin, ymin, xmax, ymax))

    def send_item(self, xmin, xmax, ymin, ymax, item_type):
        #self.add(11, json_string.encode('utf-8'))
        self.add(11, struct.pack('<IIII', xmin, xmax, ymin, ymax) + item_type.encode('utf-8'))

    def set_mode(self, mode):
        # mode 0 = nah, 1 = left , 2 = right
        self.add(12, struct.pack('<I', mode))


    def send_page_size(self, width, height):
        #self.add(11, json_string.encode('utf-8'))
        self.add(13, struct.pack('<II', width, height))

    def visualize(self):
        self.add(14, b'')

    def remove(self, key):
        self.add(16, struct.pack('<I', key))

    def remove_all(self):
        self.add(17, b'')

    def begin_display_list(self):
        self.add(18, b'')

    def end_display_list(self):
        self.add(19, b'')

    def set_target_mode(self, mode):
        self.add(20, struct.pack('<I', mode))

    # TODO: customize functionality here

    def set_point_clouds(self, key, point_clouds):
        self.add(15, struckt.pack(''))
