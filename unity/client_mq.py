
import socket
import struct
from time import sleep

host = '192.168.1.15'
port = 3816

class PrimitiveType:
    Sphere = 0
    Capsule = 1
    Cylinder = 2
    Cube = 3
    Plane = 4
    Quad = 5

def urs_get_result(s):
    data = struct.unpack('<I', s.recv(4))
    return data[0]

def urs_create_primitive(s, type):
    s.send(struct.pack('<III', 0, 4, type))

def urs_set_active(s, key, state):
    s.send(struct.pack('<IIII', 1, 8, key, state))

def urs_set_transform(s, key, local, px, py, pz, qx, qy, qz, qw, sx, sy, sz):
    s.send(struct.pack('<IIIIffffffffff', 2, 48, key, local, px, py, pz, qx, qy, qz, qw, sx, sy, sz))

def urs_set_color(s, key, r, g, b, a):
    s.send(struct.pack('<IIIffff', 3, 20, key, r, g, b, a))

def urs_set_texture(s, key, texture):
    s.send(struct.pack('<III', 4, 4+len(texture), key))
    s.send(texture)

def urs_remove(s, key):
    s.send(struct.pack('<III', 5, 4, key))

def urs_begin_display_list(s):
    s.send(struct.pack('<II', 16, 0))

def urs_end_display_list(s):
    s.send(struct.pack('<II', 17, 0))


with open('set_texture_test.jpg', mode='rb') as jpg:
    image = jpg.read()

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((host, port))
    urs_create_primitive(s, PrimitiveType.Quad)
    key = urs_get_result(s)
    urs_set_texture(s, key, image)
    urs_get_result(s)
    urs_set_transform(s, key, 1, 1440/2, 936/2, 10, 0, 0, 0, 1, 256, 256, 1)
    urs_get_result(s)


    """
    urs_create_primitive(s, PrimitiveType.Sphere)
    key = urs_get_result(s)
    urs_set_texture(s, key, image)
    urs_get_result(s)
    active = 0

    z = 0.0
    delta = 0.01
    while (True):
        z += delta
        if (z <= 0.0):
            z = 0.0
            delta = -delta
        elif (z >= 1.0):
            z = 1.0
            delta = -delta

        active = active ^ 1
        urs_set_active(s, key, active)
        urs_get_result(s)
        urs_set_color(s, key, 1.0, 1.0-z, 0.0, z)
        urs_set_transform(s, key, 0, 0.0, 0.0, z, 0, 0, 0, 1, 0.2, 0.2, 0.2)
        urs_get_result(s)
        urs_get_result(s)
        sleep(0.016)
    """
