
import struct
import numpy as np
import hl2ss

class IPCPort:
    MESSAGE_QUEUE = 3816


class PrimitiveType:
    Sphere = 0
    Capsule = 1
    Cylinder = 2
    Cube = 3
    Plane = 4
    Quad = 5


def connect_client_mq(host, port):
    c = hl2ss.client()
    c.open(host, port)
    return c


def get_results(s, n):
    return np.frombuffer(s.download(4*n, hl2ss.client.DEFAULT_CHUNK_SIZE), dtype=np.uint32)


def create_primitive(b, type):
    b.extend(struct.pack('<III', 0, 4, type))


def set_active(b, key, state):
    b.extend(struct.pack('<IIII', 1, 8, key, state))


def set_world_transform(b, key, position, rotation, scale):
    b.extend(struct.pack('<IIIffffffffff', 2, 44, key, position[0], position[1], position[2], rotation[0], rotation[1], rotation[2], rotation[3], scale[0], scale[1], scale[2]))


def set_local_transform(b, key, position, rotation, scale):
    b.extend(struct.pack('<IIIffffffffff', 3, 44, key, position[0], position[1], position[2], rotation[0], rotation[1], rotation[2], rotation[3], scale[0], scale[1], scale[2]))


def set_color(b, key, rgba):
    b.extend(struct.pack('<IIIffff', 4, 20, key, rgba[0], rgba[1], rgba[2], rgba[3]))


def set_texture(b, key, texture):
    b.extend(struct.pack('<III', 5, 4+len(texture), key))
    b.extend(texture)


def create_text(b): 
    b.extend(struct.pack('<II', 6, 0))


def set_text(b, key, font_size, rgba, string):
    data = string.encode('utf-8')
    b.extend(struct.pack('<IIIfffff', 7, 24+len(data), key, font_size, rgba[0], rgba[1], rgba[2], rgba[3]))
    b.extend(data)


def remove(b, key):
    b.extend(struct.pack('<III', 16, 4, key))


def remove_all(b):
    b.extend(struct.pack('<II', 17, 0))


def begin_display_list(b):
    b.extend(struct.pack('<II', 18, 0))


def end_display_list(b):
    b.extend(struct.pack('<II', 19, 0))


def set_target_mode(b, mode):
    b.extend(struct.pack('<III', 20, 4, mode))

