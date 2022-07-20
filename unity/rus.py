
import struct
import numpy as np

class PrimitiveType:
    Sphere = 0
    Capsule = 1
    Cylinder = 2
    Cube = 3
    Plane = 4
    Quad = 5


def download(s, bytes):
    data = bytearray()

    while (bytes > 0):
        chunk = s.recv(bytes)
        size = len(chunk)
        if (size == 0):
            raise('download failed')
        data.extend(chunk)
        bytes -= size

    return data


def get_results(s, n):
    return np.frombuffer(download(s, 4*n), dtype=np.uint32)


def create_primitive(s, type):
    s.send(struct.pack('<III', 0, 4, type))


def set_active(s, key, state):
    s.send(struct.pack('<IIII', 1, 8, key, state))


def set_world_transform(s, key, px, py, pz, qx, qy, qz, qw, sx, sy, sz):
    s.send(struct.pack('<IIIffffffffff', 2, 44, key, px, py, pz, qx, qy, qz, qw, sx, sy, sz))


def set_local_transform(s, key, px, py, pz, qx, qy, qz, qw, sx, sy, sz):
    s.send(struct.pack('<IIIffffffffff', 3, 44, key, px, py, pz, qx, qy, qz, qw, sx, sy, sz))


def set_color(s, key, r, g, b, a):
    s.send(struct.pack('<IIIffff', 4, 20, key, r, g, b, a))


def set_texture(s, key, texture):
    s.send(struct.pack('<III', 5, 4+len(texture), key))
    s.send(texture)


def create_text(s): 
    s.send(struct.pack('<II', 6, 0))


def set_text(s, key, font_size, r, g, b, a, string):
    data = string.encode('utf-8')
    s.send(struct.pack('<IIIfffff', 7, 24+len(data), key, font_size, r, g, b, a))
    s.send(data)


def remove(s, key):
    s.send(struct.pack('<III', 16, 4, key))


def remove_all(s):
    s.send(struct.pack('<II', 17, 0))


def begin_display_list(s):
    s.send(struct.pack('<II', 18, 0))


def end_display_list(s):
    s.send(struct.pack('<II', 19, 0))


def set_target_mode(s, mode):
    s.send(struct.pack('<III', 20, 4, mode))

