
import struct

class PrimitiveType:
    Sphere = 0
    Capsule = 1
    Cylinder = 2
    Cube = 3
    Plane = 4
    Quad = 5


def get_result(s):
    data = struct.unpack('<I', s.recv(4))
    return data[0]


def create_primitive(s, type):
    s.send(struct.pack('<III', 0, 4, type))


def set_active(s, key, state):
    s.send(struct.pack('<IIII', 1, 8, key, state))


def set_transform(s, key, local, px, py, pz, qx, qy, qz, qw, sx, sy, sz):
    s.send(struct.pack('<IIIIffffffffff', 2, 48, key, local, px, py, pz, qx, qy, qz, qw, sx, sy, sz))


def set_color(s, key, r, g, b, a):
    s.send(struct.pack('<IIIffff', 3, 20, key, r, g, b, a))


def set_texture(s, key, texture):
    s.send(struct.pack('<III', 4, 4+len(texture), key))
    s.send(texture)


def remove(s, key):
    s.send(struct.pack('<III', 5, 4, key))


def create_text(s, size, r, g, b, a, string):
    data = string.encode('utf-8')
    s.send(struct.pack('<IIfffff', 6, 20+len(data), size, r, g, b, a))
    s.send(data)


def begin_display_list(s):
    s.send(struct.pack('<II', 16, 0))


def end_display_list(s):
    s.send(struct.pack('<II', 17, 0))

