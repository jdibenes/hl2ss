
import struct
import numpy as np
import hl2ss


# Message Queue Port
class Port:
    IPC = 3816


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

class _CommandBuffer:
    def __init__(self):
        self._buffer = bytearray()
        self._count = 0

    def create_primitive(self, type):
        self._buffer.extend(struct.pack('<III', 0, 4, type))
        self._count += 1

    def set_active(self, key, state):
        self._buffer.extend(struct.pack('<IIII', 1, 8, key, state))
        self._count += 1

    def set_world_transform(self, key, position, rotation, scale):
        self._buffer.extend(struct.pack('<IIIffffffffff', 2, 44, key, position[0], position[1], position[2], rotation[0], rotation[1], rotation[2], rotation[3], scale[0], scale[1], scale[2]))
        self._count += 1

    def set_local_transform(self, key, position, rotation, scale):
        self._buffer.extend(struct.pack('<IIIffffffffff', 3, 44, key, position[0], position[1], position[2], rotation[0], rotation[1], rotation[2], rotation[3], scale[0], scale[1], scale[2]))
        self._count += 1

    def set_color(self, key, rgba):
        self._buffer.extend(struct.pack('<IIIffff', 4, 20, key, rgba[0], rgba[1], rgba[2], rgba[3]))
        self._count += 1

    def set_texture(self, key, texture):
        self._buffer.extend(struct.pack('<III', 5, 4+len(texture), key))
        self._buffer.extend(texture)
        self._count += 1

    def create_text(self): 
        self._buffer.extend(struct.pack('<II', 6, 0))
        self._count += 1

    def set_text(self, key, font_size, rgba, string):
        data = string.encode('utf-8')
        self._buffer.extend(struct.pack('<IIIfffff', 7, 24+len(data), key, font_size, rgba[0], rgba[1], rgba[2], rgba[3]))
        self._buffer.extend(data)
        self._count += 1

    def remove(self, key):
        self._buffer.extend(struct.pack('<III', 16, 4, key))
        self._count += 1

    def remove_all(self):
        self._buffer.extend(struct.pack('<II', 17, 0))
        self._count += 1

    def begin_display_list(self):
        self._buffer.extend(struct.pack('<II', 18, 0))
        self._count += 1

    def end_display_list(self):
        self._buffer.extend(struct.pack('<II', 19, 0))
        self._count += 1

    def set_target_mode(self, mode):
        self._buffer.extend(struct.pack('<III', 20, 4, mode))
        self._count += 1


#------------------------------------------------------------------------------
# IPC Client
#------------------------------------------------------------------------------

class client:
    def open(self, host, port):
        self._client = hl2ss._client()
        self._client.open(host, port)

    def push(self, command_buffer):
        self._client.sendall(command_buffer._buffer)

    def pop(self, command_buffer):
        return np.frombuffer(self._client.download(4 * command_buffer._count, hl2ss.ChunkSize.SINGLE_TRANSFER), dtype=np.uint32)

    def close(self):
        self._client.close()


def connect_client_mq(host, port):
    c = client()
    c.open(host, port)
    return c


def create_command_buffer():
    return _CommandBuffer()

