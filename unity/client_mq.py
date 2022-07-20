
import socket
import rus
import hl2ss
from time import sleep

host = '192.168.1.15'
port_mq = 3816

with open('set_texture_test.jpg', mode='rb') as jpg:
    image = jpg.read()

with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as s:
    s.connect((host, port_mq))

    rus.create_text(s, 0.5, 1.0, 1.0, 1.0, 1.0, 'HELLO FROM PYTHON')
    key = rus.get_result(s)
    print(key)
    
    rus.set_transform(s, key, 0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0, 1, 1, 1)
    ret = rus.get_result(s)
    print(ret)

    rus.set_active(s, key, 1)
    rus.get_result(s)
    print(ret)
    

    #urs_create_primitive(s, PrimitiveType.Quad)
    #key = urs_get_result(s)
    #urs_set_texture(s, key, image)
    #urs_get_result(s)
    #urs_set_transform(s, key, 1, 1440/2, 936/2, 10, 0, 0, 0, 1, 256, 256, 1)
    #urs_get_result(s)


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
