
import cv2
import hl2ss

cv2.namedWindow('control')

codec = hl2ss.decode_pv(hl2ss.VideoProfile.H265_MAIN)
codec.create(640, 360)
first = True

client = hl2ss._gatherer_xsm()
client.open('192.168.1.7', hl2ss.StreamPort.PERSONAL_VIDEO, 4096, hl2ss.StreamMode.MODE_1)

while (True):
    data = client.get_next_packet()

    payload = hl2ss.unpack_pv(data.payload)
    image = codec.decode(payload.image, 'bgr24')
    if (not first):
        cv2.imshow('control', image)
    first = False

    if ((cv2.waitKey(1) & 0xFF) == 27):
        break

client.close()
