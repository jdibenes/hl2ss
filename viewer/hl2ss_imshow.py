
import cv2


#------------------------------------------------------------------------------
# Extension: Workaround for the cv2/av imshow issue on some platforms (Ubuntu)
#------------------------------------------------------------------------------
# https://github.com/opencv/opencv/issues/21952
# https://github.com/PyAV-Org/PyAV/issues/978

try:
    cv2.namedWindow('_cv2_21952_av_978_workaround')
    cv2.destroyWindow('_cv2_21952_av_978_workaround')
except:
    pass

