import numpy    as    np
import cv2

cam_urls = ['http://192.168.43.171:8080/video', 'http://192.168.43.96:8080/video']
caps = [cv2.VideoCapture(cam_url) for cam_url in cam_urls]

def update(val=0):
    #	disparity	range	is	tuned	for	'aloe'	image	pair
    frames = [cap.read()[1] for cap in caps]
    #         # frames = [cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) for frame in frames]
    imgL, imgR = frames
    stereo.setBlockSize(cv2.getTrackbarPos('window_size', 'disparity'))
    stereo.setUniquenessRatio(cv2.getTrackbarPos('uniquenessRatio',
                                                 'disparity'))
    stereo.setSpeckleWindowSize(cv2.getTrackbarPos('speckleWindowSize',
                                                   'disparity'))
    stereo.setSpeckleRange(cv2.getTrackbarPos('speckleRange', 'disparity'))
    stereo.setDisp12MaxDiff(cv2.getTrackbarPos('disp12MaxDiff',
                                               'disparity'))
    print('computing	disparity…')
    disp = stereo.compute(imgL, imgR).astype(np.float32) / 16.0
    cv2.imshow('left', imgL)
    cv2.imshow('disparity', (disp - min_disp) / num_disp)


if __name__ == "__main__":
    window_size = 5
    min_disp = 16
    num_disp = 192 - min_disp
    blockSize = window_size
    uniquenessRatio = 1
    speckleRange = 3
    speckleWindowSize = 3
    disp12MaxDiff = 200
    P1 = 600
    P2 = 2400
    imgL = cv2.imread('images/color1_small.jpg')
    imgR = cv2.imread('images/color2_small.jpg')
    cv2.namedWindow('disparity')
    cv2.createTrackbar('speckleRange', 'disparity', speckleRange, 50,
                       update)
    cv2.createTrackbar('window_size', 'disparity', window_size, 21, update)
    cv2.createTrackbar('speckleWindowSize', 'disparity', speckleWindowSize,
                       200, update)
    cv2.createTrackbar('uniquenessRatio', 'disparity', uniquenessRatio, 50,
                       update)
    cv2.createTrackbar('disp12MaxDiff', 'disparity', disp12MaxDiff, 250,
                       update)
    stereo = cv2.StereoSGBM_create(
        minDisparity=min_disp,
        numDisparities=num_disp,
        blockSize=window_size,
        uniquenessRatio=uniquenessRatio,
        speckleRange=speckleRange,
        speckleWindowSize=speckleWindowSize,
        disp12MaxDiff=disp12MaxDiff,
        P1=P1,
        P2=P2
    )
    update()
    cv2.waitKey()

# import numpy as np
# import cv2
# from matplotlib import pyplot as plt
#
# # cam_urls = ['http://192.168.88.233:8080/video', 'http://192.168.88.231:8080/video']
# cam_urls = ['http://192.168.43.171:8080/video', 'http://192.168.43.96:8080/video']
#
#
# caps = [cv2.VideoCapture(cam_url) for cam_url in cam_urls]
#
# while (True):
#     try:
#
#         frames = [cap.read()[1] for cap in caps]
#         # frames = [cv2.cvtColor(frame, cv2.COLOR_BGR2GRAY) for frame in frames]
#         imgL, imgR = frames
#         cv2.imshow(f'frame L',imgL)
#         cv2.imshow(f'frame R', imgL)
#
#         # stereo = cv2.StereoBM_create(numDisparities=16, blockSize=15)
#         # disparity = stereo.compute(imgL, imgR)
#         # plt.imshow(disparity, 'gray')
#         # plt.show()
#     except:
#         pass
#
#     if cv2.waitKey(1) & 0xFF == ord('q'):
#         cv2.destroyAllWindows()
#         break