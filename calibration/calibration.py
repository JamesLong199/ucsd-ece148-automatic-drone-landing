from CamCalib import CamCalib
import os
import numpy as np
import pickle
import cv2

if __name__ == '__main__':
    myCam = CamCalib(w=8,h=6)
    # load images
    img_dir = 'new_calibration_image'
    for i,img_name in enumerate(os.listdir(img_dir)):
        # fname = img_dir + '\' + img_name
        fname = '%s\%s' % (img_dir,img_name)
        myCam.add_image(fname)

    # calibrate camera
    retval, cameraMatrix, distCoeffs, rvecs, tvecs = \
    myCam.run_calibration()

    np.set_printoptions(precision=3)
    # root mean square reprojection error in terms of pixels
    # between 0.1 and 1.0 for a good calibration
    print('retval:',retval)
    print('cameraMatrix:\n',cameraMatrix)
    print('distCoeffs:\n',distCoeffs)

    # before adding mirror mount
    # with open('drone_cam_params.pickle','wb') as f:
    #     pickle.dump([cameraMatrix,distCoeffs],f)

    # after adding mirror mount
    with open('new_drone_cam_params.pickle','wb') as f:
        pickle.dump([cameraMatrix,distCoeffs],f)

    # for i,img_name in enumerate(os.listdir(img_dir)):
    #     if i % 5 == 0:
    #         fname = '%s\%s' % (img_dir, img_name)
    #         img_re = myCam.run_undistortion(fname)
    #         cv2.imshow('',img_re)
    #         cv2.waitKey(0)
        
