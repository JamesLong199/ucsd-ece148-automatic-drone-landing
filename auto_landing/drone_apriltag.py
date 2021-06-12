from CamPoseDetector import CamPoseDetector
from TagList import TagList
from pose_conversion import transformation_to_pose,pose_to_transformation,eulerAnglesToRotationMatrix,rotationMatrixToEulerAngles
import math
import numpy as np
import time
import cv2


# assume uniform tag_size for all tags used
def drone_apriltag(q_img,q_motion,h_apriltag,is_running,cam_matrix,dist_coeffs,tag_list):
    # inputs
    # -- q_img: queue of image from main process
    # -- q_motion: queue of list_of_motion returning to main process
    # -- h_apriltag: shared double variable of drone height from apriltag output
    # -- is_running: shared flag variable that indicates if apriltag detection is running
    # -- cam_matrix: instrinsic matrix of camera
    # -- dist_coeffs: distortion coefficients of camera
    # -- tag_fam: tag families to be detected, a string separated by spaces
    # -- tag_size: tag size in meter, must not be None
    # output
    # -- (avg_pos,avg_angles): average pos and angles(degree) of camera from all
    #                          tag detections in the image
    assert cam_matrix.shape == (3, 3)
    assert dist_coeffs.shape == (1, 5)
    assert isinstance(tag_list, TagList)

    tag_families = tag_list.get_tag_families()
    tag_size = tag_list.get_tag_size()
    myCPD = CamPoseDetector(cam_matrix, dist_coeffs, tag_families)

    while(1):
        if not q_img.empty():
            is_running.value = 1
            print('Running Apriltag Algorithm!!!')

            img = q_img.get()
            img_center = (int(img.shape[1]/2),int(img.shape[0]/2))

            # obtain camera pose from image
            det_list = myCPD.get_camera_pose(img, tag_size=tag_size, verbose=False)

            # since drone lands on one apriltag, we must ensure there is only one apriltag detected
            # can modify this later to support the presence of multiple apriltags

            if len(det_list) == 1:
                det = det_list[0]
                cam_R = det.pose_R
                cam_t = (det.pose_t).reshape(3,)

                # camera position in tag frame
                cam_pos_tag = cam_R.T @ (-cam_t)
                tag_pose = tag_list.get_tag_pose(det.tag_family.decode('utf-8'),det.tag_id)
                tag_R,tag_t = pose_to_transformation(tag_pose)

                # camera global position
                cam_pos_global = tag_R @ cam_pos_tag + tag_t

                # camera global angles
                total_R = cam_R @ tag_R
                camera_angles_global = rotationMatrixToEulerAngles(total_R)

                # convert angles to degrees
                camera_angles_global = camera_angles_global * 180 / math.pi

                pose_tup = (cam_pos_global,camera_angles_global)
                # update shared height variable
                h_apriltag = camera_angles_global[2].astype('double')

                # # for vertical wall apriltag, align angle is (-90,0,0) and axis is 0(x)
                # motion = pose_to_motion(pose_tup,(-90,0,0),0,img_center,det.center)

                # for ground horizontal apriltag, align angle is (-180,0,0) and axis is 2(z)
                motion = pose_to_motion(h_apriltag,pose_tup, (-180, 0, 0), 2, img_center, det.center)

                q_motion.put(motion)

                annotate_image(img,det.corners,det.center,det.tag_family)

            elif len(det_list) == 0:
                motion = pose_to_motion(h_apriltag,None, None, None, None, None)
                q_motion.put(motion)
                annotate_image(img, None, None, None)

            if is_running.value == 1:
                is_running.value = 0

            end_time = time.time()



# display annotated the image with highlight on apriltag and tag center
# used code from https://www.pyimagesearch.com/2020/11/02/apriltag-with-python/
def annotate_image(image,corners,center,tag_fam):
    if corners is not None:
        print('Annotating Image!')
        (ptA, ptB, ptC, ptD) = corners
        ptB = (int(ptB[0]), int(ptB[1]))
        ptC = (int(ptC[0]), int(ptC[1]))
        ptD = (int(ptD[0]), int(ptD[1]))
        ptA = (int(ptA[0]), int(ptA[1]))
        # draw the bounding box of the AprilTag detection
        cv2.line(image, ptA, ptB, (0, 255, 0), 2)
        cv2.line(image, ptB, ptC, (0, 255, 0), 2)
        cv2.line(image, ptC, ptD, (0, 255, 0), 2)
        cv2.line(image, ptD, ptA, (0, 255, 0), 2)
        # draw the center (x, y)-coordinates of the AprilTag
        (cX, cY) = (int(center[0]), int(center[1]))
        cv2.circle(image, (cX, cY), 5, (0, 0, 255), -1)
        # draw the tag family on the image
        tagFamily = tag_fam.decode("utf-8")
        cv2.putText(image, tagFamily, (ptA[0], ptA[1] - 15),
                    cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 255, 0), 2)

    # show the output image after AprilTag detection
    cv2.imshow("Apriltag Detection", image)
    cv2.waitKey(10)

# given camera pose, calculate drone's movement
def pose_to_motion(h_apriltag,pose,align_angle,axis,img_center,tag_center):
    # inputs
    # -- h_apriltag: drone height from last apriltag algorithm output
    # -- pose: (position,angle) of drone in global coordinate frame
    #          with origin at tag center
    # -- align_angle: (x_angle,y_angle,z_angle) of drone when its image plane is aligned with
    #                 the tag plane, ranges from -180 to 180
    # -- axis: axis the drone rotates around, one from (0:x,1:y,2:z), we takes the advantage
    #          that the drone only rotates around one axis
    # -- img_center: center coordinate (x,y) of image
    # -- tag_center: tag center coordinate (x,y) in image

    angle_tol = 15            # angle alignment tolerance
    x_tol = 0.05
    y_tol = 0.05
    z_land = 0.5              # landing zone
    z_low_speed = 0.8         # low speed zone

    # go upward if no apriltag detection at low altitude
    motion = []
    if pose is None:
        if h_apriltag < z_low_speed:
            motion.append('up')
            motion.append('forward_0')     # add 0 to set speed to zero
            motion.append('clockwise_0')
            motion.append('left_0')
        else:
            motion.append('up_0')
            motion.append('forward_0')
            motion.append('clockwise_0')
            motion.append('left_0')
        return motion

    motion.append('up_0')

    pos, angle = pose
    angle_diff = (angle - align_angle)[axis]

    dist = np.sqrt(np.square(pos).sum())   # unit: meter
    pixel_tol = dist*70                    # unit: pixel

    if angle_diff <= -angle_tol or angle_diff >= angle_tol:
        # rotate only when image center and tag center are aligned
        rotate_flag = True

        # at low altitude, stop aligning image center with tag center
        if pos[2] > z_land:
            if img_center[0] - tag_center[0] > pixel_tol:       # unit pixel
                motion.append('left')
                rotate_flag = False
            elif img_center[0] - tag_center[0] < -pixel_tol:       # unit pixel
                motion.append('right')
                rotate_flag = False
            else:
                motion.append('left_0')

            if img_center[1] - tag_center[1] > pixel_tol:       # unit pixel
                motion.append('forward')
                rotate_flag = False
            elif img_center[1] - tag_center[1] < -(pixel_tol):       # unit pixel
                motion.append('backward')
                rotate_flag = False
            else:
                motion.append('forward_0')

        if rotate_flag == True:
            if angle_diff <= -angle_tol:
                motion.append('counter_clockwise')
            elif angle_diff >= angle_tol:
                motion.append('clockwise')
            else:
                motion.append('clockwise_0')

        if pos[2] < z_low_speed:
            # reduce speed when altitude is low, improves stability
            motion.append('low_speed')



    else:
        land_flag = True
        if pos[0] <= -x_tol:
            motion.append('right')
            land_flag = False
        elif pos[0] >= x_tol:
            motion.append('left')
            land_flag = False
        else:
            motion.append('left_0')

        pos[0] -= 0.03
        pos[1] -= 0.03

        if pos[1] <= -y_tol:
            motion.append('forward')
            land_flag = False
        elif pos[1] >= y_tol:
            motion.append('backward')
            land_flag = False
        else:
            motion.append('forward_0')

        # land when the drone position is aligned with tag center
        # lower speed below a threshold altitude
        if pos[2] < z_land and land_flag == True:
            motion.append('land')
        elif pos[2] < z_low_speed:
            # reduce speed when altitude is low, improves stability
            motion.append('down')
            motion.append('low_speed')
        else:
            motion.append('down')

    # update drone height
    h_apriltag = pos[2]

    return motion

# code for following vertical wall apriltag
# def pose_to_motion(pose,align_angle,axis,img_center,tag_center):
#     # inputs
#     # -- pose: (position,angle) of drone in global coordinate frame
#     #          with origin at tag center
#     # -- align_angle: (x_angle,y_angle,z_angle) of drone when its image plane is aligned with
#     #                 the tag plane, ranges from -180 to 180
#     # -- axis: axis the drone rotates around, one from (0:x,1:y,2:z), we takes the advantage
#     #          that the drone only rotates around one axis
#     # -- img_center: center coordinate (x,y) of image
#     # -- tag_center: tag center coordinate (x,y) in image
#     pos, angle = pose
#
#     angle_diff = (angle - align_angle)[axis]
#     angle_tol = 10
#     x_tol = 0.05
#     y_tol = 0.4
#     z_tol = 0.05
#
#     motion = []
#
#     dist = np.sqrt(np.square(pos).sum())
#     dist_tol = 0.6     # unit meter
#     pixel_tol = dist*70
#
#     if dist > dist_tol:
#         if img_center[0] - tag_center[0] > pixel_tol:       # unit pixel
#             motion.append('left')
#         elif img_center[0] - tag_center[0] < -pixel_tol:       # unit pixel
#             motion.append('right')
#
#         if img_center[1] - tag_center[1] > pixel_tol:       # unit pixel
#             motion.append('down')
#         elif img_center[1] - tag_center[1] < -(pixel_tol):       # unit pixel
#             motion.append('up')
#
#         motion.append('forward')
#
#     else:
#
#         pos_abs = np.abs(pos)
#         if np.argmax(pos_abs) == 0:
#             if pos[0] <= -x_tol:
#                 motion.append('left')
#             elif pos[0] >= x_tol:
#                 motion.append('right')
#             else:
#                 motion.append('left_0')
#
#         elif np.argmax(pos_abs) == 1:
#             if pos[1] <= -y_tol:
#                 motion.append('forward')
#             elif pos[1] >= y_tol:
#                 motion.append('backward')
#             else:
#                 motion.append('forward_0')
#
#         else:
#             if pos[2] <= -z_tol:
#                 motion.append('down')
#             elif pos[2] >= z_tol:
#                 motion.append('up')
#             else:
#                 motion.append('up_0')
#
#         if angle_diff <= -angle_tol:
#             motion.append('counter_clockwise')
#         elif angle_diff >= angle_tol:
#             motion.append('clockwise')
#         else:
#             motion.append('clockwise_0')
#
#     return motion




















