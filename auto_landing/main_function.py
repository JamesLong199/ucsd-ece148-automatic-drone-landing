# Based on example code on hanyazou/TelloPy
# https://github.com/hanyazou/TelloPy/blob/develop-0.7.0/tellopy/examples/keyboard_and_video.py

import sys
import traceback
import tellopy
import av
import cv2.cv2 as cv2  # for avoidance of pylint error
import numpy as np
import time

import pygame
import pygame.display  # pygame module to control the display window and screen
import pygame.key      # pygame module to work with the keyboard
from drone_controls import get_keyboard_input,auto_motion
from flight_data_update import update_hud,hud

from drone_apriltag import drone_apriltag
from TagList import TagList
import pickle
import multiprocessing as mp

prev_flight_data = None
font = None
date_fmt = '%Y-%m-%d_%H%M%S'

def flightDataHandler(event, sender, data):
    global prev_flight_data
    text = str(data)
    if prev_flight_data != text:
        update_hud(hud, sender, data)
        prev_flight_data = text


def main():
    # camera parameter and apriltag info for automatic landing algorithm
    with open(f'C:/Users/jianz/PycharmProjects/djitello/localization/drone_cam_params.pickle', 'rb') as f:
        cam_matrix,dist_coeffs = pickle.load(f)

    myTagList = TagList(tag_size=0.084)
    myTagList.add_tag(family='tagStandard41h12', id=0, pos=(0, 0, 0),
                      angles=(180, 0, 0), radian=False)

    # pygame display window settings
    pygame.init()
    pygame.display.init()
    pygame.display.set_mode((400, 400))    # pygame display window size

    global font
    font = pygame.font.SysFont("dejavusansmono", 32)

    global wid
    if 'window' in pygame.display.get_wm_info():
        wid = pygame.display.get_wm_info()['window']

    # drone setting
    drone = tellopy.Tello()
    drone.connect()
    drone.subscribe(drone.EVENT_FLIGHT_DATA, flightDataHandler)
    speed = 30

    auto_speed = {           # drone speed for automatic landing
        'clockwise':20,
        'counter_clockwise':20,
        'forward':15,
        'backward':15,
        'down':30,
        'up':30,
        'left':15,
        'right':15,
        'land':0
    }

    # multiprocessing
    q_img = mp.Queue()
    q_motion = mp.Queue()
    h_apriltag = mp.Value('d',1000.0)
    is_running = mp.Value('i', 0)  # 1 means apriltag detection is running
    p_april = mp.Process(target=drone_apriltag,
                         args=((q_img,q_motion,h_apriltag,is_running,cam_matrix,dist_coeffs,myTagList)))

    auto_flag = False
    motion = None
    vid = None
    # directory for storing captured images and videos
    dir = f'C:/Users/jianz/PycharmProjects/djitello/images_and_videos/'

    try:
        container = av.open(drone.get_video_stream())
        container.no_buffer = True
        frame_generator = container.decode(video=0)  # a python generator object
        p_april.start()

        while 1:
            time.sleep(0.01)  # loop with pygame.event.get() is too mush tight w/o some sleep

            frame = next(frame_generator)
            image = cv2.cvtColor(np.array(frame.to_image()), cv2.COLOR_RGB2BGR)
            # for camera mirror image
            image = np.flip(image, axis=0)

            if auto_flag == True:
                # to synchronize main process and apriltag process
                if q_img.empty() and is_running.value==0:
                    print('Putting image on queue!!!')
                    q_img.put(image)
                if not q_motion.empty():
                    motion = q_motion.get()
                    auto_motion(drone,motion,auto_speed)

            else:
                cv2.destroyWindow("Apriltag Detection")


            vid,auto_flag = get_keyboard_input(drone, speed, image, vid, dir, auto_flag)

            cv2.imshow('image', image)
            cv2.waitKey(1)

    except Exception as ex:
        exc_type, exc_value, exc_traceback = sys.exc_info()
        traceback.print_exception(exc_type, exc_value, exc_traceback)
        print(ex)

    finally:
        print('Shutting down connection to drone...')
        drone.quit()
        cv2.destroyAllWindows()

        p_april.terminate()
        print('Stopped getting motion input')

        exit(1)

if __name__ == '__main__':
    main()
