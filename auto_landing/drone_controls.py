# Based on example code on hanyazou/TelloPy
# https://github.com/hanyazou/TelloPy/blob/develop-0.7.0/tellopy/examples/keyboard_and_video.py

import cv2.cv2 as cv2  # for avoidance of pylint error
import time
import pygame
import datetime

# my own version of take_picture with openCV
def take_picture(speed,img,dir):
    if speed == 0:
        time_str = datetime.datetime.fromtimestamp(time.time()).strftime('%Y_%m_%d_%H_%M_%S')
        path = dir + time_str + '.jpg'
        cv2.imwrite(path,img)
        print('Saved photo to %s' % path)
    else:
        print('Speed is not 0, taking picture failed!')

# my own version of record_video with openCV
def record_video(img,vid,dir,stop_flag=False):
    if stop_flag == False:
        if vid is None:
            time_str = datetime.datetime.fromtimestamp(time.time()).strftime('%Y_%m_%d_%H_%M_%S')
            path = dir + time_str + '.avi'
            vid = cv2.VideoWriter(path,
                                  cv2.VideoWriter_fourcc(*'MJPG'),
                                  fps=24, frameSize=(960, 720))
            print('start recording video\npath %s' % path)
        if img is not None:
            vid.write(img)

        return vid
    else:
        vid.release()
        print('Video recording finished!')
        return None

def palm_land(drone, speed):
    if speed == 0:
        return
    drone.palm_land()

controls = {
    'w': 'forward',
    's': 'backward',
    'a': 'left',
    'd': 'right',
    'space': 'up',
    'left shift': 'down',
    'right shift': 'down',
    'q': 'counter_clockwise',
    'e': 'clockwise',
    # arrow keys for fast turns and altitude adjustments
    'left': lambda drone, speed: drone.counter_clockwise(speed*2),
    'right': lambda drone, speed: drone.clockwise(speed*2),
    'up': lambda drone, speed: drone.up(speed*2),
    'down': lambda drone, speed: drone.down(speed*2),
    'tab': lambda drone, speed: drone.takeoff(),
    'backspace': lambda drone, speed: drone.land(),
    'p': palm_land,
    'enter': take_picture,
    'return': take_picture,
    'r': record_video,
    'l': 'auto_landing'
}


def get_keyboard_input(drone,speed,img,vid,dir,auto_flag):
    if vid is not None:
        record_video(img, vid, dir, stop_flag=False)

    for e in pygame.event.get():
        # WASD for movement
        # during the interval when key is pressed
        if e.type == pygame.locals.KEYDOWN:
            print('+' + pygame.key.name(e.key))
            keyname = pygame.key.name(e.key)
            if keyname == 'escape':
                drone.quit()
                exit(0)

            # motion commands, disabled in auto_mode
            if keyname not in ['return','enter','r','l'] and auto_flag == False:
                # perform commands at current speed, e.g. forward, backward, ..
                if keyname in controls:
                    key_handler = controls[keyname]
                    if type(key_handler) == str:
                        getattr(drone, key_handler)(speed)
                    else:
                        key_handler(drone, speed)

            # press and hold 'r' to record video
            if keyname == 'r':
                key_handler = controls['r']
                vid = key_handler(img, vid, dir,stop_flag=False)

        # when key is released
        elif e.type == pygame.locals.KEYUP:
            print('-' + pygame.key.name(e.key))
            keyname = pygame.key.name(e.key)
            if keyname in controls:
                key_handler = controls[keyname]
                # take picture
                if keyname in ['return','enter']:
                    key_handler(0, img, dir)
                # stop video recording at key release
                elif keyname == 'r':
                    vid = key_handler(img, vid, dir, stop_flag=True)

                # press 'l' to trigger/end auto mode
                elif keyname == 'l':
                    auto_flag = not(auto_flag)
                    zero_speed(drone)

                # disable motion commands in auto mode
                elif type(key_handler) == str:
                    if auto_flag == False:
                        getattr(drone, key_handler)(0)
                else:
                    if auto_flag == False:
                        key_handler(drone, 0)

    return vid,auto_flag

def auto_motion(drone,motion,auto_speed):
    # inputs
    # -- drone: Tello object
    # -- motion: list of motion control
    # -- speed: speed for auto motion
    low_speed_flag = False
    if motion[-1] == 'low_speed':
        low_speed_flag = True
        motion.pop()

    for command in motion:
        if command == 'land':
            drone.land()
        else:
            if command[-1] != '0':
                if low_speed_flag == True:
                    getattr(drone, command)(auto_speed[command]/2)
                else:
                    getattr(drone,command)(auto_speed[command])
            else:
                getattr(drone, command[:-2])(0)

# set drone speed in all direction to 0
def zero_speed(drone):
    drone.left(0)
    drone.up(0)
    drone.forward(0)
    drone.clockwise(0)






