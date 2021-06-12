# copied from example code on hanyazou/TelloPy
# https://github.com/hanyazou/TelloPy/blob/develop-0.7.0/tellopy/examples/keyboard_and_video.py

import pygame
import pygame.display  # pygame module to control the display window and screen
import pygame.key      # pygame module to work with the keyboard
import pygame.locals   # contains various constants used by pygame
import pygame.font     # for loading and rendering fonts

class FlightDataDisplay(object):
    # previous flight data value and surface to overlay
    _value = None
    _surface = None
    # function (drone, data) => new value
    # default is lambda drone,data: getattr(data, self._key)
    _update = None
    def __init__(self, key, format, font, colour=(255,255,255), update=None):
        self._key = key
        self._format = format
        self._colour = colour

        if update:
            self._update = update
        else:
            # get parameter value specified by 'key', e.g. height, battery, etc.
            self._update = lambda drone,data: getattr(data, self._key)

    def update(self, drone, data):
        new_value = self._update(drone, data)
        if self._value != new_value:
            self._value = new_value
            self._surface = font.render(self._format % (new_value,), True, self._colour)
        return self._surface

def flight_data_mode(drone, *args):
    return (drone.zoom and "VID" or "PIC")

def update_hud(hud, drone, flight_data):
    (w,h) = (158,0) # width available on side of screen in 4:3 mode
    blits = []
    for element in hud:
        surface = element.update(drone, flight_data)
        if surface is None:
            continue
        blits += [(surface, (0, h))]                    # surface,position on surface
        # w = max(w, surface.get_width())
        h += surface.get_height()
    h += 64  # add some padding
    overlay = pygame.Surface((w, h), pygame.SRCALPHA)   # create a surface of (w,h)
    overlay.fill((0,0,0)) # remove for mplayer overlay mode
    for blit in blits:
        overlay.blit(*blit)                             # put display contents on surface
    pygame.display.get_surface().blit(overlay, (0,0))
    pygame.display.update(overlay.get_rect())

pygame.font.init()
font = pygame.font.SysFont("dejavusansmono", 32)
hud = [
    FlightDataDisplay('height', 'ALT %3d', font),
    FlightDataDisplay('ground_speed', 'SPD %3d', font),
    FlightDataDisplay('battery_percentage', 'BAT %3d%%', font),
    FlightDataDisplay('wifi_strength', 'NET %3d%%', font),
    FlightDataDisplay(None, 'CAM %s', font, update=flight_data_mode),
]

