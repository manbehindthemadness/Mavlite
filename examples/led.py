# Write your code here :-)
import time
import board
import digitalio
from rainbowio import colorwheel
import neopixel


red = digitalio.DigitalInOut(board.LED_RED)
green = digitalio.DigitalInOut(board.LED_GREEN)
blue = digitalio.DigitalInOut(board.LED_BLUE)
red.direction = digitalio.Direction.OUTPUT
green.direction = digitalio.Direction.OUTPUT
blue.direction = digitalio.Direction.OUTPUT


class LED:
    """
    Lighting controls.

    from led import LED; LED().test_neo()
    """
    num_pixels = 1
    pixel_pin = board.NEOPIXEL  # The neopixel can be accessed in this way
    pixels = neopixel.NeoPixel(pixel_pin, num_pixels, brightness=0.3, auto_write=False)

    RED = (255, 0, 0)
    YELLOW = (255, 150, 0)
    GREEN = (0, 255, 0)
    CYAN = (0, 255, 255)
    BLUE = (0, 0, 255)
    PURPLE = (180, 0, 255)

    halts = {
        'warn': (171, 84, 0),  # yellow.
        'err': (252, 0, 3),  # Red.
        'fail': (216, 36, 0),  # Orange.
        'crit': (120, 0, 135),  # Crimson.
        'good': (60, 195, 0),  # Green.
        'cool': (0, 24, 231),  # Blue.
        'release': (0, 120, 135)  # Cyan.
    }
    rng = list(range(255))
    rng.reverse()

    pos = 0

    def __init__(self):
        self.n_color = (0, 0, 0)

        self.red = red
        self.green = green
        self.blue = blue

        self.red.value = False
        self.green.value = True
        self.blue.value = True

    def color_chase(self, color, wait):
        for i in range(self.num_pixels):
            self.pixels[i] = color
            time.sleep(wait)
            self.pixels.show()
        time.sleep(0.5)

    def rainbow_cycle(self, wait, halt=None, test=False):

        for j in self.rng:
            j = self.pos
            for i in range(self.num_pixels):
                rc_index = (i * 256 // self.num_pixels) + j
                self.pixels[i] = colorwheel(rc_index & 255)
            self.pixels.show()
            if test:
                print(self.pixels[0])
            time.sleep(wait)
            if self.pos < max(self.rng):
                self.pos += 1
            else:
                self.pos = 0
            if halt:
                if self.pixels[0] == self.halts[halt]:
                    break

    @staticmethod
    def test():
        led = digitalio.DigitalInOut(board.LED)
        led.direction = digitalio.Direction.OUTPUT

        while True:
            led.value = True
            time.sleep(0.5)
            led.value = False
            time.sleep(0.5)

    def test_neo(self):
        while True:
            # self.pixels.fill(self.RED)
            # self.pixels.show()
            # Increase or decrease to change the speed of the solid color change.
            # time.sleep(1)
            # self.pixels.fill(self.GREEN)
            # self.pixels.show()
            # time.sleep(1)
            # self.pixels.fill(self.BLUE)
            # self.pixels.show()
            # time.sleep(1)
            # self.color_chase(self.RED, 0.1)  # Increase the number to slow down the color chase
            # self.color_chase(self.YELLOW, 0.1)
            # self.color_chase(self.GREEN, 0.1)
            # self.color_chase(self.CYAN, 0.1)
            # self.color_chase(self.BLUE, 0.1)
            # self.color_chase(self.PURPLE, 0.1)

            self.rainbow_cycle(0.5, test=True)  # Increase the number to slow down the rainbow
