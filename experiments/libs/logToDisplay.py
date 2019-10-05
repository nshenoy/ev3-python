from sys import stderr
import os

class logToDisplay:
    def __init__(self):
        # Set font for EV3 brick display
        os.system('setfont Lat15-TerminusBold14')

    def log(self, msg):
        """Logs a msg to EV3 display and output window """

        print(msg)
        print(msg, file=stderr)
