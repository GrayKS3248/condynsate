"""
This modules provides keyboard listening for I/O
"""


###############################################################################
#DEPENDENCIES
###############################################################################
from pynput import keyboard
from pynput.keyboard import Key as k
import signal
import sys


class Keys:
    """
    Keys detected keyboard events and stores key information. Used for 
    polling key presses.
    """
    def __init__(self):
        """
        Initializes the Keys class. Starts a new non-blocking thread that
        listens to the keyboard. Starts 2 new threads to detect script 
        termination events.

        Returns
        -------
        None.

        """
        # Set that no key is down
        self.down = False
        self.key = None
        
        # Asynch listen for script exit
        signal.signal(signal.SIGTERM, self._signal_handler)
        signal.signal(signal.SIGINT, self._signal_handler)
        
        # Start the keyboard listener
        self.listener = keyboard.Listener(on_press=self._on_press,
                                          on_release=self._on_release)
        self.listener.start()


    def _signal_handler(self,
                        sig,
                        frame):
        """
        Handles script termination events so the keyboard listener exits
        gracefully.

        Parameters
        ----------
        sig : int
            The signal number.
        frame : signal.frame object
            The current stack frame.

        Returns
        -------
        None.

        """
        # If exception KeyboardInterrupt, then exit gracefully
        print("Exit command detected. " +
              "Terminating keyboard listener. " + 
              "Goodbye.")
        self.listener.stop()
        sys.exit()
        

    def _on_press(self,
                  key):
        """
        Takes place on a key down event. Stores the pressed key as a unique
        string and marks that a key is down.

        Parameters
        ----------
        key : pynput.keyboard.Key object
            The current key.

        Returns
        -------
        None.

        """
        # Set that a key is down
        self.down = True
        
        # If the key is alpha numeric
        try:
            self.key = key.char
            
        # If the key is not alpha numeric
        except AttributeError:
            if key == k.space:
                self.key = "space"
            elif key == k.enter:
                self.key = "enter"
            elif key == k.backspace:
                self.key = "backspace"
            elif key == k.tab:
                self.key = "tab"
            elif key == k.esc:
                self.key = "esc"
            elif key == k.shift or key == k.shift_r:
                self.key = "shift"
            elif key == k.ctrl_l or key == k.ctrl_r:
                self.key = "ctrl"
            elif key == k.alt_l or key == k.alt_gr:
                self.key = "alt"
            else:
                self.key = None


    def _on_release(self,
                    key):
        """
        Takes place on a key up event. Removes the pressed key string and
        marks that no keys are down. When 'esc' is pressed, automatically
        terminates the keyboard listener.

        Parameters
        ----------
        key : pynput.keyboard.Key object
            The current key.

        Returns
        -------
        bool
            False on termination event ('esc')

        """
        # Determine if it's time to stop listening
        leave = self.key == "esc"
        
        # Set that no key is down
        self.down = False
        self.key = None
        
        # Stop the listener gracefully
        if leave:
            print("Termination command detected. " + 
                  "Terminating keyboard listener. " + 
                  "Goodbye")
            return False
        
        
    def is_pressed(self,
                   key):
        """
        Returns a boolean flag to indicate whether a desired key is pressed.
        The key may be alpha numeric or some special keys.

        Parameters
        ----------
        key : string
            The key to be detected. May be alpha numeric ("a", "A", "1", "!",
            "`", etc.) or some special keys. The special keys are as follows:
            "space", "enter", "backspace", "tab", "shift", "alt", "tab",
            "ctrl", and "esc". If "esc" is pressed, the keyboard listener will
            automatically stop and cannot be restarted.

        Returns
        -------
        bool
            A boolean flag to indicate whether the desired key is pressed.

        """
        return self.down and key==self.key
            