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
import time


###############################################################################
#KEYS CLASS
###############################################################################
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
        self.key_buffer = []
        self.mod_key_buffer = []
        
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
        
        
    def _get_key_string(self,
                        key):
        """
        Gets the unique key string for each alpha numeric key and some special
        keys.

        Parameters
        ----------
        key : pynput.keyboard.Key object
            The current key whose string is got.

        Returns
        -------
        key_str : string
            The associated key string.

        """
        key_str = ""
        
        # If the key is alpha numeric
        try:
            key_str = key.char.lower()
            
        # If the key is not alpha numeric
        except AttributeError:
            if key == k.space:
                key_str = "space"
            elif key == k.enter:
                key_str = "enter"
            elif key == k.backspace:
                key_str = "backspace"
            elif key == k.tab:
                key_str = "tab"
            elif key == k.esc:
                key_str = "esc"
            elif key == k.shift or key == k.shift_r:
                key_str = "shift+"
            elif key == k.ctrl_l or key == k.ctrl_r:
                key_str = "ctrl+"
            elif key == k.alt_l or key == k.alt_gr:
                key_str = "alt+"
                
        return key_str
        
    
    def _add_to_buffer(self,
                       key_str):
        # Handle modifiers
        if key_str=="shift+" or key_str=="ctrl+" or key_str=="alt+":
            if key_str not in self.mod_key_buffer:
                self.mod_key_buffer.append(key_str)
            
        # Handle other keys
        else:
            if key_str not in self.key_buffer:
                self.key_buffer.append(key_str)
    
    
    def _remove_from_buffer(self,
                            key_str):
        # Handle modifiers
        if key_str=="shift+" or key_str=="ctrl+" or key_str=="alt+":
            if key_str in self.mod_key_buffer:
                self.mod_key_buffer.pop(self.mod_key_buffer.index(key_str))
            
        # Handle other keys
        else:
            if key_str in self.key_buffer:
                self.key_buffer.pop(self.key_buffer.index(key_str))
            


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
        
        # Add the key string to the key buffer
        key_str = self._get_key_string(key)
        self._add_to_buffer(key_str)
        #print(self.key_buffer)
        #print(self.mod_key_buffer)


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
        # Get the key string
        key_str = self._get_key_string(key)
        
        # Determine if it's time to stop listening
        leave = key_str == "esc"
        
        # Remove the released key from the key buffer
        self._remove_from_buffer(key_str)
        #print(self.key_buffer)
        #print(self.mod_key_buffer)
        
        # If the buffer is empty, note that no keys are down
        if len(self.key_buffer) == 0 and len(self.mod_key_buffer):
            self.down = False
        
        # Stop the listener gracefully
        if leave:
            # Wait for 0.5 seconds before leaving
            time.sleep(0.5)
            print("Termination command detected. " + 
                  "Terminating keyboard listener. " + 
                  "Goodbye")
            return False
        
        
    def is_pressed(self,
                   key_str):
        """
        Returns a boolean flag to indicate whether a desired key is pressed.
        The key may be alpha numeric or some special keys.

        Parameters
        ----------
        key_str : string
            The key to be detected. May be alpha numeric ("a", "1", "`", etc.)
            or some special keys.
            
            The special keys are as follows:
            "space", "enter", "backspace", "tab", "tab", and "esc".
            
            The following modifiers can also be used:
            "shift+", "alt+", and "ctrl+".
            
            Modifiers are added with the following format:
            "shift+a", "ctrl+a", "alt+a", "shift+ctrl+alt+a", etc.
            
            If "esc" is pressed, the keyboard listener will
            automatically stop and cannot be restarted.

        Returns
        -------
        bool
            A boolean flag to indicate whether the desired key is pressed.

        """
        key = key_str
        mods = []
        
        # Disambiguate mods from keys
        if ("shift+" in key):
            key = key.replace('shift+','')
            mods.append('shift+')
        if ("ctrl+" in key):
            key = key.replace('ctrl+','')
            mods.append('ctrl+')
        if ("alt+" in key):
            key = key.replace('alt+','')
            mods.append('alt+')
            
        # Determine if correct key is down
        key_is_down = key in self.key_buffer
        
        # Determine if correct mods are down
        if len(mods) == 0:
            mods_are_down = len(self.mod_key_buffer)==0
        else:
            mods_are_down = all([mod in self.mod_key_buffer for mod in mods])
        
        # Return whether the proper keys and modifiers are pressed
        return key_is_down and mods_are_down
            