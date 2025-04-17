# Submodules always needs to be imported to ensure registration
from .simulator import Simulator # NOQA
from .visualizer import Visualizer # NOQA
from .animator import Animator # NOQA
from .keyboard import Keys # NOQA


__all__ = ["Simulator",
           "Visualizer",
           "Animator",
           "Keys",]


__version__ = '0.6.11'


import os
_root = os.path.split(__file__)[0]
_dirpath = os.path.join(_root, "__assets__")
vals = [os.path.join(_dirpath, f) for f in os.listdir(_dirpath)
        if f.lower().endswith('.urdf')]
keys = [os.path.split(v)[1][:-5] for v in vals]
__assets__ = dict(zip(keys, vals))
