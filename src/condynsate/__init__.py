# Submodules always needs to be imported to ensure registration
from condynsate.animator import Animator # NOQA
from condynsate.core import Simulator # NOQA
from condynsate.keyboard import Keys # NOQA
from condynsate.visualizer import Visualizer # NOQA

__all__ = ["Animator",
           "Simulator",
           "Keys",
           "Visualizer",]


__version__ = '0.7.0'


import os
_root = os.path.split(__file__)[0]
_dirpath = os.path.join(_root, "__assets__")
vals = [os.path.join(_dirpath, f) for f in os.listdir(_dirpath)
        if (f.lower().endswith('.urdf') or f.lower().endswith('.png'))]
keys = []
for v in vals:
    if v.lower().endswith('.urdf'):
        keys.append(os.path.split(v)[1][:-5])
    elif v.lower().endswith('.png'):
        keys.append(os.path.split(v)[1][:-4]+'_img')
__assets__ = dict(zip(keys, vals))
