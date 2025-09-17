import sys
from pathlib import Path
import os
import condynsate
parent = str(Path(__file__).resolve().parents[2])
source = r'src\condynsate'
sys.path.insert(0, os.path.join(parent, source))


# -- Project information -----------------------------------------------------
project = 'condynsate'
copyright = '2025, G. Schaer'
author = 'G. Schaer'
release = condynsate.__version__


# -- General configuration ---------------------------------------------------
extensions = ['sphinx.ext.autodoc', 
              'sphinx.ext.coverage', 
              'sphinx.ext.napoleon',
              "sphinxcontrib.collections",
              'numpydoc',
              'autoclasstoc',
              'sphinx_rtd_theme',
              'nbsphinx']
templates_path = ['_templates']
exclude_patterns = []


# -- Extensions configuration ------------------------------------------------
numpydoc_show_class_members = False
collections = {
    'examples': {
        'driver': 'copy_folder',
        'source': '../examples',
        'target': 'examples/',
        'ignore': ['*.py', '.sh'],
    },
    'tutorials': {
        'driver': 'copy_folder',
        'source': '../tutorials',
        'target': 'tutorials/',
        'ignore': ['*.py', '.sh'],
    }
}


# -- Options for HTML output -------------------------------------------------
html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']




