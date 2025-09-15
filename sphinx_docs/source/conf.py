import sys
from pathlib import Path
import os
import condynsate
parent = str(Path(__file__).resolve().parents[2])
source = r'src\condynsate'
sys.path.insert(0, os.path.join(parent, source))

# Configuration file for the Sphinx documentation builder.
#
# For the full list of built-in configuration values, see the documentation:
# https://www.sphinx-doc.org/en/master/usage/configuration.html

# -- Project information -----------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#project-information

project = 'condynsate'
copyright = '2025, G. Schaer'
author = 'G. Schaer'
release = condynsate.__version__

# -- General configuration ---------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#general-configuration

extensions = ['sphinx.ext.autodoc', 
              'sphinx.ext.coverage', 
              'sphinx.ext.napoleon',
              'numpydoc',
              'autoclasstoc',
              'sphinx_rtd_theme',
              'nbsphinx']

templates_path = ['_templates']
exclude_patterns = []

numpydoc_show_class_members = False


# -- Options for HTML output -------------------------------------------------
# https://www.sphinx-doc.org/en/master/usage/configuration.html#options-for-html-output

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
