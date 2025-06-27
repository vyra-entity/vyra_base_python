# Configuration file for the Sphinx documentation builder.

# -- Project information -----------------------------------------------------
project = 'vyra_base'
copyright = '2025, Holger Schernewski'
author = 'Holger Schernewski'

# -- General configuration ---------------------------------------------------
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.intersphinx',
    'sphinx.ext.napoleon',
    'sphinx_autodoc_typehints',
]

# Autodoc-Einstellungen (optional, aber hilfreich)
autodoc_default_options = {
    'members': True,
    'undoc-members': True,
    'show-inheritance': True,
}
autodoc_typehints = 'description'  # Typen in der Beschreibung statt Signatur

# Intersphinx-Referenzen
intersphinx_mapping = {
    'python': ('https://docs.python.org/3', None),
    'rclpy': ("https://docs.ros.org/en/humble/", None),
    'sqlalchemy': ('https://docs.sqlalchemy.org/en/20/', None)
}

autodoc_mock_imports = [
    "rclpy", 
    "std_msgs", 
    "geometry_msgs", 
    "sensor_msgs",
    "builtin_interfaces",
    "unique_identifier_msgs"
]

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# -- Options for HTML output -------------------------------------------------
html_theme = 'furo'
html_static_path = ['_static']

# -- Path Setup (damit src importierbar ist) ---------------------------------
import os
import sys
sys.path.insert(0, os.path.abspath('../src/'))  # ggf. anpassen
