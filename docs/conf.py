# Configuration file for the Sphinx documentation builder.

# -- Project information -----------------------------------------------------
project = 'vyra_base'
copyright = '2025, Holger Schernewski'
author = 'Holger Schernewski'
release = '0.1.0'  # Should match __version__.py

# -- General configuration ---------------------------------------------------
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.intersphinx',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',        # Add source code links
    'sphinx.ext.githubpages',     # GitHub Pages compatibility
    'sphinx_autodoc_typehints',
]

# Autodoc settings
autodoc_default_options = {
    'members': True,
    'member-order': 'bysource',
    'undoc-members': True,
    'show-inheritance': True,
    'special-members': '__init__',
    'exclude-members': '__weakref__'
}
autodoc_typehints = 'description'  # Type hints in description
autodoc_typehints_format = 'short'  # Short type names

# Napoleon settings (Google/NumPy docstring support)
napoleon_google_docstring = True
napoleon_numpy_docstring = True
napoleon_include_init_with_doc = True
napoleon_use_param = True
napoleon_use_rtype = True

# Intersphinx references
intersphinx_mapping = {
    'python': ('https://docs.python.org/3', None),
    'rclpy': ("https://docs.ros.org/en/kilted/", None),
    'sqlalchemy': ('https://docs.sqlalchemy.org/en/20/', None),
}

autodoc_mock_imports = [
    "rclpy", 
    "std_msgs", 
    "geometry_msgs", 
    "sensor_msgs",
    "builtin_interfaces",
    "unique_identifier_msgs",
    "grpc",
    "grpc.aio",
]

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store']

# -- Options for HTML output -------------------------------------------------
html_theme = 'furo'
html_static_path = ['_static']
html_title = f"{project} {release}"
html_theme_options = {
    "light_css_variables": {
        "color-brand-primary": "#336790",
        "color-brand-content": "#336790",
    },
}

# -- Path Setup (damit src importierbar ist) ---------------------------------
import os
import sys
sys.path.insert(0, os.path.abspath('../src/'))
