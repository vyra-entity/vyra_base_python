# Configuration file for the Sphinx documentation builder.

# -- Project information -----------------------------------------------------
project = 'vyra_base'
copyright = '2025, Holger Schernewski'
author = 'Holger Schernewski'
release = '0.1.8'  # Should match __version__.py

# -- General configuration ---------------------------------------------------
extensions = [
    'sphinx.ext.autodoc',
    'sphinx.ext.intersphinx',
    'sphinx.ext.napoleon',
    'sphinx.ext.viewcode',        # Add source code links
    'sphinx.ext.githubpages',     # GitHub Pages compatibility
    'sphinx.ext.graphviz',        # Graphviz diagrams
    'sphinx_autodoc_typehints',
]

# -- Internationalization (i18n) configuration --------------------------------
locale_dirs = ['locale/']   # Path where .mo files will be stored
gettext_compact = False     # Create separate .pot files per document
gettext_uuid = True         # Add UUID to .pot files for tracking
gettext_allow_fuzzy_translations = True  # Include fuzzy translations

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
    "numpy",
    "zenoh",
    "redis",
]

templates_path = ['_templates']
exclude_patterns = ['_build', 'Thumbs.db', '.DS_Store', 'DEPRECATED']

# -- Options for HTML output -------------------------------------------------
html_theme = 'furo'
html_static_path = ['_static']
html_title = f"{project} {release}"

# Language configuration
language = 'en'  # Default language (overridden by -D language=de in build)

html_theme_options = {
    "light_css_variables": {
        "color-brand-primary": "#336790",
        "color-brand-content": "#336790",
    },
}

# Language selector for multilingual documentation
html_context = {
    'languages': [
        ('en', 'English', '../en/'),
        ('de', 'Deutsch', '../de/'),
    ],
    'current_language': language,
}

# Add language selector template to sidebar
html_sidebars = {
    '**': [
        'sidebar/language.html',
        'sidebar/scroll-start.html',
        'sidebar/brand.html',
        'sidebar/search.html',
        'sidebar/navigation.html',
        'sidebar/scroll-end.html',
    ]
}

# -- Path Setup (damit src importierbar ist) ---------------------------------
import os
import sys
sys.path.insert(0, os.path.abspath('../src/'))
