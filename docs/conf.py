extensions = [
    "breathe",
    'sphinx.ext.autosectionlabel',
    'sphinx_tabs.tabs',
    'sphinx_panels'
]

html_theme = "sphinx_rtd_theme"

# General information about the project.
project = 'raisim'
copyright = '2022, RaiSim Tech Inc.'
author = 'Yeonjoo Chung and Jemin Hwangbo'
version = '1.1.7'
release = '1.1.7'

# Output file base name for HTML help builder.
htmlhelp_basename = 'raisim_doc'
html_show_sourcelink = False

# Breathe Configuration
breathe_default_project = "raisim"
autosectionlabel_prefix_document = True
autosectionlabel_maxdepth = 4
html_theme_path = ["themes"]
html_show_sphinx = False
html_logo = "image/logo.png"
