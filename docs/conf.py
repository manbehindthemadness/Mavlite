# -*- coding: UTF-8 -*-
"""
This file is part of the MagicMirror package.
Copyright (c) 2022 Kevin Eales.

This program is proprietary, redistribution is prohibited.
Please see the license file for more details.
------------------------------------------------------------------------------------------------------------------------
MagicMirror documentation build configuration file
"""

import os
import sys
import sphinx_rtd_theme


sys.path.insert(0, os.path.abspath('.'))
sys.path.insert(0, os.path.abspath('../'))
sys.path.insert(0, os.path.abspath('../src/'))

project = 'Mavlite'
copyright = '2022 Kevin Eales, Cameron Clarke'  # noqa
author = 'Kevin Eales, Cameron Clarke'

version = '0.1'
release = '0.1'

extensions = [
    "sphinx.ext.autodoc",
    "sphinx.ext.viewcode",
]

templates_path = ['_templates']
source_suffix = '.rst'
master_doc = 'contents'
language = 'en'

exclude_patterns = [
    '_build',
    'Thumbs.db',
    '.DS_Store',
    '*_cffi_backend*',
    # "source/modules.rst",
    # "source/src.rst",
    # "source/src.main.rst"
]
pygments_style = 'sphinx'
todo_include_todos = True

html_theme = 'sphinx_rtd_theme'
html_static_path = ['_static']
html_theme_path = [sphinx_rtd_theme.get_html_theme_path()]
htmlhelp_basename = 'Mavlite documentation'
html_js_files = [
    "script.js"
]
html_css_files = [
    "styles.css",
    "dark.css",
    "light.css"
]

html_theme_options = {
    'display_version': False,
    'collapse_navigation': False,
    'sticky_navigation': True,
    'navigation_depth': 0,
    'includehidden': True,
    'titles_only': False
}


html_logo = "_static/mavlite.png"
html_favicon = "_static/favicon.ico"

latex_elements = {
}

latex_documents = [
    (master_doc, 'Mavlite.tex', 'Mavlite Documentation',
     'Kevin (eales)', 'Cameron (clarke)' 'manual'),
]

texinfo_documents = [
    (master_doc, 'Mavlite', 'Mavlite Documentation',
     author, 'Mavlite', 'MAVLink extensions for micropython/circuitpython',
     'Miscellaneous'),
]


def run_apidoc(_):
    """

    :param _:
    :return:
    """
    from sphinx.ext.apidoc import main
    import os
    import sys
    import shutil
    sys.path.append(os.path.join(os.path.dirname(__file__), '..'))
    cur_dir = os.path.abspath(os.path.dirname(__file__))
    module = '../src/'
    output_path = os.path.join(cur_dir, 'source')
    main(['-o', output_path, module, '--force', '--separate'])

    static_dir = os.path.join(cur_dir, '_static')
    static_dir_files = os.listdir(static_dir)
    dest_dir = os.path.join(output_path, '_static/')
    if not os.path.exists(dest_dir):
        os.makedirs(dest_dir)
    for filename in static_dir_files:
        full_filename = os.path.join(static_dir, filename)
        dest_filename = os.path.join(dest_dir, filename)
        shutil.copy(full_filename, dest_filename)


def setup(app):
    """

    :param app:
    :return:
    """
    app.connect('builder-inited', run_apidoc)


autoclass_content = 'both'
autodoc_mock_imports = ["src.tools"]
