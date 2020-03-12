#!/usr/bin/env python
# -*- coding: utf-8 -*-

"""
Search through all of SCILPY scripts and their docstrings. The output of the
search will be the intersection of all provided keywords, found either in the
script name or in its docstring.
"""

from argparse import ArgumentParser, RawTextHelpFormatter
import ast
import inspect
import pathlib
import re

import numpy as np
import scilpy
from scilpy.io.utils import add_verbose_arg

RED = '\033[31m'
BOLD = '\033[1m'
ENDC = '\033[0m'


def _build_arg_parser():
    parser = ArgumentParser(description=__doc__,
                            formatter_class=RawTextHelpFormatter)
    parser.add_argument('keywords', nargs='+',
                        help="Search keywords")
    add_verbose_arg(parser)
    return parser


def main():
    parser = _build_arg_parser()
    args = parser.parse_args()

    scilpy_init_file = inspect.getfile(scilpy)  # scilpy/scilpy/__init__.py
    script_dir = pathlib.Path(scilpy_init_file).parent / "../scripts"
    matches = []

    kw_subs = [re.compile("(" + re.escape(kw) + ")", re.IGNORECASE)
               for kw in args.keywords]

    for script in sorted(script_dir.glob('*.py')):
        filename = script.name
        docstring = _get_docstring(str(script))

        if args.verbose:
            # Display full docstring
            display_info = docstring
        else:
            # Extract first sentence by finding the first dot
            display_info = _extract_first_sentence(docstring)
            display_info = display_info.replace("\n", " ")

        # Test intersection of all keywords, either in filename or docstring
        if not _is_matching_keywords(args.keywords, [filename, docstring]):
            continue

        matches.append(filename)

        # new_key calls regex group \1 to keep the same case
        new_key = "{}\\1{}".format(RED + BOLD, ENDC)
        for regex in kw_subs:
            filename = regex.sub(new_key, filename)
            display_info = regex.sub(new_key, display_info)

        display_info = display_info or "No docstring available!"

        if args.verbose:
            print("=====", filename, "====")
            print('"{}"'.format(display_info))
            print()
        else:
            print(filename, '"{}"'.format(display_info))

    if not matches:
        print("No results found!")


def _is_matching_keywords(keywords, texts):
    """Test multiple texts for matching keywords. Returns True only if all
    keywords are present in any of the texts.

    Parameters
    ----------
    keywords : Iterable of str
        Keywords to test for.
    texts : Iterable of str
        Strings that should contain the keywords.

    Returns
    -------
    True if all keywords were found in at least one of the texts.

    """
    matches = []
    for key in keywords:
        key_match = False
        for text in texts:
            if key.lower() in text.lower():
                key_match = True
                break
        matches.append(key_match)

    return np.all(matches)


def _extract_first_sentence(text):
    """Extract the first sentence of a string by finding the first dot. If
    there is no dot, return the full string.

    Parameters
    ----------
    text : str
        Text to parse.

    Returns
    -------
    first_sentence : str
        The first sentence, or the full text if no dot was found.

    """
    first_dot_idx = text.find('.') + 1 or None
    sentence = text[:first_dot_idx]
    return sentence


def _get_docstring(script):
    """Extract a python file's docstring from a filepath.

    Parameters
    ----------
    script : str
        Path to python file

    Returns
    -------
    docstring : str
        The file docstring, or an empty string if there was no docstring.
    """
    with open(script, 'r') as reader:
        file_contents = reader.read()
    module = ast.parse(file_contents)
    docstring = ast.get_docstring(module) or ""
    return docstring


if __name__ == '__main__':
    main()
