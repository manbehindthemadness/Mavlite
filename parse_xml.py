"""
This is an example to illustrate an idea.
"""
from pathlib import Path
import xmltodict
import pprint
import os


here = Path(os.path.abspath(os.path.join(os.path.dirname(__file__))))


def load_model():
    """
    Example.
    """
    template = here / 'mavlink-master/message_definitions/v1.0/ardupilotmega.xml'

    print('using file', template)
    with open(template, 'r', encoding='utf-8') as file:
        my_xml = file.read()

    data_model = xmltodict.parse(my_xml)

    dialect = data_model['mavlink']['enums']['enum']
    child_models = dict()

    for enum in dialect:
        name = enum['@name']
        entry = enum['entry']

        if not isinstance(entry, dict):
            child_models[name] = entry
            for child_entry in child_models[name]:
                if 'description' in child_entry.keys():
                    del child_entry['description']

    pprint.pprint(child_models, indent=2)

    return child_models
