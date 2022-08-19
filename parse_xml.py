"""
This is an example to illustrate an idea.
"""
from pathlib import Path
import xmltodict
import os


here = Path(os.path.abspath(os.path.join(os.path.dirname(__file__))))


def load_messages() -> (list, list):
    """
    Example.
    """
    template = here / 'message_definitions/v1.0/common.xml'

    with open(template, 'r', encoding='utf-8') as file:
        my_xml = file.read()

    data_model = xmltodict.parse(my_xml)

    messages = data_model['mavlink']['messages']['message']
    enums = data_model['mavlink']['enums']['enum']
    commands = dict()
    for enum in enums:
        if enum['@name'] == 'MAV_CMD':
            commands = enum['entry']
    return messages, commands
