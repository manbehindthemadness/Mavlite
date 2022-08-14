"""
This is an example to illustrate an idea.
"""
from pathlib import Path
import xmltodict
import pprint
import os


here = Path(os.path.abspath(os.path.join(os.path.dirname(__file__))))


template = here / 'mavlink-master/message_definitions/v1.0/ardupilotmega.xml'


print('using file', template)
with open(template, 'r', encoding='utf-8') as file:
    my_xml = file.read()

data_model = xmltodict.parse(my_xml)

pprint.pprint(data_model, indent=2)
