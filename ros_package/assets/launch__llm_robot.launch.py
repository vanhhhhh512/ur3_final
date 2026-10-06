"""Compatibility launch for the simulated workcell, without starting task motion."""
from llm import launching

def generate_launch_description():
    return launching.assemble_launch('workcell')
