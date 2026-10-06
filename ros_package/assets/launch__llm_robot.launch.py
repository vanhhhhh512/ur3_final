"""Compatibility launch for the simulated workcell, without starting task motion."""
from llm_va import launching

def generate_launch_description():
    return launching.assemble_launch('workcell')
