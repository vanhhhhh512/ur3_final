"""Compatibility launch for the simulated workcell, without starting task motion."""
from ur3_perception_llm_control import launching

def generate_launch_description():
    return launching.assemble_launch('workcell')
