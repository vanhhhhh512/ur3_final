"""Gazebo workcell composition entry."""
from llm_va import launching

def generate_launch_description():
    return launching.assemble_launch('workcell')
