"""MoveIt composition entry."""
from llm import launching

def generate_launch_description():
    return launching.assemble_launch('moveit')
