"""One-command camera-guided scene, local planner and sorting task."""
from llm.launching import assemble_launch


def generate_launch_description():
    return assemble_launch('workcell')
