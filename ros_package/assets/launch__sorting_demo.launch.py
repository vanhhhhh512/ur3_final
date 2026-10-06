"""One-command camera-guided scene, local planner and sorting task."""
from llm_va.launching import assemble_launch


def generate_launch_description():
    return assemble_launch('workcell')
