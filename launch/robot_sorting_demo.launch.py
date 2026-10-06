"""One-command camera-guided scene, local planner and sorting task."""
from ur3_perception_llm_control.launching import assemble_launch


def generate_launch_description():
    return assemble_launch('workcell')
