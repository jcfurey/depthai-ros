"""Explicitly report an unsupported example instead of succeeding without nodes."""

def generate_launch_description():
    raise RuntimeError('USB-camera input to the DepthAI neural network is not implemented. Use example_det2d_overlay.launch.py with an OAK camera.')
