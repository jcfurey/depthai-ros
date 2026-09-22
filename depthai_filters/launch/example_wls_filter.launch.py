"""Explicitly report an unsupported example instead of succeeding without nodes."""

def generate_launch_description():
    raise RuntimeError('The WLS camera wrapper is unavailable until the driver provides a calibrated disparity input. Load WLSFilter explicitly with disparity/image_raw, disparity/camera_info and left/image_raw.')
