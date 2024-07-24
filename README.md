# Davidbots
A virtual camera robot
import numpy as np
import pybullet as p
import pybullet_data
import imageio_ffmpeg
from base64 import b64encode
from IPython.display import HTML

# camera parameters
cam_target_pos = [.95, -0.2, 0.2]
cam_distance = 2.05
cam_yaw, cam_pitch, cam_roll = -50, -40, 0
cam_width, cam_height = 480, 368
cam_up, cam_up_axis_idx, cam_near_plane, cam_far_plane, cam_fov = [0, 0, 1], 2, 0.01, 100, 60

# physics parameters.
physicsClient = p.connect(p.DIRECT)
p.setAdditionalSearchPath(pybullet_data.getDataPath())
p.setGravity(0, 0, -10)
plane_id = p.loadURDF("plane.urdf")

# Initialize video.
vid = imageio_ffmpeg.write_frames('vid.mp4', (cam_width, cam_height), fps=30)
vid.send(None) # The first frame of the video must be a null frame.

# Simulate your world...
for t in range(0,100):

  # Create one image and add it to the video.
  cam_view_matrix = p.computeViewMatrixFromYawPitchRoll(cam_target_pos, cam_distance, cam_yaw, cam_pitch, cam_roll, cam_up_axis_idx)
  cam_projection_matrix = p.computeProjectionMatrixFOV(cam_fov, cam_width*1./cam_height, cam_near_plane, cam_far_plane)
  image = p.getCameraImage(cam_width, cam_height,cam_view_matrix, cam_projection_matrix)[2][:, :, :3]
  vid.send(np.ascontiguousarray(image))
  cam_yaw = cam_yaw + 1

  # Do physics stuff.
  p.stepSimulation()

# Shut down gracefully.
vid.close()
p.disconnect()
