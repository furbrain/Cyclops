%YAML:1.0

#--------------------------------------------------------------------------------------------
# Camera Parameters. Adjust them!
#--------------------------------------------------------------------------------------------
File.version: "1.0"

Camera.type: "PinHole"

# Right Camera calibration and distortion parameters (OpenCV)
Camera1.fx: {rgb_ci.p[0]}
Camera1.fy: {rgb_ci.p[5]}
Camera1.cx: {rgb_ci.p[2]}
Camera1.cy: {rgb_ci.p[6]}
# distortion parameters
Camera1.k1: {rgb_ci.d[0]}
Camera1.k2: {rgb_ci.d[1]}
Camera1.p1: {rgb_ci.d[2]}
Camera1.p2: {rgb_ci.d[3]}

# Camera resolution
Camera.width: {rgb_ci.width}
Camera.height: {rgb_ci.height}

# Camera frames per second 
Camera.fps: 10 # do we need to make this adjustment??

# Color order of the images (0: BGR, 1: RGB. It is ignored if images are grayscale)
Camera.RGB: 1 # this is default...

Stereo.ThDepth: 10.0
Stereo.b: 0.0745

# Depth map values factor
RGBD.DepthMapFactor: 1.0

# Transformation from body-frame (imu) to left camera
IMU.T_b_c1: !!opencv-matrix
   rows: 4
   cols: 4
   dt: f
   data: {list(transform.as_matrix().flatten())}


# Do not insert KFs when recently lost
IMU.InsertKFsWhenLost: 0

# IMU noise (Use those from VINS-mono)
# TODO
IMU.NoiseGyro: 1e-2 # 3 # 2.44e-4 #1e-3 # rad/s^0.5
IMU.NoiseAcc: 1e-1 #2 # 1.47e-3 #1e-2 # m/s^1.5
IMU.GyroWalk: 1e-6 # rad/s^1.5
IMU.AccWalk: 1e-4 # m/s^2.5
IMU.Frequency: 500.0

#--------------------------------------------------------------------------------------------
# ORB Parameters
#--------------------------------------------------------------------------------------------
# ORB Extractor: Number of features per image
ORBextractor.nFeatures: 1250

# ORB Extractor: Scale factor between levels in the scale pyramid 	
ORBextractor.scaleFactor: 1.2

# ORB Extractor: Number of levels in the scale pyramid	
ORBextractor.nLevels: 8

# ORB Extractor: Fast threshold
# Image is divided in a grid. At each cell FAST are extracted imposing a minimum response.
# Firstly we impose iniThFAST. If no corners are detected we impose a lower value minThFAST
# You can lower these values if your images have low contrast			
ORBextractor.iniThFAST: 20
ORBextractor.minThFAST: 7

#--------------------------------------------------------------------------------------------
# Viewer Parameters 
#--------------------------------------------------------------------------------------------
# These are not used as pangolin has been excised...

#Viewer.KeyFrameSize: 0.05
#Viewer.KeyFrameLineWidth: 1.0
#Viewer.GraphLineWidth: 0.9
#Viewer.PointSize: 2.0
#Viewer.CameraSize: 0.08
#Viewer.CameraLineWidth: 3.0
#Viewer.ViewpointX: 0.0
#Viewer.ViewpointY: -0.7
#Viewer.ViewpointZ: -3.5
#Viewer.ViewpointF: 500.0
