import numpy as np

class params:
   
    checkerboard = (8,5)
    pattern_size = (0.031, 0.031) # in meters

    fx = 183.7350
    fy = 184.1241
    px = 160
    py = 160
    initial_intrinsic_matrix = np.array([[fx,0,px],
                                         [0,fy,py], 
                                          [0,0,1]])
    img_shape=(320,320)
