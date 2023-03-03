import numpy as np
 
def euler_rotation_matrix(alpha,beta,gamma):
    """
    Generate a full three-dimensional rotation matrix from euler angles
 
    Input
    :param alpha: The roll angle (radians) - Rotation around the x-axis
    :param beta: The pitch angle (radians) - Rotation around the y-axis
    :param alpha: The yaw angle (radians) - Rotation around the z-axis
 
    Output
    :return: A 3x3 element matix containing the rotation matrix. 
             This rotation matrix converts a point in the local reference 
             frame to a point in the global reference frame.
 
    """
    # First row of the rotation matrix
    r00 = np.cos(gamma) * np.cos(beta)
    r01 = np.cos(gamma) * np.sin(beta) * np.sin(alpha) - np.sin(gamma) * np.cos(alpha)
    r02 = np.cos(gamma) * np.sin(beta) * np.cos(alpha) + np.sin(gamma) * np.sin(alpha)
     
    # Second row of the rotation matrix
    r10 = np.sin(gamma) * np.cos(beta)
    r11 = np.sin(gamma) * np.sin(beta) * np.sin(alpha) + np.cos(gamma) * np.cos(alpha)
    r12 = np.sin(gamma) * np.sin(beta) * np.cos(alpha) - np.cos(gamma) * np.sin(alpha)
     
    # Third row of the rotation matrix
    r20 = -np.sin(beta)
    r21 = np.cos(beta) * np.sin(alpha)
    r22 = np.cos(beta) * np.cos(alpha)
     
    # 3x3 rotation matrix
    rot_matrix = np.array([[r00, r01, r02],
                           [r10, r11, r12],
                           [r20, r21, r22]])
    print(rot_matrix)                        
    return rot_matrix
 
def rotate(p1,alpha,beta,gamma):
    """
    Rotates a point p1 in 3D space in the local reference frame to 
    a point p2 in the global reference frame.
 
    Input
    :param p1: A 3 element array containing the position of a point in the 
              local reference frame (xL,yL,zL) 
    :param alpha: The roll angle (radians) - Rotation around the x-axis
    :param beta: The pitch angle (radians) - Rotation around the y-axis
    :param alpha: The yaw angle (radians) - Rotation around the z-axis
 
    Output
    :return: p2: A 3 element array containing the position of a point in the 
             global reference frame (xG,yG,zG)
 
    """
    p2 = euler_rotation_matrix(alpha, beta, gamma) @ p1
    return p2
 
def inverse_rotation(p2,alpha,beta,gamma):  
    """
    Inverse rotation from a point p2 in global 3D reference frame 
    to a point p1 in the local (robot) reference frame.
 
    Input
    :param p2: A 3 element array containing the position of a point in the 
               global reference frame (xG,yG,zG)
    :param alpha: The roll angle (radians) - Rotation around the x-axis
    :param beta: The pitch angle (radians) - Rotation around the y-axis
    :param alpha: The yaw angle (radians) - Rotation around the z-axis
 
    Output
    :return: A 3 element array containing the position of a point in the 
             local reference frame (xL,yL,zL) 
 
    """
    # First row of the inverse rotation matrix
    r00 = np.cos(gamma) * np.cos(beta)
    r01 = np.sin(gamma) * np.cos(beta)
    r02 = -np.sin(beta)
     
    # Second row of the inverse rotation matrix 
    r10 = np.cos(gamma) * np.sin(beta) * np.sin(alpha) - np.sin(gamma) * np.cos(alpha)
    r11 = np.sin(gamma) * np.sin(beta) * np.sin(alpha) + np.cos(gamma) * np.cos(alpha)  
    r12 = np.cos(beta) * np.sin(alpha)  
     
    # Third row of the inverse rotation matrix  
    r20 = np.cos(gamma) * np.sin(beta) * np.cos(alpha) + np.sin(gamma) * np.sin(alpha)
    r21 = np.sin(gamma) * np.sin(beta) * np.cos(alpha) - np.cos(gamma) * np.sin(alpha)
    r22 = np.cos(beta) * np.cos(alpha)
     
    # 3x3 inverse rotation matrix
    inv_rot_matrix = np.array([[r00, r01, r02],
                               [r10, r11, r12],
                               [r20, r21, r22]]) 
    print(inv_rot_matrix)                    
    return inv_rot_matrix @ p2
 
def main():
 
    # Point that we want to rotate from local frame to global frame
    p1 = np.array([0.0,0.0,0.0])
     
    # Rotation angles
    alpha = (-20/180.0)*np.pi
    beta =  (-35/180.0)*np.pi
    gamma = (0/180.0)*np.pi
    #alpha = -1.9557
    #beta = 0.1197
    #gamma = -1.2955
    print(f'local coordinates p1: {p1}')
    print(f'Rotated by Roll {alpha}, Pitch {beta}, Yaw: {gamma}')
    p2 = rotate(p1,alpha,beta,gamma)
 
    print(f'global coordinates p2:{p2}')
    p1_ = inverse_rotation(p2,alpha,beta,gamma)
 
    print(f'inverse rotation back into local frame p1:{p1_}')
 
if __name__ == '__main__':
    main()
