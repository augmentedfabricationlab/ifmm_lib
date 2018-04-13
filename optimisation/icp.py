'''
. . . . . . . . . . . . . . . . . . . . . 
.                                       .
.    <<      ><      ><       >< <<     .
.    < ><   ><<     ><<<    ><    ><<   .
.    << >< > ><    ><  ><     ><        .  
.    <<  ><  ><   ><<<<<><      ><      .
.    <<      >< ><<     ><< ><    ><<   .
.    <<      ><><<       ><<  >< <<     .
.                                       .
.             DFAB 2016/17              .
. . . . . . . . . . . . . . . . . . . . . 

Created on 22.03.2017

@author: rustr
'''

import numpy as np
from scipy.spatial.distance import cdist

def best_fit_transform(A, B):
    '''
    Calculates the least-squares best-fit transform between corresponding 3D points A->B
    Input:
      A: Nx3 numpy array of corresponding 3D points
      B: Nx3 numpy array of corresponding 3D points
    Returns:
      T: 4x4 homogeneous transformation matrix
      R: 3x3 rotation matrix
      t: 3x1 column vector
    '''

    assert len(A) == len(B)

    # translate points to their centroids
    centroid_A = np.mean(A, axis=0)
    centroid_B = np.mean(B, axis=0)
    AA = A - centroid_A
    BB = B - centroid_B

    # rotation matrix
    H = np.dot(AA.T, BB)
    U, S, Vt = np.linalg.svd(H)
    R = np.dot(Vt.T, U.T)

    # special reflection case
    if np.linalg.det(R) < 0:
        Vt[2,:] *= -1
        R = np.dot(Vt.T, U.T)

    # translation
    t = centroid_B.T - np.dot(R,centroid_A.T)

    # homogeneous transformation
    T = np.identity(4)
    T[0:3, 0:3] = R
    T[0:3, 3] = t

    return T, R, t

def nearest_neighbor(src, dst):
    '''
    Find the nearest (Euclidean) neighbor in dst for each point in src
    Input:
        src: Nx3 array of points
        dst: Nx3 array of points
    Output:
        distances: Euclidean distances of the nearest neighbor
        indices: dst indices of the nearest neighbor
    '''

    all_dists = cdist(src, dst, 'euclidean')
    indices = all_dists.argmin(axis=1)
    distances = all_dists[np.arange(all_dists.shape[0]), indices]
    return distances, indices

def icp(A, B, init_guess=None, max_iterations=20, tolerance=0.001):
    '''
    The Iterative Closest Point method
    Input:
        A: Nx3 numpy array of source 3D points
        B: Nx3 numpy array of destination 3D point
        init_guess: 4x4 homogeneous transformation
        max_iterations: exit algorithm after max_iterations
        tolerance: convergence criteria
    Output:
        T: final homogeneous transformation
        distances: Euclidean distances (errors) of the nearest neighbor
        
    reference: https://github.com/ClayFlannigan/icp/blob/master/icp.py
    '''
    
    A = np.array(A)
    B = np.array(B)
    if len(init_guess):
        init_guess = np.array(init_guess)
    
    # make points homogeneous, copy them so as to maintain the originals
    src = np.ones((4,A.shape[0]))
    dst = np.ones((4,B.shape[0]))
    src[0:3,:] = np.copy(A.T)
    dst[0:3,:] = np.copy(B.T)

    # apply the initial pose estimation
    if init_guess is not None:
        src = np.dot(init_guess, src)

    prev_error = 0

    for i in range(max_iterations):
        # find the nearest neighbours between the current source and destination points
        distances, indices = nearest_neighbor(src[0:3,:].T, dst[0:3,:].T)

        # compute the transformation between the current source and nearest destination points
        T,_,_ = best_fit_transform(src[0:3,:].T, dst[0:3,indices].T)

        # update the current source
        src = np.dot(T, src)

        # check error
        mean_error = np.sum(distances) / distances.size
        if abs(prev_error-mean_error) < tolerance:
            break
        prev_error = mean_error

    # calculate final transformation
    T,_,_ = best_fit_transform(A, src[0:3,:].T)
    
    # format to non-numpy array: json doesn't like numpy
    return [[float(f) for f in a] for a in T], [float(f) for f in distances]
    #return T, distances


if __name__ == "__main__":
    
    dataA =  [[1184.60107421875, -766.2840576171875, -891.25445556640625], [1304.4188232421875, -888.66796875, -888.02142333984375], [1693.956298828125, 272.09207153320312, -891.21160888671875]]
    dataB =  [[1680.609272054043, 2572.8252477148708, -358.99999955328167], [1831.8030743871989, 2650.5454776170186, -358.99999955328167], [2674.8874877354606, 1861.7976974612438, -358.99999999990621]]

    A = np.array(dataA)
    B = np.array(dataB)
    
    print A.shape
    print B.shape

    init_guess_list = [[-0.28334373235702515, 0.95901840925216675, -2.1805554561158454e-15, 2899.379638671875], [-0.95901840925216675, -0.28334373235702515, 6.4424907745644377e-16, 3569.7255859375], [0.0, 2.2737367036103438e-15, 1.0, 537.26116943359375], [0.0, 0.0, 0.0, 1.0]]

    
    init_guess = np.array(init_guess_list)
    
    #Run the icp
    T, distances = icp(A, B, init_guess, max_iterations=20)
    
    """
    # for pasting into Rhino
    print "T = rg.Transform.Identity"
    for i in range(4):
        for j in range(4):
            print "T.M%i%i = %f" % (i,j,T[i,j])
        print
    """
    
    print T
    print distances
    
