# homography-decomp
## C++ Implementation of Homography Decomposition

This is implementation of homography decomposition to rotation(s), translation(s) and plane normals. The implementation is based on the paper by Ezio Malis, et.al. "Deeper understanding of the homography decomposition for vision-based control". 2007


Usage: Create HomographyDecomp object and call the method 

       void decomposeHomography(const cv::Matx33d& H, const cv::Matx33d& K,
                                     std::vector<CameraMotion>& camMotions);
   
Inputs:

    H    - Homography matrix between two images (3x3 )
    K    - Intrinsic camera matrix. (3x3)

Outputs:

    camMotions - Array of {R, t, n}. R-rotation matrix, t-translation vector, n-plane normal vector 
      
      
This implementation is integrated with OpenCV library. 
See the documenation in [OpenCV - decomposeHomographyMat()](https://docs.opencv.org/3.4/d9/d0c/group__calib3d.html#ga7f60bdff78833d1e3fd6d9d0fd538d92)


