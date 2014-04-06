/*
 * HomographyDecomp.h
 * This file is part of HomographyDecomp.
 *
 * Copyright 2013 Samson Yilma. All Rights Reserved.
 *
 * HomographyDecomp is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, version 2 of the License.
 *
 * HomographyDecomp is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with . If not, see <http://www.gnu.org/licenses>.
 */

#ifndef HomographyDecomp_h
#define HomographyDecomp_h

#include "opencv2/core/core.hpp"
#include <vector>

/*! struct to hold solutions of homography decomposition
 */
typedef struct _CameraMotion {
    cv::Matx33d R; //!< rotation matrix
    cv::Vec3d n; //!< normal of the plane the camera is looking at
    cv::Vec3d t; //!< translation vector
} CameraMotion;


/*! Class to decompose image-to-image homography to rotation and translation. Largely based on the papers
   Ezio Malis and Manuel Vargas, "Deeper understanding of the homography decomposition for vision-based control"
   and
   Z. Zhang, and A.R. Hanson, “3D Reconstruction based on homography mapping”
 */
class HomographyDecomp {
    
public:
    HomographyDecomp() {}
    virtual ~HomographyDecomp() {}
    virtual void decomposeHomography(const cv::Matx33d& H, const cv::Matx33d& K,
                                     std::vector<CameraMotion>& camMotions);
    static bool isRotationValid(const cv::Matx33d& R,  const double epsilon=0.01);
    
protected:
    bool passesSameSideOfPlaneConstraint(CameraMotion& motion);
    virtual void decompose(const cv::Matx33d& K, std::vector<CameraMotion>& camMotions) = 0;
    
    const cv::Matx33d& getHnorm() const {
        return _Hnorm;
    }
    
private:
    cv::Matx33d normalize(const cv::Matx33d& H, const cv::Matx33d& K);
	void removeScale();
    
    cv::Matx33d _Hnorm;
};

class HomographyDecompZhang : public HomographyDecomp {
    
public:
	   
	HomographyDecompZhang():HomographyDecomp() {}
	virtual ~HomographyDecompZhang() {}

private:
    virtual void decompose(const cv::Matx33d& K, std::vector<CameraMotion>& camMotions);
	bool findMotionFrom_tstar_n(const cv::Vec3d& tstar, const cv::Vec3d& n, CameraMotion& motion);
};

class HomographyDecompInria : public HomographyDecomp {
    
public:
	
	HomographyDecompInria():HomographyDecomp() {}
	virtual ~HomographyDecompInria() {}

private:
    virtual void decompose(const cv::Matx33d& K, std::vector<CameraMotion>& camMotions);
    
	double oppositeOfMinor(const cv::Matx33d& M, const int row, const int col);
	void findRmatFrom_tstar_n(const cv::Vec3d& tstar, const cv::Vec3d& n, const double v, cv::Matx33d& R);
	
};


#endif
