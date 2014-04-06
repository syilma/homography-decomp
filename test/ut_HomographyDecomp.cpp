/*
 * ut_HomographyDecomp.cpp
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

#include "HomographyDecomp.h"
#include "opencv2/ts/ts.hpp"

using namespace cv;
using namespace std;


class TestHomographyDecomp : public cvtest::BaseTest {

public:
    TestHomographyDecomp()
    {
        buildTestDataSet();
    }

protected:
    void run(int)
    {
        //test Inria decomposition
        auto_ptr<HomographyDecomp> hdecomp(new HomographyDecompInria);
        runHomographyDecomp(hdecomp.get());
        
        //test Zhang decomposition
        hdecomp = auto_ptr<HomographyDecomp>(new HomographyDecompZhang);
        runHomographyDecomp(hdecomp.get());
    }

private:
    
    void runHomographyDecomp(HomographyDecomp* hdecomp)
    {
        vector<CameraMotion> motions;
        hdecomp->decomposeHomography(_H, _K, motions);
        
        //there should be at least 1 solution
        ASSERT_GT(motions.size(), 0);
        
        //the solution vector should contain the correct (R,t,n) set
        ASSERT_TRUE(containsValidMotion(motions));
    }
        
    void buildTestDataSet()
    {
        //intrinsic camera and homography
        _K = Matx33d(600, 0.0,  320,
                     0,   600, 240,
                     0,    0,   1);
        _H =  Matx33d(81.1551391671253,          88.2921244076486,        -3654.60266252177,
                      97.9788368422579,          106.556736454251,        -10832.7494480565,
                      0.0194699936416257,        0.0235813413282153,       7.77972095366621);
        
        //expected solution for the given homography and intrinsic matrices
        _R = Matx33d(0,  -1,   0,
                     1,   0,   0,
                     0,  0,   1);
        _t = Vec3d(13.3021865644749, 14.7902074341139, 2.07402908846194);
        _n = Vec3d(0.56325131841032, 0.682189313334158, 0.466224938288061);
        
    }
    
    bool containsValidMotion(std::vector<CameraMotion>& motions)
    {
        double max_error = 1.0e-3;
        
        for (std::vector<CameraMotion>::iterator iter=motions.begin(); iter != motions.end(); ++iter) {
            
            double rdist = norm(iter->R, _R, NORM_INF);
            double tdist = norm(iter->t, _t, NORM_INF);
            double ndist = norm(iter->n, _n, NORM_INF);
            
            if (   rdist < max_error
                && tdist < max_error
                && ndist < max_error )
                return true;
        }
        
        return false;
    }
    
    Matx33d _R, _K, _H;
    Vec3d _t, _n;
    
};


TEST(MyTestSuite, HomographyDecompTest) {
    TestHomographyDecomp testObj;
    testObj.safe_run();
}


