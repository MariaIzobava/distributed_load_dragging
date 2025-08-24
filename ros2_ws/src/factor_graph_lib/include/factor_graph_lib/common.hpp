#ifndef COMMON_HPP
#define COMMON_HPP

#include <string>

#include <gtsam/slam/PriorFactor.h>
#include <gtsam/slam/BetweenFactor.h>
#include <gtsam/nonlinear/NonlinearFactorGraph.h>
#include <gtsam/nonlinear/LevenbergMarquardtOptimizer.h>
#include <gtsam/nonlinear/Values.h>
#include <gtsam/inference/Symbol.h>
#include <gtsam/base/Matrix.h>
#include <gtsam/base/Vector.h>
#include <gtsam/base/numericalDerivative.h>

using namespace std;
using namespace gtsam;


class CableVectorHelper {
public:
    double ex_dxr;
    double ex_dyr;
    double ex_dzr;

    double ey_dxr;
    double ey_dyr;
    double ey_dzr;

    double ez_dxr;
    double ez_dyr;
    double ez_dzr;

    double ex_dxl;
    double ex_dyl;

    double ey_dxl;
    double ey_dyl;

    double ez_dxl;
    double ez_dyl;

    double ex_dtheta;
    double ey_dtheta;
    double ap_x;
    double ap_y;
    Vector2 e_norm = Vector2::Zero();
    Vector3 e3_norm = Vector3::Zero();

    CableVectorHelper(const Vector4& xr_k, const Vector6& xl_k, int k = 1) {

        Vector2 ap = getAttachementPoint(xl_k);

        if (k == 2) {
            ap = getAttachementPoint2(xl_k);
        }

        ap_x = ap(0);
        ap_y = ap(1);

        double xd = xr_k(0) - ap(0);
        double yd = xr_k(1) - ap(1);
        Vector2 e(2);
        e << xd, yd;
        double norm = e.norm();
        
        e_norm << e / norm;

        ex_dxr = 1.0 / norm - xd * xd / (norm * norm * norm);
        ex_dyr = -yd * xd / (norm * norm * norm);
        
        ey_dxr = -yd * xd / (norm * norm * norm);
        ey_dyr = 1.0 / norm - yd * yd / (norm * norm * norm);
    
        ex_dxl = -1.0 / norm + xd * xd / (norm * norm * norm);
        ex_dyl = xd * yd / (norm * norm * norm);
        
        ey_dxl = xd * yd / (norm * norm * norm);
        ey_dyl = -1.0 / norm + yd * yd / (norm * norm * norm);
        
        ex_dtheta = -0.2 * sin(xl_k(2)) / norm - 0.2 * xd * (yd * cos(xl_k(2)) - xd * sin(xl_k(2))) / (norm * norm * norm);
        ey_dtheta = 0.2 * cos(xl_k(2)) / norm - 0.2 * yd * (yd * cos(xl_k(2)) - xd * sin(xl_k(2))) / (norm * norm * norm);

        if (k == 2) {
            ex_dtheta = 0.2 * cos(xl_k(2)) / norm - 0.2 * xd * (yd * sin(xl_k(2)) + xd * cos(xl_k(2))) / (norm * norm * norm);
            ey_dtheta = 0.2 * sin(xl_k(2)) / norm - 0.2 * yd * (yd * sin(xl_k(2)) + xd * cos(xl_k(2))) / (norm * norm * norm);
        }
    }

    CableVectorHelper(const Vector4& xr_k, const Vector4& xl_k, int k = 1) {

        double xd = xr_k(0) - xl_k(0);
        double yd = xr_k(1) - xl_k(1);
        Vector2 e(2);
        e << xd, yd;
        double norm = e.norm();
        
        e_norm << e / norm;

        ex_dxr = 1.0 / norm - xd * xd / (norm * norm * norm);
        ex_dyr = -yd * xd / (norm * norm * norm);
        
        ey_dxr = -yd * xd / (norm * norm * norm);
        ey_dyr = 1.0 / norm - yd * yd / (norm * norm * norm);
    
        ex_dxl = -1.0 / norm + xd * xd / (norm * norm * norm);
        ex_dyl = xd * yd / (norm * norm * norm);
        
        ey_dxl = xd * yd / (norm * norm * norm);
        ey_dyl = -1.0 / norm + yd * yd / (norm * norm * norm);
    }

    CableVectorHelper(const Vector6& xr_k, const Vector4& xl_k, string ap_loc, int k = 1) {

        double xl_z = (ap_loc == "bottom") ? 0.0 : 0.2;
        double xd = xr_k(0) - xl_k(0);
        double yd = xr_k(1) - xl_k(1);
        double zd = xr_k(2) - xl_z;
        Vector3 e(3);
        e << xd, yd, zd;
        double norm = e.norm();
        
        e3_norm << e / norm;
        e_norm << e3_norm(0), e3_norm(1);

        ex_dxr = 1.0 / norm - xd * xd / (norm * norm * norm);
        ex_dyr = -xd * yd / (norm * norm * norm);
        ex_dzr = -xd * zd / (norm * norm * norm);
        
        ey_dxr = -yd * xd / (norm * norm * norm);
        ey_dyr = 1.0 / norm - yd * yd / (norm * norm * norm);
        ey_dzr = -yd * zd / (norm * norm * norm);

        ez_dxr = -zd * xd / (norm * norm * norm);
        ez_dyr = -zd * yd / (norm * norm * norm);
        ez_dzr = 1.0 / norm - zd * zd / (norm * norm * norm);
        
    
        ex_dxl = -1.0 / norm + xd * xd / (norm * norm * norm);
        ex_dyl = xd * yd / (norm * norm * norm);
        
        ey_dxl = yd * xd / (norm * norm * norm);
        ey_dyl = -1.0 / norm + yd * yd / (norm * norm * norm);

        ez_dxl = zd * xd / (norm * norm * norm);
        ez_dyl = zd * yd / (norm * norm * norm);
    }

    static Vector2 getAttachementPoint(const Vector6& xl_k) {
        Vector2 p(2);
        p << xl_k(0) - 0.2 * cos(xl_k(2)), xl_k(1) - 0.2 * sin(xl_k(2));
        return p;
    }

    static Vector2 getAttachementPoint2(const Vector6& xl_k) {
        Vector2 p(2);
        p << xl_k(0) - 0.2 * sin(xl_k(2)), xl_k(1) + 0.2 * cos(xl_k(2));
        return p;
    }
};


#endif // COMMON_HPP