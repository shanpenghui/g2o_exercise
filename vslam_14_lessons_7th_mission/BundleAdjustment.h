//
// Created by sph on 2021/1/5.
//

#ifndef INC_7_2_BUNDLEADJUSTMENT_H
#define INC_7_2_BUNDLEADJUSTMENT_H

#include "data.h"
#include <vector>
#include <fstream>
#include <iostream>
#include <opencv2/core/core.hpp>
#include <Eigen/Core>
#include <Eigen/Geometry>
#include <Eigen/Dense>
#include <g2o/core/base_vertex.h>
#include <g2o/core/base_unary_edge.h>
#include <g2o/core/block_solver.h>
#include <g2o/core/optimization_algorithm_levenberg.h>
#include <g2o/solvers/csparse/linear_solver_csparse.h>
#include <g2o/types/sba/types_six_dof_expmap.h>
#include "g2o/stuff/command_args.h"
#include "g2o/core/batch_stats.h"
#include "g2o/core/sparse_optimizer.h"
#include "g2o/core/block_solver.h"
#include "g2o/core/solver.h"
#include "g2o/core/optimization_algorithm_levenberg.h"
#include "g2o/core/base_vertex.h"
#include "g2o/core/base_binary_edge.h"
#include "g2o/solvers/dense/linear_solver_dense.h"
#include "g2o/solvers/structure_only/structure_only_solver.h"
#include "g2o/solvers/pcg/linear_solver_pcg.h"

#include "g2o/EXTERNAL/ceres/autodiff.h"

#if defined G2O_HAVE_CHOLMOD
#include "g2o/solvers/cholmod/linear_solver_cholmod.h"
#else
#include "g2o/solvers/eigen/linear_solver_eigen.h"
#endif

using namespace g2o;
using namespace std;
using namespace Eigen;

/**
 * \brief camera vertex which stores the parameters for a pinhole camera
 *
 * The parameters of the camera are
 * - rx,ry,rz representing the rotation axis, whereas the angle is given by ||(rx,ry,rz)||
 * - tx,ty,tz the translation of the camera
 * - f the focal length of the camera
 * - k1, k2 two radial distortion parameters
 */
class VertexCameraBAL : public BaseVertex<9, Eigen::VectorXd>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexCameraBAL()
    {
    }

    virtual bool read(std::istream& /*is*/)
    {
        cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
        return false;
    }

    virtual bool write(std::ostream& /*os*/) const
    {
        cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
        return false;
    }

    virtual void setToOriginImpl()
    {
        cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
    }

    virtual void oplusImpl(const double* update)
    {
        Eigen::VectorXd::ConstMapType v(update, VertexCameraBAL::Dimension);
        _estimate += v;
    }
};

/**
 * \brief 3D world feature
 *
 * A 3D point feature in the world
 */
class VertexPointBAL : public BaseVertex<3, Eigen::Vector3d>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    VertexPointBAL()
    {
    }

    virtual bool read(std::istream& /*is*/)
    {
        cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
        return false;
    }

    virtual bool write(std::ostream& /*os*/) const
    {
        cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
        return false;
    }

    virtual void setToOriginImpl()
    {
        cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
    }

    virtual void oplusImpl(const double* update)
    {
        Eigen::Vector3d::ConstMapType v(update);
        _estimate += v;
    }
};

/**
 * \brief edge representing the observation of a world feature by a camera
 *
 * see: http://grail.cs.washington.edu/projects/bal/
 * We use a pinhole camera model; the parameters we estimate for each camera
 * area rotation R, a translation t, a focal length f and two radial distortion
 * parameters k1 and k2. The formula for projecting a 3D point X into a camera
 * R,t,f,k1,k2 is:
 * P  =  R * X + t       (conversion from world to camera coordinates)
 * p  = -P / P.z         (perspective division)
 * p' =  f * r(p) * p    (conversion to pixel coordinates) where P.z is the third (z) coordinate of P.
 *
 * In the last equation, r(p) is a function that computes a scaling factor to undo the radial
 * distortion:
 * r(p) = 1.0 + k1 * ||p||^2 + k2 * ||p||^4.
 *
 * This gives a projection in pixels, where the origin of the image is the
 * center of the image, the positive x-axis points right, and the positive
 * y-axis points up (in addition, in the camera coordinate system, the positive
 * z-axis points backwards, so the camera is looking down the negative z-axis,
 * as in OpenGL).
 */
class EdgeObservationBAL : public BaseBinaryEdge<2, Vector2d, VertexCameraBAL, VertexPointBAL>
{
public:
    EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
    EdgeObservationBAL()
    {
    }
    virtual bool read(std::istream& /*is*/)
    {
        cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
        return false;
    }
    virtual bool write(std::ostream& /*os*/) const
    {
        cerr << __PRETTY_FUNCTION__ << " not implemented yet" << endl;
        return false;
    }

    template<typename T>
    inline void cross(const T x[3], const T y[3], T result[3]) const
    {
        result[0] = x[1] * y[2] - x[2] * y[1];
        result[1] = x[2] * y[0] - x[0] * y[2];
        result[2] = x[0] * y[1] - x[1] * y[0];
    }

    template<typename T>
    inline T dot(const T x[3], const T y[3]) const { return (x[0] * y[0] + x[1] * y[1] + x[2] * y[2]);}

    template<typename T>
    inline T squaredNorm(const T x[3]) const { return dot<T>(x, x);}

    /**
     * templatized function to compute the error as described in the comment above
     */
    template<typename T>
    bool operator()(const T* camera, const T* point, T* error) const
    {
        // Rodrigues' formula for the rotation
        T p[3];
        T theta = sqrt(squaredNorm(camera));
        if (theta > T(0)) {
            T v[3];
            v[0] = camera[0] / theta;
            v[1] = camera[1] / theta;
            v[2] = camera[2] / theta;
            T cth = cos(theta);
            T sth = sin(theta);

            T vXp[3];
            cross(v, point, vXp);
            T vDotp = dot(v, point);
            T oneMinusCth = T(1) - cth;

            for (int i = 0; i < 3; ++i)
                p[i] = point[i] * cth + vXp[i] * sth + v[i] * vDotp * oneMinusCth;
        } else {
            // taylor expansion for theta close to zero
            T aux[3];
            cross(camera, point, aux);
            for (int i = 0; i < 3; ++i)
                p[i] = point[i] + aux[i];
        }

        // translation of the camera
        p[0] += camera[3];
        p[1] += camera[4];
        p[2] += camera[5];

        // perspective division
        T projectedPoint[2];
        projectedPoint[0] = - p[0] / p[2];
        projectedPoint[1] = - p[1] / p[2];

        // conversion to pixel coordinates
        T radiusSqr = projectedPoint[0]*projectedPoint[0] + projectedPoint[1]*projectedPoint[1];
        T f         = T(camera[6]);
        T k1        = T(camera[7]);
        T k2        = T(camera[8]);
        T r_p       = T(1) + k1 * radiusSqr + k2 * radiusSqr * radiusSqr;
        T prediction[2];
        prediction[0] = f * r_p * projectedPoint[0];
        prediction[1] = f * r_p * projectedPoint[1];

        error[0] = prediction[0] - T(measurement()(0));
        error[1] = prediction[1] - T(measurement()(1));

        return true;
    }

    void computeError()
    {
        const VertexCameraBAL* cam = static_cast<const VertexCameraBAL*>(vertex(0));
        const VertexPointBAL* point = static_cast<const VertexPointBAL*>(vertex(1));

        (*this)(cam->estimate().data(), point->estimate().data(), _error.data());
    }

    void linearizeOplus()
    {
        // use numeric Jacobians
        BaseBinaryEdge<2, Vector2d, VertexCameraBAL, VertexPointBAL>::linearizeOplus();
        return;

        const VertexCameraBAL* cam = static_cast<const VertexCameraBAL*>(vertex(0));
        const VertexPointBAL* point = static_cast<const VertexPointBAL*>(vertex(1));
        // typedef ceres::internal::AutoDiff<EdgeObservationBAL, double, VertexCameraBAL::Dimension, VertexPointBAL::Dimension> BalAutoDiff;

        Matrix<number_t, Dimension, VertexCameraBAL::Dimension, Eigen::RowMajor> dError_dCamera;
        Matrix<number_t, Dimension, VertexPointBAL::Dimension, Eigen::RowMajor> dError_dPoint;
        number_t* parameters[] = {const_cast<number_t*>(cam->estimate().data()),
                                  const_cast<number_t*>(point->estimate().data())};
        number_t* jacobians[] = {dError_dCamera.data(), dError_dPoint.data()};
        number_t value[Dimension];
        using BalAutoDiffDims =
        ceres::internal::StaticParameterDims<VertexCameraBAL::Dimension, VertexPointBAL::Dimension>;
        bool diffState =
                ceres::internal::AutoDifferentiate<EdgeObservationBAL::Dimension, BalAutoDiffDims, EdgeObservationBAL,
                        number_t>(*this, parameters, Dimension, value, jacobians);

        // copy over the Jacobians (convert row-major -> column-major)
        if (diffState) {
            _jacobianOplusXi = dError_dCamera;
            _jacobianOplusXj = dError_dPoint;
        } else {
            assert(0 && "Error while differentiating");
            _jacobianOplusXi.setZero();
            _jacobianOplusXi.setZero();
        }
    }
};


class BUNDLEADJUSTMENT {
public:
    BUNDLEADJUSTMENT();

    BUNDLEADJUSTMENT(std::string file_name);

    ~BUNDLEADJUSTMENT();

public:
    DATA data_;

private:

};


#endif //INC_7_2_BUNDLEADJUSTMENT_H
