/*
 * Software License Agreement (BSD License)
 *
 *  Point Cloud Library (PCL) - www.pointclouds.org
 *  Copyright (c) 2010, Willow Garage, Inc.
 *  Copyright (c) 2012-, Open Perception, Inc.
 *
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of the copyright holder(s) nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *
 * $Id$
 *
 */

#pragma once

#include <pcl/registration/warp_point_rigid.h>

/** \brief @b WarpPointRigid3D enables 6D (3D rotation + 3D translation)
 * transformations for points.
 *
 * \note The class is templated on the source and target point types as well as on the
 * output scalar of the transformation matrix (i.e., float or double). Default: float.
 * \author Radu B. Rusu
 * \ingroup registration
 */
namespace pcl {
namespace registration {
template <typename PointSourceT, typename PointTargetT, typename Scalar = float>
class WarpPoint3dTransRot : public WarpPointRigid<PointSourceT, PointTargetT, Scalar>
{
private:
    int rot_i_;
    int trans0_i_;
    int trans1_i_;
public:
    using WarpPointRigid<PointSourceT, PointTargetT, Scalar>::transform_matrix_;

    using Matrix4 = typename WarpPointRigid<PointSourceT, PointTargetT, Scalar>::Matrix4;
    using VectorX = typename WarpPointRigid<PointSourceT, PointTargetT, Scalar>::VectorX;

    using Ptr = shared_ptr<WarpPoint3dTransRot<PointSourceT, PointTargetT, Scalar>>;
    using ConstPtr =
        shared_ptr<const WarpPoint3dTransRot<PointSourceT, PointTargetT, Scalar>>;

    WarpPoint3dTransRot() : rot_i_(2), trans0_i_(0), trans1_i_(1), WarpPointRigid<PointSourceT, PointTargetT, Scalar>(6) {}

    /** \brief Empty destructor */
    ~WarpPoint3dTransRot() override = default;

    void set_axes(int rot_i, int trans0_i, int trans1_i)
    {
        assert(rot_i > 0 && rot_i < 3);
        assert(trans0_i > 0 && trans0_i < 3);
        assert(trans1_i > 0 && trans1_i < 3);
        rot_i_ = rot_i; 
        trans0_i_ = trans0_i; 
        trans1_i_ = trans1_i; 
    }

    /** \brief Set warp parameters.
        * \note Assumes the quaternion parameters are normalized.
        * \param[in] p warp parameters (tx ty tz qx qy qz)
        */
    void
    setParam(const VectorX &p) override
    {
        assert(p.rows() == this->getDimension());

        // Copy the rotation and translation components
        double p_constr[6] = {0};
        p_constr[trans0_i_] = p[trans0_i_]; 
        p_constr[trans1_i_] = p[trans1_i_]; 
        p_constr[3 + rot_i_] = p[3 + rot_i_]; 
        transform_matrix_.setZero();
        transform_matrix_(0, 3) = p_constr[0];
        transform_matrix_(1, 3) = p_constr[1];
        transform_matrix_(2, 3) = p_constr[2];
        transform_matrix_(3, 3) = 1;

        // Compute w from the unit quaternion
        Eigen::Quaternion<Scalar> q(0, p_constr[3], p_constr[4], p_constr[5]);
        q.w() = static_cast<Scalar>(std::sqrt(1 - q.dot(q)));
        q.normalize();
        transform_matrix_.topLeftCorner(3, 3) = q.toRotationMatrix();
    }
};


}
}