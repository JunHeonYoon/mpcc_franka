// Copyright 2019 Alexander Liniger

// Licensed under the Apache License, Version 2.0 (the "License");
// you may not use this file except in compliance with the License.
// You may obtain a copy of the License at

//     http://www.apache.org/licenses/LICENSE-2.0

// Unless required by applicable law or agreed to in writing, software
// distributed under the License is distributed on an "AS IS" BASIS,
// WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
// See the License for the specific language governing permissions and
// limitations under the License.
///////////////////////////////////////////////////////////////////////////
///////////////////////////////////////////////////////////////////////////

#include "Cost/cost.h"
namespace ttmpc{
Cost::Cost() 
{
    std::cout << "default constructor, not everything is initialized properly" << std::endl;
}

Cost::Cost(const PathToJson &path) 
:cost_param_(path.cost_path),
param_(path.param_path)
{
}

Cost::Cost(const PathToJson &path,const ParamValue &param_value) 
:cost_param_(path.cost_path,param_value.cost),
param_(path.param_path,param_value.param)
{
}

double CubicSpline(double x, double x_0, double x_f, double y_0, double y_f)
{
    double x_temp =  (x - x_0) / (x_f - x_0);
    double x_temp2 = pow(x_temp, 2);
    double x_temp3 = pow(x_temp, 3);

    return y_0 + (y_f - y_0) * (3 * x_temp2 - 2 * x_temp3);
}

void Cost::getPositionCost(const Eigen::Vector3d &ref_posi,const State &x,const RobotData &rb,int k, 
                             double* obj,CostGrad* grad,CostHess* hess)
{
    const double PositionCost = cost_param_.q_e * ((k == N) ? cost_param_.q_e_N_mult : 1.0);

    // Exact position error cost
    if(obj)
    {
        (*obj) = PositionCost * (rb.EE_position_ - ref_posi).squaredNorm();
    }

    // Gradient of position error cost
    if(grad)
    {
        grad->setZero();
        grad->f_x = 2.0 * PositionCost * rb.Jv_.transpose() * (rb.EE_position_ - ref_posi);
    }

    // Hessian of position error cost
    if(hess)
    {
        hess->setZero();
        hess->f_xx = 2.0 * PositionCost * rb.Jv_.transpose() * rb.Jv_;
    }
    return;
}

void Cost::getHeadingCost(const Eigen::Matrix3d &ref_ori,const State &x,const RobotData &rb,
                          double* obj,CostGrad* grad,CostHess* hess)
{
    // compute heading orientation error cost
    const Eigen::Matrix3d ori_ee = rb.EE_orientation_;
    const Eigen::Vector3d Phi = getInverseSkewVector(LogMatrix(ref_ori.transpose() * ori_ee));

    // Exact Heading error cost
    if(obj)
    {
        (*obj) = cost_param_.q_ori * Phi.squaredNorm();
    }


    if(grad || hess)
    {
        // Linearize heading error function by jacobian. 
        Eigen::Matrix3d J_r_inv;
        if(Phi.norm() < 1e-8) J_r_inv = Eigen::Matrix3d::Identity();
        else J_r_inv = Eigen::Matrix3d::Identity() + 1./2.*getSkewMatrix(Phi) + ( 1. / Phi.squaredNorm() + ( 1. + cos(Phi.norm()) ) / ( 2. * Phi.norm() * sin(Phi.norm()) ) )*getSkewMatrix(Phi)*getSkewMatrix(Phi);

        Eigen::MatrixXd d_Phi = J_r_inv * ori_ee.transpose() * rb.Jw_;

        // Gradient of Heading error cost
        if(grad)
        {
            grad->setZero();
            grad->f_x = 2.0 * cost_param_.q_ori * d_Phi.transpose() * Phi;
        }

        // Hessian of Heading error cost
        if(hess)
        {
            hess->setZero();
            hess->f_xx = 2.0 * cost_param_.q_ori * d_Phi.transpose() * d_Phi;
        }
    }

    return;
}

void Cost::getInputCost(const State &x,const Input &u,const RobotData &rb,int k,
                        double* obj,CostGrad* grad,CostHess* hess)
{
    // compute control input cost, formed by joint velocity, joint acceleration, acceleration of path parameter 
    dJointVector dq = inputTodJointVector(u);

    // Exact Input cost
    if(obj)
    {
        (*obj) = 0;
        if(k != N) (*obj) = cost_param_.r_dq * dq.squaredNorm();    }


    // Gradient of Input cost
    if(grad)
    {
        grad->setZero();
        if(k != N)
        {
            grad->f_u.segment(si_index.dq1,PANDA_DOF) = 2.0 * cost_param_.r_dq * dq;
        }
    }
    
    if(hess)
    {
        hess->setZero();
        if(k != N)
        {
            hess->f_uu.block(si_index.dq1,si_index.dq1,PANDA_DOF,PANDA_DOF) = 2.0 * cost_param_.r_dq * Eigen::MatrixXd::Identity(PANDA_DOF,PANDA_DOF);
        }
    }

    return;
}

void Cost::getSingularityCost(const State &x,const RobotData &rb,
                              double* obj,CostGrad* grad,CostHess* hess)
{
    if(obj)
    {
        (*obj) = -cost_param_.q_sing * rb.manipul_;
    }
    if(grad)
    {
        grad->setZero();
        grad->f_x.segment(si_index.q1, PANDA_DOF) = -cost_param_.q_sing * rb.d_manipul_;
    }
    if(hess)
    {
        hess->setZero();
    }
}

void Cost::getCost(const Eigen::Vector3d &ref_posi,const Eigen::Matrix3d &ref_ori,const State &x,const Input &u,const RobotData &rb,int k,
                   double* obj,CostGrad* grad,CostHess* hess)
{
    double ratio = std::min(rb.sel_min_dist_ / (param_.tol_selcol*2.0), rb.manipul_ / (param_.tol_sing*2.0));
    if(ratio <= 1.0)
    {
        contouring_cost_ = cost_param_.q_e * CubicSpline(ratio, 0.5, 1.0, cost_param_.q_c_red_ratio, 1.0);
        heading_cost_ = cost_param_.q_ori * CubicSpline(ratio, 0.5, 1.0, cost_param_.q_ori_red_ratio, 1.0);
    }
    else
    {
        contouring_cost_ = cost_param_.q_e;
        heading_cost_ = cost_param_.q_ori;
    }
    double obj_contouring, obj_heading, obj_input, obj_sing;
    CostGrad grad_contouring, grad_heading, grad_input, grad_sing;
    CostHess hess_contouring, hess_heading, hess_input, hess_sing;

    if(obj && !grad && !hess)
    {
        getPositionCost(ref_posi, x, rb, k, &obj_contouring, NULL, NULL);
        getHeadingCost(ref_ori, x, rb, &obj_heading, NULL, NULL);
        getInputCost(x, u, rb, k, &obj_input, NULL, NULL);
        getSingularityCost(x, rb, &obj_sing, NULL, NULL);

    }
    else if(obj && grad && !hess)
    {
        getPositionCost(ref_posi, x, rb, k, &obj_contouring, &grad_contouring, NULL);
        getHeadingCost(ref_ori, x, rb, &obj_heading, &grad_heading, NULL);
        getInputCost(x, u, rb, k, &obj_input, &grad_input, NULL);
        getSingularityCost(x, rb, &obj_sing, &grad_sing, NULL);
    }
    else if(obj && grad && hess)
    {
        getPositionCost(ref_posi, x, rb, k, &obj_contouring, &grad_contouring, &hess_contouring);
        getHeadingCost(ref_ori, x, rb, &obj_heading, &grad_heading, &hess_heading);
        getInputCost(x, u, rb, k, &obj_input, &grad_input, &hess_input);
        getSingularityCost(x, rb, &obj_sing, &grad_sing, &hess_sing);
    }

    if(obj)
    {
        (*obj) =  obj_contouring + obj_heading + obj_input + obj_sing;
    }

    if(grad)
    {
        grad->f_x = grad_contouring.f_x + grad_heading.f_x + grad_input.f_x + grad_sing.f_x;
        grad->f_u = grad_contouring.f_u + grad_heading.f_u + grad_input.f_u + grad_sing.f_u;
    }

    if(hess)
    {
        hess->f_xx = hess_contouring.f_xx + hess_heading.f_xx + hess_input.f_xx + hess_sing.f_xx;
        hess->f_uu = hess_contouring.f_uu + hess_heading.f_uu + hess_input.f_uu + hess_sing.f_uu;
        hess->f_xu = hess_contouring.f_xu + hess_heading.f_xu + hess_input.f_xu + hess_sing.f_xu;

        hess->f_xx += Q_MPC::Identity()*1e-6;
        hess->f_uu += R_MPC::Identity()*1e-6;
    }
    return;
}
}