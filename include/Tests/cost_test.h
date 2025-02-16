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
#ifndef TTMPC_COST_TEST_H
#define TTMPC_COST_TEST_H

#include "Cost/cost.h"
#include "constraints_test.h"
#include "Constraints/bounds.h"
#include "Model/robot_data.h"
#include "gtest/gtest.h"
#include <nlohmann/json.hpp>
using json = nlohmann::json;

TEST(TestCost, TestSPD)
{
    std::ifstream iConfig(ttmpc::pkg_path + "Params/config.json");
    json jsonConfig;
    iConfig >> jsonConfig;

    ttmpc::PathToJson json_paths {ttmpc::pkg_path + std::string(jsonConfig["model_path"]),
                                 ttmpc::pkg_path + std::string(jsonConfig["cost_path"]),
                                 ttmpc::pkg_path + std::string(jsonConfig["bounds_path"]),
                                 ttmpc::pkg_path + std::string(jsonConfig["track_path"]),
                                 ttmpc::pkg_path + std::string(jsonConfig["normalization_path"]),
                                 ttmpc::pkg_path + std::string(jsonConfig["sqp_path"])};
    ttmpc::Bounds bound = ttmpc::Bounds(ttmpc::BoundsParam(json_paths.bounds_path), ttmpc::Param(json_paths.param_path));
    ttmpc::Cost cost = ttmpc::Cost(json_paths);
    std::unique_ptr<ttmpc::RobotModel> robot = std::make_unique<ttmpc::RobotModel>();
    std::unique_ptr<ttmpc::SelCollNNmodel> selcolNN = std::make_unique<ttmpc::SelCollNNmodel>();
    ttmpc::ArcLengthSpline track = ttmpc::ArcLengthSpline(json_paths);

    Eigen::Vector2d sel_col_n_hidden;
    sel_col_n_hidden << 256, 64;
    selcolNN->setNeuralNetwork(ttmpc::PANDA_DOF, 1, sel_col_n_hidden, true);

    genRoundTrack(track);

    ttmpc::Bounds_x x_LB = bound.getBoundsLX();
    ttmpc::Bounds_x x_UB = bound.getBoundsUX();
    ttmpc::Bounds_x x_range = x_UB - x_LB;
    ttmpc::Bounds_u u_LB = bound.getBoundsLU();
    ttmpc::Bounds_u u_UB = bound.getBoundsUU();
    ttmpc::Bounds_u u_range = u_UB - u_LB;

    ttmpc::StateVector xk_vec;
    xk_vec = ttmpc::StateVector::Random();
    xk_vec = (xk_vec + ttmpc::StateVector::Ones()) / 2;
    xk_vec = (xk_vec.array() * x_range.array()).matrix() + x_LB; 
    ttmpc::State xk = ttmpc::vectorToState(xk_vec);

    ttmpc::InputVector uk_vec;
    uk_vec = ttmpc::InputVector::Random();
    uk_vec = (uk_vec + ttmpc::InputVector::Ones()) / 2;
    uk_vec = (uk_vec.array() * u_range.array()).matrix() + u_LB; 
    ttmpc::Input uk = ttmpc::vectorToInput(uk_vec);

    ttmpc::RobotData rbk;
    rbk.update(xk_vec.head(ttmpc::PANDA_DOF),robot,selcolNN);

    // calculate cost matrix
    double temp_obj;
    ttmpc::CostGrad cost_grad;
    ttmpc::CostHess cost_hess;
    cost.getCost(track,xk,uk,rbk,1,&temp_obj,&cost_grad,&cost_hess);

    bool is_symQ = (cost_hess.f_xx.transpose() - cost_hess.f_xx).norm() < 1e-5;
    bool is_symR = (cost_hess.f_uu.transpose() - cost_hess.f_uu).norm() < 1e-5;
    std::cout<< "Is symmetric(Q, R): " << is_symQ << is_symR  << std::endl;

    Eigen::EigenSolver<Eigen::MatrixXd> eigensolverQ, eigensolverR;
    eigensolverQ.compute(cost_hess.f_xx); 
    eigensolverR.compute(cost_hess.f_uu); 

    Eigen::VectorXd eigen_valuesQ = eigensolverQ.eigenvalues().real();
    Eigen::VectorXd eigen_valuesR = eigensolverR.eigenvalues().real();

    bool eigen_positiveQ = eigen_valuesQ.minCoeff() > 0;
    bool eigen_positiveR = eigen_valuesR.minCoeff() > 0;

    bool is_spdQ = is_symQ && eigen_positiveQ;
    bool is_spdR = is_symR && eigen_positiveR;

    std::cout<< "Is semi positive definite(Q, R): " << is_spdQ << is_spdR << std::endl;
    if(!eigen_positiveQ) std::cout << "min eigenvalue Q: " << eigen_valuesQ.minCoeff() << std::endl;
    if(!eigen_positiveR) std::cout << "min eigenvalue R: " << eigen_valuesR.minCoeff() << std::endl;

    
    EXPECT_TRUE(is_symQ && is_symR && is_spdQ && is_spdR);
}

TEST(TestCost, TestLinearization)
{
    std::ifstream iConfig(ttmpc::pkg_path + "Params/config.json");
    json jsonConfig;
    iConfig >> jsonConfig;

    ttmpc::PathToJson json_paths {ttmpc::pkg_path + std::string(jsonConfig["model_path"]),
                                 ttmpc::pkg_path + std::string(jsonConfig["cost_path"]),
                                 ttmpc::pkg_path + std::string(jsonConfig["bounds_path"]),
                                 ttmpc::pkg_path + std::string(jsonConfig["track_path"]),
                                 ttmpc::pkg_path + std::string(jsonConfig["normalization_path"]),
                                 ttmpc::pkg_path + std::string(jsonConfig["sqp_path"])};
    ttmpc::Bounds bound = ttmpc::Bounds(ttmpc::BoundsParam(json_paths.bounds_path), ttmpc::Param(json_paths.param_path));
    std::unique_ptr<ttmpc::RobotModel> robot = std::make_unique<ttmpc::RobotModel>();
    std::unique_ptr<ttmpc::SelCollNNmodel> selcolNN = std::make_unique<ttmpc::SelCollNNmodel>();
    ttmpc::Cost cost = ttmpc::Cost(json_paths);
    ttmpc::CostParam cost_param = ttmpc::CostParam(json_paths.cost_path);
    ttmpc::Param param = ttmpc::Param(json_paths.param_path);
    ttmpc::ArcLengthSpline track = ttmpc::ArcLengthSpline(json_paths);

    genRoundTrack(track);

    Eigen::Vector2d sel_col_n_hidden;
    sel_col_n_hidden << 256, 64;
    selcolNN->setNeuralNetwork(ttmpc::PANDA_DOF, 1, sel_col_n_hidden, true);

    ttmpc::Bounds_x x_LB = bound.getBoundsLX();
    ttmpc::Bounds_x x_UB = bound.getBoundsUX();
    ttmpc::Bounds_x x_range = x_UB - x_LB;
    ttmpc::Bounds_u u_LB = bound.getBoundsLU();
    ttmpc::Bounds_u u_UB = bound.getBoundsUU();
    ttmpc::Bounds_u u_range = u_UB - u_LB;

    ttmpc::StateVector xk_vec, d_xk_vec, xk1_vec;
    xk_vec = ttmpc::StateVector::Random();
    xk_vec = (xk_vec + ttmpc::StateVector::Ones()) / 2;
    xk_vec = (xk_vec.array() * x_range.array()).matrix() + x_LB; 
    d_xk_vec = ttmpc::StateVector::Constant(0.01);
    xk1_vec = xk_vec + d_xk_vec;

    ttmpc::InputVector uk_vec, d_uk_vec, uk1_vec;
    uk_vec = ttmpc::InputVector::Random();
    uk_vec = (uk_vec + ttmpc::InputVector::Ones()) / 2;
    uk_vec = (uk_vec.array() * u_range.array()).matrix() + u_LB; 
    d_uk_vec = ttmpc::InputVector::Constant(0.01);
    uk1_vec = uk_vec + d_uk_vec;

    ttmpc::State xk = ttmpc::vectorToState(xk_vec);
    ttmpc::State xk1 = ttmpc::vectorToState(xk1_vec);
    ttmpc::Input uk = ttmpc::vectorToInput(uk_vec);
    ttmpc::Input uk1 = ttmpc::vectorToInput(uk1_vec);

    ttmpc::RobotData rbk, rbk1;
    rbk.update(xk_vec.head(ttmpc::PANDA_DOF),robot,selcolNN);
    rbk1.update(xk1_vec.head(ttmpc::PANDA_DOF),robot,selcolNN);

    double obj, obj1;
    ttmpc::CostGrad cost_grad;
    ttmpc::CostHess cost_hess;

    // caluculate real cost at xk, uk
    cost.getCost(track,xk,uk,rbk,1,&obj,&cost_grad,&cost_hess);
    cost.getCost(track,xk1,uk1,rbk1,1,&obj1,NULL,NULL);


    // calculate linearized cost at xk, uk
    double obj_lin1 = (0.5*d_xk_vec.transpose()*cost_hess.f_xx*d_xk_vec + 
                       0.5*d_uk_vec.transpose()*cost_hess.f_uu*d_uk_vec + 
                       d_xk_vec.transpose()*cost_hess.f_xu*d_uk_vec +  
                       cost_grad.f_x.transpose()*d_xk_vec + 
                       cost_grad.f_u.transpose()*d_uk_vec).value() + obj;

    
    std::cout << "X0\t: " << xk_vec.transpose()  <<"\t||\t U0\t:" << uk_vec.transpose() << std::endl;
    std::cout << "X1\t: " << xk1_vec.transpose() <<"\t||\t U1\t:" << uk1_vec.transpose() << std::endl;
    std::cout << "real cost on X0     : " << obj << std::endl;
    std::cout << "real cost on X0 + dX: " << obj1 << std::endl;
    std::cout << "lin  cost on X0 + dX: " << obj_lin1 << std::endl;
    std::cout << "Error[%]: " << fabs((obj1 - obj_lin1) / obj1)*100 <<std::endl;

    EXPECT_TRUE(fabs((obj1 - obj_lin1) / obj1) <= 1e-2);
}

#endif //TTMPC_COST_TEST_H
