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

#ifndef TTMPC_CONSTRATINS_TEST_H
#define TTMPC_CONSTRATINS_TEST_H


#include "Spline/arc_length_spline.h"
#include "Constraints/constraints.h"
#include "Constraints/bounds.h"
#include "Constraints/SelfCollision/SelfCollisionModel.h"
#include "Model/robot_model.h"
#include "Model/robot_data.h"
#include "gtest/gtest.h"
#include <nlohmann/json.hpp>
using json = nlohmann::json;

void genRoundTrack(ttmpc::ArcLengthSpline &track)
{
    int NT = 100;    //number of track points
    double TrackRadius = 0.2; // track radius

    Eigen::VectorXd X;
    Eigen::VectorXd Y;
    Eigen::VectorXd Z;
    std::vector<Eigen::Matrix3d> R;
    Eigen::VectorXd phiR;

    X.setZero(NT);
    Y.setZero(NT);
    Z.setZero(NT);
    R.resize(NT);

    // generate equally spaced points around circle
    phiR.setLinSpaced(NT,0,2*M_PI);
    // compute circle points
    for(int i=0;i<NT;i++){
        X(i) = 0;
        Y(i) = TrackRadius*std::cos(phiR(i));
        Z(i) = TrackRadius*std::sin(phiR(i));
        R[i] << 1,0,0,0,-1,0,0,0,-1;
    }

    // give points to arc length based 3-D curve fitting
    track.gen6DSpline(X,Y,Z,R);
}

TEST(TestConstraints, TestSelfCollision)
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
    ttmpc::Bounds bound = ttmpc::Bounds(ttmpc::BoundsParam(json_paths.bounds_path),ttmpc::Param(json_paths.param_path));
    std::unique_ptr<ttmpc::RobotModel> robot = std::make_unique<ttmpc::RobotModel>();
    std::unique_ptr<ttmpc::SelCollNNmodel> selcolNN = std::make_unique<ttmpc::SelCollNNmodel>();
    ttmpc::Constraints constraints = ttmpc::Constraints(0.02,json_paths);
    ttmpc::ArcLengthSpline track = ttmpc::ArcLengthSpline(json_paths);
    ttmpc::Model model = ttmpc::Model(0.02, json_paths);
    ttmpc::Param param = ttmpc::Param(json_paths.param_path);

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

    ttmpc::StateVector xk_vec,d_xk_vec,xk1_vec;
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
    uk1_vec = d_uk_vec + uk_vec;

    ttmpc::State xk = ttmpc::vectorToState(xk_vec);
    ttmpc::State xk1 = ttmpc::vectorToState(xk1_vec);
    ttmpc::Input uk = ttmpc::vectorToInput(uk_vec);
    ttmpc::Input uk1 = ttmpc::vectorToInput(uk1_vec);

    ttmpc::RobotData rbk, rbk1;
    rbk.update(xk_vec.head(ttmpc::PANDA_DOF),robot,selcolNN);
    rbk1.update(xk1_vec.head(ttmpc::PANDA_DOF),robot,selcolNN);


    ttmpc::ConstraintsInfo constr_info, constr_info1;
    ttmpc::ConstraintsJac jac_constr;

    constraints.getConstraints(xk,uk,rbk,1,&constr_info,&jac_constr);
    constraints.getConstraints(xk1,uk1,rbk1,1,&constr_info1,NULL);

    // real inequality constraint on xk1, uk1
    double r_l1 = constr_info1.c_lvec(ttmpc::si_index.con_selcol);
    double r_x1 = constr_info1.c_vec(ttmpc::si_index.con_selcol);
    double r_u1 = constr_info1.c_uvec(ttmpc::si_index.con_selcol);

    // Linearization inequality constriant on xk, uk
    double l_l1 = constr_info.c_lvec(ttmpc::si_index.con_selcol);
    double l_x1 = (jac_constr.c_x*d_xk_vec + jac_constr.c_u*d_uk_vec + constr_info.c_vec)(ttmpc::si_index.con_selcol);
    double l_u1 = constr_info.c_uvec(ttmpc::si_index.con_selcol);

    std::cout<< "Self Collision inequality condition"<<std::endl;
    std::cout<< "real on X0: " << constr_info.c_lvec(ttmpc::si_index.con_selcol) << " < " << constr_info.c_vec(ttmpc::si_index.con_selcol) << " < " << constr_info.c_uvec(ttmpc::si_index.con_selcol) << std::endl;
    std::cout<< "real on X1: " << r_l1 << " < " << r_x1 << " < " << r_u1 << std::endl;
    std::cout<< "lin  on X1: " << l_l1 << " < " << l_x1 << " < " <<  l_u1 << std::endl;
    std::cout << "Error[%]: " << fabs((l_x1 - r_x1) / r_x1)*100 << std::endl;
    
    EXPECT_TRUE(fabs((l_x1 - r_x1) / r_x1) < 0.05);
}

TEST(TestConstraints, TestSingularity)
{
    std::ifstream iConfig(ttmpc::pkg_path +"Params/config.json");
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
    ttmpc::Constraints constraints = ttmpc::Constraints(0.02,json_paths);
    ttmpc::ArcLengthSpline track = ttmpc::ArcLengthSpline(json_paths);
    ttmpc::Model model = ttmpc::Model(0.02, json_paths);
    ttmpc::Param param = ttmpc::Param(json_paths.param_path);

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

    ttmpc::StateVector xk_vec,d_xk_vec,xk1_vec;
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
    uk1_vec = d_uk_vec + uk_vec;

    ttmpc::State xk = ttmpc::vectorToState(xk_vec);
    ttmpc::State xk1 = ttmpc::vectorToState(xk1_vec);
    ttmpc::Input uk = ttmpc::vectorToInput(uk_vec);
    ttmpc::Input uk1 = ttmpc::vectorToInput(uk1_vec);

    ttmpc::RobotData rbk, rbk1;
    rbk.update(xk_vec.head(ttmpc::PANDA_DOF),robot,selcolNN);
    rbk1.update(xk1_vec.head(ttmpc::PANDA_DOF),robot,selcolNN);

    bool result = false;

    ttmpc::ConstraintsInfo constr_info, constr_info1;
    ttmpc::ConstraintsJac jac_constr;

    constraints.getConstraints(xk,uk,rbk,1,&constr_info,&jac_constr);
    constraints.getConstraints(xk1,uk1,rbk1,1,&constr_info1,NULL);

    // real inequality constraint on xk1, uk1
    double r_l1 = constr_info1.c_lvec(ttmpc::si_index.con_sing);
    double r_x1 = constr_info1.c_vec(ttmpc::si_index.con_sing);
    double r_u1 = constr_info1.c_uvec(ttmpc::si_index.con_sing);

    // Linearization inequality constriant on xk, uk
    double l_l1 = constr_info.c_lvec(ttmpc::si_index.con_sing);
    double l_x1 = (jac_constr.c_x*d_xk_vec + jac_constr.c_u*d_uk_vec + constr_info.c_vec)(ttmpc::si_index.con_sing);
    double l_u1 = constr_info.c_uvec(ttmpc::si_index.con_sing);

    std::cout<< "Singularity inequality condition"<<std::endl;
    std::cout<< "real on X0: " << constr_info.c_lvec(ttmpc::si_index.con_sing) << " < " << constr_info.c_vec(ttmpc::si_index.con_sing) << " < " << constr_info.c_uvec(ttmpc::si_index.con_sing) << std::endl;
    std::cout<< "real on X1: " << r_l1 << " < " << r_x1 << " < " << r_u1 << std::endl;
    std::cout<< "lin  on X1: " << l_l1 << " < " << l_x1 << " < " << l_u1 << std::endl;
    std::cout << "Error[%]: " << fabs((l_x1 - r_x1) / r_x1)*100 << std::endl;
    
    EXPECT_TRUE(fabs((l_x1 - r_x1) / r_x1) < 0.05);
}
#endif //TTMPC_CONSTRATINS_TEST_H