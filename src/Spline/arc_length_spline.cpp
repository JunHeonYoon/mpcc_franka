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

#include "Spline/arc_length_spline.h"

namespace mpcc{
ArcLengthSpline::ArcLengthSpline()
{ 
}
ArcLengthSpline::ArcLengthSpline(const PathToJson &path)
:param_(Param(path.param_path))
{
}

ArcLengthSpline::ArcLengthSpline(const PathToJson &path,const ParamValue &param_value)
:param_(Param(path.param_path,param_value.param))
{
}

void ArcLengthSpline::setData(const Eigen::VectorXd &X_in,const Eigen::VectorXd &Y_in,const Eigen::VectorXd &Z_in,const std::vector<Eigen::Matrix3d> &R_in)
{
    // set input data if x, y, z and orienatation have same length
    // compute arc length based on an piecewise linear approximation
    if(X_in.size() == Y_in.size() && X_in.size() == Z_in.size() && X_in.size() == R_in.size())
    {
        path_data_.X = X_in;
        path_data_.Y = Y_in;
        path_data_.Z = Z_in;
        path_data_.R = R_in;
        path_data_.n_points = X_in.size();
        Eigen::VectorXd arc_length = compArcLength(X_in,Y_in,Z_in,R_in);
        path_data_.arc_length = arc_length.tail(1)(0);
        // path_data_.s = arc_length / path_data_.arc_length; // normalization
        path_data_.s.setLinSpaced(path_data_.n_points,0., 1.); // normalization
    }
    else
    {
        std::cout << "input data does not have the same length" << std::endl;
    }
}

void ArcLengthSpline::setRegularData(const Eigen::VectorXd &X_in,const Eigen::VectorXd &Y_in,const Eigen::VectorXd &Z_in,const std::vector<Eigen::Matrix3d> &R_in,const Eigen::VectorXd &s_in) {
    // set final x-y-z-R data if x, y, z and orieatation have same length
    // x-y points are space such that they are very close to arc length parametrized
    if(X_in.size() == Y_in.size() && X_in.size() == Z_in.size() && X_in.size() == R_in.size())
    {
        path_data_.X = X_in;
        path_data_.Y = Y_in;
        path_data_.Z = Z_in;
        path_data_.R = R_in;
        path_data_.n_points = X_in.size();
        path_data_.arc_length = compArcLength(X_in,Y_in,Z_in,R_in).tail(1)(0);
        path_data_.s = s_in;
    }
    else
    {
        std::cout << "input data does not have the same length" << std::endl;
    }
}

Eigen::VectorXd ArcLengthSpline::compArcLength(const Eigen::VectorXd &X_in,const Eigen::VectorXd &Y_in,const Eigen::VectorXd &Z_in,const std::vector<Eigen::Matrix3d> &R_in) const
{
    //compute arc length based on straight line distance between the data points
    double dx,dy,dz,dR;
    double dist;

    int n_points = X_in.size();
    // initailize a as zero
    Eigen::VectorXd a;
    a.setZero(n_points);
    for(int i=0;i<n_points-1;i++)
    {
        dx = X_in(i+1)-X_in(i);
        dy = Y_in(i+1)-Y_in(i);
        dz = Z_in(i+1)-Z_in(i);

        dR = getInverseSkewVector(LogMatrix(R_in[i].transpose() * R_in[i+1])).norm();

        dist = std::sqrt(dx*dx + dy*dy + dz*dz) + dR;    //dist is straight line distance between points
        a(i+1) = a(i)+dist;       //s is cumulative sum of dist
    }
    return a;
}

PathData ArcLengthSpline::resamplePath(const CubicSpline &initial_spline_x,const CubicSpline &initial_spline_y,const CubicSpline &initial_spline_z,const CubicSplineRot &initial_spline_r) const
{
    // re-sample arc length parametrized X-Y-Z-R spline path with N_spline data points
    // using equidistant arc length values
    // successively re-sample, computing the arc length and then fit the path should
    // result in close to equidistant points w.r.t. arc length

    // s -> "normalized arc length" where points should be extracted
    // equilly spaced between 0 and 1
    PathData resampled_path;
    resampled_path.n_points=N_SPLINE;
    resampled_path.s.setLinSpaced(N_SPLINE,0.,1.);

    // initialize new points as zero
    resampled_path.X.setZero(N_SPLINE);
    resampled_path.Y.setZero(N_SPLINE);
    resampled_path.Z.setZero(N_SPLINE);
    resampled_path.R.clear();
    resampled_path.R.resize(N_SPLINE);
    for(size_t i=0;i<N_SPLINE;i++) resampled_path.R[i] = Matrix3d::Zero();

    // extract X-Y-Z-R points
    for(int i=0;i<N_SPLINE;i++)
    {
        resampled_path.X(i) = initial_spline_x.getPoint(resampled_path.s(i));
        resampled_path.Y(i) = initial_spline_y.getPoint(resampled_path.s(i));
        resampled_path.Z(i) = initial_spline_z.getPoint(resampled_path.s(i));
        resampled_path.R[i] = initial_spline_r.getPoint(resampled_path.s(i));
    }
    return resampled_path;
}

RawPath ArcLengthSpline::outlierRemoval(const Eigen::VectorXd &X_original,const Eigen::VectorXd &Y_original,const Eigen::VectorXd &Z_original,const std::vector<Eigen::Matrix3d> &R_original) const
{

    // remove points which are not at all equally spaced, to avoid fitting problems
    // compute mean distance between points and then process the points such that points
    // are not closer than 1.5 the mean distance

    double dx,dy,dz;       // difference between points in x, y and z
    Eigen::VectorXd distVec;   // vector with all the distances
    double meanDist;    // mean distance
    double dist;        // temp variable for distance
    RawPath resampled_path;
    int k = 0;          // indicies
    int j = 0;

    if (X_original.size() != Y_original.size() || X_original.size() != Z_original.size() || X_original.size() != R_original.size()){
        //error
    }
//    std::cout << X_original << std::endl;

    int n_points = X_original.size();

    // initialize with zero
    resampled_path.X.setZero(n_points);
    resampled_path.Y.setZero(n_points);
    resampled_path.Z.setZero(n_points);
    resampled_path.R.clear();
    resampled_path.R.resize(n_points);
    for(size_t i=0;i<n_points;i++) resampled_path.R[i] = Matrix3d::Zero();


    // compute distance between points in X-Y-Z data
    distVec.setZero(n_points-1);
    for(int i=0;i<n_points-1;i++){
        dx = X_original(i+1)-X_original(i);
        dy = Y_original(i+1)-Y_original(i);
        dz = Z_original(i+1)-Z_original(i);
        distVec(i) = std::sqrt(dx*dx + dy*dy + dz*dz);
    }
    // compute mean distance between points
    meanDist = distVec.sum()/(n_points-1);

    // compute the new points
    // start point is the original start point
    resampled_path.X(k) = X_original(k);
    resampled_path.Y(k) = Y_original(k);
    resampled_path.Z(k) = Z_original(k);
    resampled_path.R[k] = R_original[k];
    k++;
    for(int i=1;i<n_points-1;i++){
        // compute distance between currently checked point and the one last added to the new X-Y-Z path
        dx = X_original(i)-X_original(j);
        dy = Y_original(i)-Y_original(j);
        dz = Z_original(i)-Z_original(j);
        dist = std::sqrt(dx*dx + dy*dy);
        // if this distance is larger than 1.5 the mean distance add this point to the new X-Y-Z path
        if(dist >= 1.5*meanDist)
        {
            resampled_path.X(k) = X_original(i);
            resampled_path.Y(k) = Y_original(i);
            resampled_path.Z(k) = Z_original(i);
            resampled_path.R[k] = R_original[k];
            k++;
            j = i;
        }
    }
    // always add the last point
    resampled_path.X(k) = X_original(n_points-1);
    resampled_path.Y(k) = Y_original(n_points-1);
    resampled_path.Z(k) = Z_original(n_points-1);
    resampled_path.R[k] = R_original[n_points-1];
    k++;

//    std::cout << "not resiszed " << X_new.transpose() << std::endl;
    // set the new X-Y data
//    setData(X.head(k),Y.head(k));
    resampled_path.X.conservativeResize(k);
    resampled_path.Y.conservativeResize(k);
    resampled_path.Z.conservativeResize(k);
    resampled_path.R.resize(k);

//    std::cout << "resiszed " << X_new.transpose()  << std::endl;
    return resampled_path;
}

double ArcLengthSpline::unwrapInput(double x) const
{
    return std::max(0., std::min(x,1.));
}

void ArcLengthSpline::fitSpline(const Eigen::VectorXd &X,const Eigen::VectorXd &Y,const Eigen::VectorXd &Z,const std::vector<Eigen::Matrix3d> &R)
{
    // successively fit spline -> re-sample path -> compute arc length
    // temporary spline class only used for fitting
    Eigen::VectorXd s_approximation;
    PathData first_refined_path,second_refined_path;

    // s_approximation = compArcLength(X,Y,Z,R);
    // s_approximation = s_approximation / s_approximation.tail(1)(0);
    s_approximation.setLinSpaced(X.size(),0.,1.);
    // std::cout << s_approximation << std::endl;

    CubicSpline first_spline_x,first_spline_y,first_spline_z; CubicSplineRot first_spline_r;
    CubicSpline second_spline_x,second_spline_y,second_spline_z; CubicSplineRot second_spline_r;
    // 1. spline fit
    first_spline_x.genSpline(s_approximation,X,false);
    first_spline_y.genSpline(s_approximation,Y,false);
    first_spline_z.genSpline(s_approximation,Z,false);
    first_spline_r.genSpline(s_approximation,R,false);
    // 1. re-sample
    first_refined_path = resamplePath(first_spline_x,first_spline_y,first_spline_z,first_spline_r);
    // s_approximation = compArcLength(first_refined_path.X,first_refined_path.Y,first_refined_path.Z,first_refined_path.R);
    // s_approximation = s_approximation / s_approximation.tail(1)(0);
    s_approximation.setLinSpaced(first_refined_path.X.size(),0.,1.);
    // std::cout << s_approximation << std::endl;
    ////////////////////////////////////////////
    // 2. spline fit
    second_spline_x.genSpline(s_approximation,first_refined_path.X,false);
    second_spline_y.genSpline(s_approximation,first_refined_path.Y,false);
    second_spline_z.genSpline(s_approximation,first_refined_path.Z,false);
    second_spline_r.genSpline(s_approximation,first_refined_path.R,false);
    // 2. re-sample
    second_refined_path = resamplePath(second_spline_x,second_spline_y,second_spline_z,second_spline_r);
    ////////////////////////////////////////////
    setRegularData(second_refined_path.X,second_refined_path.Y,second_refined_path.Z,second_refined_path.R,second_refined_path.s);
    // setData(second_refined_path.X,second_refined_path.Y);
    // Final spline fit with fixed Delta_s
    spline_x_.genSpline(path_data_.s,path_data_.X,true);
    spline_y_.genSpline(path_data_.s,path_data_.Y,true);
    spline_z_.genSpline(path_data_.s,path_data_.Z,true);
    spline_r_.genSpline(path_data_.s,path_data_.R,true);
}

void ArcLengthSpline::gen6DSpline(const Eigen::VectorXd &X,const Eigen::VectorXd &Y,const Eigen::VectorXd &Z,const std::vector<Eigen::Matrix3d> &R)
{
    // generate 6-D arc length parametrized spline given X-Y-Z-R data

    // remove outliers, depending on how irregular the points are this can help
    // RawPath clean_path;
    // clean_path = outlierRemoval(X,Y,Z,R);
    // successively fit spline and re-sample
    // fitSpline(clean_path.X,clean_path.Y,clean_path.Z,clean_path.R);
    fitSpline(X,Y,Z,R);
}

Eigen::Vector3d ArcLengthSpline::getPosition(const double s) const
{
    Eigen::Vector3d s_path;
    s_path(0) = spline_x_.getPoint(s);
    s_path(1) = spline_y_.getPoint(s);
    s_path(2) = spline_z_.getPoint(s);

    return s_path;
}

Eigen::Matrix3d ArcLengthSpline::getOrientation(const double s) const
{
    Eigen::Matrix3d s_path;
    s_path = spline_r_.getPoint(s);

    return s_path;
}

Eigen::Vector3d ArcLengthSpline::getDerivative(const double s) const
{
    Eigen::Vector3d ds_path;
    ds_path(0) = spline_x_.getDerivative(s);
    ds_path(1) = spline_y_.getDerivative(s);
    ds_path(2) = spline_z_.getDerivative(s);

    return ds_path;
}

Eigen::Vector3d ArcLengthSpline::getOrientationDerivative(const double s) const
{
    Eigen::Vector3d ds_path;
    ds_path = spline_r_.getDerivative(s);

    return ds_path;
}

Eigen::Vector3d ArcLengthSpline::getSecondDerivative(const double s) const
{
    Eigen::Vector3d dds_path;
    dds_path(0) = spline_x_.getSecondDerivative(s);
    dds_path(1) = spline_y_.getSecondDerivative(s);
    dds_path(2) = spline_z_.getSecondDerivative(s);

    return dds_path;
}

double ArcLengthSpline::getLength() const
{
    return path_data_.arc_length;
}

double ArcLengthSpline::projectOnSpline(const double &s, const Eigen::Matrix4d ee_pose) const
{
    double s_guess = s;
    Eigen::Vector3d pos_path = getPosition(s_guess);
    Eigen::Matrix3d ori_path = getOrientation(s_guess);

    double s_opt = s_guess;
    double dist = (ee_pose.block(0,3,3,1)-pos_path).norm() + 0.1*getInverseSkewVector(LogMatrix(ori_path.transpose() * ee_pose.block(0,0,3,3))).norm();

    if (dist >= param_.max_dist_proj)
    {
        std::cout << "dist too large" << std::endl;
        Eigen::ArrayXd diff_x_all = path_data_.X.array() - ee_pose(0,3);
        Eigen::ArrayXd diff_y_all = path_data_.Y.array() - ee_pose(1,3);
        Eigen::ArrayXd diff_z_all = path_data_.Z.array() - ee_pose(2,3);
        Eigen::ArrayXd dist_square = diff_x_all.square() + diff_y_all.square() + diff_z_all.square();
        std::vector<double> dist_square_vec(dist_square.data(),dist_square.data() + dist_square.size());
        for(size_t i=0; i++; i<dist_square_vec.size())
        {
            dist_square_vec[i] += 0.1*getInverseSkewVector(LogMatrix(path_data_.R[i].transpose() * ee_pose.block(0,0,3,3))).squaredNorm();
        }

        Eigen::ArrayXd diff_s_all = path_data_.s.array() - s_guess;
        Eigen::Array<bool, Eigen::Dynamic, 1> valid_mask = (diff_s_all.abs() <= param_.max_dist_proj);
        Eigen::ArrayXd filtered_dist = dist_square * valid_mask.cast<double>();
        filtered_dist = filtered_dist + (1.0 - valid_mask.cast<double>()) * std::numeric_limits<double>::infinity();
        Eigen::Index min_idx;
        filtered_dist.minCoeff(&min_idx);
        if (!valid_mask.any()) 
        {
            auto min_iter = std::min_element(dist_square_vec.begin(),dist_square_vec.end());
            s_opt = path_data_.s(std::distance(dist_square_vec.begin(), min_iter));
        } 
        else 
        {
            s_opt = path_data_.s(min_idx);
        }
    }
    else return s_opt;

    if(s_opt >= 1.) return 1.;

    double s_old = s_opt;
    for(int i=0; i<20; i++)
    {
        pos_path = getPosition(s_opt);
        Eigen::Vector3d ds_posi = getDerivative(s_opt);
        Eigen::Vector3d dds_posi = getSecondDerivative(s_opt);
        Eigen::Vector3d diff_posi = pos_path - ee_pose.block(0,3,3,1);

        ori_path = getOrientation(s_opt);
        Eigen::Vector3d diff_ori = getInverseSkewVector(LogMatrix(ori_path.transpose()*ee_pose.block(0,0,3,3)));
        Eigen::Matrix3d J_r_inv;
        if(diff_ori.norm() < 1e-8) J_r_inv = Eigen::Matrix3d::Identity();
        else J_r_inv = Eigen::Matrix3d::Identity() + 1./2.*getSkewMatrix(diff_ori) + ( 1. / diff_ori.squaredNorm() + ( 1. + cos(diff_ori.norm()) ) / ( 2. * diff_ori.norm() * sin(diff_ori.norm()) ) )*getSkewMatrix(diff_ori)*getSkewMatrix(diff_ori);
        Eigen::Vector3d ds_diff_ori = -J_r_inv * ee_pose.block(0,0,3,3).transpose() * ori_path * getOrientationDerivative(s_opt);

        double jac = 2.0 * ds_posi.dot(diff_posi) + 
                     0.1*2.0 * ds_diff_ori.dot(diff_ori);
        double hessian = 2.0 * ds_posi.dot(ds_posi) + 2.0 * dds_posi.dot(diff_posi) +
                         0.1*2.0 * ds_diff_ori.dot(ds_diff_ori); // gauss-newton approximation about orientation
        s_opt -= 0.2*jac/hessian;
        s_opt = unwrapInput(s_opt);

        if(std::abs(s_old - s_opt) <= 1e-5)
            return s_opt;
        s_old = s_opt;
    }
    // something is strange if it did not converge within 20 iterations, give back the initial guess
    return s_guess;
}
}