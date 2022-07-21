/*
# Copyright (c) 2022 Murilo Marques Marinho
#
#    This file is part of sas_common.
#
#    sas_common is free software: you can redistribute it and/or modify
#    it under the terms of the GNU Lesser General Public License as published by
#    the Free Software Foundation, either version 3 of the License, or
#    (at your option) any later version.
#
#    sas_common is distributed in the hope that it will be useful,
#    but WITHOUT ANY WARRANTY; without even the implied warranty of
#    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
#    GNU Lesser General Public License for more details.
#
#    You should have received a copy of the GNU Lesser General Public License
#    along with sas_common.  If not, see <https://www.gnu.org/licenses/>.
#
# ################################################################
#
#   Author: Murilo M. Marinho, email: murilo@g.ecc.u-tokyo.ac.jp
#
# ################################################################*/
#include <sas_common/sas_common.h>
#include <mutex>

namespace sas
{

/**
 * @brief concatenate two VectorXd.
 * @param a a VectorXd.
 * @param b a VectorXd.
 * @return the result of the concatenated vectors.
 */
VectorXd concatenate(const VectorXd& a, const VectorXd& b)
{
    return (VectorXd (a.size() + b.size()) << a, b).finished();
}

/**
 * @brief concatenate a std::vector of VectorXd.
 * @param a an std::vector of VectorXd.
 * @return the result of the concatenated vectors.
 */
VectorXd concatenate(const std::vector<VectorXd>& as)
{
    VectorXd b;
    for(const auto& c : as)
    {
        b = concatenate(b, c);
    }
    return b;
}

/**
 * @brief vstack vertically (row-wise) stack two MatrixXd.
 * @param A the first MatrixXd.
 * @param B the second MatrixXd.
 * @return the vstacked MatrixXd.
 * @exception a std::range_error if @a A and @a B don't have
 * the same number of columns.
 * @note returns an empty matrix if both arguments are empty
 * or return the other argument of only one of the arguments
 * is empty.
 */
MatrixXd vstack(const MatrixXd& A, const MatrixXd& B)
{
    if((A.size() == 0) && (B.size()==0))
        return MatrixXd();
    if(A.size() == 0)
        return B;
    if(B.size() == 0)
        return A;
    if(A.cols()!=B.cols())
        throw std::range_error("vstack needs inputs a and b with the same number of columns.");

    return(MatrixXd(A.rows()+B.rows(),A.cols()) << A, B).finished();
}

/**
 * @brief block_diag creates a block diagonal matrix
 * using an input of std::vector<MatrixXd>.
 * e.g. if As= [A, B, C],
 * then
 * block_diag(As) =
 * |A 0 0|
 * |0 B 0|
 * |0 0 C|
 * @param As the std::vector<MatrixXd> contaning
 * the matrix to form the block diagonal matrix.
 * @return the block diagonal matrix.
 */
MatrixXd block_diag(const std::vector<MatrixXd>& As)
{
    int rows = 0;
    int cols = 0;
    for(const auto& A : As)
    {
        rows+=A.rows();
        cols+=A.cols();
    }

    MatrixXd B = MatrixXd::Zero(rows,cols);
    int start_row = 0;
    int start_col = 0;
    for(const auto& A : As)
    {
        const int& this_rows = A.rows();
        const int& this_columns = A.cols();

        B.block(start_row,start_col,this_rows,this_columns) = A;

        start_row+=this_rows;
        start_col+=this_columns;
    }

    return B;
}

/**
 * @brief split splits the input VectorXd @a as into a set of subvectors
 * defined by ns.
 * @param as the VectorXd to be split.
 * @param ns the sizes of the subvectors.
 * @return an std::vector<VectorXd> of the splitted vectors.
 */
std::vector<VectorXd> split(const VectorXd& a, const std::vector<int>& ns)
{
    std::vector<VectorXd> as;
    int n_acc = 0;
    for(const auto& n : ns)
    {
        as.push_back(a.segment(n_acc,n));
        n_acc+=n;
    }
    return as;
}


namespace common
{

/**
 * @brief get_static_node_handle a version of roscpp_initializer without using boost.
 * @return a reference to the global NodeHandle. Should only be used for the Python wrappers of the sas Classes.
 * Inspired on: https://github.com/ros-planning/moveit/blob/master/moveit_ros/planning_interface/py_bindings_tools/src/roscpp_initializer.cpp
 */
ros::NodeHandle& get_static_node_handle()
{
    std::lock_guard<std::mutex> lock(node_handle_mutex);
    if(!ros::isInitialized())
    {
        //ROS_INFO_STREAM("sas_common::Initializing roscpp.");
        int argc = 0;
        char** argv = nullptr;
        ros::init(argc,argv,"sas_common_nodehandle_dummy", ros::init_options::AnonymousName | ros::init_options::NoSigintHandler);
    }
    if(not node_handle)
    {
        //ROS_INFO_STREAM("sas_common::Initializing ros::NodeHandle.");
        node_handle = {new ros::NodeHandle(),
                       [](ros::NodeHandle* nh)
                       {
                           //std::cout << "sas_common::Shutdown roscpp."<< std::endl;
                           if (ros::isInitialized() && !ros::isShuttingDown())
                           {
                               async_spinner->stop();
                               ros::shutdown();
                           }
                           if(nh != nullptr)
                           {
                               delete nh;
                           }
                       }
                      };
        async_spinner.reset(new ros::AsyncSpinner(1));
        async_spinner->start();
    }
    return (*node_handle.get());
}
}
}
