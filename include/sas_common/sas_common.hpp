/*
# Copyright (c) 2022-2023 Murilo Marques Marinho
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
#   Author: Murilo M. Marinho, email: murilomarinho@ieee.org
#
# ################################################################*/
#pragma once

#include <eigen3/Eigen/Dense>

using namespace Eigen;

namespace sas
{

VectorXd concatenate(const VectorXd& a, const VectorXd& b);
VectorXd concatenate(const std::vector<VectorXd>& as);

MatrixXd vstack(const MatrixXd& A, const MatrixXd& B);

MatrixXd block_diag(const std::vector<MatrixXd>& As);

std::vector<VectorXd> split(const VectorXd& a, const std::vector<int>& ns);

}
