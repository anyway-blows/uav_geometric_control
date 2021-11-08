/*
 * Copyright (c) 2020 Flight Dynamics and Control Lab
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in 
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "fdcl/matrix_utils.hpp"


Eigen::Matrix3d hat(const Eigen::Vector3d v)
{
    Eigen::Matrix3d V;
    V.setZero();

    V(2,1) = v(0);
    V(1,2) = -V(2, 1);
    V(0,2) = v(1);
    V(2,0) = -V(0, 2);
    V(1,0) = v(2);
    V(0,1) = -V(1, 0);

    return V;
}


Eigen::Vector3d vee(const Eigen::Matrix3d V)
{
    // improve code by: https://codereview.stackexchange.com/questions/77546/multiply-vector-elements-by-a-scalar-value-using-stl-and-templates
    Eigen::Matrix3d E = V + V.transpose();
    
    if(E.norm() > 1.e-6)
    {
        std::cout << "VEE: E.norm() = " << E.norm() << std::endl;
    }
    Eigen::Vector3d v(V(2, 1), V(0, 2), V(1, 0));
    return v;
}


void saturate(Eigen::Vector3d &x, const double x_min, const double x_max)
{
    for (int i = 0; i < 3; i++)
    {
        if (x(i) > x_max) x(i) = x_max;
        else if (x(i) < x_min) x(i) = x_min;
    }
}


void deriv_unit_vector(
    const Eigen::Vector3d &A,
    const Eigen::Vector3d &A_dot,
    const Eigen::Vector3d &A_ddot,
    Eigen::Vector3d &q,
    Eigen::Vector3d &q_dot,
    Eigen::Vector3d &q_ddot
)
{
    double nA = A.norm();
    double nA3 = pow(nA, 3);
    double nA5 = pow(nA, 5);

    q = A / nA;
    q_dot = A_dot / nA \
        - A * A.dot(A_dot) / nA3;

    q_ddot = A_ddot / nA \
        - A_dot / nA3 * (2 * A.dot(A_dot)) \
        - A / nA3 * (A_dot.dot(A_dot) + A.dot(A_ddot)) \
        + 3 * A / nA5 * pow(A.dot(A_dot), 2);
}