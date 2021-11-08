/** \file common_types.hpp
*  \brief Data types used throughout the code.
*
* Data structures and type definitions used are defined here.
*/

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

#ifndef COMMON_TYPE_HPP
#define COMMON_TYPE_HPP

// external libraries
#include "Eigen/Dense"

namespace fdcl
{

/** \brief Data structure to store current states.
 */
struct state_t
{
    Eigen::Vector3d x = Eigen::Vector3d::Zero(); /**< Position */
    Eigen::Vector3d v = Eigen::Vector3d::Zero(); /**< Velocity */
    Eigen::Vector3d a = Eigen::Vector3d::Zero(); /**< Acceleration */
    Eigen::Matrix3d R = Eigen::Matrix3d::Identity(); /**< Attitude in SO(3) */
    Eigen::Vector3d W = Eigen::Vector3d::Zero(); /**< Body angular velocity */
};


/** \brief Data structure to store command (or desired) values.
 */
class command_t
{
public:
    Eigen::Vector3d Wd = Eigen::Vector3d::Zero(); /**< desired angular velocity*/
    Eigen::Vector3d Wd_dot = Eigen::Vector3d::Zero();
    Eigen::Vector3d Wd_2dot = Eigen::Vector3d::Zero();
    Eigen::Matrix3d Rd = Eigen::Matrix3d::Identity();
    Eigen::Vector3d xd = Eigen::Vector3d::Zero(); /**< desired position, aka. position coordinates*/
    Eigen::Vector3d xd_dot = Eigen::Vector3d::Zero(); /**< desired position 1 order derivatives, aka. velocity*/
    Eigen::Vector3d xd_2dot = Eigen::Vector3d::Zero(); /**< desired position 2 order derivatives, aka. acceleration*/
    Eigen::Vector3d xd_3dot = Eigen::Vector3d::Zero(); /**< desired position 3 order derivatives, aka. jerk*/
    Eigen::Vector3d xd_4dot = Eigen::Vector3d::Zero(); /**< desired position 4 order derivatives, aka. snap*/
    Eigen::Vector3d b1d = Eigen::Vector3d::Zero(); /**< desired b1 heading*/
    Eigen::Vector3d b1d_dot = Eigen::Vector3d::Zero();
    Eigen::Vector3d b1d_ddot = Eigen::Vector3d::Zero();
    Eigen::Vector3d b3d = Eigen::Vector3d::Zero(); /**< desired b3 heading*/
    Eigen::Vector3d b3d_dot = Eigen::Vector3d::Zero();
    Eigen::Vector3d b3d_ddot = Eigen::Vector3d::Zero();
    Eigen::Vector3d b1c = Eigen::Vector3d::Zero();
    double wc3 = 0.0;
    double wc3_dot = 0.0;

    Eigen::Vector3d b1 = Eigen::Vector3d::Zero();
    Eigen::Vector3d b2 = Eigen::Vector3d::Zero();
    Eigen::Vector3d b3 = Eigen::Vector3d::Zero();
};

}  // end of namespace fdcl

#endif
