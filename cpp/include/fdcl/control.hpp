/** \file control.hpp
*  \brief Control class used to generate motors outputs for a desired trajectory
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

#ifndef FDCL_CONTROL_HPP
#define FDCL_CONTROL_HPP


// user headers
#include "common_types.hpp"
#include "fdcl/param.hpp"
#include "fdcl/integral_utils.hpp"
#include "fdcl/matrix_utils.hpp"


// external headers
#include "Eigen/Dense"


namespace fdcl
{
/** \brief Controller functions for the rover
 *
 * This class includes all the controllers used to control the rover	 
 * All controller related functions for the rover are included in this
 * class. This inlcudes two attitude controllers, a geometric controller and
 * a decoupled-yaw controller (ACC-2019) with a geometric position controller.
 */
class control
{
public:
    double t = 0.0;  /**< Current loop time in seconds */
    double t_pre = 0.0;  /**< Time of the previous loop in seconds */
    double dt = 1e-9;  /**< Time step size in seconds */
    int freq = 100;  /**< Control thread frequence */

    bool use_integral = false;  /**< Flag to enable/disable integral control */

    // for integral controller
    fdcl::integral_error_vec3 eIR; /**< Attitude integral error */
    fdcl::integral_error eI1; /**< Attitude integral error for roll axis */
    fdcl::integral_error eI2; /**< Attitude integral error for pitch axis */
    fdcl::integral_error eIy; /**< Attitude integral error for yaw axis */
    fdcl::integral_error_vec3 eIX; /**< Position integral error */

    Eigen::Vector3d eR = Eigen::Vector3d::Zero(); /**< Attitude error */
    Eigen::Vector3d eW = Eigen::Vector3d::Zero(); /**< Angular rate error */
    Eigen::Vector3d ei = Eigen::Vector3d::Zero(); /**< Position integral error */
    Eigen::Vector3d M = Eigen::Vector3d::Zero();  /**< Control moments */

    Eigen::Vector3d eX = Eigen::Vector3d::Zero(); /**< Position error */
    Eigen::Vector3d eV = Eigen::Vector3d::Zero(); /**< Velocity error */

    Eigen::Vector3d b1 = Eigen::Vector3d::Zero(); /**< Direction of the first body axis */
    Eigen::Vector3d b2 = Eigen::Vector3d::Zero(); /**< Direction of the second body axis */
    Eigen::Vector3d b3 = Eigen::Vector3d::Zero(); /**< Direction of the third body axis */
    Eigen::Vector3d b3_dot = Eigen::Vector3d::Zero(); /**< Desired rate of change of b3 axis */

    Eigen::Matrix4d fM_to_forces_inv; /**< Force to force-moment
        * conversion matrix
        */
    Eigen::Vector4d f_motor; /**< Calculated forces required by each moter */

    Eigen::Vector4d fM = Eigen::Vector4d::Zero(); /**< Force-moment vector */

    double f_total = 0.0;  /**< Total propeller thrust */
   
    control(
        fdcl::state_t *&state_, /**< Pointer to the current states */
        fdcl::command_t *&command_, /**< Pointer to the desired states */
        fdcl::param *config_file_  /**< Pointer to the external fdcl::param
            * object, which is used to load configuration parameters
            */
    );
    control(void); // Default constructor (this is should not be used)
    ~control(void);

    /** \fn void init(void)
     * Initialize the variables in the controller class
     */
    void init(void);

    /** \fn void load_config(void)
     * Loads the class parameters from the config file.
     */
    void load_config(void);

    /** \fn void set_error_to_zero(void)
     * Set all integral errors to zero.
     */
    void set_error_to_zero(void);

    /** \fn void attitude_control(void)
     * Decouple-yaw controller proposed on "Control of Complex Maneuvers
     * for a Quadrotor UAV using Geometric Methods on SE(3)"
     * URL: https://arxiv.org/pdf/1003.2005.pdf
     */
    void attitude_control(void);

    /** \fn void attitude_control_decoupled_yaw(void)
     * Decouple-yaw controller proposed on "Geometric Controls of a Quadrotor
     * with a Decoupled Yaw control"
     * URL: https://doi.org/10.23919/ACC.2019.8815189
     */
    void attitude_control_decoupled_yaw(void);


    /** \fn void position_control(void)
     * Position controller as proposed in "Geometric Controls of a Quadrotor
     * with a Decoupled Yaw control"
     */
    void position_control(void);


    /** \fn double get_time(void)
     * Returns the current time in seonds from the start of the class.
     * @return  current time in seonds from the start of the class
     */
    double get_time(void);


    /** \fn void output_uav_properties(double &m_out, Eigen::Matrix3d &J_out)
     * Outputs the mass and the inertia matrix of the UAV.
     * @param m_out 
     * @param J_out
     */
    void output_uav_properties(double &m, Eigen::Matrix3d &J);


    /** \fn void output_fM(double &f, Eigen::Vector3d &M)
     * Outputs the control force and moments.
     * @param f 
     * @param M
     */
    void output_fM(double &f, Eigen::Vector3d &M);


private:
    fdcl::state_t *state = nullptr; /**< Pointer to the current states */
    fdcl::command_t *command = nullptr; /**< Pointer to the desired states */
    fdcl::param *config_file = nullptr; /**< The fdcl::param object to load the 
     * configuration parameters from
     */

    bool use_decoupled_yaw = true; /**< Whether to use decoupled-yaw controller
     * or to use regular non-decoupled control for attitude
     */

    Eigen::Vector3d e1; /**< Direction of the first axis of the fixed frame */
    Eigen::Vector3d e2; /**< Direction of the second axis of the fixed frame */
    Eigen::Vector3d e3; /**< Direction of the third axis of the fixed frame */

    double m = 0.0;  /**< Mass of the rover (kg) */
    double g = 9.81;  /**< Gravitational acceleration (m/s^2) */
    Eigen::Matrix3d J = Eigen::Matrix3d::Zero();  /**< Inertia matrix for the rover */
    double c_tf = 0.0;  /**< Torsional moment generated by the propellers */
    double l = 0.0;  /**< Length of the rover arm */

    struct timespec tspec_init; /**< Time data at the beggining of the class */
    struct timespec tspec_curr; /**< Current time data */

     // Attitude gains
    Eigen::Matrix3d kR = Eigen::Matrix3d::Zero();  /**< Attitude gains */
    Eigen::Matrix3d kW = Eigen::Matrix3d::Zero();  /**< Angular rate gains */
    double kyw = 0.0; /**< Yaw angular rate gain for decoupled-yaw controller */

    // Position gains
    Eigen::Matrix3d kX = Eigen::Matrix3d::Zero(); /**< Position gains */
    Eigen::Matrix3d kV = Eigen::Matrix3d::Zero(); /**< Velocity gains */

    // Integral gains
    double kIR = 0.0;  /**< Attitude integral gain */
    double ki = 0.0;  /**< Position integral gain */
    double kI = 0.0;  /**< Attitude integral gain for roll and pitch */
    double kyI = 0.0;  /**< Attitude integral gain for yaw */
    double kIX = 0.0;  /**< Position integral gains */
    double c1 = 0.0;  /**< Parameters for decoupled-yaw integral control */
    double c2 = 0.0;  /**< Parameters for decoupled-yaw integral control */
    double c3 = 0.0;  /**< Parameters for decoupled-yaw integral control */

};  // end of control class

}  // end of namespace fdcl

#endif
