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
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO eVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include "fdcl/control.hpp"


fdcl::control::control(
    fdcl::state_t *&state_,
    fdcl::command_t *&command_,
    fdcl::param *config_file_) : \
    state(state_), command(command_), config_file(config_file_)
{
    // initialize clock for this class
    clock_gettime(CLOCK_REALTIME, &tspec_init);

    // init uninitialized parameters
    fdcl::control::init();

    // load parameters from the config file
    fdcl::control::load_config();
};


fdcl::control::control(void)
{
    // This should not happen as this leads to uninitialized pointers.
    std::cout << "CTRL: control class is initiated without required parameters."
              << "\n\tThis might lead to undefined behavoirs."
              << "\n\tCalling the destructor .."
              << std::endl;

    this->~control();
}


fdcl::control::~control(void)
{
    // do nothing
};


void fdcl::control::init(void)
{
    e1 << 1.0, 0.0, 0.0;
    e2 << 0.0, 1.0, 0.0;
    e3 << 0.0, 0.0, 1.0;
}


void fdcl::control::position_control(void)
{
    // Position controller as proposed in "Geometric Controls of a Quadrotor
    // with a Decoupled Yaw control"
    // translational error functions
    eX = state->x - command->xd;     // position error - eq (11)
    eV = state->v - command->xd_dot; // velocity error - eq (12)

    // position integral terms
    // "use_integral" must be set through the config file
    if (use_integral)
    {
        eIX.integrate(c1 * eX + eV, dt); // eq (13)

        double sat_sigma = 1.8;
        saturate(eIX.error, -sat_sigma, sat_sigma);
    }
    else
    {
        eIX.set_zero();
    }

    // force 'f' along negative b3-axis - eq (14)
    // this term equals to R.e3
    Eigen::Vector3d A = -kX * eX \
                - kV * eV \
                - kIX * eIX.error \
                - m * g * e3 \
                + m * command->xd_2dot;

    Eigen::Vector3d b3 = state->R * e3;
    Eigen::Vector3d b3_dot = state->R * hat(state->W) * e3; // eq (22)
    f_total = -A.dot(b3);

    // intermediate terms for rotational errors
    Eigen::Vector3d ea = g * e3 - f_total / m * b3 - command->xd_2dot;
    Eigen::Vector3d A_dot = -kX * eV - kV * ea + m * command->xd_3dot;

    double fdot = -A_dot.dot(b3) - A.dot(b3_dot);
    Eigen::Vector3d eb = -fdot / m * b3 - f_total / m * b3_dot - command->xd_3dot;
    Eigen::Vector3d A_ddot = -kX * ea - kV * eb + m * command->xd_4dot;

    Eigen::Vector3d b3c, b3c_dot, b3c_ddot;
    deriv_unit_vector(-A, -A_dot, -A_ddot, b3c, b3c_dot, b3c_ddot);

    Eigen::Vector3d A2 = -hat(command->b1d) * b3c;
    Eigen::Vector3d A2_dot = -hat(command->b1d_dot) * b3c - hat(command->b1d) * b3c_dot;
    Eigen::Vector3d A2_ddot = -hat(command->b1d_ddot) * b3c \
                      - 2.0 * hat(command->b1d_dot) * b3c_dot 
                      - hat(command->b1d) * b3c_ddot;

    Eigen::Vector3d b2c, b2c_dot, b2c_ddot;
    deriv_unit_vector(A2, A2_dot, A2_ddot, b2c, b2c_dot, b2c_ddot);

    Eigen::Vector3d b1c = hat(b2c) * b3c;
    Eigen::Vector3d b1c_dot = hat(b2c_dot) * b3c + hat(b2c) * b3c_dot;
    Eigen::Vector3d b1c_ddot = hat(b2c_ddot) * b3c \
                       + 2.0 * hat(b2c_dot) * b3c_dot \
                       + hat(b2c) * b3c_ddot;

    Eigen::Matrix3d Rddot, Rdddot;

    command->Rd << b1c, b2c, b3c;
    Rddot << b1c_dot, b2c_dot, b3c_dot;
    Rdddot << b1c_ddot, b2c_ddot, b3c_ddot;

    command->Wd = vee(command->Rd.transpose() * Rddot);
    command->Wd_dot = vee(command->Rd.transpose() * Rdddot \
                      - hat(command->Wd) * hat(command->Wd));

    // roll / pitch
    command->b3d = b3c;
    command->b3d_dot = b3c_dot;
    command->b3d_ddot = b3c_ddot;

    // yaw
    command->b1c = b1c;
    command->wc3 = e3.dot(state->R.transpose() * command->Rd * command->Wd);
    command->wc3_dot = e3.dot(state->R.transpose() * command->Rd \
        * command->Wd_dot) \
        - e3.dot(hat(state->W) * state->R.transpose() \
        * command->Rd * command->Wd);

    // This is set through the config file.
    if (use_decoupled_yaw)
    {
        this->attitude_control_decoupled_yaw();
    }
    else
    {
        this->attitude_control();
    }
    
}


void fdcl::control::attitude_control(void)
{
    //  This uses the controller defined in "Control of Complex Maneuvers
    //  for a Quadrotor UAV using Geometric Methods on SE(3)"
    //  URL: https://arxiv.org/pdf/1003.2005.pdf
    Eigen::Matrix3d RdtR = command->Rd.transpose() * state->R;
    eR = 0.5 * vee(RdtR - RdtR.transpose());
    eW = state->W - state->R.transpose() * command->Rd * command->Wd;

    if(use_integral)
    {
        eIR.integrate(eW + c2 * eR, dt);
    }

    M = - kR * eR \
        - kW * eW \
        - kI * eIR.error \
        + hat(state->R.transpose() * command->Rd * command->Wd) * J * \
            state->R.transpose() * command->Rd * command->Wd \
        + J * state->R.transpose() * command->Rd * command->Wd_dot;

    fM(0) = f_total;
    fM.block<3,1>(1,0) = M;
    f_motor = fM_to_forces_inv * fM;
}


void fdcl::control::attitude_control_decoupled_yaw(void)
{
    // This uses the controller defined in "Geometric Controls of a Quadrotor
    // with a Decoupled Yaw Control" 
    // URL: https://doi.org/10.23919/ACC.2019.8815189
    b1 = state->R * e1;
    b2 = state->R * e2;
    b3 = state->R * e3;

    double ky = kR(2, 2);
    double kwy = kW(2, 2);

    // roll/pitch angular velocity vector
    Eigen::Vector3d W_12 = state->W(0) * b1 + state->W(1) * b2;
    b3_dot = hat(W_12) * b3; // eq (26)

    Eigen::Vector3d W_12d = hat(command->b3d) * command->b3d_dot;
    Eigen::Vector3d W_12d_dot = hat(command->b3d) * command->b3d_ddot;

    Eigen::Vector3d eb = hat(command->b3d) * b3;           // eq (27)
    Eigen::Vector3d ew = W_12 + hat(b3) * hat(b3) * W_12d; // eq (28)

    // yaw
    double ey = -b2.dot(command->b1c);
    double ewy = state->W(2) - command->wc3;

    // attitude integral terms
    Eigen::Vector3d eI = ew + c2 * eb;

    eI1.integrate(eI.dot(b1), dt); // b1 axis - eq (29)
    eI2.integrate(eI.dot(b2), dt); // b2 axis - eq (30)
    eIy.integrate(ewy + c3 * ey, dt);

    // control moment for the roll/pitch dynamics - eq (31)
    Eigen::Vector3d tau;
    tau = -kR(0, 0) * eb \
        - kW(0, 0) * ew \
        - J(0, 0) * b3.transpose() * W_12d * b3_dot \
        - J(0, 0) * hat(b3) * hat(b3) * W_12d_dot;
    if (use_integral)
    {
        tau += -kI * eI1.error * b1 - kI * eI2.error * b2;
    }

    double M1, M2, M3;

    // control moment around b1 axis - roll - eq (24)
    M1 = b1.transpose() * tau + J(2, 2) * state->W(2) * state->W(1);

    // control moment around b2 axis - pitch - eq (24)
    M2 = b2.transpose() * tau - J(2, 2) * state->W(2) * state->W(0);

    // control moment around b3 axis - yaw - eq (52)
    M3 = -ky * ey - kwy * ewy + J(2, 2) * command->wc3_dot;
    if (use_integral)
    {
        M3 += -kyI * eIy.error;
    }

    M << M1, M2, M3;

    fM(0) = f_total;
    fM.block<3, 1>(1, 0) = M;
    f_motor = fM_to_forces_inv * fM;

    // for saving:
    Eigen::Matrix3d RdtR = command->Rd.transpose() * state->R;
    eR = 0.5 * vee(RdtR - RdtR.transpose());
    eIR.error << eI1.error, eI2.error, eIy.error;
    eW = state->W - state->R.transpose() * command->Rd * command->Wd;
}


void fdcl::control::output_fM(double &f, Eigen::Vector3d &M)
{
    f = this->f_total;
    M = this->M;
}


void fdcl::control::output_orientation(Eigen::Orientationd &orientation)
{
    orientation = this->orientation;
}


void fdcl::control::geometric_track_control(void)
{   
    // T. Lee, M. Leok and N. H. McClamroch, 
    // "Geometric tracking control of a quadrotor UAV on SE(3)," 
    // 49th IEEE Conference on Decision and Control (CDC), Atlanta, GA, 2010, 
    // pp. 5420-5425, doi: 10.1109/CDC.2010.5717652.

    eX = state->x - command->xd;
    eV = state->v - command->xd_dot;

    Eigen::Vector3d e3(0.0, 0.0, 1.0);
    Eigen::Vector3d A = -kX * eX - kV * eV - m * g * e3 + m * command->xd_2dot;

    Eigen::Vector3d b3 = state->R * e3;
    Eigen::Vector3d b3_dot = state->R * hat(state->W) * e3;
    double f = -A.dot(b3);

    /* Eigen::Vector3d eA = state->a - command->xd_2dot; */
    Eigen::Vector3d eA = g * e3 - f / m * b3 - command->xd_2dot;
    Eigen::Vector3d A_1dot = -kX * eV - kV * eA + m * command->xd_3dot;

    double f_dot = -A_1dot.dot(b3) - A.dot(b3_dot);

    Eigen::Vector3d ej = -f_dot / m * b3 - f / m * b3_dot - command->xd_3dot;
    Eigen::Vector3d A_2dot = -kX * eA - kV * ej + m * command->xd_4dot;

    Eigen::Vector3d b3c, b3c_1dot, b3c_2dot;
    deriv_unit_vector(-A, -A_1dot, -A_2dot, b3c, b3c_1dot, b3c_2dot);

    // double nA = A.norm();
    // Eigen::Vector3d b3c = -A / A.norm();
    // Eigen::Vector3d b3c_1dot = -A_1dot / nA + (A.dot(A_1dot) / pow(nA, 3)) * A;
    // Eigen::Vector3d b3c_2dot = -A_2dot / nA + (2.0 / pow(nA, 3)) * A.dot(A_1dot) * A_1dot +
    //         (pow(A_1dot.norm(), 2) + A.dot(A_2dot) / (pow(nA, 3))) * A - (3.0 / pow(nA, 5)) * (pow(A.dot(A_1dot), 2)) * A;
    

    Eigen::Vector3d C = -hat(command->b1d) * b3c;
    Eigen::Vector3d C_1dot = -hat(command->b1d) * b3c_1dot - hat(command->b1d_dot) * b3c;
    Eigen::Vector3d C_2dot = -hat(command->b1d) * b3c_2dot - hat(command->b1d_2dot) * b3c 
                             -2.0 * hat(command->b1d_dot) * b3c_1dot;
    
    Eigen::Vector3d b2c, b2c_1dot, b2c_2dot;
    deriv_unit_vector(C, C_1dot, C_2dot, b2c, b2c_1dot, b2c_2dot);

    // double nC = C.norm();
    // Eigen::Vector3d b2c = C / C.norm();
    // Eigen::Vector3d b2c_1dot = C / nC - C.dot(C_1dot) / (pow(nC, 3)) * C;
    // Eigen::Vector3d b2c_2dot = C_2dot / nC - 2.0 / (pow(nC, 3)) * C.dot(C_1dot) * C_1dot - (((pow(C_2dot.norm(), 2) + C.dot(C_2dot))) / pow(nC, 3)) * C
    //                 + (3.0 / pow(nC, 5)) * (pow(C.dot(C_1dot), 2)) * C;
    
    Eigen::Vector3d b1c = hat(b2c) * b3c;
    Eigen::Vector3d b1c_1dot = hat(b2c) * b3c_1dot + hat(b2c_1dot) * b3c;
    Eigen::Vector3d b1c_2dot = hat(b2c) * b3c_2dot + hat(b2c_2dot) * b3c + 2.0 * hat(b2c_1dot) * b3c_1dot;

    Eigen::Matrix3d Rc, Rc_1dot, Rc_2dot;
    Rc << b1c, b2c, b3c;
    Rc_1dot << b1c_1dot, b2c_1dot, b3c_1dot;
    Rc_2dot << b1c_2dot, b2c_2dot, b3c_2dot;

    Eigen::Vector3d Wc = vee(Rc.transpose() * Rc_1dot);
    Eigen::Vector3d Wc_1dot = vee(Rc.transpose() * Rc_2dot - hat(Wc) * hat(Wc));

    //attitude control part
    eR = 0.5 * vee(Rc.transpose() * state->R - state->R.transpose() * Rc);
    eW = state->W - state->R.transpose() * Rc * Wc;

    M = -kR * eR - kW * eW + 
        state->W.cross(J * state->W) - 
        J * (hat(state->W) * state->R.transpose() * Rc * Wc - state->R.transpose() * Rc * Wc_1dot);

    // for save
    fM(0) = f_total;
    fM.block<3,1>(1,0) = M;
    f_motor = fM_to_forces_inv * fM;

    // control_out
    f_total = f;
    orientation = Eigen::Quaterniond(Rc);
}


void fdcl::control::set_error_to_zero(void)
{
    // Set integral terms to zero.
    eIX.set_zero();
    eIR.set_zero();
    eI1.set_zero();
    eI2.set_zero();
    eIy.set_zero();
    ei.setZero();
}


void fdcl::control::load_config(void)
{
    config_file->read("integral.use_integral", use_integral);
    config_file->read("control.use_decoupled_yaw", use_decoupled_yaw);

    Eigen::Vector3d temp_3x1;
    config_file->read("control.kX", temp_3x1);
    kX(0, 0) = temp_3x1[0];
    kX(1, 1) = temp_3x1[1];
    kX(2, 2) = temp_3x1[2];

    config_file->read("control.kV", temp_3x1);
    kV(0, 0) = temp_3x1[0];
    kV(1, 1) = temp_3x1[1];
    kV(2, 2) = temp_3x1[2];

    config_file->read("integral.kIX", kIX);
    config_file->read("integral.c1", c1);

    config_file->read("control.kR", temp_3x1);
    kR(0, 0) = temp_3x1[0];
    kR(1, 1) = temp_3x1[1];
    kR(2, 2) = temp_3x1[2];

    config_file->read("control.kW", temp_3x1);
    kW(0, 0) = temp_3x1[0];
    kW(1, 1) = temp_3x1[1];
    kW(2, 2) = temp_3x1[2];

    config_file->read("integral.kIR", kIR);
    config_file->read("integral.c2", c2);
    config_file->read("integral.c3", c3);

    config_file->read("integral.kyI", kyI); // yaw
    config_file->read("integral.ki", ki);   //position
    config_file->read("integral.kI", kI);   //roll & pitch

    config_file->read("control.c_tf", c_tf);
    config_file->read("control.l", l);

    config_file->read("uav.J", J);
    config_file->read("uav.m", m);
    config_file->read("uav.g", g);

    Eigen::Matrix4d fM_to_forces;
    fM_to_forces << 1.0, 1.0, 1.0, 1.0,
                    0.0,  -l, 0.0,   l,
                    l,   0.0,  -l, 0.0,
                    -c_tf, c_tf, -c_tf, c_tf;
    fM_to_forces_inv = fM_to_forces.inverse();
}


void fdcl::control::output_uav_properties(double &m, Eigen::Matrix3d &J)
{
    m = this->m;
    J = this->J;
}


double fdcl::control::get_time()
{
    double t;
    clock_gettime(CLOCK_REALTIME, &tspec_curr);
    t = (double)tspec_curr.tv_sec + ((double)tspec_curr.tv_nsec) / 1.e9;
    t -= (double)tspec_init.tv_sec + ((double)tspec_init.tv_nsec) / 1.e9;
    return t;
}