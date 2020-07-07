#include "fdcl/control.hpp"


fdcl::control::control(
    fdcl::state_t *&state_,
    fdcl::command_t *&command_,
    fdcl::param *config_file_) : \
    state(state_), command(command_), config_file(config_file_)
{
    clock_gettime(CLOCK_REALTIME, &tspec_init);
    fdcl::control::init();
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
    delete state;
    delete command;
    delete config_file;
};


void fdcl::control::init(void)
{
    e1 << 1.0, 0.0, 0.0;
    e2 << 0.0, 1.0, 0.0;
    e3 << 0.0, 0.0, 1.0;
}


void fdcl::control::position_control(void)
{
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
    Vector3 A = -kX * eX \
        - kV * eV \
        - kIX * eIX.error \
        - m * g * e3 \
        + m * command->xd_2dot;

    Vector3 L = state->R * e3;
    Vector3 Ldot = state->R * hat(state->W) * e3; // eq (22)
    f_total = -A.dot(state->R * e3);

    // intermediate terms for rotational errors
    Vector3 ea = g * e3 - f_total / m * L - command->xd_2dot;
    Vector3 Adot = -kX * eV - kV * ea + m * command->xd_3dot;
    // Lee Matlab: -ki * satdot(sigma,ei,ev + c1 * ex);

    double fdot = -Adot.dot(L) - A.dot(Ldot);
    Vector3 eb = -fdot / m * L - f_total / m * Ldot - command->xd_3dot;
    Vector3 Addot = -kX * ea - kV * eb + m * command->xd_4dot;
    // Lee Matlab: -ki * satdot(sigma,ei,ea + c1 * ev);

    double nA = A.norm();
    Vector3 Ld = -A / nA;
    Vector3 Lddot = -Adot / nA + A * A.dot(Adot) / pow(nA, 3);
    Vector3 Ldddot = -Addot / nA \
        + Adot / pow(nA, 3) * (2 * A.dot(Adot)) \
        + A / pow(nA, 3) * (Adot.dot(Adot) + A.dot(Addot)) \
        - 3.0 * A / pow(nA, 5) * pow(A.dot(Adot), 2);

    Vector3 Ld2 = -hat(command->b1d) * Ld;
    Vector3 Ld2dot = -hat(command->b1d_dot) * Ld - hat(command->b1d) * Lddot;
    Vector3 Ld2ddot = -hat(command->b1d_ddot) * Ld \
        - 2.0 * hat(command->b1d_dot) * Lddot 
        - hat(command->b1d) * Ldddot;

    double nLd2 = Ld2.norm();
    Vector3 Rd2 = Ld2 / nLd2;
    Vector3 Rd2dot = Ld2dot / nLd2 - Ld2.dot(Ld2dot) / pow(nLd2, 3) * Ld2;
    Vector3 Rd2ddot = Ld2ddot / nLd2 \
        - Ld2.dot(Ld2dot) / pow(nLd2, 3) * Ld2dot \
        - Ld2dot.dot(Ld2dot) / pow(nLd2, 3) * Ld2 \
        - Ld2.dot(Ld2ddot) / pow(nLd2, 3) * Ld2 \
        - Ld2.dot(Ld2dot) / pow(nLd2, 3) * Ld2dot \
        + 3.0 * pow(Ld2.dot(Ld2dot), 2) / pow(nLd2, 5) * Ld2;

    Vector3 Rd1 = hat(Rd2) * Ld;
    Vector3 Rd1dot = hat(Rd2dot) * Ld + hat(Rd2) * Lddot;
    Vector3 Rd1ddot = hat(Rd2ddot) * Ld \
        + 2.0 * hat(Rd2dot) * Lddot \
        + hat(Rd2) * Ldddot;

    Matrix3 Rddot, Rdddot;

    command->Rd << Rd1, Rd2, Ld;
    Rddot << Rd1dot, Rd2dot, Lddot;
    Rdddot << Rd1ddot, Rd2ddot, Ldddot;

    command->Wd = vee(command->Rd.transpose() * Rddot);
    command->Wd_dot = vee(command->Rd.transpose() * Rdddot \
        - hat(command->Wd) * hat(command->Wd));

    // roll / pitch
    command->b3d = Ld;
    command->b3d_dot = Lddot;
    command->b3d_ddot = Ldddot;

    // yaw
    command->b1c = Rd1;
    command->wc3 = e3.dot(state->R.transpose() * command->Rd * command->Wd);
    command->wc3_dot = (e3).dot(state->R.transpose() * command->Rd \
        * command->Wd_dot) \
        - e3.dot(hat(state->W) * state->R.transpose() \
        * command->Rd * command->Wd);

    this->attitude_control();
}


void fdcl::control::attitude_control(void)
{
    b1 = state->R * e1;
    b2 = state->R * e2;
    b3 = state->R * e3;

    double ky = kR(2, 2);
    double kwy = kW(2, 2);

    // roll/pitch angular velocity vector
    Vector3 W_12 = state->W(0) * b1 + state->W(1) * b2;
    b3_dot = hat(W_12) * b3; // eq (26)

    Vector3 W_12d = hat(command->b3d) * command->b3d_dot;
    Vector3 W_12d_dot = hat(command->b3d) * command->b3d_ddot;

    Vector3 eb = hat(command->b3d) * b3;           // eq (27)
    Vector3 ew = W_12 + hat(b3) * hat(b3) * W_12d; // eq (28)

    // yaw
    double ey = -b2.dot(command->b1c);
    double ewy = state->W(2) - command->wc3;

    // attitude integral terms
    Vector3 eI = ew + c2 * eb;

    eI1.integrate(eI.dot(b1), dt); // b1 axis - eq (29)
    eI2.integrate(eI.dot(b2), dt); // b2 axis - eq (30)
    eIy.integrate(ewy + c3 * ey, dt);

    // control moment for the roll/pitch dynamics - eq (31)
    Vector3 tau;
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
        M3 += -kyI * eIy.error;

    M << M1, M2, M3;

    fM(0) = f_total;
    fM.block<3, 1>(1, 0) = M;
    f_motor = fM_to_forces_inv * fM;

    // for saving:
    Matrix3 RdtR = command->Rd.transpose() * state->R;
    eR = 0.5 * vee(RdtR - RdtR.transpose());
    eIR.error << eI1.error, eI2.error, eIy.error;
    eW = state->W - state->R.transpose() * command->Rd * command->Wd;
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
    config_file->read("Integral.use_integral", use_integral);

    Vector3 temp_3x1;
    config_file->read("Control.kX", temp_3x1);
    kX(0, 0) = temp_3x1[0];
    kX(1, 1) = temp_3x1[1];
    kX(2, 2) = temp_3x1[2];

    config_file->read("Control.kV", temp_3x1);
    kV(0, 0) = temp_3x1[0];
    kV(1, 1) = temp_3x1[1];
    kV(2, 2) = temp_3x1[2];

    config_file->read("Integral.kIX", kIX);
    config_file->read("Integral.c1", c1);

    config_file->read("Control.kR", temp_3x1);
    kR(0, 0) = temp_3x1[0];
    kR(1, 1) = temp_3x1[1];
    kR(2, 2) = temp_3x1[2];

    config_file->read("Control.kW", temp_3x1);
    kW(0, 0) = temp_3x1[0];
    kW(1, 1) = temp_3x1[1];
    kW(2, 2) = temp_3x1[2];

    config_file->read("Integral.kIR", kIR);
    config_file->read("Integral.c2", c2);
    config_file->read("Integral.c3", c3);

    config_file->read("Integral.kyI", kyI); // yaw
    config_file->read("Integral.ki", ki);   //position
    config_file->read("Integral.kI", kI);   //roll & pitch

    config_file->read("Control.c_tf", c_tf);
    config_file->read("Control.l", l);

    config_file->read("UAV.J", J);
    config_file->read("UAV.m", m);
    config_file->read("UAV.g", g);


    Eigen::Matrix<double, 4, 4> fM_to_forces;
    fM_to_forces << 1.0, 1.0, 1.0, 1.0,
        0.0, -l, 0.0, l,
        l, 0.0, -l, 0.0,
        -c_tf, c_tf, -c_tf, c_tf;
    fM_to_forces_inv = fM_to_forces.inverse();
}


double fdcl::control::get_time()
{
    double t;
    clock_gettime(CLOCK_REALTIME, &tspec_curr);
    t = (double)tspec_curr.tv_sec + ((double)tspec_curr.tv_nsec) / 1.e9;
    t -= (double)tspec_init.tv_sec + ((double)tspec_init.tv_nsec) / 1.e9;
    return t;
}