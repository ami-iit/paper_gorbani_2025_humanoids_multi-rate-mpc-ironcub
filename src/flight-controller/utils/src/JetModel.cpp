/**
 * @file JetModel.cpp
 * @author Gabriele Nava
 * @author Hosameldin Mohamed
 * @date 2024
 */
#include "JetModel.h"
#include <cmath>
#include <iostream>
#include <sstream>
#include <yarp/os/Bottle.h>
#include <yarp/os/Log.h>
#include <yarp/os/LogStream.h>
#include <yarp/os/Searchable.h>

using namespace Eigen::numext;

JetModel::JetModel()
{
    // initialize the jet models
    m_dt = 0.02;
    m_TMax = 237.63;
    m_TMin = 10.84;
    m_airTemperature = 20;
    m_ambientCoeff = {-0.00621661, 1.13212876};
    m_rpm2TCoeff = {2.749e-4, -0.03007, 1.958, -33.08};
    m_u2rpmCoeff = {8.2, 0.5, 35};
    m_T2uCoeff = {0.8130, -4.4723, 8.3406, -7.1713, 2.7883, 0.7017, 0.0};
    m_u2TCoeff = {-0.02567311,
                  -0.48316774,
                  -0.70734742,
                  0.218006,
                  -0.01652024,
                  0.00704737,
                  0.50263685,
                  0.090101,
                  -0.13025216,
                  -0.04018778,
                  -0.00632942,
                  -0.31548423,
                  -0.07279471};
    m_u2Tnormalization
        = {101.72479750418607, 69.3478685933187, 44.4176940196032, 33.64622030338592};
    m_T2u_normalization = {83.611, 77.589, 34.115, 35.933};
    m_RPM2Tnormalization = {83.611, 77.589, 70.923, 31.452};
    m_ambientCorrection = m_ambientCoeff[0] * m_airTemperature + m_ambientCoeff[1];
}

bool JetModel::configModels(yarp::os::Searchable& config)
{
    bool ok = getCoeffFromConfig(config, "T2u_coefficients", &m_T2uCoeff);
    ok = ok && getCoeffFromConfig(config, "u2T_coefficients", &m_u2TCoeff);
    ok = ok && getCoeffFromConfig(config, "u2T_normalization", &m_u2Tnormalization);
    ok = ok && getCoeffFromConfig(config, "T2u_normalization", &m_T2u_normalization);
    ok = ok && getCoeffFromConfig(config, "RPM2T_normalization", &m_RPM2Tnormalization);
    ok = ok && getCoeffFromConfig(config, "rpm2T_coeff", &m_rpm2TCoeff);
    ok = ok && getCoeffFromConfig(config, "u2rpm_coeff", &m_u2rpmCoeff);
    ok = ok && getCoeffFromConfig(config, "ambient_correction_coefficients", &m_ambientCoeff);
    ok = ok && getScalarFromConfig(config, "T_max", &m_TMax);
    ok = ok && getScalarFromConfig(config, "T_min", &m_TMin);
    ok = ok && getScalarFromConfig(config, "air_temperature", &m_airTemperature);
    ok = ok && getScalarFromConfig(config, "dt", &m_dt);

    if (!ok)
    {
        yError() << "JetModel ::"
                 << "Error parsing parameters from configuration file.";
    }

    m_ambientCorrection = m_ambientCoeff[0] * m_airTemperature + m_ambientCoeff[1];

    return ok;
}

bool JetModel::getCoeffFromConfig(yarp::os::Searchable& config,
                                  std::string coeffName,
                                  std::vector<double>* coeff)
{
    yarp::os::Bottle* coeffList = config.find(coeffName).asList();
    if (coeffList == 0)
    {
        yError() << "JetModel ::"
                 << "Error parsing parameter:" << coeffName;
        return false;
    } else
    {
        coeff->clear();
        std::stringstream ss;
        for (int i = 0; i < coeffList->size(); i++)
        {
            coeff->push_back(coeffList->get(i).asFloat64());
            ss << coeff->at(i) << " ";
        }
        yDebug() << "JetModel ::"
                 << "Parsing parameter:" << coeffName << "=\n"
                 << ss.str();
        return true;
    }
}

bool JetModel::getScalarFromConfig(yarp::os::Searchable& config,
                                   std::string scalarName,
                                   double* scalar)
{
    if (config.find(scalarName).isFloat64() || config.find(scalarName).isInt32())
    {
        *scalar = config.find(scalarName).asFloat64();
        yDebug() << "JetModel ::"
                 << "Parsing parameter:" << scalarName << "=" << *scalar;
        return true;
    } else
    {
        yError() << "JetModel ::"
                 << "Error parsing parameter:" << scalarName;
        return false;
    }
}

void JetModel::getTMax(double* TMax)
{
    *TMax = m_TMax;
}

void JetModel::getTMin(double* TMin)
{
    *TMin = m_TMin;
}

/* Ambient correction*/

void JetModel::setAirTemperature(double airTemperature)
{
    m_airTemperature = airTemperature;
    m_ambientCorrection = m_ambientCoeff[0] * m_airTemperature + m_ambientCoeff[1];
}

double JetModel::getAmbientCorrection()
{
    return m_ambientCorrection;
}

void JetModel::getAirTemperature(double* airTemperature)
{
    *airTemperature = m_airTemperature;
}

/* dt */

void JetModel::setDt(double dt)
{
    m_dt = dt;
}

void JetModel::getDt(double* dt)
{
    *dt = m_dt;
}

/* T2RPM and RPM2T models */

double JetModel::T2RPM(double thrust)
{
    double thrustBar = standardizeThrust_RPM2T(thrust);
    // Computing the observation model using inverse RPM2T model
    // the 'compute' function accept the coefficient of the polynomial in increasing order
    Eigen::Vector4d coeff((m_rpm2TCoeff[3] - thrustBar),
                          m_rpm2TCoeff[2],
                          m_rpm2TCoeff[1],
                          m_rpm2TCoeff[0]);
    Eigen::PolynomialSolver<double, Eigen::Dynamic> solver;
    solver.compute(coeff);
    // computing the roots of a * krpmBar**3 + b * krpmBar**2 + c * krpmBar + (d - thrustBar) = 0
    const Eigen::PolynomialSolver<double, Eigen::Dynamic>::RootsType& r = solver.roots();

    double krpmBar = 35;

    for (size_t i = 0; i < 3; i++)
    {
        // selecting the real roots
        if (r[i].imag() < 1e-6 && r[i].imag() > -1e-6)
        {
            krpmBar = r[i].real();
        }
    }
    return destandardizeRPM_RPM2T(krpmBar);
}

double JetModel::RPM2T(double krpm)
{
    double krpmBar = standardizeRPM_RPM2T(krpm);
    // computing thrustBar = a * krpmBar**3 + b * krpmBar**2 + c * krpmBar + d
    double thrustBar = m_rpm2TCoeff[0] * pow(krpmBar, 3.0) + m_rpm2TCoeff[1] * pow(krpmBar, 2.0)
                       + m_rpm2TCoeff[2] * krpmBar + m_rpm2TCoeff[3];

    return destandardizeThrust_RPM2T(thrustBar);
}

double JetModel::standardizeThrust_RPM2T(double thrust)
{
    return (thrust / m_ambientCorrection - m_RPM2Tnormalization[0]) / m_RPM2Tnormalization[1];
}

double JetModel::standardizeRPM_RPM2T(double krpm)
{
    return (krpm - m_RPM2Tnormalization[2]) / m_RPM2Tnormalization[3];
}

double JetModel::destandardizeThrust_RPM2T(double thrustBar)
{
    // destandardize thrust value
    return (thrustBar * m_RPM2Tnormalization[1] + m_RPM2Tnormalization[0]) * m_ambientCorrection;
}

double JetModel::destandardizeRPM_RPM2T(double krpmBar)
{
    // destandardize krpm
    return krpmBar * m_RPM2Tnormalization[3] + m_RPM2Tnormalization[2];
}

/* RPM2u model */

double JetModel::RPM2u(double krpm)
{
    return pow(((krpm - m_u2rpmCoeff[2]) / m_u2rpmCoeff[0]), 1.0 / m_u2rpmCoeff[1]);
}

/* T2u model */

double JetModel::T2u(double thrust)
{
    double thrustBar = standardizeThrust_T2u(thrust);
    // u_tilde and T_tilde are the non-dimensional throttle and thrust
    double u_tilde = 0;
    double power = 1.0;
    int n = m_T2uCoeff.size();
    for (int i = 0; i < n; i++)
    {
        // u_tilde = a6 + a5 * T_tilde + a4 * T_tilde**2 + a3 * T_tilde**3 + a2 * T_tilde**4 + a1 *
        // T_tilde**5 + a0 * T_tilde**6
        u_tilde += m_T2uCoeff[n - i - 1] * power;
        power *= thrustBar;
    }
    // destandardize the throttle
    return destandardizeThrottle_T2u(u_tilde);
}

double JetModel::standardizeThrust_T2u(double thrust)
{
    return (thrust / m_ambientCorrection - m_T2u_normalization[0]) / m_T2u_normalization[1];
}

double JetModel::standardizeThrottle_T2u(double throttle)
{
    return (throttle - m_T2u_normalization[2]) / m_T2u_normalization[3];
}

double JetModel::destandardizeThrottle_T2u(double throttle)
{
    // destandardize the throttle
    double throttleBar = throttle * m_T2u_normalization[3] + m_T2u_normalization[2];
    // check if the throttle is within the limits
    if (throttleBar < 0)
    {
        throttleBar = 0;
    } else if (throttleBar > 100)
    {
        throttleBar = 100;
    }
    return throttleBar;
}

double JetModel::getThrustStandardDeviation_T2u()
{
    return m_T2u_normalization[1];
}

/* u2T model */

double JetModel::compute_f(double T, double Tdot)
{
    return m_u2TCoeff[0] + m_u2TCoeff[1] * T + m_u2TCoeff[2] * Tdot + m_u2TCoeff[3] * T * Tdot
           + m_u2TCoeff[4] * pow(T, 2.0) + m_u2TCoeff[5] * pow(Tdot, 2.0);
}

double JetModel::compute_df_dT(double T, double Tdot)
{
    return m_u2TCoeff[1] + m_u2TCoeff[3] * Tdot + 2 * m_u2TCoeff[4] * T;
}

double JetModel::compute_df_dTdot(double T, double Tdot)
{
    return m_u2TCoeff[2] + m_u2TCoeff[3] * T + 2 * m_u2TCoeff[5] * Tdot;
}

double JetModel::compute_dg_dT(double T, double Tdot)
{
    return m_u2TCoeff[7] + m_u2TCoeff[9] * Tdot + 2 * m_u2TCoeff[10] * T;
}

double JetModel::compute_dg_dTdot(double T, double Tdot)
{
    return m_u2TCoeff[8] + m_u2TCoeff[9] * T + 2 * m_u2TCoeff[11] * Tdot;
}

double JetModel::compute_g(double T, double Tdot)
{
    return m_u2TCoeff[6] + m_u2TCoeff[7] * T + m_u2TCoeff[8] * Tdot + m_u2TCoeff[9] * T * Tdot
           + m_u2TCoeff[10] * pow(T, 2.0) + m_u2TCoeff[11] * pow(Tdot, 2.0);
}

double JetModel::compute_v(double u)
{
    return u + m_u2TCoeff[12] * pow(u, 2.0);
}

double JetModel::Tddot(double T, double Tdot, double u)
{
    return compute_f(T, Tdot) + compute_g(T, Tdot) * compute_v(u);
}

Eigen::VectorXd JetModel::predict(Eigen::VectorXd x, double u)
{
    // normalize
    x(0) = standardizeThrust_u2T(x(0));
    x(1) = standardizeThrustDot_u2T(x(1));
    u = standardizeThrottle_u2T(u);

    Eigen::VectorXd xnext(m_nStates);

    xnext << x(0) + x(1) * m_dt + 0.5 * Tddot(x(0), x(1), u) * pow(m_dt, 2.0),
        x(1) + Tddot(x(0), x(1), u) * m_dt;

    // denormalize
    xnext(0) = destandardizeThrust_u2T(xnext(0));
    xnext(1) = destandardizeThrustDot_u2T(xnext(1));

    return xnext;
}

double JetModel::standardizeThrust_u2T(double thrust)
{
    return (thrust / m_ambientCorrection - m_u2Tnormalization[0]) / m_u2Tnormalization[1];
}

double JetModel::standardizeThrustDot_u2T(double thrustDot)
{
    return (thrustDot / m_ambientCorrection) / m_u2Tnormalization[1];
}

double JetModel::standardizeThrottle_u2T(double throttle)
{
    return (throttle - m_u2Tnormalization[2]) / m_u2Tnormalization[3];
}

double JetModel::destandardizeThrust_u2T(double thrustBar)
{
    // destandardize thrust value
    return (thrustBar * m_u2Tnormalization[1] + m_u2Tnormalization[0]) * m_ambientCorrection;
}

double JetModel::destandardizeThrustDot_u2T(double thrustDotBar)
{
    // destandardize thrust derivative value
    return thrustDotBar * m_u2Tnormalization[1] * m_ambientCorrection;
}

double JetModel::destandardizeThrottle_u2T(double v)
{
    double u;
    // solve the equation Buu * u^2 + u - v = 0
    u = (-1 + sqrt(1 + 4 * m_u2TCoeff[12] * v)) / (2 * m_u2TCoeff[12]);
    // destandardize the throttle
    u = u * m_u2Tnormalization[3] + m_u2Tnormalization[2];
    // check if the throttle is within the limits
    if (u < 0)
    {
        u = 0;
    } else if (u > 100)
    {
        u = 100;
    }
    return u;
}

double JetModel::getThrustStandardDeviation_u2T()
{
    return m_u2Tnormalization[1];
}
