/**
 * @file JetModel.h
 * @author Gabriele Nava
 * @author Hosameldin Mohamed
 * @date 2024
 */
#ifndef JET_MODEL_H
#define JET_MODEL_H

#include <Eigen/Dense>
#include <unsupported/Eigen/Polynomials>
#include <yarp/os/Searchable.h>

/**
 * @class JetModel
 * @brief The prediction and observation models for jet thrust estimation.
 */
class JetModel
{
public:
    /**
     * @brief Construct a JetModel object with inputs.
     */
    JetModel();

    /* Ambient correction*/

    /**
     * @brief Set the air temperature parameter value.
     * @return None.
     */
    void setAirTemperature(double airTemperature);
    /**
     * @brief Get the air temperature parameter value.
     * @return None.
     */
    void getAirTemperature(double* airTemperature);
    /**
     * @brief Get the ambient correction value.
     * @return ambient correction value.
     */
    double getAmbientCorrection();

    /* u2T model */

    /**
     * @brief Compute the value of the function f given T and Tdot
     * refer to
     * https://github.com/ami-iit/element_sim2real-bridge/issues/109#issuecomment-2468338870
     * @return f value.
     */
    double compute_f(double T, double Tdot);
    /**
     * @brief Compute the value of the function g given T and Tdot
     * refer to
     * https://github.com/ami-iit/element_sim2real-bridge/issues/109#issuecomment-2468338870
     * @return g value.
     */
    double compute_g(double T, double Tdot);
    /**
     * @brief Compute the value of the function v given u
     * refer to
     * https://github.com/ami-iit/element_sim2real-bridge/issues/109#issuecomment-2468338870
     * @return v value.
     */
    double compute_v(double u);
    /**
     * @brief Compute Tddot from the throttle u applying the identified dynamics.
     * @return Tddot value.
     */
    double Tddot(double T, double Tdot, double u);
    /**
     * @brief Standardize the thrust value for the u2T model.
     * @return None.
     */
    double standardizeThrust_u2T(double thrust);
    /**
     * @brief Standardize the thrustDot value for the u2T model.
     * @return None.
     */
    double standardizeThrustDot_u2T(double thrustDot);
    /**
     * @brief Standardize the throttle value for the u2T model.
     * @return None.
     */
    double standardizeThrottle_u2T(double throttle);
    /**
     * @brief destandardize the thrust value for the u2T model.
     * @return None.
     */
    double destandardizeThrust_u2T(double thrustBar);
    /**
     * @brief destandardize the thrust derivative value for the u2T model.
     * @return None.
     */
    double destandardizeThrustDot_u2T(double thrustDotBar);
    /**
     * @brief Denormalize the throttle value for the u2T model.
     * @return None.
     */
    double destandardizeThrottle_u2T(double v);
    /**
     * @brief Get the thrust standard deviation value for the u2T model.
     * @return thrust standard deviation value.
     */
    double getThrustStandardDeviation_u2T();
    /**
     * compute the partial derivative of f with respect to T
     * @return df_dT value.
     */
    double compute_df_dT(double T, double Tdot);
    /**
     * compute the partial derivative of f with respect to Tdot
     * @return df_dTdot value.
     */
    double compute_df_dTdot(double T, double Tdot);
    /**
     * compute the partial derivative of g with respect to T
     * @return dg_dT value.
     */
    double compute_dg_dT(double T, double Tdot);
    /**
     * compute the partial derivative of g with respect to Tdot
     * @return dg_dTdot value.
     */
    double compute_dg_dTdot(double T, double Tdot);

private:
    // parameters of models
    double m_airTemperature;
    double m_ambientCorrection;
    std::vector<double> m_ambientCoeff;
    std::vector<double> m_u2TCoeff;
    std::vector<double> m_u2Tnormalization;
};

#endif /* end of include guard JET_MODEL_H */
