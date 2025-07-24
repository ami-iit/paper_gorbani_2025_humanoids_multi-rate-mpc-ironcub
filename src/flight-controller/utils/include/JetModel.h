/**
 * @file JetModel.h
 * @author Gabriele Nava
 * @author Hosameldin Mohamed
 * @date 2024
 */
#ifndef JET_MODEL_H
#define JET_MODEL_H

#include <Eigen/Dense>
#include <memory>
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
    /**
     * @brief Configure Models with param from config file
     * @param config
     * @return None.
     */
    bool configModels(yarp::os::Searchable& config);
    /**
     * @brief Get the scalar value from the configuration file.
     * @param config configuration parameters;
     * @param scalarName name of the scalar parameter;
     * @param scalar pointer to the scalar value.
     * @return True if the scalar value is found in the configuration file, false otherwise.
     */
    bool getCoeffFromConfig(yarp::os::Searchable& config,
                            std::string coeffName,
                            std::vector<double>* coeff);
    /**
     * @brief Get the scalar value from the configuration file.
     * @param config configuration parameters;
     * @param scalarName name of the scalar parameter;
     * @param scalar pointer to the scalar value.
     * @return True if the scalar value is found in the configuration file, false otherwise.
     */
    bool getScalarFromConfig(yarp::os::Searchable& config, std::string scalarName, double* scalar);
    /**
     * @brief Get the TMax parameter value.
     * @param TMax pointer to the TMax value.
     * @return None.
     */
    void getTMax(double* TMax);
    /**
     * @brief Get the TMin parameter value.
     * @param TMin pointer to the TMin value.
     * @return None.
     */
    void getTMin(double* TMin);

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

    /* dt */

    /**
     * @brief Set the value of the sampling period dt
     * @return None.
     */
    void setDt(double dt);
    /**
     * @brief Get the value of the sampling period dt
     * @return None.
     */
    void getDt(double* dt);

    /* T2RPM and RPM2T models */

    /**
     * @brief Convert thrust to RPM.
     * @return RPM value.
     */
    double T2RPM(double thrust);
    /**
     * @brief Convert RPM to thrust.
     * @return thrust value.
     */
    double RPM2T(double krpm);
    /**
     * @brief Standardize the thrust value for the RPM2T model.
     * @return None.
     */
    double standardizeThrust_RPM2T(double thrust);
    /**
     * @brief Standardize the RPM value for the RPM2T model.
     * @return None.
     */
    double standardizeRPM_RPM2T(double krpm);
    /**
     * @brief Denormalize the thrust value for the RPM2T model.
     * @return None.
     */
    double destandardizeThrust_RPM2T(double thrustBar);
    /**
     * @brief Denormalize the krpm value for the RPM2T model.
     * @return None.
     */
    double destandardizeRPM_RPM2T(double krpmBar);

    /* RPM2u model */

    /**
     * @brief Convert RPM to throttle.
     * @return Throttle value.
     */
    double RPM2u(double krpm);

    /* T2u model */

    /**
     * @brief Convert thrust to throttle.
     * @return Throttle value.
     */
    double T2u(double thrust);
    /**
     * @brief Standardize the thrust value for the T2u model.
     * @return None.
     */
    double standardizeThrust_T2u(double thrust);
    /**
     * @brief Standardize the throttle value for the T2u model.
     * @return None.
     */
    double standardizeThrottle_T2u(double throttle);
    /**
     * @brief Denormalize the throttle value for the T2u model.
     * @return None.
     */
    double destandardizeThrottle_T2u(double throttleBar);
    /**
     * @brief Get the thrust standard deviation value for the T2u model.
     * @return thrust standard deviation value.
     */
    double getThrustStandardDeviation_T2u();

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
     * @brief Performs 1-step prediction given the state x
     * @return next value of the state xnext.
     */
    Eigen::VectorXd predict(Eigen::VectorXd x, double u);
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
    double m_TMax, m_TMin;
    double m_airTemperature;
    double m_ambientCorrection;
    std::vector<double> m_ambientCoeff;
    std::vector<double> m_rpm2TCoeff;
    std::vector<double> m_u2rpmCoeff;
    std::vector<double> m_T2uCoeff;
    std::vector<double> m_u2TCoeff;
    std::vector<double> m_u2Tnormalization;
    std::vector<double> m_T2u_normalization;
    std::vector<double> m_RPM2Tnormalization;
    // time step
    double m_dt;
    // no of states
    int m_nStates{2};
    // unscented kalman Filter class
    friend class UnscentedKF;
};

#endif /* end of include guard JET_MODEL_H */
