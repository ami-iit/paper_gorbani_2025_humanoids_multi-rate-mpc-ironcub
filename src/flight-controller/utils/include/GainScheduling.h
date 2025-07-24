#ifndef GAIN_SCHEDULING_H
#define GAIN_SCHEDULING_H

#include <Eigen/Dense>
#include <vector>
#include <yarp/os/Log.h>

/**
 * @brief Class to implement a gain scheduling algorithm for a scalar gain; the gain is increased or
 * decreased based on the input value.
 */
class GainScheduling
{
public:
    /**
     * @brief Set the parameters for the gain scheduling algorithm
     * @param initialValue Initial value of the gain
     * @param finalValue Final value of the gain
     * @param inputTriggerValue Starting value of the input at which the gain starts to increase or
     * decrease
     * @param deltaTime Time interval to change the gain
     * @param period Period of the gain scheduling algorithm
     * @return True if the parameters are set correctly
     */
    bool setParams(double initialValue,
                   double finalValue,
                   double inputTriggerValue,
                   double deltaTime,
                   double period);

    /**
     * @brief Update the gain value, to be called at each iteration before getting the gain
     * @param input Input value to trigger the gain scheduling
     */
    void updateGain(double input);

    /**
     * @brief Get the current gain value
     * @return Current gain value
     */
    double getGain();

private:
    double m_finalGain; /* Final value of the gain */
    double m_inputTriggerValue; /* Starting value of the input at which the gain starts to increase
                                   or decrease */
    double m_slope; /* Slope of the gain scheduling algorithm */
    double m_period; /* Period of the gain scheduling algorithm */
    double m_gain; /* Current value of the gain */
};

/**
 * @brief Class to implement a gain scheduling algorithm for a vector of values; the gain is
 * increased or decreased based on the input value. A diagonal matrix is returned as the gain
 * matrix.
 */
class GainSchedulingVector
{
public:
    GainSchedulingVector(); // Constructor

    /**
     * @brief Set the parameters for the gain scheduling algorithm
     * @param initialValue Pointer to the std::vector<double> containing the initial value of the
     * gains
     * @param finalValue Pointer to the std::vector<double> containing the final value of the gains
     * @param size Size of the vector of the initialValue and finalValue vectors
     * @param inputTriggerValue Starting value of the input at which the gain starts to increase or
     * decrease
     * @param deltaTime Time interval to change the gain
     * @param period Period of the gain scheduling algorithm
     * @return True if the parameters are set correctly
     */
    bool setParams(const std::vector<double>& initialValue,
                   const std::vector<double>& finalValue,
                   double inputTriggerValue,
                   double deltaTime,
                   double period);

    /**
     * @brief Update the gain matrix, to be called at each iteration before getting the gain matrix
     * @param input Input value to trigger the gain scheduling
     */
    void updateGainMatrix(double input);

    /**
     * @brief Get the current gain matrix
     * @return Current gain matrix
     */
    Eigen::MatrixXd getGainMatrix();

private:
    std::vector<GainScheduling> m_gainsVect; /* Vector of GainScheduling objects */
    Eigen::MatrixXd m_gainMatrix; /* Gain matrix */
};

#endif // GAIN_SCHEDULING_H
