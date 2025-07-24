#include "GainScheduling.h"

bool GainScheduling::setParams(double initialValue,
                               double finalValue,
                               double inputTriggerValue,
                               double deltaTime,
                               double period)
{
    m_gain = initialValue;
    m_finalGain = finalValue;
    m_inputTriggerValue = inputTriggerValue;
    // if deltaTime is too small, the slope is calculated based on the period
    if (deltaTime < m_period)
    {
        m_slope = (finalValue - initialValue) / m_period;
    } else
    {
        m_slope = (finalValue - initialValue) / deltaTime;
    }
    m_period = period;
    return true;
}

void GainScheduling::updateGain(double input)
{
    bool isRampFinished = false;
    if (m_slope > 0)
    {
        isRampFinished = (m_gain >= m_finalGain);
    } else
    {
        isRampFinished = (m_gain <= m_finalGain);
    }
    // Update gain
    if ((input > m_inputTriggerValue) && (!isRampFinished))
    {
        m_gain += m_slope * m_period;
    }
    // Check if gain is within limits
    if ((m_slope > 0) && (m_gain > m_finalGain))
    {
        m_gain = m_finalGain;
    } else if ((m_slope < 0) && (m_gain < m_finalGain))
    {
        m_gain = m_finalGain;
    }
}

double GainScheduling::getGain()
{
    return m_gain;
}

GainSchedulingVector::GainSchedulingVector()
{
}

bool GainSchedulingVector::setParams(const std::vector<double>& initialValue,
                                     const std::vector<double>& finalValue,
                                     double inputTriggerValue,
                                     double deltaTime,
                                     double period)
{
    if (initialValue.size() != finalValue.size())
    {
        yError("Initial and final values of the gains must have the same size");
        return false;
    }
    m_gainsVect.resize(initialValue.size());
    m_gainMatrix.resize(initialValue.size(), initialValue.size());
    for (int i = 0; i < initialValue.size(); i++)
    {
        m_gainsVect[i].setParams(initialValue[i],
                                 finalValue[i],
                                 inputTriggerValue,
                                 deltaTime,
                                 period);
    }
    return true;
}

void GainSchedulingVector::updateGainMatrix(double input)
{
    m_gainMatrix.setZero();
    for (int i = 0; i < m_gainsVect.size(); i++)
    {
        m_gainsVect[i].updateGain(input);
        m_gainMatrix(i, i) = m_gainsVect[i].getGain();
    }
}

Eigen::MatrixXd GainSchedulingVector::getGainMatrix()
{
    return m_gainMatrix;
}
