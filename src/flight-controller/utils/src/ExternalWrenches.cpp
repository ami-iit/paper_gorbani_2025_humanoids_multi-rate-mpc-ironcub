#include "ExternalWrenches.h"

const bool ExternalWrenches::configure(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
    std::string prefixReadPortNameList)
{
    // Read the parameters from the configuration file.
    auto ptr = parametersHandler.lock();
    if (!ptr->getParameter("refFrameNameList", m_refFrameNames))
    {
        yError() << "Parameter 'refFrameNameList' not found in the config file.";
        return false;
    }
    if (!ptr->getParameter("srcPortNameList", m_srcPortName))
    {
        yError() << "Parameter 'srcPortNameList' not found in the config file.";
        return false;
    }
    if (!ptr->getParameter("readPortNameList", m_readPortName))
    {
        yError() << "Parameter 'readPortNameList' not found in the config file.";
        return false;
    }

    // Add the prefix to the read port names.
    for (auto& name : m_readPortName)
    {
        name = prefixReadPortNameList + name;
    }

    // Check that the number of elements in the lists is the same.
    if (m_refFrameNames.size() != m_srcPortName.size()
        || m_refFrameNames.size() != m_readPortName.size())
    {
        yError() << "The number of elements in the lists 'm_refFrameNames', 'm_srcPortName' and "
                    "'m_readPortName' must be the same.";
        return false;
    }

    // Resize m_values
    m_values.resize(m_refFrameNames.size());

    // Open the ports and connect them.
    for (size_t i = 0; i < m_refFrameNames.size(); i++)
    {
        auto port = std::make_unique<yarp::os::BufferedPort<yarp::os::Bottle>>();
        m_ports.emplace_back(std::move(port));
        yInfo() << "ExternalWrenches: Opening port " << m_readPortName[i] << " and connecting to "
                << m_srcPortName[i];
        if (!m_ports.back()->open(m_readPortName[i]))
        {
            yError() << "ExternalWrenches: Failed to open port " << m_readPortName[i];
            return false;
        }
        if (!yarp::os::Network::connect(m_srcPortName[i], m_readPortName[i]))
        {
            yError() << "ExternalWrenches: Failed to connect port " << m_readPortName[i] << " to "
                     << m_srcPortName[i];
            return false;
        }
    }
    return true;
}

const bool ExternalWrenches::read(const bool shouldWait)
{
    bool success = true;
    for (size_t i = 0; i < m_ports.size(); i++)
    {
        yarp::os::Bottle* bottle = m_ports[i]->read(shouldWait);
        if (bottle != nullptr)
        {
            m_values[i][0] = bottle->get(0).asFloat64();
            m_values[i][1] = bottle->get(1).asFloat64();
            m_values[i][2] = bottle->get(2).asFloat64();
            m_values[i][3] = bottle->get(3).asFloat64();
            m_values[i][4] = bottle->get(4).asFloat64();
            m_values[i][5] = bottle->get(5).asFloat64();
        } else
        {
            success = false;
        }
    }
    return success;
}

void ExternalWrenches::closePort()
{
    for (size_t i = 0; i < m_ports.size(); i++)
    {
        m_ports[i]->close();
    }
    yInfo() << "ExternalWrenches: Closed ports";
}

const std::vector<std::string>& ExternalWrenches::getRefFrameNames() const
{
    return m_refFrameNames;
}

const std::vector<Eigen::Vector6d>& ExternalWrenches::getValues() const
{
    return m_values;
}
