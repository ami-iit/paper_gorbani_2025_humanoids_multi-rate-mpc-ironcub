#ifndef EXTERNAL_WRENCH_H
#define EXTERNAL_WRENCH_H

#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>
#include <iDynTree/EigenHelpers.h>
#include <yarp/os/Bottle.h>
#include <yarp/os/BufferedPort.h>
#include <yarp/os/Log.h>
#include <yarp/sig/all.h>

namespace Eigen
{
using Vector6d = Eigen::Matrix<double, 6, 1>;
}

/**
 * @class ExternalWrenches
 * @brief Class to read external wrenches from YARP ports.
 *
 * This class provides functionality to add, read, and close ports for reading external wrenches.
 */
class ExternalWrenches
{
public:
    /**
     * @brief Configure the wrenches by opening the ports.
     * @param parametersHandler Reference to the ParametersHandler object (to access configuration
     * parameters).
     * @param prefixReadPortNameList Prefix for the read port names. Default is an empty string.
     * @return True if the wrenches were successfully configured, false otherwise.
     */
    const bool configure(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        std::string prefixReadPortNameList = "");

    /**
     * @brief Update the wrenches by reading from the ports.
     * @param shouldWait If true, the function will wait for new data to be available. Default is
     * false.
     * @return True if the wrenches were successfully updated, false otherwise.
     */
    const bool read(const bool shouldWait = false);

    /**
     * @brief Close the ports used to read the wrenches.
     * @return nothing.
     */
    void closePort();

    /**
     * @brief Get the reference frame names associated with the wrenches.
     * @return A reference to the vector of reference frame names.
     */
    const std::vector<std::string>& getRefFrameNames() const;

    /**
     * @brief Get the wrench values.
     * @return A reference to the vector of wrench values.
     */
    const std::vector<Eigen::Vector6d>& getValues() const;

private:
    std::vector<std::unique_ptr<yarp::os::BufferedPort<yarp::os::Bottle>>> m_ports; /**< Vector of
                                                                                       ports for
                                                                                       reading
                                                                                       wrenches. */
    std::vector<Eigen::Vector6d> m_values; /**< Vector of wrench values. */
    std::vector<std::string> m_refFrameNames; /**< Vector of reference frame names. */
    std::vector<std::string> m_srcPortName; /**< Vector of source port names. */
    std::vector<std::string> m_readPortName; /**< Vector of read port names. */
};

#endif /* end of include guard EXTERNAL_WRENCHES_H */
