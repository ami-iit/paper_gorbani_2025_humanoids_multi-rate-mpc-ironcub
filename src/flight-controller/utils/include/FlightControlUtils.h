#ifndef FLIGHT_CONTROL_UTILS_H
#define FLIGHT_CONTROL_UTILS_H

/**
 * @brief Utility functions used by the flight controller.
 *
 * Provides utility functions to be used in the flight controller.
 */

// YARP headers
#include <yarp/os/LogStream.h>
#include <yarp/os/Property.h>
#include <yarp/robotinterface/Types.h>
#include <yarp/robotinterface/XMLReader.h>
#include <yarp/sig/Vector.h>

// iDynTree headers
#include <Eigen/Dense>
#include <iDynTree/EigenHelpers.h>

// BLF headers
#include <BipedalLocomotion/ParametersHandler/YarpImplementation.h>

void addVectorOfStringToProperty(yarp::os::Property& prop,
                                 std::string key,
                                 const std::vector<std::string>& list);

yarp::os::Property paramsAsProperty(yarp::robotinterface::ParamList& params);

bool readXMLFile(
    const std::string& pathToFile,
    std::shared_ptr<BipedalLocomotion::ParametersHandler::YarpImplementation> parameterHandler);

void convertDegToRad(const yarp::sig::Vector& vecDeg, yarp::sig::Vector& vecRad);

void convertRadToDeg(const yarp::sig::Vector& vecRad, yarp::sig::Vector& vecDeg);

iDynTree::Matrix3x3 skewSymmetricFrom3DMatrix(const iDynTree::Matrix3x3& inputMatrix);

Eigen::Matrix3d fromVecToSkew(const Eigen::Vector3d& inputVect);

Eigen::Vector3d fromSkewToVec(const Eigen::Matrix3d& skewMatrix);

bool getParameterAndCheckSize(
    const std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler>& parametersHandler,
    const std::string& paramName,
    Eigen::Ref<Eigen::VectorXd> param,
    double size = 3);

/**
 * @brief Compute the rotation error between the desired rotation matrix and the current
 * rotation matrix as:
 *
 *   \f$ e_R = 0.5 * veeMap(R_{des}^T R_{curr} - R_{curr}^T R_{des}) \f$
 *
 * Valid when the equations that use this error as feedback are expressed in body coordinates.
 * @param R_des Desired rotation matrix.
 * @param R_curr Current rotation matrix.
 * @return The rotation error.
 */
Eigen::Vector3d computeRotationErrorInBodyCoord(const Eigen::Ref<Eigen::Matrix3d> R_des,
                                                const Eigen::Ref<Eigen::Matrix3d> R_curr);

/**
 * @brief Compute the rotation error between the desired rotation matrix and the current rotation
 * matrix as:
 * \f$ e_R = 0.5 * veeMap(R_{curr} R_{des}^T - R_{des} R_{curr}^T) \f$
 * @param R_des Desired rotation matrix.
 * @param R_curr Current rotation matrix.
 * @return The rotation error.
 */
Eigen::Vector3d computeRotationErrorInInertialCoord(const Eigen::Ref<Eigen::Matrix3d> R_des,
                                                    const Eigen::Ref<Eigen::Matrix3d> R_curr);

class LowPassFilter
{
private:
    double alpha; // Filter coefficient
    double prevY; // Previous output
    double cutoffFreq; // Cutoff frequency
    double samplingFreq; // Sampling frequency

    // Helper function to calculate the filter coefficient (alpha)
    void calculateAlpha();

public:
    // Constructor to initialize the filter with a cutoff frequency and sampling frequency
    LowPassFilter(double cutoffFreq, double samplingFreq);

    // Set a new cutoff frequency
    void setCutoffFrequency(double newCutoffFreq);

    // Set a new sampling frequency
    void setSamplingFrequency(double newSamplingFreq);

    // Process the input signal and return the filtered output
    double process(double input);

    // Reset the filter's state
    void reset();
};

#endif /* end of include guard FLIGHT_CONTROL_UTILS_H */
