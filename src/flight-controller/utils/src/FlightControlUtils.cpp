#include "FlightControlUtils.h"

yarp::os::Property paramsAsProperty(yarp::robotinterface::ParamList& params)
{
    yarp::robotinterface::ParamList p = yarp::robotinterface::mergeDuplicateGroups(params);

    yarp::os::Property prop;

    for (yarp::robotinterface::ParamList::const_iterator it = p.begin(); it != p.end(); ++it)
    {
        const yarp::robotinterface::Param& param = *it;

        // check if parentheses are balanced
        std::string stringFormatValue = param.value();
        int counter = 0;
        for (size_t i = 0; i < stringFormatValue.size() && counter >= 0; i++)
        {
            if (stringFormatValue[i] == '(')
            {
                counter++;
            } else if (stringFormatValue[i] == ')')
            {
                counter--;
            }
        }
        if (counter != 0)
        {
            yWarning() << "Parentheses not balanced for param " << param.name();
        }

        std::string s = "(" + param.name() + " " + param.value() + ")";
        prop.fromString(s, false);
    }
    return prop;
}

bool readXMLFile(
    const std::string& pathToFile,
    std::shared_ptr<BipedalLocomotion::ParametersHandler::YarpImplementation> parameterHandler)
{
    yarp::robotinterface::XMLReader reader;
    yarp::robotinterface::XMLReaderResult result = reader.getRobotFromFile(pathToFile);
    if (!result.parsingIsSuccessful)
    {
        yError() << "Could not open file " << pathToFile;
        return false;
    }
    yarp::robotinterface::ParamList params
        = result.robot.device("flight_control_cpp_config").params();
    yInfo() << "Printing the values of the XML file";
    for (auto param : params)
    {
        std::string name = param.name();
        std::string value = param.value();
        yInfo() << "Name: " << name << " Value: " << value;
    }
    yInfo() << "Read the XML file correctly!!";
    parameterHandler->set(paramsAsProperty(params));
    return true;
}

void addVectorOfStringToProperty(yarp::os::Property& prop,
                                 std::string key,
                                 const std::vector<std::string>& list)
{
    prop.addGroup(key);

    yarp::os::Bottle& bot = prop.findGroup(key).addList();

    for (size_t i = 0; i < list.size(); i++)
    {
        bot.addString(list[i].c_str());
    }
    return;
}

void convertDegToRad(const yarp::sig::Vector& vecDeg, yarp::sig::Vector& vecRad)
{
    if (vecDeg.size() != vecRad.size())
    {
        yError() << "convertDegToRad: wrong vector size";
        return;
    }

    for (size_t i = 0; i < vecDeg.size(); i++)
    {
        vecRad[i] = (M_PI / 180.0) * vecDeg[i];
    }
}

void convertRadToDeg(const yarp::sig::Vector& vecRad, yarp::sig::Vector& vecDeg)
{
    if (vecDeg.size() != vecRad.size())
    {
        yError() << "convertRadToDeg: wrong vector size";
        return;
    }

    for (size_t i = 0; i < vecRad.size(); i++)
    {
        vecDeg[i] = (180.0 / M_PI) * vecRad[i];
    }
}

iDynTree::Matrix3x3 skewSymmetricFrom3DMatrix(const iDynTree::Matrix3x3& inputMatrix)
{
    // Given a 3x3 matrix, extracts its skew-symmetric part
    iDynTree::Matrix3x3 outputMatrix;

    iDynTree::toEigen(outputMatrix)
        = 0.5 * (iDynTree::toEigen(inputMatrix) - iDynTree::toEigen(inputMatrix).transpose());
    return outputMatrix;
}

Eigen::Matrix3d fromVecToSkew(const Eigen::Vector3d& inputVect)
{
    Eigen::Matrix3d skew(3, 3);

    skew << 0, -inputVect(2), inputVect(1), inputVect(2), 0, -inputVect(0), -inputVect(1),
        inputVect(0), 0;

    return skew;
}

Eigen::Vector3d fromSkewToVec(const Eigen::Matrix3d& skewMatrix)
{
    // Given a 3x3 skew matrix, extracts its elements in a 3x1 vector
    Eigen::Vector3d outputVector;

    outputVector(0) = -skewMatrix(1, 2);
    outputVector(1) = skewMatrix(0, 2);
    outputVector(2) = -skewMatrix(0, 1);

    return outputVector;
}

bool getParameterAndCheckSize(
    const std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler>& parametersHandler,
    const std::string& paramName,
    Eigen::Ref<Eigen::VectorXd> param,
    double size)
{
    auto ptr = parametersHandler.lock();
    if (!ptr->getParameter(paramName, param))
    {
        yError() << "Parameter '" << paramName << "' not found in the config file.";
        return false;
    }
    if (param.size() != size)
    {
        yError() << "The size of the vector is not correct.";
        return false;
    }
    return true;
}

Eigen::Vector3d computeRotationErrorInBodyCoord(const Eigen::Ref<Eigen::Matrix3d> R_des,
                                                const Eigen::Ref<Eigen::Matrix3d> R_curr)
{
    // Given two rotation matrices, computes the error between them. Valid when the equations
    // that use this error as feedback are expressed in body coordinates.
    Eigen::Vector3d error;

    error = 0.5 * fromSkewToVec(R_des.transpose() * R_curr - R_curr.transpose() * R_des);

    return error;
}

Eigen::Vector3d computeRotationErrorInInertialCoord(const Eigen::Ref<Eigen::Matrix3d> R_des,
                                                    const Eigen::Ref<Eigen::Matrix3d> R_curr)
{
    // Given two rotation matrices, computes the error between them
    Eigen::Vector3d error;

    error = 0.5 * fromSkewToVec(R_curr * R_des.transpose() - R_des * R_curr.transpose());

    return error;
}

// Helper function to calculate the filter coefficient (alpha)
void LowPassFilter::calculateAlpha()
{
    double dt = 1.0 / samplingFreq;
    double RC = 1.0 / (2 * M_PI * cutoffFreq);
    alpha = dt / (RC + dt);
}

// Constructor to initialize the filter with a cutoff frequency and sampling frequency
LowPassFilter::LowPassFilter(double cutoffFreq, double samplingFreq)
    : cutoffFreq(cutoffFreq)
    , samplingFreq(samplingFreq)
    , prevY(0.0)
{
    calculateAlpha();
}

// Set a new cutoff frequency and recalculate alpha
void LowPassFilter::setCutoffFrequency(double newCutoffFreq)
{
    cutoffFreq = newCutoffFreq;
    calculateAlpha();
}

// Set a new sampling frequency and recalculate alpha
void LowPassFilter::setSamplingFrequency(double newSamplingFreq)
{
    samplingFreq = newSamplingFreq;
    calculateAlpha();
}

// Process the input signal and return the filtered output
double LowPassFilter::process(double input)
{
    double output = alpha * input + (1 - alpha) * prevY;
    prevY = output; // Store the output for the next step
    return output;
}

// Reset the filter's state
void LowPassFilter::reset()
{
    prevY = 0.0;
}
