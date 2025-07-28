#include <fstream>
#include <variableSamplingMPC/VSconstant.h>
#include <variableSamplingMPC/constraintsVSMPC.h>

ConstraintSystemDynamicVS::ConstraintSystemDynamicVS(
    const int nVar, const int nStates, const int nJoints, const int nThrottle, const int nIter)
    : IQPConstraint(nVar, nStates * nIter)
{
    m_systemDynamicVS = std::make_unique<SystemDynamicVS>(nStates, nJoints, nThrottle);
    m_A.resize(nStates, nStates);
    m_BJoints.resize(nStates, nJoints);
    m_BThrottle.resize(nStates, nThrottle);
    m_c.resize(nStates);
    m_nStates = nStates;
    m_nJoints = nJoints;
    m_nThrottle = nThrottle;
    m_nIter = nIter;
}

const bool ConstraintSystemDynamicVS::readConfigParameters(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
    QPInput& qpInput)
{
    auto ptr = parametersHandler.lock();
    if (!ptr->getParameter("nIterSmall", m_nSmallSteps))
    {
        yError() << "Parameter 'nIterSmall' not found in the config file.";
        return false;
    }
    if (!ptr->getParameter("controlHorizon", m_ctrlHorizon))
    {
        yError() << "Parameter 'controlHorizon' not found in the config file.";
        return false;
    }
    if (!ptr->getParameter("periodMPCSmallSteps", m_deltaTSmallSteps))
    {
        yError() << "Parameter 'periodMPCSmallSteps' not found in the config file.";
        return false;
    }
    if (!ptr->getParameter("periodMPCLargeSteps", m_deltaTLargeSteps))
    {
        yError() << "Parameter 'periodMPCLargeSteps' not found in the config file.";
        return false;
    }
    // the warp function is computed as: w(tau) = beta1 * tau + beta2 * tau^2
    // the coefficients beta1 and beta2 are computed solving the linear system:
    // w(1) - w(0) = deltaTSmallSteps
    // w(nSmallSteps) = 0.1
    m_beta2 = (m_deltaTLargeSteps - m_nSmallSteps * m_deltaTSmallSteps)
              / (m_nSmallSteps * (m_nSmallSteps - 1));
    m_beta1 = m_deltaTSmallSteps - m_beta2;
    return m_systemDynamicVS->configure(parametersHandler, qpInput);
}

void ConstraintSystemDynamicVS::configureDynVectorsSize(QPInput& qpInput)
{
    m_updateThrottle = true;
    m_counter = 0;
}

const bool ConstraintSystemDynamicVS::computeConstraintsMatrixAndBounds(QPInput& qpInput)
{
    if (!m_systemDynamicVS->updateDynamicMatrices(qpInput))
    {
        yError() << "ConstraintSystemDynamicVS::computeConstraintsMatrixAndBounds: error in "
                    "updateDynamicMatrices";
        return false;
    }
    m_systemDynamicVS->getAMatrix(m_A);
    m_systemDynamicVS->getBJointsMatrix(m_BJoints);
    m_systemDynamicVS->getBThrottleMatrix(m_BThrottle);
    m_systemDynamicVS->getCVector(m_c);
    m_linearMatrix.setZero();
    m_lowerBound.setZero();
    m_upperBound.setZero();
    for (int i = 0; i < m_nIter; i++)
    {
        if (i < m_nSmallSteps)
        {
            m_deltaT = warp_function(i + 1) - warp_function(i);
        } else
        {
            m_deltaT = m_deltaTLargeSteps;
        }
        m_linearMatrix.block(i * m_nStates, i * m_nStates, m_nStates, m_nStates)
            = Eigen::MatrixXd::Identity(m_nStates, m_nStates) + m_deltaT * m_A;
        m_linearMatrix.block(i * m_nStates, (i + 1) * m_nStates, m_nStates, m_nStates)
            = -Eigen::MatrixXd::Identity(m_nStates, m_nStates);
        if (i < m_ctrlHorizon)
        {
            m_linearMatrix.block(i * m_nStates,
                                 m_nStates * (m_nIter + 1) + i * m_nJoints,
                                 m_nStates,
                                 m_nJoints)
                = m_deltaT * m_BJoints;
        } else
        {
            m_linearMatrix.block(i * m_nStates,
                                 m_nStates * (m_nIter + 1) + (m_ctrlHorizon - 1) * m_nJoints,
                                 m_nStates,
                                 m_nJoints)
                = m_deltaT * m_BJoints;
        }
        if (i < m_nSmallSteps)
        {
            m_linearMatrix.block(i * m_nStates,
                                 m_nStates * (m_nIter + 1) + m_ctrlHorizon * m_nJoints,
                                 m_nStates,
                                 m_nThrottle)
                = m_deltaT * m_BThrottle;

        } else if (i >= m_nSmallSteps && i < m_ctrlHorizon)
        {
            m_linearMatrix.block(i * m_nStates,
                                 m_nStates * (m_nIter + 1) + m_ctrlHorizon * m_nJoints
                                     + (i - (m_nSmallSteps - 1)) * m_nThrottle,
                                 m_nStates,
                                 m_nThrottle)
                = m_deltaT * m_BThrottle;
        } else
        {
            m_linearMatrix.block(i * m_nStates,
                                 m_nStates * (m_nIter + 1) + m_ctrlHorizon * m_nJoints
                                     + (m_ctrlHorizon - (m_nSmallSteps)) * m_nThrottle,
                                 m_nStates,
                                 m_nThrottle)
                = m_deltaT * m_BThrottle;
        }
        m_lowerBound.segment(i * m_nStates, m_nStates) = -m_deltaT * m_c;
        m_upperBound.segment(i * m_nStates, m_nStates) = -m_deltaT * m_c;
    }
    if (m_counter == m_nSmallSteps)
    {
        m_updateThrottle = true;
        m_counter = 0;
    } else
    {
        m_updateThrottle = false;
    }
    m_counter++;
    return true;
}

const bool
ConstraintSystemDynamicVS::populateVectorsCollection(QPInput& qpInput,
                                                     const Eigen::Ref<Eigen::VectorXd> qpSolution)
{
    return true;
}

const bool ConstraintSystemDynamicVS::configureVectorsCollectionServer(QPInput& qpInput)
{
    return true;
}

double ConstraintSystemDynamicVS::warp_function(double t)
{
    return m_beta1 * t + m_beta2 * t * t;
}

ConstraintInitialState::ConstraintInitialState(const unsigned int nStates, const unsigned int nVar)
    : IQPConstraintInitialState(nStates, nVar)
{
}

const bool ConstraintInitialState::readConfigParameters(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
    QPInput& qpInput)
{
    auto ptr = parametersHandler.lock();
    if (!ptr->getParameter("useEstimatedThrust", m_useEstimatedThrust))
    {
        yError() << "Parameter 'useEstimatedThrust' not found in the config file.";
        return false;
    }
    if (!ptr->getParameter("periodMPC", m_periodMPC))
    {
        yError() << "Parameter 'periodMPC' not found in the config file.";
        return false;
    }
    return true;
}

void ConstraintInitialState::configureDynVectorsSize(QPInput& qpInput)
{
    m_robot = qpInput.getRobot();
    m_vectorsCollectionServer = qpInput.getVectorsCollectionServer();
    m_initialThrust.resize(m_robot->getNJets());
    m_initialThrustDot.resize(m_robot->getNJets());
    m_initialThrust.resize(m_robot->getNJets());
    m_initialThrustDot.resize(m_robot->getNJets());
    m_initialThrust = m_robot->getJetThrusts();
    m_initialThrustDot.setZero();
    m_initialAngMom = m_robot->getMomentum(true).bottomRows(3);
    m_initialCoMPos = m_robot->getPositionCoM();
    m_initialLinMom = m_robot->getMomentum(true).topRows(3);
    m_initialRPY = iDynTree::toEigen(m_robot->getBasePose().getRotation().asRPY());
    m_rpyOld = m_initialRPY;
    m_RPYinit = m_initialRPY;
    m_nTurns.setZero();
    m_initialAlphaGravity = 0.08;
    m_integralCoMError.setZero();
    m_integralRPYError.setZero();
}

bool ConstraintInitialState::updateInitialState(QPInput& qpInput)
{
    this->unwrapRPY();
    m_initialState.setZero();
    m_initialState.segment(CoMPosIdx[0], CoMPosIdx.size()) = m_robot->getPositionCoM();
    m_initialState.segment(linMomIdx[0], linMomIdx.size()) = m_robot->getMomentum(true).topRows(3);
    m_initialState.segment(rpyIdx[0], rpyIdx.size()) = m_rpyUnwrapped;
    m_initialState.segment(angMomIdx[0], angMomIdx.size())
        = m_robot->getMomentum(true).bottomRows(3);
    if (m_useEstimatedThrust)
    {
        m_initialState.segment(thrustIdx[0], thrustIdx.size()) = m_robot->getJetThrusts();
        m_initialState.segment(thrustDotIdx[0], thrustDotIdx.size())
            = qpInput.getEstimatedThrustDot();
    } else
    {
        m_initialState.segment(thrustIdx[0], thrustIdx.size()) = qpInput.getThrustDesMPC();
        m_initialState.segment(thrustDotIdx[0], thrustDotIdx.size()) = qpInput.getThrustDotDesMPC();
    }
    m_initialState.segment(positionErrorIdx[0], positionErrorIdx.size())
        = m_robot->getPositionCoM() - qpInput.getPosCoMReference();
    m_initialState.segment(rpyErrorIdx[0], rpyErrorIdx.size())
        = m_rpyUnwrapped - qpInput.getRPYReference();
    return true;
}

void ConstraintInitialState::unwrapRPY()
{
    for (int i = 0; i < 3; i++)
    {
        if (m_robot->getBasePose().getRotation().asRPY()(i) - m_rpyOld(i) > M_PI)
        {
            m_nTurns(i)--;
        } else if (m_robot->getBasePose().getRotation().asRPY()(i) - m_rpyOld(i) < -M_PI)
        {
            m_nTurns(i)++;
        }
    }
    m_rpyUnwrapped
        = iDynTree::toEigen(m_robot->getBasePose().getRotation().asRPY()) + 2 * M_PI * m_nTurns;
    m_rpyOld = iDynTree::toEigen(m_robot->getBasePose().getRotation().asRPY());
}

const bool ConstraintInitialState::updateQPSolution(Eigen::Ref<Eigen::VectorXd> qpSolution)
{
    m_initialCoMPos = qpSolution.segment(CoMPosIdx[0], CoMPosIdx.size());
    m_initialLinMom = qpSolution.segment(linMomIdx[0], linMomIdx.size());
    m_initialRPY = qpSolution.segment(rpyIdx[0], rpyIdx.size());
    m_initialAngMom = qpSolution.segment(angMomIdx[0], angMomIdx.size());
    m_initialThrust = qpSolution.segment(thrustIdx[0], thrustIdx.size());
    m_initialThrustDot = qpSolution.segment(thrustDotIdx[0], thrustDotIdx.size());
    return true;
}

const bool ConstraintInitialState::configureVectorsCollectionServer(QPInput& qpInput)
{
    m_vectorsCollectionServer->populateMetadata("measures::unwrappedRPY", {"roll", "pitch", "yaw"});
    m_vectorsCollectionServer->populateMetadata("measures::integralErrorRPY",
                                                {"roll", "pitch", "yaw"});
    m_vectorsCollectionServer->populateMetadata("measures::integralErrorCoM", {"x", "y", "z"});
    return true;
}

const bool
ConstraintInitialState::populateVectorsCollection(QPInput& qpInput,
                                                  const Eigen::Ref<Eigen::VectorXd> qpSolution)
{
    m_vectorsCollectionServer->populateData("measures::unwrappedRPY", m_rpyUnwrapped);
    m_vectorsCollectionServer->populateData("measures::integralErrorRPY", m_integralRPYError);
    m_vectorsCollectionServer->populateData("measures::integralErrorCoM", m_integralCoMError);
    return true;
}

ThrottleConstraint::ThrottleConstraint(const int nVar,
                                       const int nStates,
                                       const int nIter,
                                       const int nSmallSteps)
    : IQPConstraint(nVar, N_THRUSTS * (nIter - nSmallSteps + 1))
{
    m_nIter = nIter;
    m_nSmallSteps = nSmallSteps;
    m_nStates = nStates;
}

const bool ThrottleConstraint::readConfigParameters(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
    QPInput& qpInput)
{
    auto ptr = parametersHandler.lock();
    if (!ptr->getParameter("throttleMax", m_throttleMaxValue))
    {
        yError() << "Parameter 'throttleMax' not found in the config file.";
        return false;
    }
    if (!ptr->getParameter("throttleMin", m_throttleMinValue))
    {
        yError() << "Parameter 'throttleMin' not found in the config file.";
        return false;
    }
    double periodMPCLargeSteps;
    if (!ptr->getParameter("periodMPCLargeSteps", periodMPCLargeSteps))
    {
        yError() << "Parameter 'periodMPCLargeSteps' not found in the config file.";
        return false;
    }
    double periodMPCSmallSteps;
    if (!ptr->getParameter("periodMPCSmallSteps", periodMPCSmallSteps))
    {
        yError() << "Parameter 'periodMPCSmallSteps' not found in the config file.";
        return false;
    }
    if (!ptr->getParameter("controlHorizon", m_ctrlHorizon))
    {
        yError() << "Parameter 'controlHorizon' not found in the config file.";
        return false;
    }
    m_ratioSmallLargeStepsPeriod = round(periodMPCLargeSteps / periodMPCSmallSteps);
    return true;
}

void ThrottleConstraint::configureDynVectorsSize(QPInput& qpInput)
{
    m_jetModel = qpInput.getJetModel();
    m_throttleMaxValue = m_jetModel->standardizeThrottle_u2T(m_throttleMaxValue);
    m_throttleMinValue = m_jetModel->standardizeThrottle_u2T(m_throttleMinValue);
    m_vMax = m_jetModel->compute_v(m_throttleMaxValue);
    m_vMin = m_jetModel->compute_v(m_throttleMinValue);
    m_nJoints = deltaJointIdx.size();
    m_nThrottle = throttleIdx.size();
    m_counter = m_ratioSmallLargeStepsPeriod - 1;
}

const bool ThrottleConstraint::computeConstraintsMatrixAndBounds(QPInput& qpInput)
{
    m_linearMatrix.setZero();
    m_lowerBound.setZero();
    m_upperBound.setZero();
    for (int i = 0; i < m_ctrlHorizon - m_nSmallSteps + 1; i++)
    {
        m_linearMatrix
            .block(i * m_nThrottle,
                   m_nStates * (m_nIter + 1) + m_nJoints * m_ctrlHorizon + i * m_nThrottle,
                   m_nThrottle,
                   m_nThrottle)
            .setIdentity();
        if (m_counter != (m_ratioSmallLargeStepsPeriod - 1) && i == 0)
        {
            for (int j = 0; j < N_THRUSTS; j++)
            {
                m_lowerBound[j] = m_jetModel->compute_v(
                    m_jetModel->standardizeThrottle_u2T(qpInput.getThrottleMPC()[j]));
                m_upperBound[j] = m_jetModel->compute_v(
                    m_jetModel->standardizeThrottle_u2T(qpInput.getThrottleMPC()[j]));
            }
        } else
        {
            m_lowerBound.segment(i * m_nThrottle, m_nThrottle).setConstant(m_vMin);
            m_upperBound.segment(i * m_nThrottle, m_nThrottle).setConstant(m_vMax);
        }
    }
    if (m_counter == (m_ratioSmallLargeStepsPeriod - 1))
    {
        m_counter = 0;
    } else
    {
        m_counter++;
    }
    return true;
}

const bool ThrottleConstraint::configureVectorsCollectionServer(QPInput& qpInput)
{
    return true;
}

const bool
ThrottleConstraint::populateVectorsCollection(QPInput& qpInput,
                                              const Eigen::Ref<Eigen::VectorXd> qpSolution)
{
    return true;
}

JointPositionConstraint::JointPositionConstraint(const int nVar,
                                                 const int nStates,
                                                 const int nJoints,
                                                 const int nIter)
    : IQPConstraint(nVar, nJoints * nIter)
{
    m_nIter = nIter;
    m_nStates = nStates;
    m_nJoints = nJoints;
}

const bool JointPositionConstraint::readConfigParameters(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
    QPInput& qpInput)
{
    auto ptr = parametersHandler.lock();
    m_jointPositionMax.resize(m_nJoints);
    m_jointPositionMin.resize(m_nJoints);
    if (!ptr->getParameter("jointPos_max", m_jointPositionMax))
    {
        yError() << "Parameter 'jointPos_max' not found in the config file.";
        return false;
    }
    if (!ptr->getParameter("jointPos_min", m_jointPositionMin))
    {
        yError() << "Parameter 'jointPos_min' not found in the config file.";
        return false;
    }
    if (!ptr->getParameter("controlHorizon", m_ctrlHorizon))
    {
        yError() << "Parameter 'controlHorizon' not found in the config file.";
        return false;
    }
    // convert the joint position limits from degrees to radians
    m_jointPositionMax = m_jointPositionMax * M_PI / 180.0;
    m_jointPositionMin = m_jointPositionMin * M_PI / 180.0;
    return true;
}

void JointPositionConstraint::configureDynVectorsSize(QPInput& qpInput)
{
    m_robot = qpInput.getRobot();
    m_linearMatrix.setZero();
    m_firstIteriation = true;
}

const bool JointPositionConstraint::computeConstraintsMatrixAndBounds(QPInput& qpInput)
{
    m_lowerBound.setZero();
    m_upperBound.setZero();
    for (int i = 0; i < m_ctrlHorizon; i++)
    {
        if (m_firstIteriation)
        {
            m_linearMatrix
                .block(i * m_nJoints,
                       m_nStates * (m_nIter + 1) + i * m_nJoints,
                       m_nJoints,
                       m_nJoints)
                .setIdentity();
            m_firstIteriation = false;
        }
        m_lowerBound.segment(i * m_nJoints, m_nJoints)
            = m_jointPositionMin - qpInput.getOutputQPJointsPosition().segment(3, m_nJoints);
        m_upperBound.segment(i * m_nJoints, m_nJoints)
            = m_jointPositionMax - qpInput.getOutputQPJointsPosition().segment(3, m_nJoints);
    }
    return true;
}

const bool JointPositionConstraint::configureVectorsCollectionServer(QPInput& qpInput)
{
    return true;
}

const bool
JointPositionConstraint::populateVectorsCollection(QPInput& qpInput,
                                                   const Eigen::Ref<Eigen::VectorXd> qpSolution)
{
    return true;
}
