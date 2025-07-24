// #include <linearMomentumBasedMPC/constraintsLinearMomentumMPC.h>
#include <variableSamplingMPC/VSconstant.h>
#include <variableSamplingMPC/constraintsVSMPC.h>
#include <variableSamplingMPC/costsVSMPC.h>
#include <variableSamplingMPC/variableSamplingMPC.h>

const bool VariableSamplingMPC::setCostAndConstraints(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
    QPInput& qpInput)
{
    auto ptr = parametersHandler.lock();
    if (!ptr->getParameter("controlledJoints", m_controlledJoints))
    {
        yError() << "Parameter 'controlledJoints' not found in the config file.";
        return false;
    }
    m_nCtrlJoints = m_controlledJoints.size();
    if (m_nCtrlJoints != deltaJointIdx.size())
    {
        yError() << "The number of controlled joints defined in the systemDynamic.h file is "
                    "different from the size of the 'controlledJoints' parameter";
        return false;
    }
    if (!ptr->getParameter("nIter", m_nIter))
    {
        yError() << "Parameter 'nIter' not found in the config file.";
        return false;
    }
    if (!ptr->getParameter("nIterSmall", m_nIterSmall))
    {
        yError() << "Parameter 'nIterSmall' not found in the config file.";
        return false;
    }
    if (!ptr->getParameter("controlHorizon", m_ctrlHorizon))
    {
        yError() << "Parameter 'controlHorizon' not found in the config file.";
        return false;
    }
    m_robot = qpInput.getRobot();
    m_nJets = m_robot->getNJets();
    m_jetModel = qpInput.getJetModel();
    m_nStates = rpyErrorIdx[2] + 1;
    m_nInput = m_nCtrlJoints + m_nJets;
    m_nVar = m_nStates * (m_nIter + 1) + m_nCtrlJoints * m_ctrlHorizon
             + m_nJets * (m_ctrlHorizon - m_nIterSmall + 1);
    m_jointSelectorVector.clear();
    for (auto joint : m_controlledJoints)
    {
        for (int i = 0; i < m_robot->getNJoints(); i++)
        {
            if (joint == m_robot->getJointName(i))
            {
                m_jointSelectorVector.emplace_back(i);
            }
        }
    }

    // resize the vectors
    m_jointsPositionReference.resize(m_robot->getNJoints());
    m_jointsPositionReference = m_robot->getJointPos();
    m_previousState.resize(m_nStates);
    m_QPSolution.resize(m_nVar);
    m_deltaJointsPositionReference.resize(m_nCtrlJoints);
    m_thrustReference.resize(m_nJets);
    m_thrustDotReference.resize(m_nJets);
    m_throttleReference.resize(m_nJets);
    m_statesSolution.resize(m_nStates * (m_nIter + 1));
    m_inputSolution.resize(m_nCtrlJoints * m_nIter + m_nJets * (m_nIter - m_nIterSmall + 1));

    m_vectorCosts.emplace_back(std::make_unique<ReferenceTrackingCost>(m_nVar, m_nStates, m_nIter));
    m_vectorCosts.emplace_back(
        std::make_unique<RegualarizationCost>(m_nVar, m_nStates, m_nCtrlJoints, m_nJets));
    m_vectorCosts.emplace_back(
        std::make_unique<ThrottleInitialValueCost>(m_nVar, m_nStates, m_nCtrlJoints, m_nJets));
    m_vectorCosts.emplace_back(
        std::make_unique<JointPositionRegularizationCost>(m_nVar, m_nStates, m_nCtrlJoints));
    m_vectorConstraints.emplace_back(std::make_unique<ConstraintSystemDynamicVS>(m_nVar,
                                                                                 m_nStates,
                                                                                 m_nCtrlJoints,
                                                                                 m_nJets,
                                                                                 m_nIter));
    m_vectorConstraints.emplace_back(std::make_unique<ConstraintInitialState>(m_nStates, m_nVar));
    m_vectorConstraints.emplace_back(
        std::make_unique<ThrottleConstraint>(m_nVar, m_nStates, m_nIter, m_nIterSmall));
    return true;
}

const bool VariableSamplingMPC::solveMPC()
{
    this->solve();
    if (this->getQPProblemStatus() == OsqpEigen::Status::Solved)
    {
        m_QPSolution = this->getSolution();

        // extract the solution
        m_statesSolution = m_QPSolution.head(m_nStates * (m_nIter + 1));
        m_inputSolution = m_QPSolution.tail(m_nCtrlJoints * m_ctrlHorizon
                                            + m_nJets * (m_ctrlHorizon - m_nIterSmall + 1));
        m_deltaJointsPositionReference = m_inputSolution.segment(0, m_nCtrlJoints);
        m_throttleReference = m_inputSolution.segment(m_nCtrlJoints * m_ctrlHorizon, m_nJets);
        m_thrustReference = m_statesSolution.segment(m_nStates + thrustIdx[0], m_nJets);
        m_thrustDotReference = m_statesSolution.segment(m_nStates + thrustDotIdx[0], m_nJets);
        m_finalState = m_statesSolution.tail(m_nStates);
        for (int i = 0; i < m_jointSelectorVector.size(); i++)
        {
            m_jointsPositionReference(m_jointSelectorVector[i])
                += m_deltaJointsPositionReference(i);
        }
    }

    return true;
}

const bool VariableSamplingMPC::getMPCSolution(Eigen::Ref<Eigen::VectorXd> qpSolution)
{
    if (qpSolution.size() != m_nInput)
    {
        yError() << "VariableSamplingMPC::getMPCSolution: wrong size of the input vector";
        return false;
    }
    qpSolution = m_inputSolution;
    return true;
}

const bool
VariableSamplingMPC::getJointsReferencePosition(Eigen::Ref<Eigen::VectorXd> jointsPosition)
{
    if (jointsPosition.size() != m_robot->getNJoints())
    {
        yError() << "VariableSamplingMPC::getJointsReferencePosition: wrong size of the input "
                    "vector";
        return false;
    }
    jointsPosition = m_jointsPositionReference;
    return true;
}

const bool VariableSamplingMPC::getThrottleReference(Eigen::Ref<Eigen::VectorXd> throttle)
{
    if (throttle.size() != m_nJets)
    {
        yError() << "VariableSamplingMPC::getThrottleReference: wrong size of the input vector";
        return false;
    }

    for (int i = 0; i < m_nJets; i++)
    {
        throttle(i) = m_jetModel->destandardizeThrottle_u2T(m_throttleReference(i));
    }
    return true;
}

const bool VariableSamplingMPC::getThrustReference(Eigen::Ref<Eigen::VectorXd> thrust)
{
    if (thrust.size() != m_nJets)
    {
        yError() << "VariableSamplingMPC::getThrustReference: wrong size of the input vector";
        return false;
    }
    thrust = m_thrustReference;
    return true;
}

const bool VariableSamplingMPC::getThrustDotReference(Eigen::Ref<Eigen::VectorXd> thrustDot)
{
    if (thrustDot.size() != m_nJets)
    {
        yError() << "VariableSamplingMPC::getThrustDotReference: wrong size of the input vector";
        return false;
    }
    thrustDot = m_thrustDotReference;
    return true;
}

const bool VariableSamplingMPC::getFinalCoMPosition(Eigen::Ref<Eigen::VectorXd> finalCoMPosition)
{
    if (finalCoMPosition.size() != 3)
    {
        yError() << "VariableSamplingMPC::getFinalCoMPosition: wrong size of the input vector";
        return false;
    }
    finalCoMPosition = m_finalState.segment(CoMPosIdx[0], CoMPosIdx.size());
    return true;
}

const bool VariableSamplingMPC::getFinalLinMom(Eigen::Ref<Eigen::VectorXd> finalLinMom)
{
    if (finalLinMom.size() != 3)
    {
        yError() << "VariableSamplingMPC::getFinalLinMom: wrong size of the input vector";
        return false;
    }
    finalLinMom = m_finalState.segment(linMomIdx[0], linMomIdx.size());
    return true;
}

const bool VariableSamplingMPC::getFinalRPY(Eigen::Ref<Eigen::VectorXd> finalRPY)
{
    if (finalRPY.size() != 3)
    {
        yError() << "VariableSamplingMPC::getFinalRPY: wrong size of the input vector";
        return false;
    }
    finalRPY = m_finalState.segment(rpyIdx[0], rpyIdx.size());
    return true;
}

const bool VariableSamplingMPC::getFinalAngMom(Eigen::Ref<Eigen::VectorXd> finalAngMom)
{
    if (finalAngMom.size() != 3)
    {
        yError() << "VariableSamplingMPC::getFinalAngMom: wrong size of the input vector";
        return false;
    }
    finalAngMom = m_finalState.segment(angMomIdx[0], angMomIdx.size());
    return true;
}

double VariableSamplingMPC::getNStatesMPC() const
{
    return m_nStates;
}

double VariableSamplingMPC::getNInputMPC() const
{
    return m_nInput;
}
