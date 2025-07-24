#include "QPExample.h"

const bool QPExample::setCostAndConstraints(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
    QPInput& qpInput)
{
    const std::shared_ptr<Robot> robot = qpInput.getRobot();
    m_nVar = robot->getNJoints() + 6;
    m_vectorCosts.emplace_back(std::make_unique<CostConfigurationVelocity>(m_nVar));
    m_vectorConstraints.emplace_back(std::make_unique<ConstraintBaseVel>(m_nVar));
    m_vectorConstraints.emplace_back(
        std::make_unique<ConstraintJointVel>(m_nVar, robot->getNJoints()));
    return true;
};

// -- Define the constraints and costs --

//// ConstraintBaseVel

ConstraintBaseVel::ConstraintBaseVel(const unsigned int nVar)
    : IQPConstraint(nVar, 6){};

void ConstraintBaseVel::configureDynVectorsSize(QPInput& qpInput)
{
    m_vectorsCollectionServer = qpInput.getVectorsCollectionServer();
};

const bool ConstraintBaseVel::readConfigParameters(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
    QPInput& qpInput)
{
    return true;
};

const bool ConstraintBaseVel::computeConstraintsMatrixAndBounds(QPInput& qpInput)
{
    m_linearMatrix.block(0, 0, m_nConstraints, m_nVar).setZero();
    m_linearMatrix.block(0, 0, m_nConstraints, m_nConstraints).setIdentity();
    m_lowerBound.setZero();
    m_upperBound.setZero();
    return true;
};

const bool ConstraintBaseVel::configureVectorsCollectionServer(QPInput& qpInput)
{
    return true;
};

const bool
ConstraintBaseVel::populateVectorsCollection(QPInput& qpInput,
                                             const Eigen::Ref<Eigen::VectorXd> qpSolution)
{
    return true;
};

//// ConstraintJointVel

ConstraintJointVel::ConstraintJointVel(const unsigned int nVar, const unsigned int n_dof)
    : IQPConstraint(nVar, n_dof){};

const bool ConstraintJointVel::readConfigParameters(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
    QPInput& qpInput)
{
    auto ptr = parametersHandler.lock();
    if (!ptr->getParameter("joint_speed_limit", m_speedLimit))
    {
        yError() << "Parameter 'joint_speed_limit' not found in the config file.";
        return false;
    }
    return true;
};

void ConstraintJointVel::configureDynVectorsSize(QPInput& qpInput)
{
    m_vectorsCollectionServer = qpInput.getVectorsCollectionServer();
};

const bool ConstraintJointVel::computeConstraintsMatrixAndBounds(QPInput& qpInput)
{
    m_linearMatrix.setZero();
    m_linearMatrix.block(0, 6, m_nConstraints, m_nConstraints).setIdentity();
    m_lowerBound.setOnes();
    m_lowerBound *= -m_speedLimit;
    m_upperBound.setOnes();
    m_upperBound *= m_speedLimit;
    return true;
};

const bool ConstraintJointVel::configureVectorsCollectionServer(QPInput& qpInput)
{
    return true;
};

const bool
ConstraintJointVel::populateVectorsCollection(QPInput& qpInput,
                                              const Eigen::Ref<Eigen::VectorXd> qpSolution)
{
    return true;
};

//// CostConfigurationVelocity

CostConfigurationVelocity::CostConfigurationVelocity(const unsigned int nVar)
    : IQPCost(nVar){};

const bool CostConfigurationVelocity::readConfigParameters(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
    QPInput& qpInput)
{
    auto ptr = parametersHandler.lock();
    if (!ptr->getParameter("weight_postural", m_weight))
    {
        yError() << "Parameter 'weight_postural' not found in the config file.";
        return false;
    }
    return true;
};

void CostConfigurationVelocity::configureDynVectorsSize(QPInput& qpInput)
{
    m_vectorsCollectionServer = qpInput.getVectorsCollectionServer();
};

const bool CostConfigurationVelocity::computeHessianAndGradient(QPInput& qpInput)
{
    m_hessian.setIdentity();
    m_hessian *= m_weight;
    m_gradient.setOnes();
    m_gradient *= -1.0;
    return true;
};

const bool CostConfigurationVelocity::configureVectorsCollectionServer(QPInput& qpInput)
{
    return true;
};

const bool
CostConfigurationVelocity::populateVectorsCollection(QPInput& qpInput,
                                                     const Eigen::Ref<Eigen::VectorXd> qpSolution)
{
    return true;
};
