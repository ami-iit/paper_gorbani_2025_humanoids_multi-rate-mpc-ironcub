#ifndef QPEXAMPLE
#define QPEXAMPLE

#include "IQPConstraint.h"
#include "IQPCost.h"
#include "IQPProblem.h"
#include <Eigen/Dense>
#include <OsqpEigen/OsqpEigen.h>
#include <iDynTree/EigenHelpers.h>
#include <iDynTree/KinDynComputations.h>
#include <yarp/os/LogStream.h>

class QPExample : public IQPProblem
{
private:
    const bool setCostAndConstraints(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput) override;
};

class ConstraintBaseVel : public IQPConstraint
{
public:
    ConstraintBaseVel(const unsigned int nVar);
    const bool readConfigParameters(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput) override;
    void configureDynVectorsSize(QPInput& qpInput) override;
    const bool computeConstraintsMatrixAndBounds(QPInput& qpInput) override;
    const bool configureVectorsCollectionServer(QPInput& qpInput) override;
    const bool populateVectorsCollection(QPInput& qpInput,
                                         const Eigen::Ref<Eigen::VectorXd> qpSolution) override;

private:
    std::shared_ptr<BipedalLocomotion::YarpUtilities::VectorsCollectionServer>
        m_vectorsCollectionServer;
};

class ConstraintJointVel : public IQPConstraint
{
public:
    ConstraintJointVel(const unsigned int nVar, const unsigned int n_dof);
    const bool readConfigParameters(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput) override;
    void configureDynVectorsSize(QPInput& qpInput) override;
    const bool computeConstraintsMatrixAndBounds(QPInput& qpInput) override;
    const bool configureVectorsCollectionServer(QPInput& qpInput) override;
    const bool populateVectorsCollection(QPInput& qpInput,
                                         const Eigen::Ref<Eigen::VectorXd> qpSolution) override;
    double m_speedLimit;

private:
    std::shared_ptr<BipedalLocomotion::YarpUtilities::VectorsCollectionServer>
        m_vectorsCollectionServer;
};

class CostConfigurationVelocity : public IQPCost
{
public:
    CostConfigurationVelocity(const unsigned int nVar);
    const bool readConfigParameters(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput) override;
    void configureDynVectorsSize(QPInput& qpInput) override;
    const bool computeHessianAndGradient(QPInput& qpInput) override;
    const bool configureVectorsCollectionServer(QPInput& qpInput) override;
    const bool populateVectorsCollection(QPInput& qpInput,
                                         const Eigen::Ref<Eigen::VectorXd> qpSolution) override;
    double m_weight;

private:
    std::shared_ptr<BipedalLocomotion::YarpUtilities::VectorsCollectionServer>
        m_vectorsCollectionServer;
};

#endif /* end of include guard QPEXAMPLE */
