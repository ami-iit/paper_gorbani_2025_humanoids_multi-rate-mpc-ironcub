#ifndef CONSTRAINT_VS_MPC_H
#define CONSTRAINT_VS_MPC_H

#include "systemDynamicsVSMPC.h"
#include <IMPCProblem/IQPUtilsMPC.h>

class ConstraintSystemDynamicVS : public IQPConstraint
{
public:
    ConstraintSystemDynamicVS(
        const int nVar, const int nStates, const int nJoints, const int nThrottle, const int nIter);
    ~ConstraintSystemDynamicVS() = default;

    const bool readConfigParameters(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput) override;

    void configureDynVectorsSize(QPInput& qpInput) override;

    const bool computeConstraintsMatrixAndBounds(QPInput& qpInput) override;

    const bool populateVectorsCollection(QPInput& qpInput,
                                         const Eigen::Ref<Eigen::VectorXd> qpSolution) override;

    const bool configureVectorsCollectionServer(QPInput& qpInput) override;

private:
    double warp_function(double t);

    bool m_updateThrottle;
    double m_counter;
    int m_nStates;
    int m_nJoints;
    int m_nThrottle;
    int m_nIter;
    int m_nSmallSteps;
    int m_ctrlHorizon;
    double m_deltaT;
    double m_deltaTSmallSteps;
    double m_deltaTLargeSteps;
    double m_beta1;
    double m_beta2;
    std::unique_ptr<SystemDynamicVS> m_systemDynamicVS;
    Eigen::MatrixXd m_A;
    Eigen::MatrixXd m_BJoints;
    Eigen::MatrixXd m_BThrottle;
    Eigen::VectorXd m_c;
};

class ConstraintInitialState : public IQPConstraintInitialState
{
public:
    ConstraintInitialState(const unsigned int nStates, const unsigned int nVar);
    const bool readConfigParameters(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput) override;

    void configureDynVectorsSize(QPInput& qpInput) override;

    bool updateInitialState(QPInput& qpInput) override;

    const bool configureVectorsCollectionServer(QPInput& qpInput) override;

    const bool populateVectorsCollection(QPInput& qpInput,
                                         const Eigen::Ref<Eigen::VectorXd> qpSolution) override;

    const bool updateQPSolution(Eigen::Ref<Eigen::VectorXd> qpSolution);

private:
    void unwrapRPY();

    bool m_useEstimatedThrust;
    Eigen::Vector3d m_initialCoMPos;
    Eigen::Vector3d m_initialLinMom;
    Eigen::Vector3d m_initialRPY;
    Eigen::Vector3d m_RPYinit;
    Eigen::Vector3d m_initialAngMom;
    Eigen::VectorXd m_initialThrust;
    Eigen::VectorXd m_initialThrustDot;
    Eigen::Vector3d m_rpyOld;
    Eigen::Vector3d m_rpyUnwrapped;
    Eigen::Vector3d m_integralRPYError;
    Eigen::Vector3d m_integralCoMError;
    double m_initialAlphaGravity;
    double m_periodMPC;
    int m_nStates;
    int m_nInput;
    int m_nIter;

    Eigen::Vector3d m_nTurns;
    std::shared_ptr<Robot> m_robot;
    std::shared_ptr<BipedalLocomotion::YarpUtilities::VectorsCollectionServer>
        m_vectorsCollectionServer;
};

class ThrottleConstraint : public IQPConstraint
{
public:
    ThrottleConstraint(const int nVar, const int nStates, const int nIter, const int nSmallSteps);
    ~ThrottleConstraint() = default;

    const bool readConfigParameters(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput) override;

    void configureDynVectorsSize(QPInput& qpInput) override;

    const bool computeConstraintsMatrixAndBounds(QPInput& qpInput) override;

    const bool populateVectorsCollection(QPInput& qpInput,
                                         const Eigen::Ref<Eigen::VectorXd> qpSolution) override;

    const bool configureVectorsCollectionServer(QPInput& qpInput) override;

private:
    int m_nIter;
    int m_ctrlHorizon;
    int m_nSmallSteps;
    int m_nStates;
    int m_nJoints;
    int m_nThrottle;
    int m_counter;
    int m_ratioSmallLargeStepsPeriod;
    double m_throttleMaxValue;
    double m_throttleMinValue;
    double m_vMax;
    double m_vMin;

    std::shared_ptr<Robot> m_robot;
    std::shared_ptr<BipedalLocomotion::YarpUtilities::VectorsCollectionServer>
        m_vectorsCollectionServer;
    std::shared_ptr<JetModel> m_jetModel;
};

class JointPositionConstraint : public IQPConstraint
{
public:
    JointPositionConstraint(const int nVar, const int nStates, const int nJoints, const int nIter);
    ~JointPositionConstraint() = default;

    const bool readConfigParameters(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput) override;

    void configureDynVectorsSize(QPInput& qpInput) override;

    const bool computeConstraintsMatrixAndBounds(QPInput& qpInput) override;

    const bool populateVectorsCollection(QPInput& qpInput,
                                         const Eigen::Ref<Eigen::VectorXd> qpSolution) override;

    const bool configureVectorsCollectionServer(QPInput& qpInput) override;

private:
    int m_nIter;
    int m_ctrlHorizon;
    int m_nStates;
    int m_nJoints;
    int m_counter;
    bool m_firstIteriation;
    Eigen::VectorXd m_jointPositionMax;
    Eigen::VectorXd m_jointPositionMin;
    std::shared_ptr<Robot> m_robot;
    std::shared_ptr<BipedalLocomotion::YarpUtilities::VectorsCollectionServer>
        m_vectorsCollectionServer;
    std::shared_ptr<JetModel> m_jetModel;
};

#endif // CONSTRAINT_VS_MPC_H
