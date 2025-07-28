#ifndef VS_COSTS
#define VS_COSTS

#include "TrajectoryManager.h"
#include <IMPCProblem/IQPUtilsMPC.h>
#include <IQPCost.h>

class ReferenceTrackingCost : public IQPCost
{
public:
    ReferenceTrackingCost(const unsigned int nVar,
                          const unsigned int nStates,
                          const unsigned int nIter);

    const bool readConfigParameters(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput) override;
    void configureDynVectorsSize(QPInput& qpInput) override;
    const bool configureVectorsCollectionServer(QPInput& qpInput) override;

    const bool computeHessianAndGradient(QPInput& qpInput) override;
    const bool populateVectorsCollection(QPInput& qpInput,
                                         const Eigen::Ref<Eigen::VectorXd> qpSolution) override;

    const bool setPositionCoMReference(const Eigen::Ref<const Eigen::MatrixXd> CoMPosRef);
    const bool setLinearMomentumReference(const Eigen::Ref<const Eigen::MatrixXd> linMomRef);
    const bool setRPYReference(const Eigen::Ref<const Eigen::MatrixXd> RPYRef);
    const bool setAngularMomentumReference(const Eigen::Ref<const Eigen::MatrixXd> angMomRef);

private:
    void updateInertiaMatrix();

    int m_nStates;
    int m_nIter;
    int m_nIterSmall;
    int m_counter;
    int m_ratioSmallLargeStepsPeriod;
    bool m_firstUpdate{true};
    Eigen::Vector3d m_weightCoMPos;
    Eigen::Vector3d m_weightCoMPosError;
    Eigen::Vector3d m_weightLinMom;
    Eigen::Vector3d m_weightRPY;
    Eigen::Vector3d m_weightRPYError;
    Eigen::Vector3d m_weightAngMom;
    Eigen::Vector3d m_initialCoMPos;
    Eigen::Vector3d m_initialRPY;
    TrajectoryManager m_trajManager;
    Eigen::MatrixXd m_positionCoMReference;
    Eigen::MatrixXd m_linearMomentumReference;
    Eigen::MatrixXd m_RPYReference;
    Eigen::MatrixXd m_angularMomentumReference;
    Eigen::Matrix3d m_inertia;
    Eigen::Matrix3d m_W;

    std::shared_ptr<BipedalLocomotion::YarpUtilities::VectorsCollectionServer>
        m_vectorsCollectionServer;
    Eigen::MatrixXd m_Q;
    Eigen::MatrixXd m_stateReference;
    std::shared_ptr<Robot> m_robot;
};

class RegualarizationCost : public IQPCost
{
public:
    RegualarizationCost(const unsigned int nVar,
                        const unsigned int nState,
                        const unsigned int nJoints,
                        const unsigned int nThrottle);

    const bool readConfigParameters(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput) override;

    void configureDynVectorsSize(QPInput& qpInput) override;

    const bool computeHessianAndGradient(QPInput& qpInput) override;

    const bool configureVectorsCollectionServer(QPInput& qpInput) override;

    const bool populateVectorsCollection(QPInput& qpInput,
                                         const Eigen::Ref<Eigen::VectorXd> qpSolution) override;

private:
    bool m_firstUpdate{true};
    int m_nIter;
    int m_ctrlHorizon;
    int m_nStates;
    int m_nCtrlJoints;
    int m_nJets;
    int m_nSmallSteps;
    double m_weightThrottle;
    Eigen::MatrixXd m_weightDeltaJoint;
    Eigen::MatrixXd m_weightThrottleMatrix;
    std::shared_ptr<Robot> m_robot;
};

class ThrottleInitialValueCost : public IQPCost
{
public:
    ThrottleInitialValueCost(const unsigned int nVar,
                             const unsigned int nState,
                             const unsigned int nJoints,
                             const unsigned int nThrottle);

    const bool readConfigParameters(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput) override;

    void configureDynVectorsSize(QPInput& qpInput) override;

    const bool computeHessianAndGradient(QPInput& qpInput) override;

    const bool configureVectorsCollectionServer(QPInput& qpInput) override;

    const bool populateVectorsCollection(QPInput& qpInput,
                                         const Eigen::Ref<Eigen::VectorXd> qpSolution) override;

private:
    bool m_firstUpdate{true};
    int m_nIter;
    int m_ctrlHorizon;
    int m_nStates;
    int m_nCtrlJoints;
    int m_nJets;
    double m_weightThrottle;
    Eigen::MatrixXd m_weightThrottleMatrix;
    std::shared_ptr<JetModel> m_jetModel;
};

class JointPositionRegularizationCost : public IQPCost
{
public:
    JointPositionRegularizationCost(const unsigned int nVar,
                                    const unsigned int nState,
                                    const unsigned int nJoints);

    const bool readConfigParameters(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput) override;

    void configureDynVectorsSize(QPInput& qpInput) override;

    const bool computeHessianAndGradient(QPInput& qpInput) override;

    const bool configureVectorsCollectionServer(QPInput& qpInput) override;

    const bool populateVectorsCollection(QPInput& qpInput,
                                         const Eigen::Ref<Eigen::VectorXd> qpSolution) override;

private:
    bool m_firstUpdate{true};
    int m_nIter;
    int m_ctrlHorizon;
    int m_nStates;
    int m_nCtrlJoints;
    double m_weightJointPos;
    Eigen::MatrixXd m_weightJointPosMatrix;
    std::shared_ptr<Robot> m_robot;
    std::shared_ptr<Robot> m_robotReference;
    std::vector<std::string> m_ctrlJointsNames;
    Eigen::VectorXd m_jointPosReference;
};

#endif // VS_COSTS
