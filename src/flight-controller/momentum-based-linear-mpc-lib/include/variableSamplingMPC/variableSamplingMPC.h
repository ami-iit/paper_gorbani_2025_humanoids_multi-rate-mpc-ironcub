/**
 * @file linearMomentumBasedMPC.h
 * @authors Davide Gorbani
 * @copyright 2025 Istituto Italiano di Tecnologia (IIT). This software may be modified and
 * distributed under the terms of the BSD-3-Clause license.
 */

#ifndef VARIABLE_SAMPLING_MPC_H
#define VARIABLE_SAMPLING_MPC_H

#include <IMPCProblem/IMPCProblem.h>
#include <IMPCProblem/IQPUtilsMPC.h>
#include <IQPCost.h>

class VariableSamplingMPC : public IMPCProblem
{
public:
    const bool getMPCSolution(Eigen::Ref<Eigen::VectorXd> qpSolution);

    const bool getJointsReferencePosition(Eigen::Ref<Eigen::VectorXd> jointsPosition);

    const bool getThrottleReference(Eigen::Ref<Eigen::VectorXd> throttle);

    const bool getThrustReference(Eigen::Ref<Eigen::VectorXd> thrust);

    const bool getThrustDotReference(Eigen::Ref<Eigen::VectorXd> thrustDot);

    const bool getFinalCoMPosition(Eigen::Ref<Eigen::VectorXd> finalCoMPosition);

    const bool getFinalLinMom(Eigen::Ref<Eigen::VectorXd> finalLinMom);

    const bool getFinalRPY(Eigen::Ref<Eigen::VectorXd> finalRPY);

    const bool getFinalAngMom(Eigen::Ref<Eigen::VectorXd> finalAngMom);

    const bool solveMPC();

    double getNStatesMPC() const;

    double getNInputMPC() const;

private:
    const bool setCostAndConstraints(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput) override;

    int m_nStates;
    int m_nInput;
    int m_nIter;
    int m_ctrlHorizon;
    int m_nIterSmall;
    int m_nCtrlJoints;
    int m_nJets;
    std::vector<int> m_jointSelectorVector;
    std::vector<std::string> m_controlledJoints;
    std::shared_ptr<Robot> m_robot;
    std::shared_ptr<JetModel> m_jetModel;

    Eigen::VectorXd m_previousState;
    Eigen::VectorXd m_QPSolution;
    Eigen::VectorXd m_jointsPositionReference;
    Eigen::VectorXd m_deltaJointsPositionReference;
    Eigen::VectorXd m_thrustReference;
    Eigen::VectorXd m_thrustDotReference;
    Eigen::VectorXd m_throttleReference;
    Eigen::VectorXd m_statesSolution;
    Eigen::VectorXd m_inputSolution;
    Eigen::VectorXd m_finalState;
};

#endif // VARIABLE_SAMPLING_MPC_H
