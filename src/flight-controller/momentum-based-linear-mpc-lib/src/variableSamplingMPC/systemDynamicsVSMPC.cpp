#include "FlightControlUtils.h"
#include <variableSamplingMPC/VSconstant.h>
#include <variableSamplingMPC/systemDynamicsVSMPC.h>
#define EIGEN_INITIALIZE_MATRICES_BY_NAN

AngularMomentumDynamicVS::AngularMomentumDynamicVS(const int nStates,
                                                   const int nJoints,
                                                   const int nThrottle)
    : DynamicTemplateVariableSampling(nStates, nJoints, nThrottle)
{
}

const bool AngularMomentumDynamicVS::configure(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
    QPInput& qpInput)
{
    auto ptr = parametersHandler.lock();
    if (!ptr->getParameter("periodMPC", m_periodMPC))
    {
        yError() << "Parameter 'periodMPC' not found in the config file.";
        return false;
    }
    std::vector<std::string> controlledJoints;
    if (!ptr->getParameter("controlledJoints", controlledJoints))
    {
        yError() << "Parameter 'controlledJoints' not found in the config file.";
        return false;
    }
    std::string jointsLambdaOption;
    if (!ptr->getParameter("jointsLambdaOption", jointsLambdaOption))
    {
        yError() << "Parameter 'jointsLambdaOption' not found in the config file.";
        return false;
    }
    if (jointsLambdaOption == "unfiltered")
    {
        m_jointsLambda = UNFILTERED;
    } else if (jointsLambdaOption == "constant")
    {
        m_jointsLambda = CONSTANT;
    } else
    {
        yError() << "Parameter 'jointsLambdaOption' should be 'unfiltered' or "
                    "'constant'.";
        return false;
    }
    m_robot = qpInput.getRobot();
    m_robotReference = qpInput.getRobotReference();
    m_vectorsCollectionServer = qpInput.getVectorsCollectionServer();
    m_SiAngMom.resize(3, 6);
    m_SiAngMom.setZero();
    m_lambdaAng.resize(3, m_robotReference->getNJoints());
    m_lambdaAngB.resize(3, deltaJointIdx.size());
    m_relJacobianInit = m_robot->getRelativeJacobianJetsBodyFrame();
    m_matJetAxisInit = m_robot->getMatrixOfJetAxes();
    m_matJetArmsInit = m_robot->getMatrixOfJetArms();
    for (auto joint : controlledJoints)
    {
        for (int i = 0; i < m_robot->getNJoints(); i++)
        {
            if (joint == m_robot->getJointName(i))
            {
                m_jointSelectorVector.emplace_back(i);
            }
        }
    }
    m_rpyInit = iDynTree::toEigen(m_robotReference->getBasePose().getRotation().asRPY());

    return true;
}

const bool AngularMomentumDynamicVS::updateInitialStates(QPInput& qpInput)
{
    this->updateRPY();
    this->computeLambdaAng(qpInput);
    return this->computeAngularMomentumMatrices();
}

const bool AngularMomentumDynamicVS::computeAngularMomentumMatrices()
{
    m_ASystem.setZero();
    m_BSystemJoints.setZero();
    m_BSystemThrottle.setZero();
    m_cSystem.setZero();
    // phi_{k+1} = phi_{k} + dt * (I * W)^{-1} * w_{k}
    m_ASystem.block(rpyIdx[0], angMomIdx[0], angMomIdx.size(), angMomIdx.size())
        = m_WInverse * m_inertia.inverse();

    // w_{k+1} = w_{k} + dt * (A * T_{k} - S(omega) * w_{k} + lambda * deltaJoint_{k})
    m_ASystem.block(angMomIdx[0], angMomIdx[0], angMomIdx.size(), angMomIdx.size())
        -= fromVecToSkew(m_B_omega_B);
    m_ASystem.block(angMomIdx[0], thrustIdx[0], angMomIdx.size(), thrustIdx.size())
        = m_robotReference->getMatrixAmomJets(true).bottomRows(3);
    m_BSystemJoints.block(angMomIdx[0], deltaJointIdx[0], angMomIdx.size(), deltaJointIdx.size())
        = m_lambdaAngB;

    // rpyError_{k+1} = rpyError_{k} + dt * (rpy_{k} - rpy_{k}^{ref})
    m_ASystem.block(rpyErrorIdx[0], rpyIdx[0], rpyErrorIdx.size(), rpyIdx.size())
        = Eigen::MatrixXd::Identity(rpyErrorIdx.size(), rpyIdx.size());
    m_cSystem.segment(rpyErrorIdx[0], rpyErrorIdx.size()) = -m_rpyInit;

    return true;
}

void AngularMomentumDynamicVS::updateRPY()
{
    m_wRb = iDynTree::toEigen(m_robotReference->getBasePose().getRotation());
    m_B_omega_B = m_wRb.transpose() * iDynTree::toEigen(m_robot->getBaseVel().getAngularVec3());

    iDynTree::Position r;
    iDynTree::toEigen(r) = m_robotReference->getPositionCoM()
                           - iDynTree::toEigen(m_robotReference->getBasePose().getPosition());
    iDynTree::Transform G_H_B;
    G_H_B.setPosition(r);
    G_H_B.setRotation(m_robotReference->getBasePose().getRotation());
    Eigen::MatrixXd M_b = m_robotReference->getMassMatrix().block(0, 0, 6, 6);
    Eigen::MatrixXd M_bs
        = m_robotReference->getMassMatrix().block(0, 5, 6, m_robotReference->getNJoints());
    Eigen::MatrixXd J_s = M_b.inverse() * M_bs;

    Eigen::MatrixXd G_T_B(6 + m_robotReference->getNJoints(), 6 + m_robotReference->getNJoints());
    G_T_B.setIdentity();
    G_T_B.block(0, 0, 6, 6) = iDynTree::toEigen(G_H_B.asAdjointTransform());
    G_T_B.block(0, 6, 6, m_robotReference->getNJoints()) = J_s;
    Eigen::MatrixXd M_G;
    M_G = G_T_B.inverse().transpose() * m_robotReference->getMassMatrix() * G_T_B.inverse();
    // std::cout << "M_G:\n"
    //           << iDynTree::toEigen(G_H_B.asAdjointTransform()).transpose() * M_b
    //                  * iDynTree::toEigen(G_H_B.asAdjointTransform())
    //           << std::endl;
    m_inertia = (iDynTree::toEigen(G_H_B.asAdjointTransform()).transpose() * M_b
                 * iDynTree::toEigen(G_H_B.asAdjointTransform()))
                    .block(3, 3, 3, 3);

    // std::cout << "determinant inertia:\n" << m_inertia.determinant() << std::endl;

    m_RPY = iDynTree::toEigen(m_robotReference->getBasePose().getRotation().asRPY());
    m_W.setZero();
    m_W(0, 0) = 1.0;
    m_W(1, 1) = cos(m_RPY(0));
    m_W(2, 1) = -sin(m_RPY(0));
    m_W(0, 2) = -sin(m_RPY(1));
    m_W(1, 2) = cos(m_RPY(1)) * sin(m_RPY(0));
    m_W(2, 2) = cos(m_RPY(0)) * cos(m_RPY(1));
    m_WInverse.setZero();
    m_WInverse(0, 0) = 1.0;
    m_WInverse(0, 1) = sin(m_RPY(0)) * tan(m_RPY(1));
    m_WInverse(1, 1) = cos(m_RPY(0));
    m_WInverse(2, 1) = sin(m_RPY(0)) / cos(m_RPY(1));
    m_WInverse(0, 2) = cos(m_RPY(0)) * tan(m_RPY(1));
    m_WInverse(1, 2) = -sin(m_RPY(0));
    m_WInverse(2, 2) = cos(m_RPY(0)) / cos(m_RPY(1));
    m_RPYDot = m_WInverse * m_B_omega_B;
    m_WDot.setZero();
    m_WDot(1, 1) = -m_RPYDot(0) * sin(m_RPY(0));
    m_WDot(2, 1) = -m_RPYDot(0) * cos(m_RPY(0));
    m_WDot(0, 2) = -m_RPYDot(1) * cos(m_RPY(1));
    m_WDot(1, 2) = -m_RPYDot(1) * sin(m_RPY(0)) * sin(m_RPY(1))
                   + m_RPYDot(0) * cos(m_RPY(0)) * cos(m_RPY(1));
    m_WDot(2, 2) = -m_RPYDot(1) * cos(m_RPY(0)) * sin(m_RPY(1))
                   - m_RPYDot(0) * sin(m_RPY(0)) * cos(m_RPY(1));
}

void AngularMomentumDynamicVS::computeLambdaAng(QPInput& qpInput)
{
    int i_lambda = 0;
    m_lambdaAng.setZero();

    switch (m_jointsLambda)
    {
    case UNFILTERED:
        for (auto& jetAxisName : m_robotReference->getJetsList())
        {
            m_lambdaAng.noalias()
                -= m_robotReference->getJetThrusts()[i_lambda]
                   * fromVecToSkew(
                       m_wRb.transpose()
                       * iDynTree::toEigen(m_robotReference->getMatrixOfJetAxes()[i_lambda]))
                   * getRelativeJacobianCoM(jetAxisName);

            m_lambdaAng.noalias()
                -= m_robotReference->getJetThrusts()[i_lambda]
                   * fromVecToSkew(m_wRb.transpose()
                                   * m_robotReference->getMatrixOfJetArms()[i_lambda])
                   * fromVecToSkew(
                       m_wRb.transpose()
                       * iDynTree::toEigen(m_robotReference->getMatrixOfJetAxes()[i_lambda]))
                   * m_robotReference->getRelativeJacobianJetsBodyFrame()[i_lambda].bottomRows(3);
            ++i_lambda;
        }
        break;
    case CONSTANT:
        for (auto& jetAxisName : m_robotReference->getJetsList())
        {
            m_SiAngMom.block(0, 0, 3, 3)
                = fromVecToSkew(m_wRb.transpose() * iDynTree::toEigen(m_matJetAxisInit[i_lambda]));
            m_SiAngMom.block(0, 3, 3, 3)
                = fromVecToSkew(m_wRb.transpose() * m_matJetArmsInit[i_lambda])
                  * fromVecToSkew(m_wRb.transpose()
                                  * iDynTree::toEigen(m_matJetAxisInit[i_lambda]));
            m_SiAngMom *= m_robot->getJetThrusts()[i_lambda];
            m_lambdaAng.noalias() -= m_SiAngMom * m_relJacobianInit[i_lambda];
            ++i_lambda;
        }
        break;
    }
    for (int i = 0; i < m_jointSelectorVector.size(); i++)
    {
        m_lambdaAngB.col(i) = m_lambdaAng.col(m_jointSelectorVector[i]);
    }
}

const Eigen::MatrixXd AngularMomentumDynamicVS::getRelativeJacobianCoM(std::string frameName)
{
    Eigen::MatrixXd jacobian
        = m_robotReference->getJacobian(frameName).topRightCorner(3,
                                                                  m_robotReference->getNJoints());
    iDynTree::Position m_posCoMInBody;
    iDynTree::toEigen(m_posCoMInBody)
        = -iDynTree::toEigen(m_robotReference->getBasePose().getRotation().inverse())
          * m_robotReference->getPositionCoM();
    m_forcesTransform.setPosition(m_posCoMInBody);
    m_forcesTransform.setRotation(m_robotReference->getBasePose().getRotation().inverse());
    jacobian.noalias()
        -= m_robotReference->getJacobianCoM().topRightCorner(3, m_robotReference->getNJoints());

    // WARNING: this relation between jacobian holds only if the frame B (body) is also the
    // floating base frame
    jacobian = iDynTree::toEigen(m_forcesTransform.getRotation()) * jacobian;
    return jacobian;
}

LinearMomentumDynamicVS::LinearMomentumDynamicVS(const int nStates,
                                                 const int nJoints,
                                                 const int nThrottle)
    : DynamicTemplateVariableSampling(nStates, nJoints, nThrottle)
{
}

const bool LinearMomentumDynamicVS::configure(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
    QPInput& qpInput)
{
    auto ptr = parametersHandler.lock();
    if (!ptr->getParameter("periodMPC", m_periodMPC))
    {
        yError() << "Parameter 'periodMPC' not found in the config file.";
        return false;
    }
    std::string jointsLambdaOption;
    if (!ptr->getParameter("jointsLambdaOption", jointsLambdaOption))
    {
        yError() << "Parameter 'jointsLambdaOption' not found in the config file.";
        return false;
    }
    if (jointsLambdaOption == "unfiltered")
    {
        m_jointsLambda = UNFILTERED;
    } else if (jointsLambdaOption == "constant")
    {
        m_jointsLambda = CONSTANT;
    } else
    {
        yError() << "Parameter 'jointsLambdaOption' should be 'unfiltered' or "
                    "'constant'.";
        return false;
    }
    auto paramHandlerYarpImpl
        = std::make_shared<BipedalLocomotion::ParametersHandler::YarpImplementation>();
    auto group = ptr->getGroup("TRAJECTORY_MANAGER");
    if (group.lock() == nullptr)
    {
        yError() << "Group [TRAJECTORY_MANAGER] not found in the config file.";
        return false;
    }
    paramHandlerYarpImpl->setGroup("TRAJECTORY_MANAGER", group.lock());
    m_trajectoryManager.configure(paramHandlerYarpImpl, 1 / m_periodMPC);
    m_robot = qpInput.getRobot();
    m_robotReference = qpInput.getRobotReference();
    m_lambda.resize(3, m_robotReference->getNJoints());
    m_lambdaB.resize(3, deltaJointIdx.size());
    m_relJacobianInit = m_robot->getRelativeJacobianJetsBodyFrame();
    m_matJetAxisInit = m_robot->getMatrixOfJetAxes();
    return true;
}

const bool LinearMomentumDynamicVS::updateInitialStates(QPInput& qpInput)
{
    this->computeLambdaLin(qpInput);
    return this->computeLinearMomentumMatrices(qpInput);
}

const bool LinearMomentumDynamicVS::computeLinearMomentumMatrices(QPInput& qpInput)
{
    m_ASystem.setZero();
    m_BSystemJoints.setZero();
    m_BSystemThrottle.setZero();
    m_cSystem.setZero();

    // CoM_{k+1} = CoM_{k} + dt / mass * w_R_b * linMom_{k}
    m_ASystem.block(CoMPosIdx[0], linMomIdx[0], CoMPosIdx.size(), linMomIdx.size())
        = 1 / m_robotReference->getTotalMass() * m_wRb;

    // linMom_{k+1} = linMom_{k} + dt * (A * T_{k} - S(omega) * linMom_{k} + lambda *
    // deltaJoint_{k} + alpha_{gravity_k} * mass * ^{W}R_{B} * g)
    m_ASystem.block(linMomIdx[0], linMomIdx[0], linMomIdx.size(), linMomIdx.size())
        -= fromVecToSkew(m_B_omega_B);
    m_ASystem.block(linMomIdx[0], thrustIdx[0], linMomIdx.size(), thrustIdx.size())
        = m_robotReference->getMatrixAmomJets(true).topRows(3);
    m_BSystemJoints.block(linMomIdx[0], deltaJointIdx[0], linMomIdx.size(), deltaJointIdx.size())
        = m_lambdaB;
    m_cSystem.segment(linMomIdx[0], linMomIdx.size())
        = m_trajectoryManager.getCurrentValue("alphaGravity")[0] * m_robotReference->getTotalMass()
          * m_wRb.transpose() * iDynTree::toEigen(m_robotReference->getGravity());
    qpInput.setAlphaGravity(m_trajectoryManager.getCurrentValue("alphaGravity")[0]);
    m_trajectoryManager.advanceTrajectory();

    // err_{position, k+1} = err_{position, k} + dt * (CoM_{k} - CoM_{ref, k})
    m_ASystem.block(positionErrorIdx[0], CoMPosIdx[0], positionErrorIdx.size(), CoMPosIdx.size())
        = Eigen::MatrixXd::Identity(positionErrorIdx.size(), CoMPosIdx.size());
    m_cSystem.segment(positionErrorIdx[0], positionErrorIdx.size()) = -qpInput.getPosCoMReference();

    return true;
}

const bool LinearMomentumDynamicVS::computeLambdaLin(QPInput& qpInput)
{
    m_lambda.setZero();
    m_wRb = iDynTree::toEigen(m_robotReference->getBasePose().getRotation());
    m_B_omega_B = m_wRb.transpose() * iDynTree::toEigen(m_robot->getBaseVel().getAngularVec3());
    int count = 0;
    switch (m_jointsLambda)
    {
    case CONSTANT:
        for (auto jetAxis : m_matJetAxisInit)
        {
            m_lambda -= m_robotReference->getJetThrusts()[count]
                        * fromVecToSkew(m_wRb.transpose() * iDynTree::toEigen(jetAxis))
                        * m_relJacobianInit[count].bottomRows(3);
            count++;
        }
        break;
    case UNFILTERED:
        for (auto jetAxis : m_robotReference->getMatrixOfJetAxes())
        {
            m_lambda -= m_robotReference->getJetThrusts()[count]
                        * fromVecToSkew(m_wRb.transpose() * iDynTree::toEigen(jetAxis))
                        * m_robotReference->getRelativeJacobianJetsBodyFrame()[count].bottomRows(3);
            count++;
        }
        break;
    }
    m_lambdaB = m_lambda.middleCols(3, deltaJointIdx.size());
    return true;
}

JetDynamicVS::JetDynamicVS(const int nStates, const int nJoints, const int nThrottle)
    : DynamicTemplateVariableSampling(nStates, nJoints, nThrottle)
{
}

const bool JetDynamicVS::configure(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
    QPInput& qpInput)
{
    auto ptr = parametersHandler.lock();
    if (!ptr->getParameter("periodMPC", m_periodMPC))
    {
        yError() << "Parameter 'periodMPC' not found in the config file.";
        return false;
    }
    if (!ptr->getParameter("useJetDynamic", m_useJetDynamic))
    {
        yError() << "Parameter 'Kt' not found in the config file.";
        return false;
    }
    if (!ptr->getParameter("useEstimatedThrust", m_useEstimatedThrust))
    {
        yError() << "Parameter 'useEstimatedThrust' not found in the config file.";
        return false;
    }
    m_robot = qpInput.getRobot();
    m_robotReference = qpInput.getRobotReference();
    m_jetModel = qpInput.getJetModel();

    return true;
}

const bool JetDynamicVS::updateInitialStates(QPInput& qpInput)
{
    m_ASystem.setZero();
    m_BSystemJoints.setZero();
    m_BSystemThrottle.setZero();
    m_cSystem.setZero();
    if (m_useJetDynamic)
    {
        // T_{k+1} = T_{k} + dt * \dot{T}_{k}
        m_ASystem.block(thrustIdx[0], thrustDotIdx[0], thrustIdx.size(), thrustDotIdx.size())
            = Eigen::MatrixXd::Identity(thrustIdx.size(), thrustDotIdx.size());

        // \dot{T}_{k+1} = \dot{T}_{k}
        // + dt * (f(T_{0}, \dot{T}_{0}) + g(T_{0}, \dot{T}_{0}) * u_{k})

        for (int i = 0; i < m_robot->getNJets(); i++)
        {
            if (m_useEstimatedThrust)
            {
                m_thrust = m_robot->getJetThrusts()[i];
                m_thrustDot = qpInput.getEstimatedThrustDot()[i];
            } else
            {
                m_thrust = qpInput.getThrustDesMPC()[i];
                m_thrustDot = qpInput.getThrustDotDesMPC()[i];
            }
            m_ASystem(thrustDotIdx[i], thrustIdx[i])
                = compute_dh_dT(m_thrust, m_thrustDot, qpInput.getThrottleMPC()[i]);
            m_ASystem(thrustDotIdx[i], thrustDotIdx[i])
                += compute_dh_dTDot(m_thrust, m_thrustDot, qpInput.getThrottleMPC()[i]);
            m_BSystemThrottle(thrustDotIdx[i], throttleIdx[i])
                = computeG(qpInput.getThrustDesMPC()[i], qpInput.getThrustDotDesMPC()[i]);
            m_cSystem(thrustDotIdx[i])
                = computeF(m_thrust, m_thrustDot)
                  - compute_dh_dT(m_thrust, m_thrustDot, qpInput.getThrottleMPC()[i]) * m_thrust
                  - compute_dh_dTDot(m_thrust, m_thrustDot, qpInput.getThrottleMPC()[i])
                        * m_thrustDot;
        }
    } else
    {
        m_BSystemThrottle.block(thrustIdx[0], throttleIdx[0], thrustIdx.size(), throttleIdx.size())
            .setIdentity();
    }

    return true;
}

double JetDynamicVS::computeF(double T, double Tdot)
{
    T = m_jetModel->standardizeThrust_u2T(T);
    Tdot = m_jetModel->standardizeThrustDot_u2T(Tdot);
    return m_jetModel->compute_f(T, Tdot) * m_jetModel->getThrustStandardDeviation_u2T();
}

double JetDynamicVS::computeG(double T, double Tdot)
{
    T = m_jetModel->standardizeThrust_u2T(T);
    Tdot = m_jetModel->standardizeThrustDot_u2T(Tdot);
    return m_jetModel->compute_g(T, Tdot) * m_jetModel->getThrustStandardDeviation_u2T();
}

double JetDynamicVS::compute_dh_dT(double T, double Tdot, double throttle)
{
    T = m_jetModel->standardizeThrust_u2T(T);
    Tdot = m_jetModel->standardizeThrustDot_u2T(Tdot);
    throttle = m_jetModel->standardizeThrottle_u2T(throttle);
    return (m_jetModel->compute_df_dT(T, Tdot)
            + m_jetModel->compute_dg_dT(T, Tdot) * m_jetModel->compute_v(throttle));
}

double JetDynamicVS::compute_dh_dTDot(double T, double Tdot, double throttle)
{
    T = m_jetModel->standardizeThrust_u2T(T);
    Tdot = m_jetModel->standardizeThrustDot_u2T(Tdot);
    throttle = m_jetModel->standardizeThrottle_u2T(throttle);
    return (m_jetModel->compute_df_dTdot(T, Tdot)
            + m_jetModel->compute_dg_dTdot(T, Tdot) * m_jetModel->compute_v(throttle));
}

SystemDynamicVS::SystemDynamicVS(const int nStates, const int nJoints, const int nThrottle)
{
    m_nStates = nStates;
    m_nJoints = nJoints;
    m_nThrottle = nThrottle;
    m_A = Eigen::MatrixXd::Zero(m_nStates, m_nStates);
    m_BJoints = Eigen::MatrixXd::Zero(m_nStates, m_nJoints);
    m_BThrottle = Eigen::MatrixXd::Zero(m_nStates, m_nThrottle);
    m_c = Eigen::VectorXd::Zero(m_nStates);
}

const bool SystemDynamicVS::configure(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
    QPInput& qpInput)
{
    m_vectorDynamic.emplace_back(
        std::make_unique<AngularMomentumDynamicVS>(m_nStates, m_nJoints, m_nThrottle));
    m_vectorDynamic.emplace_back(
        std::make_unique<LinearMomentumDynamicVS>(m_nStates, m_nJoints, m_nThrottle));
    m_vectorDynamic.emplace_back(std::make_unique<JetDynamicVS>(m_nStates, m_nJoints, m_nThrottle));

    for (auto& dynamic : m_vectorDynamic)
    {
        if (!dynamic->configure(parametersHandler, qpInput))
        {
            yError() << "Could not configure the dynamic.";
            return false;
        }
    }
    return true;
}

const bool SystemDynamicVS::updateDynamicMatrices(QPInput& qpInput)
{
    for (auto& dynamic : m_vectorDynamic)
    {
        if (!dynamic->updateInitialStates(qpInput))
        {
            yError() << "Could not update the dynamic matrices of the class: "
                     << boost::core::demangle(typeid(*dynamic).name());
            return false;
        }
    }
    return true;
}

const bool SystemDynamicVS::getAMatrix(Eigen::Ref<Eigen::MatrixXd> A)
{
    if (A.cols() != m_A.cols() || A.rows() != m_A.rows())
    {
        yError() << "SystemDynamicVS::getAMatrix: The size of the input matrix is not correct.";
        return false;
    }
    m_A.setZero();
    Eigen::MatrixXd A_dyn;
    A_dyn.resize(m_nStates, m_nStates);
    for (auto& dynamic : m_vectorDynamic)
    {
        dynamic->getAMatrixDynamicClass(A_dyn);
        m_A += A_dyn;
    }
    A = m_A;
    return true;
}

const bool SystemDynamicVS::getBJointsMatrix(Eigen::Ref<Eigen::MatrixXd> BJoints)
{
    if (BJoints.cols() != m_BJoints.cols() || BJoints.rows() != m_BJoints.rows())
    {
        yError() << "SystemDynamicVS::getBJointsMatrix: The size of the input matrix is not "
                    "correct.";
        return false;
    }
    m_BJoints.setZero();
    Eigen::MatrixXd BJoints_dyn;
    BJoints_dyn.resize(m_nStates, m_nJoints);
    for (auto& dynamic : m_vectorDynamic)
    {
        dynamic->getBJointsMatrixDynamicClass(BJoints_dyn);
        m_BJoints += BJoints_dyn;
    }
    BJoints = m_BJoints;
    return true;
}

const bool SystemDynamicVS::getBThrottleMatrix(Eigen::Ref<Eigen::MatrixXd> BThrottle)
{
    if (BThrottle.cols() != m_BThrottle.cols() || BThrottle.rows() != m_BThrottle.rows())
    {
        yError() << "SystemDynamicVS::getBThrottleMatrix: The size of the input matrix is not "
                    "correct.";
        return false;
    }
    m_BThrottle.setZero();
    Eigen::MatrixXd BThrottle_dyn;
    BThrottle_dyn.resize(m_nStates, m_nThrottle);
    for (auto& dynamic : m_vectorDynamic)
    {
        dynamic->getBThrottleMatrixDynamicClass(BThrottle_dyn);
        m_BThrottle += BThrottle_dyn;
    }
    BThrottle = m_BThrottle;
    return true;
}

const bool SystemDynamicVS::getCVector(Eigen::Ref<Eigen::VectorXd> c)
{
    if (c.size() != m_c.size())
    {
        yError() << "SystemDynamicVS::getCVector: The size of the input vector is not correct.";
        return false;
    }
    m_c.setZero();
    for (auto& dynamic : m_vectorDynamic)
    {
        Eigen::VectorXd c_dyn;
        c_dyn.resize(m_nStates);
        dynamic->getCVectorDynamicClass(c_dyn);
        m_c += c_dyn;
    }
    c = m_c;
    return true;
}
