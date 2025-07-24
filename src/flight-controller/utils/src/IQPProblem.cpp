#include "IQPProblem.h"

const bool IQPProblem::configure(
    std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
    QPInput& qpInput)
{

    m_debugModeActive = false;

    if (!setCostAndConstraints(parametersHandler, qpInput))
    {
        yError() << "QPProblem::configure: error in setting the list of costs and constraints";
        return false;
    }
    for (auto& cost : m_vectorCosts)
    {
        if (!cost->readConfigParameters(parametersHandler, qpInput))
        {
            const auto& costRef = *cost; // Dereference the pointer first
            std::string costTypeName = boost::core::demangle(typeid(costRef).name());
            yError() << "QPProblem::configure: error in reading the configuration parameters of "
                     << costTypeName;
            return false;
        }
    }
    for (auto& constraint : m_vectorConstraints)
    {
        if (!constraint->readConfigParameters(parametersHandler, qpInput))
        {
            const auto& constrRef = *constraint; // Dereference the pointer first
            std::string constrTypeName = boost::core::demangle(typeid(constrRef).name());
            yError() << "QPProblem::configure: error in reading the configuration parameters of "
                     << constrTypeName;
            return false;
        }
    }

    // resize matrices
    m_hessian.resize(m_nVar, m_nVar);
    m_gradient.resize(m_nVar);
    m_linearMatrix.resize(0, m_nVar);
    m_lowerBound.resize(0);
    m_upperBound.resize(0);

    m_hessian.setZero();
    m_gradient.setZero();
    m_linearMatrix.setZero();
    m_lowerBound.setZero();
    m_upperBound.setZero();

    // configure costs
    for (auto& cost : m_vectorCosts)
    {
        cost->configureDynVectorsSize(qpInput);
        cost->configureSizeHessianAndGradient();
    }
    for (auto& cost : m_vectorCosts)
    {
        const auto& costRef = *cost; // Dereference the pointer first
        std::string costTypeName = boost::core::demangle(typeid(costRef).name());
        if (!cost->computeHessianAndGradient(qpInput))
        {
            yError() << "QPProblem::configure: error in cost computation of " << costTypeName;
            return false;
        }
        yDebug() << "QPProblem::configure: computation of " << costTypeName << " done";

        if (cost->getHessian().rows() != m_nVar || cost->getHessian().cols() != m_nVar)
        {
            yError() << "QPProblem::configure: error in cost computation of " << costTypeName
                     << " Hessian matrix has wrong size!";
            return false;
        }
        m_hessian += cost->getHessian();
        if (cost->getGradient().rows() != m_nVar)
        {
            yError() << "QPProblem::configure: error in cost computation of " << costTypeName
                     << " gradient vector has wrong size!";
            return false;
        }
        m_gradient += cost->getGradient();
    }

    // configure constraints
    m_countConstraints = 0;
    for (auto& constraint : m_vectorConstraints)
    {
        constraint->configureDynVectorsSize(qpInput);
        constraint->configureSizeConstraintMatrixAndBounds();
        m_linearMatrix.conservativeResize(m_countConstraints + constraint->getNConstraints(),
                                          Eigen::NoChange);
        m_lowerBound.conservativeResize(m_countConstraints + constraint->getNConstraints());
        m_upperBound.conservativeResize(m_countConstraints + constraint->getNConstraints());
        m_countConstraints += constraint->getNConstraints();
    }
    m_linearMatrix.setZero();
    m_lowerBound.setZero();
    m_upperBound.setZero();
    m_countConstraints = 0;
    for (auto& constraint : m_vectorConstraints)
    {
        const auto& constrRef = *constraint; // Dereference the pointer first
        std::string constrTypeName = boost::core::demangle(typeid(constrRef).name());
        if (!constraint->computeConstraintsMatrixAndBounds(qpInput))
        {
            yError() << "QPProblem::configure: error in constraint computation of "
                     << constrTypeName;
            return false;
        }
        yDebug() << "QPProblem::configure: computation of " << constrTypeName << " done";

        if (constraint->getLinearConstraintMatrix().rows() != constraint->getNConstraints()
            || constraint->getLinearConstraintMatrix().cols() != m_nVar)
        {
            yError() << "QPProblem::configure: error in constraint computation of "
                     << constrTypeName << " linear matrix has wrong size!";
            return false;
        }
        m_linearMatrix.block(m_countConstraints, 0, constraint->getNConstraints(), m_nVar)
            = constraint->getLinearConstraintMatrix();
        if (constraint->getLowerBound().rows() != constraint->getNConstraints())
        {
            yError() << "QPProblem::configure: error in constraint computation of "
                     << constrTypeName << " lower bound vector has wrong size!";
            return false;
        }
        m_lowerBound.segment(m_countConstraints, constraint->getNConstraints())
            = constraint->getLowerBound();
        if (constraint->getUpperBound().rows() != constraint->getNConstraints())
        {
            yError() << "QPProblem::configure: error in constraint computation of "
                     << constrTypeName << " upper bound vector has wrong size!";
            return false;
        }
        m_upperBound.segment(m_countConstraints, constraint->getNConstraints())
            = constraint->getUpperBound();
        m_countConstraints += constraint->getNConstraints();
    }

    // configure vectors collection server
    this->configureVectorsCollectionServerIQP(qpInput);

    m_nConstraints = m_countConstraints;
    m_solver.settings()->setWarmStart(true);
    m_solver.settings()->setVerbosity(false);
    m_solver.data()->setNumberOfVariables(m_nVar);
    m_solver.data()->setNumberOfConstraints(m_nConstraints);
    m_outputQP = Eigen::VectorXd::Zero(m_nVar);

    // Scaling
    m_scalingMatrix = Eigen::MatrixXd::Identity(m_nVar, m_nVar);
    m_hessianScaled = Eigen::MatrixXd::Zero(m_nVar, m_nVar);
    m_gradientScaled = Eigen::VectorXd::Zero(m_nVar);
    m_linearMatrixScaled = Eigen::MatrixXd::Zero(m_nConstraints, m_nVar);
    m_outputQPScaled = Eigen::VectorXd::Zero(m_nVar);

    return true;
}

const bool IQPProblem::update(QPInput& qpInput)
{
    m_hessian.setZero();
    m_gradient.setZero();
    for (auto& cost : m_vectorCosts)
    {
        if (!cost->computeHessianAndGradient(qpInput))
        {
            const auto& costRef = *cost; // Dereference the pointer first
            std::string costTypeName = boost::core::demangle(typeid(costRef).name());
            yError() << "QPProblem::update: error in cost computation of " << costTypeName;
            return false;
        }
        m_hessian += cost->getHessian();
        m_gradient += cost->getGradient();
    }
    m_countConstraints = 0;
    for (auto& constraint : m_vectorConstraints)
    {
        if (!constraint->computeConstraintsMatrixAndBounds(qpInput))
        {
            const auto& constrRef = *constraint; // Dereference the pointer first
            std::string constrTypeName = boost::core::demangle(typeid(constrRef).name());
            yError() << "QPProblem::update: error in constraint computation of " << constrTypeName;
            return false;
        }
        m_linearMatrix.block(m_countConstraints, 0, constraint->getNConstraints(), m_nVar)
            = constraint->getLinearConstraintMatrix();
        m_lowerBound.segment(m_countConstraints, constraint->getNConstraints())
            = constraint->getLowerBound();
        m_upperBound.segment(m_countConstraints, constraint->getNConstraints())
            = constraint->getUpperBound();
        m_countConstraints += constraint->getNConstraints();
    }
    return true;
}

const bool IQPProblem::solve()
{
    m_exitFlagQPProblem
        = OsqpEigen::ErrorExitFlag::DataValidationError; /** Initialize
                                                            m_exitFlagQPProblem with
                                                            an error to prevent
                                                            m_exitFlagQPProblem from
                                                            being equal to
                                                            OsqpEigen::ErrorExitFlag::NoError
                                                            if code fails before
                                                            m_exitFlagQPProblem =
                                                            m_solver.solveProblem().
                                                          */

    // Convert matrices to sparse

    m_hessianScaled = m_scalingMatrix.transpose() * m_hessian * m_scalingMatrix;
    m_gradientScaled = m_scalingMatrix.transpose() * m_gradient;
    m_linearMatrixScaled = m_linearMatrix * m_scalingMatrix;

    m_hessianSparseScaled = m_hessianScaled.sparseView();
    m_linearMatrixSparseScaled = m_linearMatrixScaled.sparseView();

    // Debugging
    // std::cout << "Hessian: " << m_hessian << std::endl;
    // std::cout << "Gradient: " << m_gradient << std::endl;
    // std::cout << "Linear Matrix: " << m_linearMatrix << std::endl;
    // std::cout << "Lower Bound: " << m_lowerBound << std::endl;
    // std::cout << "Upper Bound: " << m_upperBound << std::endl;

    if (!m_solver.isInitialized())
    {
        yInfo() << "QPProblem::solve : initialising solver";
        if (!m_solver.data()->setHessianMatrix(m_hessianSparseScaled))
        {
            yError() << "QPProblem::solve : error in setting hessian matrix";
            return false;
        }
        if (!m_solver.data()->setGradient(m_gradientScaled))
        {
            yError() << "QPProblem::solve : error in setting gradient";
            return false;
        }
        if (!m_solver.data()->setLinearConstraintsMatrix(m_linearMatrixSparseScaled))
        {
            yError() << "QPProblem::solve : error in setting linear matrix";
            return false;
        }
        if (!m_solver.data()->setLowerBound(m_lowerBound))
        {
            yError() << "QPProblem::solve : error in setting lower bound";
            return false;
        }
        if (!m_solver.data()->setUpperBound(m_upperBound))
        {
            yError() << "QPProblem::solve : error in setting upper bound";
            return false;
        }
        if (!m_solver.initSolver())
        {
            yError() << "QPProblem::solve : error in initializing solver";
            printMatricesByTask();
            return false;
        }
    } else
    {
        if (!m_solver.updateHessianMatrix(m_hessianSparseScaled))
        {
            yError() << "QPProblem::solve : error in updating hessian matrix";
            return false;
        }
        if (!m_solver.updateGradient(m_gradientScaled))
        {
            yError() << "QPProblem::solve : error in updating gradient";
            return false;
        }
        if (!m_solver.updateLinearConstraintsMatrix(m_linearMatrixSparseScaled))
        {
            yError() << "QPProblem::solve : error in updating linear matrix";
            return false;
        }
        if (!m_solver.updateBounds(m_lowerBound, m_upperBound))
        {
            yError() << "QPProblem::solve : error in updating bounds";
            return false;
        }
    }
    m_exitFlagQPProblem = m_solver.solveProblem();
    if (m_exitFlagQPProblem != OsqpEigen::ErrorExitFlag::NoError)
    {
        yError() << "QPProblem::solve : error in solving problem";
        return false;
    }
    m_statusQPProblem = m_solver.getStatus();
    if (m_statusQPProblem != OsqpEigen::Status::Solved
        && m_statusQPProblem != OsqpEigen::Status::SolvedInaccurate)
    {
        yWarning() << "QPProblem::solve : osqp was not able to find a feasible solution";
    }
    if (m_statusQPProblem == OsqpEigen::Status::SolvedInaccurate)
    {
        yWarning() << "QPProblem::solve : osqp found an inaccurate feasible solution.";
    }

    m_outputQPScaled = m_solver.getSolution();
    m_outputQP = m_scalingMatrix * m_outputQPScaled;
    m_costValue = 0.5 * (m_outputQP.transpose() * m_hessian * m_outputQP)(0, 0)
                  + (m_gradient.transpose() * m_outputQP)(0, 0);
    return true;
}

const bool IQPProblem::configureVectorsCollectionServerIQP(QPInput& qpInput)
{
    std::string nameTag;
    m_vectorsCollectionServer = qpInput.getVectorsCollectionServer();
    {
        nameTag = removeNamespace(boost::core::demangle(typeid(*this).name()));
        m_vectorsCollectionServer->populateMetadata(nameTag + "::solution", {});
        m_vectorsCollectionServer->populateMetadata(nameTag + "::solved", {});
        if (m_debugModeActive)
        {
            m_vectorsCollectionServer->populateMetadata(nameTag + "::cost", {});
            m_vectorsCollectionServer->populateMetadata(nameTag + "::hessianDiagonal", {});
        }
    }
    for (auto& cost : m_vectorCosts)
    {
        if (m_debugModeActive)
        {
            const auto& costRef = *cost; // Dereference the pointer first
            std::string costTypeName = boost::core::demangle(typeid(costRef).name());
            nameTag = removeNamespace(costTypeName);
            m_vectorsCollectionServer->populateMetadata(nameTag + "::xHx_gx", {});
        }
        cost->configureVectorsCollectionServer(qpInput);
    }
    for (auto& constraint : m_vectorConstraints)
    {
        if (m_debugModeActive)
        {
            const auto& constrRef = *constraint; // Dereference the pointer first
            std::string constrTypeName = boost::core::demangle(typeid(constrRef).name());
            nameTag = removeNamespace(constrTypeName);
            for (unsigned int i = 0; i < constraint->getNConstraints(); i++)
            {
                m_vectorsCollectionServer->populateMetadata(
                    nameTag + "::constraint_"
                        + convertNumberToStringWithDigits(i,
                                                          std::to_string(
                                                              constraint->getNConstraints() - 1)
                                                              .length()),
                    {"lb", "Ax", "ub"});
            }
        }
        constraint->configureVectorsCollectionServer(qpInput);
    }
    yInfo() << "QPProblem::configureVectorsCollectionServerIQP : configuration vectors collection "
               "server done!";
    return true;
}

const bool IQPProblem::sendVectorsCollectionToLog(QPInput& qpInput)
{
    std::string nameTag;
    m_vectorsCollectionServer = qpInput.getVectorsCollectionServer();
    {
        nameTag = removeNamespace(boost::core::demangle(typeid(*this).name()));
        m_vectorsCollectionServer->populateData(nameTag + "::solution", m_outputQP);
        double QPFails;
        if (getQPProblemStatus() != OsqpEigen::Status::Solved)
        {
            QPFails = 1.0;
        } else
        {
            QPFails = 0.0;
        }
        m_vectorsCollectionServer->populateData(nameTag + "::solved", std::vector<double>{QPFails});

        if (m_debugModeActive)
        {
            m_vectorsCollectionServer->populateData(nameTag + "::cost",
                                                    std::vector<double>{m_costValue});
            m_vectorsCollectionServer->populateData(nameTag + "::hessianDiagonal",
                                                    m_hessian.diagonal());
        }
    }
    double costValue;
    Eigen::VectorXd Ax;
    std::vector<double> lbAxUb;
    lbAxUb.resize(3);
    for (auto& cost : m_vectorCosts)
    {
        if (m_debugModeActive)
        {
            const auto& costRef = *cost; // Dereference the pointer first
            std::string costTypeName = boost::core::demangle(typeid(costRef).name());
            nameTag = removeNamespace(costTypeName);
            costValue = 0.5 * m_outputQP.transpose() * cost->getHessian() * m_outputQP;
            costValue += m_outputQP.transpose() * cost->getGradient();
            if (m_statusQPProblem != OsqpEigen::Status::Solved
                && m_statusQPProblem != OsqpEigen::Status::SolvedInaccurate)
            {
                costValue = 0.0;
            }
            m_vectorsCollectionServer->populateData(nameTag + "::xHx_gx",
                                                    std::vector<double>{costValue});
        }
        cost->populateVectorsCollection(qpInput, m_outputQP);
    }
    for (auto& constraint : m_vectorConstraints)
    {
        if (m_debugModeActive)
        {
            const auto& constrRef = *constraint; // Dereference the pointer first
            std::string constrTypeName = boost::core::demangle(typeid(constrRef).name());
            nameTag = removeNamespace(constrTypeName);
            Ax.resize(constraint->getNConstraints());
            Ax = constraint->getLinearConstraintMatrix() * m_outputQP;
            for (unsigned int i = 0; i < constraint->getNConstraints(); i++)
            {
                lbAxUb[0] = constraint->getLowerBound()(i);
                lbAxUb[1] = Ax(i);
                lbAxUb[2] = constraint->getUpperBound()(i);
                if (m_statusQPProblem != OsqpEigen::Status::Solved
                    && m_statusQPProblem != OsqpEigen::Status::SolvedInaccurate)
                {
                    lbAxUb[1] = 0.0;
                }
                m_vectorsCollectionServer
                    ->populateData(nameTag + "::constraint_"
                                       + convertNumberToStringWithDigits(i,
                                                                         std::to_string(
                                                                             constraint
                                                                                 ->getNConstraints()
                                                                             - 1)
                                                                             .length()),
                                   lbAxUb);
            }
        }
        constraint->populateVectorsCollection(qpInput, m_outputQP);
    }
    return true;
}

const std::string IQPProblem::removeNamespace(const std::string& nameTag)
{
    std::string nameTagCopy = nameTag;
    size_t pos = nameTagCopy.find("::");
    if (pos != std::string::npos)
    {
        nameTagCopy.erase(0, pos + 2);
    }
    return nameTagCopy;
}

const std::string IQPProblem::convertNumberToStringWithDigits(const int& number, const int& digits)
{
    std::string result = std::to_string(number);
    int numDigits = result.length();
    if (numDigits < digits)
    {
        result = std::string(digits - numDigits, '0') + result;
    }
    return result;
}

void IQPProblem::enableDebugLogMode()
{
    m_debugModeActive = true;
    yWarning() << "QPProblem::enableDebugLogMode : debug mode enabled";
}

Eigen::Ref<const Eigen::VectorXd> IQPProblem::getSolution() const
{
    return m_outputQP;
}

double IQPProblem::getCostValue() const
{
    return m_costValue;
}

const OsqpEigen::ErrorExitFlag IQPProblem::getQPProblemExitFlag()
{
    return m_exitFlagQPProblem;
}

const OsqpEigen::Status IQPProblem::getQPProblemStatus()
{
    return m_statusQPProblem;
}

const unsigned int IQPProblem::getNOptimizationVariables() const
{
    return m_nVar;
}

const unsigned int IQPProblem::getNConstraints() const
{
    return m_nConstraints;
}

Eigen::Ref<const Eigen::MatrixXd> IQPProblem::getHessian() const
{
    return m_hessian;
}

Eigen::Ref<const Eigen::VectorXd> IQPProblem::getGradient() const
{
    return m_gradient;
}

Eigen::Ref<const Eigen::MatrixXd> IQPProblem::getLinearConstraintMatrix() const
{
    return m_linearMatrix;
}

Eigen::Ref<const Eigen::VectorXd> IQPProblem::getLowerBound() const
{
    return m_lowerBound;
}

Eigen::Ref<const Eigen::VectorXd> IQPProblem::getUpperBound() const
{
    return m_upperBound;
}

void IQPProblem::printMatricesByTask() const
{
    for (auto& cost : m_vectorCosts)
    {
        const auto& costRef = *cost; // Dereference the pointer first
        std::string costTypeName = boost::core::demangle(typeid(costRef).name());
        std::cout << "========= " << costTypeName << " =========" << std::endl;
        std::cout << "Hessian:\n" << cost->getHessian() << std::endl;
        std::cout << "Gradient:\n" << cost->getGradient() << std::endl;
        std::cout << "=============================\n" << std::endl;
    }

    for (auto& constraint : m_vectorConstraints)
    {
        const auto& constrRef = *constraint; // Dereference the pointer first
        std::string constrTypeName = boost::core::demangle(typeid(constrRef).name());
        std::cout << "========= " << constrTypeName << " =========" << std::endl;
        std::cout << "Linear Matrix:\n" << constraint->getLinearConstraintMatrix() << std::endl;
        std::cout << "Lower Bound:\n" << constraint->getLowerBound() << std::endl;
        std::cout << "Upper Bound:\n" << constraint->getUpperBound() << std::endl;
        std::cout << "=============================\n" << std::endl;
    }

    std::cout << "=========== Recap ===========" << std::endl;
    std::cout << "N Optimization Variables : " << m_nVar << std::endl;
    std::cout << "N Constraints : " << m_nConstraints << std::endl;
    std::cout << "Hessian:\n" << m_hessian << std::endl;
    std::cout << "Gradient:\n" << m_gradient << std::endl;
    std::cout << "Linear Matrix:\n" << m_linearMatrix << std::endl;
    std::cout << "Lower Bound:\n" << m_lowerBound << std::endl;
    std::cout << "Upper Bound:\n" << m_upperBound << std::endl;
    if (m_exitFlagQPProblem == OsqpEigen::ErrorExitFlag::NoError)
    {
        std::cout << "Solution:\n" << m_outputQP << std::endl;
    }
    std::cout << "=============================\n" << std::endl;
}

void IQPProblem::useScaling(const Eigen::VectorXd& scalingVector)
{
    m_scalingMatrix = scalingVector.asDiagonal();
}
