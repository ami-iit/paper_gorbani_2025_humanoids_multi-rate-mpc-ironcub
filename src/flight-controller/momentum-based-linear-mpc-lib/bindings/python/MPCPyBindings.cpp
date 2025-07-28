#include "PyIMPCProblem.h"
#include <iDynTree/Transform.h>
#include <pybind11/eigen.h>
#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <variableSamplingMPC/variableSamplingMPC.h>
#include <yarp/os/ResourceFinder.h>
#include <yarp/os/Searchable.h>

namespace py = pybind11;

PYBIND11_MODULE(bindingsMPC, m)
{

    py::class_<IMPCProblem, PyIMPCProblem>(m, "IMPCProblem")
        .def(py::init<>())
        .def("setCostAndConstraints",
             &IMPCProblem::setCostAndConstraints,
             py::arg("parametersHandler"),
             py::arg("mpcInput"));

    py::class_<VariableSamplingMPC, IMPCProblem>(m, "VariableSamplingMPC")
        .def(py::init<>())
        .def("configure",
             [](VariableSamplingMPC& self,
                std::shared_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler>
                    parametersHandler,
                QPInput& mpcInput) {
                 std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler>
                     parametersHandlerWeak = parametersHandler;
                 return self.configure(parametersHandlerWeak, mpcInput);
             })
        .def("setCostAndConstraints",
             &IMPCProblem::setCostAndConstraints,
             py::arg("parametersHandler"),
             py::arg("mpcInput"))
        .def("update", &IMPCProblem::update, py::arg("mpcInput"))
        .def("solveMPC", &VariableSamplingMPC::solveMPC)
        .def("getMPCSolution", &VariableSamplingMPC::getMPCSolution)
        .def("getJointsReferencePosition",
             [](VariableSamplingMPC& self) {
                 Eigen::VectorXd jointsReferencePosition;
                 jointsReferencePosition.resize(23);
                 self.getJointsReferencePosition(jointsReferencePosition);
                 return jointsReferencePosition;
             })
        .def("getThrottleReference",
             [](VariableSamplingMPC& self) {
                 Eigen::VectorXd throttleReference(4);
                 self.getThrottleReference(throttleReference);
                 return throttleReference;
             })
        .def("getThrustReference",
             [](VariableSamplingMPC& self) {
                 Eigen::VectorXd jointsReferenceVelocity(4);
                 self.getThrustReference(jointsReferenceVelocity);
                 return jointsReferenceVelocity;
             })
        .def("getThrustDotReference",
             [](VariableSamplingMPC& self) {
                 Eigen::VectorXd thrustDotReference(4);
                 self.getThrustDotReference(thrustDotReference);
                 return thrustDotReference;
             })
        .def("getFinalCoMPosition",
             [](VariableSamplingMPC& self) {
                 Eigen::VectorXd finalCoMPosition;
                 self.getFinalCoMPosition(finalCoMPosition);
                 return finalCoMPosition;
             })
        .def("getFinalLinMom",
             [](VariableSamplingMPC& self) {
                 Eigen::VectorXd finalLinMom;
                 self.getFinalLinMom(finalLinMom);
                 return finalLinMom;
             })
        .def("getFinalRPY",
             [](VariableSamplingMPC& self) {
                 Eigen::VectorXd finalRPY;
                 self.getFinalRPY(finalRPY);
                 return finalRPY;
             })
        .def("getFinalAngMom",
             [](VariableSamplingMPC& self) {
                 Eigen::VectorXd finalAngMom;
                 self.getFinalAngMom(finalAngMom);
                 return finalAngMom;
             })
        .def("getNStatesMPC", &VariableSamplingMPC::getNStatesMPC)
        .def("getNInputMPC", &VariableSamplingMPC::getNInputMPC);
}
