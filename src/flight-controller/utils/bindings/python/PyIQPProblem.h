#include <pybind11/pybind11.h>
#include <pybind11/stl.h>
#include <pybind11/complex.h>
#include <pybind11/functional.h>
#include <pybind11/chrono.h>
#include "IQPProblem.h"

class PyIQPProblem : public IQPProblem
{
    using IQPProblem::IQPProblem;
    
    const bool setCostAndConstraints(
        std::weak_ptr<BipedalLocomotion::ParametersHandler::IParametersHandler> parametersHandler,
        QPInput& qpInput) override
    {
        PYBIND11_OVERLOAD_PURE(
            const bool,
            IQPProblem,
            setCostAndConstraints,
            parametersHandler,
            qpInput);
    }
    
};
