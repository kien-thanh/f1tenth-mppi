import os
from abc import ABC, abstractmethod

import numpy as np
from acados_template import AcadosModel
from casadi import MX, vertcat, cos, sin, tan, Function

class BaseVehicleModel(ABC):
    """
    A base class for the vehicle model.

    Attributes
    ----------
    L : float
        The length of the wheelbase (unit: m).
    model : acados_template.AcadosModel
        The model used for acados optimization.

    Methods
    -------
    setup_model() :
        Abstract method to be implemented in subclasses to setup the model dynamics.
    """
    def __init__(self, L):
        self.L = L
        self.model = AcadosModel()
        self.setup_model()

    @abstractmethod
    def setup_model(self):
        pass

class ModelInputAccelSteering(BaseVehicleModel):
    """
    A subclass of BaseVehicleModel.
    States x = [x, y, theta, v]^T,
    Inputs u = [a, delta]^T,
    State-space model xdot = [v*cos(theta), v*sin(theta), v*tan(theta)/L, a]^T

    Methods
    -------
    setup_model() :
        Implements the setup of the model.
        Following properties are set:
        - name: model name
        - x: CasADi system state variables
        - u: CasADi system input variables
        - xdot: CasADi system state variable derivatives
        - f_expl_expr: CasADi expression for the explicit dynamics xdot = f(x,u), use when in combination with
          explicit Runge-Kutta (ERK) integrator.
        - f_impl_expr: CasADi expression for the implicit dynamics f(xdot, x,u)=0, use when in combination with
          implicit Runge-Kutta (IRK) integrator.
        - x_labels: state variable names
        - y_labels: input variable names
        - t_label: time variable label
    """
    def setup_model(self):
        # state variables
        X = MX.sym("X")
        Y = MX.sym("Y")
        theta = MX.sym("theta")  # heading
        v = MX.sym("v")

        x = vertcat(X, Y, theta, v)

        # input variables
        a = MX.sym("a")
        delta = MX.sym("delta")  # steering angle

        u = vertcat(a, delta)

        # xdot
        Xdot = MX.sym("Xdot")
        Ydot = MX.sym("Ydot")
        thetadot = MX.sym("thetadot")
        vdot = MX.sym("vdot")

        xdot = vertcat(Xdot, Ydot, thetadot, vdot)

        # dynamics xdot = f(x,u)
        f_expl_expr = vertcat(
            v * cos(theta),
            v * sin(theta),
            v * tan(delta) / self.L,
            a,
        )

        f_impl_expr = xdot - f_expl_expr

        self.model.name = "model_input_accel_steering"
        self.model.x = x
        self.model.u = u
        self.model.xdot = xdot
        self.model.f_expl_expr = f_expl_expr
        self.model.f_impl_expr = f_impl_expr

        # store meta information
        self.model.x_labels = ["x", "y", "theta", "v"]
        self.model.u_labels = ["a", "delta"]
        self.model.t_label = "t"


class ModelInputAccelSteeringrate(BaseVehicleModel):
    """
    A subclass of BaseVehicleModel.
    States x = [x, y, theta, v, delta]^T,
    Inputs u = [a, omega]^T,
    State-space model xdot = [v*cos(theta), v*sin(theta), v*tan(theta)/L, a, omega]^T

    Methods
    -------
    setup_model() :
        Implements the setup of the model.
    """
    def setup_model(self):
        # state variables
        X = MX.sym("X")
        Y = MX.sym("Y")
        theta = MX.sym("theta")  # heading
        v = MX.sym("v")
        delta = MX.sym("delta")  # steering angle

        x = vertcat(X, Y, theta, v, delta)

        # input variables
        a = MX.sym("a")
        omega = MX.sym("omega")  # steering rate

        u = vertcat(a, omega)

        # xdot
        Xdot = MX.sym("Xdot")
        Ydot = MX.sym("Ydot")
        thetadot = MX.sym("thetadot")
        vdot = MX.sym("vdot")
        deltadot = MX.sym("deltadot")

        xdot = vertcat(Xdot, Ydot, thetadot, vdot, deltadot)

        # dynamics xdot = f(x,u)
        f_expl_expr = vertcat(
            v*cos(theta),
            v*sin(theta),
            v*tan(delta)/ self.L,
            a,
            omega,
        )

        f_impl_expr = xdot - f_expl_expr

        self.model.name = "model_input_accel_steeringrate"
        self.model.x = x
        self.model.u = u
        self.model.xdot = xdot
        self.model.f_expl_expr = f_expl_expr
        self.model.f_impl_expr = f_impl_expr

        # store meta information
        self.model.x_labels = ["x", "y", "theta", "v", "delta"]
        self.model.u_labels = ["a", "omega"]
        self.model.t_label = "t"
