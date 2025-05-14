import scipy
from acados_template import AcadosOcp, AcadosOcpSolver
import numpy as np
from mpc.vehicle_model import ModelInputAccelSteering, ModelInputAccelSteeringrate


class MPCsolver:
    """
    A MPC solver class utilizing Acados.

    Attributes
    ----------
    model_type : str
        Model type.
    L : float
        Vehicle wheelbase (unit: m).
    vref : float
        Vehicle reference velocity (unit: m/s).
    Tf : float
        MPC predict horizon (unit: s).
    N:  int
        MPC number of shooting nodes.
    n_drive_control: int
    model: acados_template.AcadosModel
        The model used for acados optimization.
    state_names: list of str
        State names, e.g., ["x", "y", "theta", "v"]
    input_names: list of str
        Input names, e.g., ["a", "delta"]
    nx: int
        Number of states.
    nu: int
        Number of inputs.
    ny: int
        y = [[x],[u]], number of states and inputs.
    ny_e: int
        y_e = x_e, number of terminal states.
    ocp: acados_template.AcadosOcp
        Instance of class AcadosOcp, to be set up for formulating an optical control problem.
    ocp_solver: acados_template.AcadosOcpSolver
        Instance of class AcadosOcpSolver, to be set up using an instance of class AcadosOcp
    Methods
    -------
    setup_ocp(ocp_config)
        Formulates the optical control problem based on the given configuration.
    """
    def __init__(self, config):
        self.model_type = config["model_type"]
        self.L = config["L"]
        self.vref = config["vref"]

        ocp_config = config["models"][self.model_type]

        horizon_config = config["horizon"]
        self.Tf = horizon_config["Tf"]
        self.N = horizon_config["N"]
        self.n_drive_control = horizon_config["n_drive_control"]

        self.model = self.setup_model()
        self.state_names = self.model.x_labels
        self.input_names = self.model.u_labels
        # dimensions
        self.nx = self.model.x.rows()
        self.nu = self.model.u.rows()
        self.ny = self.nx + self.nu
        self.ny_e = self.nx
        # create ocp object to formulate the OCP
        self.ocp = AcadosOcp()
        self.setup_ocp(ocp_config)
        self.ocp_solver = AcadosOcpSolver(self.ocp)

    def setup_model(self):
        if self.model_type == "model_input_accel_steering":
            model = ModelInputAccelSteering(self.L)
            return model.model
        elif self.model_type == "model_input_accel_steeringrate":
            model = ModelInputAccelSteeringrate(self.L)
            return model.model
        raise ValueError(f"{self.model_type} is not supported")

    def setup_ocp(self, ocp_config):
        """
        Here set up model (AcadosModel), cost(AcadosCost), constraints(AcadosOcpConstraints),
        solver_options (AcadosOcpOptions) for ocp(AcadosOcp).

        Parameters
        ----------
        ocp_config: dict
            The dictionary structure is as follows:
            - 'constraints' : dict
                Has the following keys:
                - 'x_min' : list of float or None
                    List of lower bounds of each state, float if corresponding state is bounded, None if unbounded.
                - 'x_max' : list of float or None
                    List of upper bounds of each state, float if corresponding state is bounded, None if unbounded.
                - 'u_min' : list of float or None
                    List of lower bounds of each input, float if corresponding state is bounded, None if unbounded.
                - 'u_max' : list of float or None
                    List of upper bounds of each input, float if corresponding state is bounded, None if unbounded.
            - 'weights': dict
                Has the following keys:
                - 'Q' : list of float
                    Diagonal terms of MPC Q matrix
                - 'Q_e' : list of float
                    Diagonal terms of MPC Q_e matrix
                - 'R' : list of float
                    Diagonal terms of MPC R matrix

        Note
        ----
        Here the cost properties:
        - cost_type: 'LINEAR_LS',
           stage cost l(x,u)= 1/2 (Vx*x + Vu*u - yref)^T W (Vx*x + Vu*u - yref), y = [[x],[u]]
           terminal cost m(x,u) = 1/2 (Vx_e*x - yref_e)^T W (Vx_e*x - yref_e);
        - Vx, Vx_e:  [[I_nx],[0]]
        - Vu: [[0], [I_nu]]
        - W: diag(Q, R)
        - W_e: Q_e
        - yref: [[xref], [uref]], reference at intermediate shooting nodes (1 to N-1), first set to zeros
        - yref_e: xref_e, reference at terminal shooting node (N), first set to zeros
        """
        # set model
        self.ocp.model = self.model

        # path cost
        self.ocp.cost.cost_type = "LINEAR_LS"

        Vx = np.zeros((self.ny, self.nx))
        Vx[:self.nx, :] = np.eye(self.nx)
        self.ocp.cost.Vx = Vx

        Vu = np.zeros((self.ny, self.nu))
        Vu[-self.nu:, :] = np.eye(self.nu)
        self.ocp.cost.Vu = Vu

        self.ocp.cost.W = scipy.linalg.block_diag(
            np.diag(ocp_config["weights"]["Q"]),
            np.diag(ocp_config["weights"]["R"])
        )

        # terminal cost
        self.ocp.cost.cost_type_e = "LINEAR_LS"

        Vx_e = np.zeros((self.ny_e, self.nx))
        Vx_e[:self.nx, :] = np.eye(self.nx)
        self.ocp.cost.Vx_e = Vx_e

        self.ocp.cost.W_e = np.diag(ocp_config["weights"]["Q_e"])

        # set initial references
        self.ocp.cost.yref = np.zeros((self.ny,))
        self.ocp.cost.yref_e = np.zeros((self.ny_e,))

        # set prediction horizon
        self.ocp.solver_options.N_horizon = self.N
        self.ocp.solver_options.tf = self.Tf

        # set constraints
        self.setup_constraints(ocp_config["constraints"])

        # set solver options
        self.ocp.solver_options.qp_solver = "PARTIAL_CONDENSING_HPIPM"
        self.ocp.solver_options.nlp_solver_type = "SQP_RTI"
        self.ocp.solver_options.hessian_approx = "GAUSS_NEWTON"
        self.ocp.solver_options.integrator_type = "ERK"
        self.ocp.solver_options.sim_method_num_stages = 4
        self.ocp.solver_options.nlp_solver_max_iter = 200
        self.ocp.solver_options.qp_solver_iter_max = 400
        

    def setup_constraints(self, constraints):
        """ Set constraints(AcadosOcpConstraints) properties: lbx, ubx, idxbx, lbu, ubu, idxbu, x0"""
        def set_bounds(attr, min_key, max_key):
            lb = self.get_constraint_bounds(constraints[min_key])
            if lb.size > 0:
                setattr(self.ocp.constraints, f"l{attr}", lb)

            ub = self.get_constraint_bounds(constraints[max_key])
            if ub.size > 0:
                setattr(self.ocp.constraints, f"u{attr}", ub)

            idx = self.get_constraint_indices(constraints[min_key])
            if idx.size > 0:
                setattr(self.ocp.constraints, f"idx{attr}", idx)

        self.ocp.constraints.x0 = np.zeros((self.nx,))
        set_bounds("bu", "u_min", "u_max")
        set_bounds("bx", "x_min", "x_max")
        # set_bounds("bx_e", "x_min", "x_max")

    def get_constraint_indices(self, constraint):
        indices = [i for i, b in enumerate(constraint) if b is not None]
        return np.array(indices)

    def get_constraint_bounds(self, constraint):
        bounds = [b for b in constraint if b is not None]
        return np.array(bounds)

    def solve(self, x0, yref, yref_e):
        """
        In each odom_callback (see mpc_node.py), solve the MPC OCP problem, get predicted x,u.

        Parameters
        ----------
        x0: ndarray
            (nx,), initial state, the current state of the vehicle.
        yref: ndarray
            (nx+nu, N), yref = [[xref],[uref]] for 0 ~ N-1 shooting node.
        yref_e: ndarray
            (nx,), yref_e = x_e for the terminal (N-th) node.
        Returns
        -------
        simX: ndarray
            (nx, N+1), predicted x.
        simU: ndarray
            (nu, N), predicted u.
        """
        for i in range(self.N):
            self.ocp_solver.set(i, "yref", yref[i])
        self.ocp_solver.set(self.N, "yref", yref_e)
        self.ocp_solver.set(0, "lbx", x0)
        self.ocp_solver.set(0, "ubx", x0)

        simX = np.zeros((self.N + 1, self.nx))
        simU = np.zeros((self.N, self.nu))

        status = self.ocp_solver.solve()
        # self.ocp_solver.print_statistics()  # encapsulates: stat = ocp_solver.get_stats("statistics")

        if status != 0:
            #raise Exception(f"acados returned status {status}.")
            print(f"acados returned status {status}.")

        # get solution
        for i in range(self.N):
            simX[i, :] = self.ocp_solver.get(i, "x")
            simU[i, :] = self.ocp_solver.get(i, "u")
        simX[self.N, :] = self.ocp_solver.get(self.N, "x")

        return simX, simU
