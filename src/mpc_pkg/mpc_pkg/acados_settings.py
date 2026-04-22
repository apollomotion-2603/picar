"""
Acados OCP solver setup for 6-state spatial bicycle model.

Based on Nhã's acados_settings.py (test10/src/race_cars),
adapted for Phi's package structure and configurable params.

Track curvature (kappa) is a runtime parameter: call
    acados_solver.set(j, "p", np.array([kappa_j]))
for each stage j before every solve() call.
"""

import os
import numpy as np
import scipy.linalg

os.environ['ACADOS_SOURCE_DIR'] = os.path.expanduser('~/acados')
from acados_template import AcadosModel, AcadosOcp, AcadosOcpSolver

from .bicycle_model import bicycle_model


def acados_settings(
    Tf,
    N,
    build_dir,
    # Vehicle dynamic params (passed from yaml or defaults)
    m=1.5,
    lf=0.11,
    lr=0.11,
    Cm1=0.28,
    Cm2=0.05,
    Cr0=0.006,
    Cr2=0.011,
    cr3=5.0,
    # Bounds
    n_max=0.20,
    delta_max=0.6109,
    throttle_min=-1.0,
    throttle_max=1.0,
    ddelta_max=2.0,
    dthrottle_max=10.0,
    alat_max=4.0,
    along_max=4.0,
):
    """
    Build and return the acados OCP solver for the 6-state spatial bicycle model.

    Args:
        Tf        : prediction horizon length [s]
        N         : number of discretisation steps
        build_dir : absolute path to solver build directory

    Returns:
        (constraint, model, acados_solver)
    """
    os.makedirs(build_dir, exist_ok=True)
    os.chdir(build_dir)

    # ── Build model (parameterized) ───────────────────────────────────
    model, constraint = bicycle_model(
        m=m, lf=lf, lr=lr,
        Cm1=Cm1, Cm2=Cm2, Cr0=Cr0, Cr2=Cr2, cr3=cr3,
        n_min=-n_max, n_max=n_max,
        throttle_min=throttle_min, throttle_max=throttle_max,
        delta_min=-delta_max, delta_max=delta_max,
        ddelta_min=-ddelta_max, ddelta_max=ddelta_max,
        dthrottle_min=-dthrottle_max, dthrottle_max=dthrottle_max,
        alat_min=-alat_max, alat_max=alat_max,
        along_min=-along_max, along_max=along_max,
    )

    # ── Create OCP ────────────────────────────────────────────────────
    ocp = AcadosOcp()

    # Fill AcadosModel
    model_ac = AcadosModel()
    model_ac.f_impl_expr = model.f_impl_expr
    model_ac.f_expl_expr = model.f_expl_expr
    model_ac.x    = model.x
    model_ac.xdot = model.xdot
    model_ac.u    = model.u
    model_ac.z    = model.z
    model_ac.p    = model.p    # np = 1 : kappa_p
    model_ac.con_h_expr = constraint.expr
    model_ac.name = model.name
    ocp.model = model_ac

    # ── Dimensions ────────────────────────────────────────────────────
    nx  = model.x.rows()    # 6
    nu  = model.u.rows()    # 2
    ny  = nx + nu           # 8  (stage cost residuals)
    ny_e = nx               # 6  (terminal cost residuals)

    ocp.solver_options.N_horizon = N
    ns  = 2
    nsh = 2

    # ── Cost (LINEAR_LS) ──────────────────────────────────────────────
    # Lane-following weights (NOT racing: penalize n, α heavily)
    #   s:     small  — progress is secondary to staying centered
    #   n:     large  — penalize lateral offset strongly
    #   α:     large  — penalize heading error
    #   v:     small  — velocity tracked by adaptive reference
    #   D:     small  — regularise throttle state
    #   δ:     moderate — prevent steering oscillation on straights
    #
    # Re-tuned 2026-04-22 after C2→lf/lr model fix:
    #   Old model C2=15.5 gave α̇ gain ~15.5, new correct gain ~4.55.
    #   Solver now uses ~3.4× larger δ → steering weights ×10 (≈3.4²)
    #   to restore straight-line stability.
    Q = np.diag([1e-1, 5e1, 1e1, 1e-1, 1e-3, 5e-2])
    R = np.eye(nu)
    R[0, 0] = 1e-3   # derD weight
    R[1, 1] = 5e-2   # derDelta weight (5e-3→5e-2: penalize steering oscillation)
    Qe = np.diag([5e-1, 1e2, 2e1, 1e-1, 5e-3, 2e-2])

    ocp.cost.cost_type   = "LINEAR_LS"
    ocp.cost.cost_type_e = "LINEAR_LS"
    unscale = N / Tf

    ocp.cost.W   = unscale * scipy.linalg.block_diag(Q, R)
    ocp.cost.W_e = Qe / unscale

    Vx = np.zeros((ny, nx))
    Vx[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx = Vx

    Vu = np.zeros((ny, nu))
    Vu[6, 0] = 1.0
    Vu[7, 1] = 1.0
    ocp.cost.Vu = Vu

    Vx_e = np.zeros((ny_e, nx))
    Vx_e[:nx, :nx] = np.eye(nx)
    ocp.cost.Vx_e = Vx_e

    ocp.cost.zl = 100 * np.ones((ns,))
    ocp.cost.Zl =   0 * np.ones((ns,))
    ocp.cost.zu = 100 * np.ones((ns,))
    ocp.cost.Zu =   0 * np.ones((ns,))

    # Initial references
    ocp.cost.yref   = np.array([1, 0, 0, 0, 0, 0, 0, 0])
    ocp.cost.yref_e = np.array([0, 0, 0, 0, 0, 0])

    # ── Constraints ───────────────────────────────────────────────────
    ocp.constraints.lbx    = np.array([-12])
    ocp.constraints.ubx    = np.array([ 12])
    ocp.constraints.idxbx  = np.array([1])

    ocp.constraints.lbu   = np.array([model.dthrottle_min, model.ddelta_min])
    ocp.constraints.ubu   = np.array([model.dthrottle_max, model.ddelta_max])
    ocp.constraints.idxbu = np.array([0, 1])

    ocp.constraints.lh = np.array([
        constraint.along_min,
        constraint.alat_min,
        model.n_min,
        model.throttle_min,
        model.delta_min,
    ])
    ocp.constraints.uh = np.array([
        constraint.along_max,
        constraint.alat_max,
        model.n_max,
        model.throttle_max,
        model.delta_max,
    ])
    ocp.constraints.lsh    = np.zeros(nsh)
    ocp.constraints.ush    = np.zeros(nsh)
    ocp.constraints.idxsh  = np.array([0, 2])

    # Initial condition
    ocp.constraints.x0 = model.x0

    # Runtime parameter initialisation (kappa_p = 0 until first callback)
    ocp.parameter_values = np.array([0.0])

    # ── Solver options ────────────────────────────────────────────────
    ocp.solver_options.tf              = Tf
    ocp.solver_options.qp_solver       = "PARTIAL_CONDENSING_HPIPM"
    ocp.solver_options.nlp_solver_type = "SQP_RTI"
    ocp.solver_options.hessian_approx  = "GAUSS_NEWTON"
    ocp.solver_options.integrator_type = "ERK"
    ocp.solver_options.sim_method_num_stages = 4
    ocp.solver_options.sim_method_num_steps  = 3
    ocp.solver_options.print_level     = 0

    # ── Create solver ─────────────────────────────────────────────────
    json_file = os.path.join(build_dir, "acados_ocp_6state.json")
    acados_solver = AcadosOcpSolver(ocp, json_file=json_file)

    return constraint, model, acados_solver
