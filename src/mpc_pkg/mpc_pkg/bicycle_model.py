"""
Spatial bicycle model for NMPC — 6-state Kloeser 2020.

State   x = [s, n, alpha, v, D, delta]
Control u = [derD, derDelta]
Param   p = [kappa_p]   -- curvature at current horizon stage [1/m]

Based on Nhã's bicycle_model.py (test10/src/race_cars),
adapted for Phi's RC car Gazebo sim.
"""

import types
import numpy as np
from casadi import *


def bicycle_model(
    m=1.5,
    C1=0.5,
    C2=15.5,
    Cm1=0.28,
    Cm2=0.05,
    Cr0=0.006,
    Cr2=0.011,
    cr3=5.0,
    n_min=-0.20,
    n_max=0.20,
    throttle_min=-1.0,
    throttle_max=1.0,
    delta_min=-0.6109,
    delta_max=0.6109,
    ddelta_min=-2.0,
    ddelta_max=2.0,
    dthrottle_min=-10.0,
    dthrottle_max=10.0,
    alat_min=-4.0,
    alat_max=4.0,
    along_min=-4.0,
    along_max=4.0,
):
    """
    Build 6-state spatial bicycle model.

    Track curvature (kappa) is supplied as a runtime CasADi parameter `kappa_p`
    instead of a compile-time bspline from a track file.

    Returns:
        (model, constraint)  — SimpleNamespace objects
    """
    constraint = types.SimpleNamespace()
    model = types.SimpleNamespace()

    model_name = "Spatialbicycle_model"

    # ── CasADi Model — states ─────────────────────────────────────────
    s     = MX.sym("s")
    n     = MX.sym("n")
    alpha = MX.sym("alpha")
    v     = MX.sym("v")
    D     = MX.sym("D")
    delta = MX.sym("delta")
    x = vertcat(s, n, alpha, v, D, delta)

    # ── Controls ──────────────────────────────────────────────────────
    derD     = MX.sym("derD")
    derDelta = MX.sym("derDelta")
    u = vertcat(derD, derDelta)

    # ── State derivatives (for implicit form) ─────────────────────────
    sdot     = MX.sym("sdot")
    ndot     = MX.sym("ndot")
    alphadot = MX.sym("alphadot")
    vdot     = MX.sym("vdot")
    Ddot     = MX.sym("Ddot")
    deltadot = MX.sym("deltadot")
    xdot = vertcat(sdot, ndot, alphadot, vdot, Ddot, deltadot)

    # ── Algebraic variables (none) ────────────────────────────────────
    z = vertcat([])

    # ── Runtime parameter: track curvature per stage ──────────────────
    kappa_p = MX.sym("kappa_p")   # [1/m]
    p = vertcat(kappa_p)

    # ── Dynamics (spatial bicycle model — Kloeser 2020 Eq. 10) ────────
    Fxd   = (Cm1 - Cm2 * v) * D - Cr2 * v * v - Cr0 * tanh(cr3 * v)
    sdota = (v * cos(alpha + C1 * delta)) / (1 - kappa_p * n)
    f_expl = vertcat(
        sdota,
        v * sin(alpha + C1 * delta),
        v * C2 * delta - kappa_p * sdota,
        Fxd / m * cos(C1 * delta),
        derD,
        derDelta,
    )

    # ── Nonlinear constraint expressions ──────────────────────────────
    a_lat  = C2 * v * v * delta + Fxd * sin(C1 * delta) / m
    a_long = Fxd / m

    # ── Model bounds ──────────────────────────────────────────────────
    model.n_min = n_min
    model.n_max = n_max

    model.throttle_min = throttle_min
    model.throttle_max = throttle_max

    model.delta_min = delta_min
    model.delta_max = delta_max

    model.ddelta_min    = ddelta_min
    model.ddelta_max    = ddelta_max
    model.dthrottle_min = dthrottle_min
    model.dthrottle_max = dthrottle_max

    # ── Constraint bounds ─────────────────────────────────────────────
    constraint.alat_min  = alat_min
    constraint.alat_max  = alat_max
    constraint.along_min = along_min
    constraint.along_max = along_max

    # ── Initial condition ─────────────────────────────────────────────
    model.x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    # ── Constraint struct ─────────────────────────────────────────────
    constraint.alat = Function("a_lat", [x, u, p], [a_lat])
    constraint.expr = vertcat(a_long, a_lat, n, D, delta)

    # ── Model struct ──────────────────────────────────────────────────
    params = types.SimpleNamespace()
    params.C1  = C1;  params.C2  = C2
    params.Cm1 = Cm1; params.Cm2 = Cm2
    params.Cr0 = Cr0; params.Cr2 = Cr2; params.cr3 = cr3

    model.f_expl_expr = f_expl
    model.f_impl_expr = xdot - f_expl
    model.x    = x
    model.xdot = xdot
    model.u    = u
    model.z    = z
    model.p    = p           # np = 1 (kappa_p)
    model.name = model_name
    model.params = params

    return model, constraint
