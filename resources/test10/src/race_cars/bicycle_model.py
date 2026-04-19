#
# Copyright (c) The acados authors.
#
# This file is part of acados.
#
# The 2-Clause BSD License
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice,
# this list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.;
#

# author: Daniel Kloeser

import types
import numpy as np
from casadi import *


def bicycle_model():
    """
    Spatial bicycle model for NMPC.

    Track curvature (kappa) is supplied as a runtime CasADi parameter `kappa_p`
    instead of a compile-time bspline from a track file.  This makes the model
    track-agnostic: kappa is fed at each solver step from /line/kappa_s.

    State   x = [s, n, alpha, v, D, delta]
    Control u = [derD, derDelta]
    Param   p = [kappa_p]   -- curvature at current horizon stage [1/m]
    """
    constraint = types.SimpleNamespace()
    model = types.SimpleNamespace()

    model_name = "Spatialbicycle_model"

    ## Race car parameters
    # --- Original parameters (commented for reference) ---
    # m   = 1.5      # vehicle_mass
    # C1  = 0.5
    # C2  = 15.5
    # Cm1 = 0.287    # cm1
    # Cm2 = 0.054    # cm2
    # Cr0 = 0.051    # cr0
    # Cr2 = 0.0      # cr2
    # cr3 = 50.0     # cr3

    # --- Default/typical parameters (restore to common values) ---
    m   = 1.5      # vehicle_mass
    C1  = 0.5
    C2  = 15.5
    Cm1 = 0.28    # cm1
    Cm2 = 0.05    # cm2
    Cr0 = 0.006    # cr0
    Cr2 = 0.011      # cr2
    cr3 = 5.0      # cr3 (giảm về giá trị nhỏ hơn, phổ biến hơn)

    ## CasADi Model — states
    s     = MX.sym("s")
    n     = MX.sym("n")
    alpha = MX.sym("alpha")
    v     = MX.sym("v")
    D     = MX.sym("D")
    delta = MX.sym("delta")
    x = vertcat(s, n, alpha, v, D, delta)

    ## Controls
    derD     = MX.sym("derD")
    derDelta = MX.sym("derDelta")
    u = vertcat(derD, derDelta)

    ## State derivatives (for implicit form)
    sdot     = MX.sym("sdot")
    ndot     = MX.sym("ndot")
    alphadot = MX.sym("alphadot")
    vdot     = MX.sym("vdot")
    Ddot     = MX.sym("Ddot")
    deltadot = MX.sym("deltadot")
    xdot = vertcat(sdot, ndot, alphadot, vdot, Ddot, deltadot)

    ## Algebraic variables (none)
    z = vertcat([])

    ## Runtime parameter: track curvature supplied per stage from /line/kappa_s
    kappa_p = MX.sym("kappa_p")   # [1/m]
    p = vertcat(kappa_p)

    ## Dynamics (spatial bicycle model)
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

    ## Nonlinear constraint expressions
    a_lat  = C2 * v * v * delta + Fxd * sin(C1 * delta) / m
    a_long = Fxd / m

    ## Model bounds
    model.n_min = -0.12   # track half-width [m]
    model.n_max =  0.12

    model.throttle_min = -1.0
    model.throttle_max =  1.0

    model.delta_min = -0.40   # max steering angle [rad]
    model.delta_max =  0.40

    model.ddelta_min    = -2.0    # max steering rate [rad/s]
    model.ddelta_max    =  2.0
    model.dthrottle_min = -10.0   # max throttle rate
    model.dthrottle_max =  10.0

    ## Constraint bounds
    constraint.alat_min  = -4.0   # lateral accel [m/s^2]
    constraint.alat_max  =  4.0
    constraint.along_min = -4.0   # longitudinal accel [m/s^2]
    constraint.along_max =  4.0

    ## Initial condition
    model.x0 = np.array([0.0, 0.0, 0.0, 0.0, 0.0, 0.0])

    ## Constraint struct
    constraint.alat = Function("a_lat", [x, u, p], [a_lat])
    constraint.expr = vertcat(a_long, a_lat, n, D, delta)

    ## Model struct
    params = types.SimpleNamespace()
    params.C1 = C1; params.C2 = C2
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
