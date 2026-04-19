// =============================================================================
// NMPC Support Library — RTI-SQP, Eigen, qpOASES
// Dựa trên: Kloeser et al., IFAC 2020 — Singularity-Free Path-Parametric NMPC
//
// Kiến trúc:
//   nmpc_types   → Eigen types, constants, parameters structs
//   nmpc_model   → Dynamics f(x,u), RK4 integrator, Analytic Jacobians (6x6)
//   nmpc_solver  → Condensing (H, g, G, Phi), QP solve via qpOASES
// =============================================================================
#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <vector>
#include <memory>
#include <string>
#include <qpOASES.hpp>

#include "rclcpp/rclcpp.hpp"

namespace nmpc {

// =============================================================================
// MODULE 1 — TYPES & CONSTANTS
// =============================================================================

// Kích thước hệ thống
constexpr int State_Dimension = 6;       // Number of states:   [s, n, alpha, v, D, delta]
constexpr int Control_Dimension = 2;     // Number of controls: [dD, ddelta]
constexpr int N = 5;    // Prediction horizon length

// Kích thước QP condensed
constexpr int Num_Qp_Decision_Variables = N * Control_Dimension;               // decision variables: Δu₀...Δu_{N-1}
constexpr int Num_Qp_Inequality_Constraints = N;           // track-bound rows (n_k)

// Eigen type aliases
using StateVector   = Eigen::Matrix<double, State_Dimension, 1>;
using ControlVector = Eigen::Matrix<double, Control_Dimension, 1>;
using StateMatrix   = Eigen::Matrix<double, State_Dimension, State_Dimension>;
using ControlMatrix = Eigen::Matrix<double, State_Dimension, Control_Dimension>;
using StateCostMatrix = Eigen::Matrix<double, State_Dimension, State_Dimension>;
using ControlCostMatrix = Eigen::Matrix<double, Control_Dimension, Control_Dimension>;

// Stacked (horizon) types
using StackedStateTransitionMatrix = Eigen::Matrix<double, N * State_Dimension, State_Dimension>;           // Phi stacked
using StackedControlMatrix   = Eigen::Matrix<double, N * State_Dimension, N * Control_Dimension>;       // G condensed
using QpHessianMatrix   = Eigen::Matrix<double, Num_Qp_Decision_Variables, Num_Qp_Decision_Variables>; // QP Hessian
using QpGradientVector  = Eigen::Matrix<double, Num_Qp_Decision_Variables, 1>;         // QP gradient
using QpConstraintMatrix = Eigen::Matrix<double, Num_Qp_Inequality_Constraints, Num_Qp_Decision_Variables>; // QP constraint matrix

// =============================================================================
// MODULE 2 — PARAMETERS
// =============================================================================

struct VehicleParams {
  // Geometry
  double lf  = 0.11;   // [m] distance CG → front axle
  double lr  = 0.11;   // [m] distance CG → rear axle

  // Inertia
  double m   = 1.5;    // [kg]

  // Motor model (Eq. 4)
  double Cm1 = 0.287;
  double Cm2 = 0.054;
  double Cr0 = 0.051;
  double Cr2 = 0.0;
  double Cr3 = 50.0;

  // Constraints
  double v_max          = 10.0;   // [m/s]
  double a_lateral_max  =  4.0;   // [m/s^2] — model validity
  double delta_min      = -0.6109; // [rad]
  double delta_max      =  0.6109; // [rad]
  double D_min          = -1.0;
  double D_max          =  1.0;
  double dD_min         = -5.0;
  double dD_max         =  5.0;
  double ddelta_min     = -2.0;
  double ddelta_max     =  2.0;
};

struct NMPCParams {
  double Ts         = 0.05;  // [s] sampling time (50 ms)
  double s_ref_N    = 3.0;   // [m] desired progress over horizon

  // Cost matrices (block-diagonal → just store diagonal)
  // Q  = diag(0.1, 1e-4, 1e-4, 1e-4, 1e-3, 5e-3)
  // R  = diag(1e-3, 5e-3)
  // QN = diag(5,   100,  1e-4, 1e-4, 1e-3, 5e-3)
  VecX  Q_diag  = (VecX() << 0.1, 1e-4, 1e-4, 1e-4, 1e-3, 5e-3).finished();
  VecU  R_diag  = (VecU() << 1e-3, 5e-3).finished();
  VecX  QN_diag = (VecX() << 5.0, 100.0, 1e-4, 1e-4, 1e-3, 5e-3).finished();

  // Track half-width (symmetric, can be overridden per step)
  double track_half_width  = 0.5;   // [m] nửa chiều rộng đường (track half-width)
};

// =============================================================================
// MODULE 3 — MODEL: Dynamics, RK4, Analytic Jacobians
// =============================================================================

// ----------------------------------------------------------------------------
// 3.1  Continuous-time dynamics  f(x, u, kappa)
//      x = [s, n, alpha, v, D, delta]
//      u = [dD, ddelta]
//      kappa = track curvature at s
// ----------------------------------------------------------------------------
inline VecX f_continuous(const VecX& x, const VecU& u,
                         double kappa,
                         const VehicleParams& vp)
{
  const double s     = x(0);
  const double n     = x(1);
  const double alpha = x(2);
  const double v     = x(3);
  const double D     = x(4);
  const double delta = x(5);

  (void)s; // s itself not needed inside f, only kappa(s) which is provided

  const double lambda = vp.lr / (vp.lr + vp.lf);   // β ≈ λ·δ (small angle)
  const double beta   = lambda * delta;

  const double cab = std::cos(alpha + beta);
  const double sab = std::sin(alpha + beta);
  const double cb  = std::cos(beta);
  const double sb  = std::sin(beta);

  const double rho = 1.0 / (1.0 - n * kappa);      // 1/(1-n·κ^c)

  // Longitudinal drive force (Eq. 4) with tanh rolling resistance
  const double Fx = (vp.Cm1 - vp.Cm2 * v) * D
                  - vp.Cr2 * v * v
                  - vp.Cr0 * std::tanh(vp.Cr3 * v);

  VecX dx;
  dx(0) = v * cab * rho;                              // ṡ
  dx(1) = v * sab;                                    // ṅ
  dx(2) = (v / vp.lr) * sb - kappa * dx(0);          // α̇
  dx(3) = (Fx / vp.m) * cb;                           // v̇
  dx(4) = u(0);                                        // Ḋ
  dx(5) = u(1);                                        // δ̇
  return dx;
}

// ----------------------------------------------------------------------------
// 3.2  RK4 integrator (for forward simulation)
// ----------------------------------------------------------------------------
inline VecX rk4_step(const VecX& x, const VecU& u,
                     double kappa,
                     double dt,
                     const VehicleParams& vp)
{
  const VecX k1 = f_continuous(x,               u, kappa, vp);
  const VecX k2 = f_continuous(x + 0.5*dt*k1,  u, kappa, vp);
  const VecX k3 = f_continuous(x + 0.5*dt*k2,  u, kappa, vp);
  const VecX k4 = f_continuous(x +     dt*k3,  u, kappa, vp);
  return x + (dt / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4);
}

// ----------------------------------------------------------------------------
// 3.3  Analytic Jacobians (Euler discretization of ∂f/∂x, ∂f/∂u)
//      Returns discrete A_k = I + dt·(∂f/∂x),  B_k = dt·(∂f/∂u)
//
//  Full analytic derivation follows NMPC_NOTES.md §5.
// ----------------------------------------------------------------------------
inline void compute_jacobians(const VecX& x, const VecU& u,
                               double kappa,
                               double dt,
                               const VehicleParams& vp,
                               MatA& Ad, MatB& Bd)
{
  const double n     = x(1);
  const double alpha = x(2);
  const double v     = x(3);
  const double D     = x(4);
  const double delta = x(5);

  const double lambda = vp.lr / (vp.lr + vp.lf);
  const double beta   = lambda * delta;

  const double cab = std::cos(alpha + beta);
  const double sab = std::sin(alpha + beta);
  const double cb  = std::cos(beta);
  const double sb  = std::sin(beta);

  const double rho  = 1.0 / (1.0 - n * kappa);
  const double rho2 = rho * rho;

  // Drive force and its partial w.r.t. v
  const double tanh_cr3v  = std::tanh(vp.Cr3 * v);
  const double sech2_cr3v = 1.0 - tanh_cr3v * tanh_cr3v; // sech^2
  const double Fx = (vp.Cm1 - vp.Cm2 * v) * D
                  - vp.Cr2 * v * v
                  - vp.Cr0 * tanh_cr3v;
  const double dFx_dv = -vp.Cm2 * D
                        - 2.0 * vp.Cr2 * v
                        - vp.Cr0 * vp.Cr3 * sech2_cr3v;
  const double dFx_dD = vp.Cm1 - vp.Cm2 * v;

  // -------------------------------------------------------------------------
  // ∂f/∂x  (6×6) — continuous
  // Row 0: ṡ = v·cos(α+β)/(1-n·κ)
  // Row 1: ṅ = v·sin(α+β)
  // Row 2: α̇ = (v/lr)·sin(β) - κ·ṡ
  // Row 3: v̇ = (Fx/m)·cos(β)
  // Row 4: Ḋ = u₀  → all zeros
  // Row 5: δ̇ = u₁  → all zeros
  // -------------------------------------------------------------------------
  Eigen::Matrix<double, NX, NX> Ac = Eigen::Matrix<double, NX, NX>::Zero();

  // ṡ partials
  // ∂ṡ/∂n     = v·cos(α+β)·κ·ρ²
  const double ds_dn     =  v * cab * kappa * rho2;
  // ∂ṡ/∂α     = -v·sin(α+β)·ρ
  const double ds_dalpha = -v * sab * rho;
  // ∂ṡ/∂v     =  cos(α+β)·ρ
  const double ds_dv     =  cab * rho;
  // ∂ṡ/∂delta = -v·sin(α+β)·λ·ρ
  const double ds_ddelta = -v * sab * lambda * rho;

  Ac(0, 1) = ds_dn;
  Ac(0, 2) = ds_dalpha;
  Ac(0, 3) = ds_dv;
  Ac(0, 5) = ds_ddelta;

  // ṅ partials
  // ∂ṅ/∂α     = v·cos(α+β)
  // ∂ṅ/∂v     = sin(α+β)
  // ∂ṅ/∂delta = v·cos(α+β)·λ
  Ac(1, 2) =  v * cab;
  Ac(1, 3) =  sab;
  Ac(1, 5) =  v * cab * lambda;

  // α̇ = (v/lr)·sin(β) - κ·ṡ
  // ∂α̇/∂n     = -κ·∂ṡ/∂n
  // ∂α̇/∂α     = -κ·∂ṡ/∂α
  // ∂α̇/∂v     = sin(β)/lr - κ·∂ṡ/∂v
  // ∂α̇/∂delta = (v·λ·cos(β))/lr - κ·∂ṡ/∂delta
  Ac(2, 1) = -kappa * ds_dn;
  Ac(2, 2) = -kappa * ds_dalpha;
  Ac(2, 3) =  sb / vp.lr - kappa * ds_dv;
  Ac(2, 5) =  (v * lambda * cb) / vp.lr - kappa * ds_ddelta;

  // v̇ partials
  // ∂v̇/∂v     = (dFx_dv·cos(β)) / m
  // ∂v̇/∂D     = (dFx_dD·cos(β)) / m
  // ∂v̇/∂delta = (-Fx·sin(β)·λ) / m
  Ac(3, 3) = (dFx_dv * cb) / vp.m;
  Ac(3, 4) = (dFx_dD * cb) / vp.m;
  Ac(3, 5) = (-Fx * sb * lambda) / vp.m;

  // Rows 4,5 → all zero (trivial augmented states)

  // Euler discretization: Ad = I + dt·Ac
  Ad = MatA::Identity() + dt * Ac;

  // -------------------------------------------------------------------------
  // ∂f/∂u  (6×2) — continuous (and discrete, since B_d = dt·B_c)
  // Only rows 4 and 5 are nonzero (identity)
  // -------------------------------------------------------------------------
  Bd = MatB::Zero();
  Bd(4, 0) = dt;   // ∂(Ḋ)/∂(dD)       = 1 → discretized: dt
  Bd(5, 1) = dt;   // ∂(δ̇)/∂(ddelta)   = 1 → discretized: dt
}

// =============================================================================
// MODULE 4 — SOLVER: Condensing + qpOASES
// =============================================================================

// Container for condensed QP data (built in preparation phase)
struct CondensedQP {
  MatH   H;        // (N·NU × N·NU)  Hessian
  VecG   g;        // (N·NU)         gradient (partial, completed in feedback)
  MatC   C;        // (Num_Qp_Inequality_Constraints × NumQpDecisionVars)  inequality constraint matrix
  Eigen::VectorXd lbA; // lower bound for C·Δu
  Eigen::VectorXd ubA; // upper bound for C·Δu
  Eigen::VectorXd lb;  // box lower bound on Δu
  Eigen::VectorXd ub;  // box upper bound on Δu

  // Stored for feedback phase
  MatPhi Phi;           // (N·NX × NX)
  MatG   G;             // (N·NX × N·NU)
  Eigen::Matrix<double, N * NX, 1> d_hat; // residual accumulation

  CondensedQP() {
    H.setZero(); g.setZero();
    C.setZero();
    lbA.resize(Num_Qp_Inequality_Constraints); lbA.setZero();
    ubA.resize(Num_Qp_Inequality_Constraints); ubA.setZero();
    lb.resize(NumQpDecisionVars);  lb.setZero();
    ub.resize(NumQpDecisionVars);  ub.setZero();
    Phi.setZero(); G.setZero(); d_hat.setZero();
  }
};

// ----------------------------------------------------------------------------
// 4.1  Build condensed QP (preparation phase)
//
//  Inputs:
//    x_bar[N+1]  — nominal trajectory states
//    u_bar[N]    — nominal trajectory controls
//    kappa[N]    — track curvature at each step
//    params      — vehicle + NMPC parameters
//
//  Outputs:
//    qp           — fully populated CondensedQP (H, G, Phi, d_hat)
//    NOTE: g is finalized in feedback phase after Δx₀ is known
// ----------------------------------------------------------------------------
inline void condense(
  const std::vector<VecX>& x_bar,
  const std::vector<VecU>& u_bar,
  const std::vector<double>& kappa,
  const VehicleParams& vp,
  const NMPCParams& mp,
  CondensedQP& qp)
{
  const double dt = mp.Ts;

  // Cost matrices
  MatQ Q  = mp.Q_diag.asDiagonal();
  MatR R  = mp.R_diag.asDiagonal();
  MatQ QN = mp.QN_diag.asDiagonal();

  // ---- Step A: Compute A_k, B_k, and simulate residuals ------------------
  std::vector<MatA> Ad(N);
  std::vector<MatB> Bd(N);
  std::vector<VecX> d(N);   // defect: x_bar[k+1] - F_RK4(x_bar[k], u_bar[k])

  for (int k = 0; k < N; ++k) {
    compute_jacobians(x_bar[k], u_bar[k], kappa[k], dt, vp, Ad[k], Bd[k]);
    VecX x_rk4 = rk4_step(x_bar[k], u_bar[k], kappa[k], dt, vp);
    d[k] = x_bar[k + 1] - x_rk4;   // linearization residual
  }

  // ---- Step B: Build Phi and G via recursion ------------------------------
  // Phi_k = A_{k-1}·...·A_0        (state transition from step 0 to step k)
  // G(k,j) = A_{k-1}·...·A_{j+1}·B_j   for j < k,  B_k for j == k
  qp.Phi.setZero();
  qp.G.setZero();
  qp.d_hat.setZero();

  // d_hat_k = Phi_k·d_0 + A_{k-1}·...·A_1·d_1 + ... + d_{k-1}
  // Computed incrementally:
  MatA Phi_k = MatA::Identity();
  VecX d_hat_k = VecX::Zero();

  for (int k = 0; k < N; ++k) {
    // Before applying A_k:  Phi_k = A_{k-1}·...·A_0
    // Phi for row k (1-indexed from 1 to N): Phi_{k+1} = A_k · Phi_k
    MatA Phi_next = Ad[k] * Phi_k;

    // d_hat_{k+1} = A_k · d_hat_k + d_k
    VecX d_hat_next = Ad[k] * d_hat_k + d[k];

    // Store
    qp.Phi.block(k * NX, 0, NX, NX) = Phi_next;
    qp.d_hat.segment(k * NX, NX)     = d_hat_next;

    // G(k+1, j) = A_k · G(k, j) for j <= k,  G(k+1, k+1... wait indices
    // G is (N·NX × N·NU), indexing: G.block(row_step*NX, col_step*NU, NX, NU)
    // For step k (0-indexed), diagonal block is Bd[k]:
    qp.G.block(k * NX, k * NU, NX, NU) = Bd[k];
    // Off-diagonal blocks: propagate previous column blocks
    for (int j = 0; j < k; ++j) {
      qp.G.block(k * NX, j * NU, NX, NU) =
          Ad[k] * qp.G.block((k - 1) * NX, j * NU, NX, NU);
    }

    Phi_k   = Phi_next;
    d_hat_k = d_hat_next;
  }

  // ---- Step C: Build stacked Q̄ (block diagonal Q...Q QN) -----------------
  Eigen::Matrix<double, N * NX, N * NX> Q_bar =
      Eigen::Matrix<double, N * NX, N * NX>::Zero();
  for (int k = 0; k < N - 1; ++k) {
    Q_bar.block(k * NX, k * NX, NX, NX) = Q;
  }
  Q_bar.block((N - 1) * NX, (N - 1) * NX, NX, NX) = QN;

  // Stacked R̄ (block diagonal R...R)
  Eigen::Matrix<double, N * NU, N * NU> R_bar =
      Eigen::Matrix<double, N * NU, N * NU>::Zero();
  for (int k = 0; k < N; ++k) {
    R_bar.block(k * NU, k * NU, NU, NU) = R;
  }

  // ---- Step D: H = G^T · Q̄ · G + R̄ ----------------------------------------
  qp.H = qp.G.transpose() * Q_bar * qp.G + R_bar;

  // ---- Step E: Reference tracking error e = x_bar - x_ref ----------------
  // Reference: only s progresses, rest stay at 0 (can be refined later)
  // g is partially built here; Phi·Δx₀ term added in feedback phase.
  //   g_partial = G^T · Q̄ · (Phi·Δx₀ + d_hat - e_ref)
  //   We precompute:  q = Q̄·(d_hat - e_ref), g = G^T·q + G^T·Q̄·Phi·Δx₀
  // Here we only precompute the d_hat part; Φ·Δx₀ is added in feedback.

  // Build stacked reference deviation (x_bar - x_ref)
  Eigen::Matrix<double, N * NX, 1> e_bar;
  const double s0   = x_bar[0](0);
  const double ds   = mp.s_ref_N / static_cast<double>(N);
  for (int k = 0; k < N; ++k) {
    VecX x_ref = VecX::Zero();
    x_ref(0) = s0 + ds * (k + 1);  // s_k,ref = s0 + ds*(k+1)
    e_bar.segment(k * NX, NX) = x_bar[k + 1] - x_ref;
  }

  // g_partial = G^T · Q̄ · (d_hat + e_bar)  (Phi·Δx₀ added in feedback)
  qp.g = qp.G.transpose() * (Q_bar * (qp.d_hat + e_bar));

  // ---- Step F: Inequality constraints — track bounds on n_k ---------------
  // lbA ≤ C·Δu ≤ ubA    where C selects n-row of G
  // [G]_{n-rows} · Δu + [Phi]_{n-rows} · Δx₀ + d_hat_{n,k} ∈ [n_min, n_max]
  // (Phi·Δx₀ contribution is added in feedback phase; here we set C)
  qp.C.setZero();
  for (int k = 0; k < N; ++k) {
    // n is state index 1
    qp.C.row(k) = qp.G.block(k * NX + 1, 0, 1, N_QP_VARS);
    // bounds shifted by nominal n̄_k and d_hat_n,k
    double n_bar_k = x_bar[k + 1](1);
    double d_n_k   = qp.d_hat(k * NX + 1);
    qp.lbA(k)  = -mp.track_half_width - n_bar_k - d_n_k;
    qp.ubA(k)  =  mp.track_half_width - n_bar_k - d_n_k;
  }

  // ---- Step G: Box constraints on Δu -------------------------------------
  for (int k = 0; k < N; ++k) {
    // dD limits
    qp.lb(k * NU + 0) = vp.dD_min;
    qp.ub(k * NU + 0) = vp.dD_max;
    // ddelta limits
    qp.lb(k * NU + 1) = vp.ddelta_min;
    qp.ub(k * NU + 1) = vp.ddelta_max;
  }
}

// ----------------------------------------------------------------------------
// 4.2  Feedback phase: finalize g with Δx₀, solve QP, return Δu*
//
//  Returns true on success.
// ----------------------------------------------------------------------------
inline bool solve_qp(
  CondensedQP& qp,
  const VecX& x_current,
  const VecX& x_bar_0,
  Eigen::VectorXd& du_opt)  // output: Δu* of size N_QP_VARS
{
  // Δx₀ = x_c - x̄₀
  VecX dx0 = x_current - x_bar_0;

  // Build stacked Q̄ (same as in condense — we need Q̄·Phi·Δx₀ term)
  // For efficiency, precompute Q̄·Phi:
  // g_full = g_partial + G^T · Q̄ · Phi · Δx₀
  // Since we don't store Q̄·Phi separately, recompute inline.
  // (In RTI, this is lightweight — N*NX × NX matrix × NX vector)
  const NMPCParams mp_default;  // use defaults — caller should pass mp ideally
  // Note: proper design would pass mp here too; simplified for now.
  // Use NMPCParams defaults for Q, QN:
  MatQ Q  = mp_default.Q_diag.asDiagonal();
  MatQ QN = mp_default.QN_diag.asDiagonal();

  // Stack Q_bar·Phi·Δx₀
  Eigen::Matrix<double, N * NX, 1> QPhi_dx0;
  QPhi_dx0.setZero();
  for (int k = 0; k < N; ++k) {
    MatQ Qk = (k == N - 1) ? QN : Q;
    QPhi_dx0.segment(k * NX, NX) =
        Qk * qp.Phi.block(k * NX, 0, NX, NX) * dx0;
  }
  VecG g_full = qp.g + qp.G.transpose() * QPhi_dx0;

  // Tighten lbA/ubA with Phi·Δx₀ n-row contribution
  Eigen::VectorXd lbA_full = qp.lbA;
  Eigen::VectorXd ubA_full = qp.ubA;
  for (int k = 0; k < N; ++k) {
    double phi_n_dx0 = (qp.Phi.block(k * NX + 1, 0, 1, NX) * dx0)(0);
    lbA_full(k) -= phi_n_dx0;
    ubA_full(k) -= phi_n_dx0;
  }

  // ---- Solve with qpOASES ------------------------------------------------
  du_opt.resize(N_QP_VARS);
  du_opt.setZero();

  qpOASES::int_t nWSR = 100;
  qpOASES::QProblem qp_solver(N_QP_VARS, N_QP_INEQ);

  // Suppress output
  qpOASES::Options opts;
  opts.printLevel = qpOASES::PL_NONE;
  qp_solver.setOptions(opts);

  // qpOASES expects row-major; Eigen is col-major by default → use .data()
  // on RowMajor copies.
  Eigen::Matrix<double, NumQpDecisionVars, NumQpDecisionVars, Eigen::RowMajor> H_rm = qp.H;
  Eigen::Matrix<double, Num_Qp_Inequality_Constraints, NumQpDecisionVars, Eigen::RowMajor> C_rm = qp.C;

  qpOASES::returnValue ret = qp_solver.init(
      H_rm.data(), g_full.data(),
      C_rm.data(),
      qp.lb.data(), qp.ub.data(),
      lbA_full.data(), ubA_full.data(),
      nWSR);

  if (ret != qpOASES::SUCCESSFUL_RETURN && ret != qpOASES::RET_MAX_NWSR_REACHED) {
    return false;
  }

  qp_solver.getPrimalSolution(du_opt.data());
  return true;
}

// ----------------------------------------------------------------------------
// 4.3  Shift trajectory (start of each RTI cycle)
//      Shifts x_bar and u_bar by one step, extrapolates last entry.
// ----------------------------------------------------------------------------
inline void shift_trajectory(
  std::vector<VecX>& x_bar,
  std::vector<VecU>& u_bar,
  const double kappa_last,
  const double dt,
  const VehicleParams& vp)
{
  // Shift states
  for (int k = 0; k < N; ++k) {
    x_bar[k] = x_bar[k + 1];
  }
  // Extrapolate last state using RK4
  x_bar[N] = rk4_step(x_bar[N - 1], u_bar[N - 1], kappa_last, dt, vp);

  // Shift controls (hold last)
  for (int k = 0; k < N - 1; ++k) {
    u_bar[k] = u_bar[k + 1];
  }
  // u_bar[N-1] unchanged (warm-start hold)
}

// ----------------------------------------------------------------------------
// 4.4  Apply Δu* to nominal trajectory (SQP update)
// ----------------------------------------------------------------------------
inline void apply_sqp_update(
  std::vector<VecU>& u_bar,
  const Eigen::VectorXd& du_opt)
{
  for (int k = 0; k < N; ++k) {
    u_bar[k] += du_opt.segment<NU>(k * NU);
  }
}

} // namespace nmpc
