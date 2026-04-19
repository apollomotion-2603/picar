#pragma once

#include <Eigen/Dense>
#include <cmath>
#include <array>
#include <vector>
#include <qpOASES.hpp>

namespace nmpc {

// =============================================================================
// MODULE 1 — CẤU HÌNH KÍCH THƯỚC (COMPILE-TIME CONSTANTS)
// Ép toàn bộ cấp phát lên Stack Memory để đảm bảo thời gian thực tuyệt đối
// =============================================================================
constexpr int Nx_core = 4;        // Trạng thái lõi: [s, n, alpha, v]
constexpr int Nx = 6;             // Trạng thái mở rộng: [s, n, alpha, v, D, delta]
constexpr int Nu = 2;             // Tín hiệu điều khiển: [dD, ddelta]
constexpr int N  = 5;             // Chân trời dự báo (Prediction Horizon)

constexpr int N_QP_VARS = N * Nu; // Số biến quyết định cho QP (10)
constexpr int N_QP_INEQ = N;      // Số ràng buộc n_k (5)

// Định nghĩa kiểu dữ liệu tĩnh (Fixed-size Eigen Matrices)
using VecX  = Eigen::Matrix<double, Nx, 1>;
using VecU  = Eigen::Matrix<double, Nu, 1>;
using MatA  = Eigen::Matrix<double, Nx, Nx>;
using MatB  = Eigen::Matrix<double, Nx, Nu>;

// Các ma trận cho Batch/Condensing
using MatPhi = Eigen::Matrix<double, N * Nx, Nx>;
using MatG   = Eigen::Matrix<double, N * Nx, N * Nu>;
using MatH   = Eigen::Matrix<double, N_QP_VARS, N_QP_VARS>;
using VecG   = Eigen::Matrix<double, N_QP_VARS, 1>;
using MatC   = Eigen::Matrix<double, N_QP_INEQ, N_QP_VARS>;

// =============================================================================
// MODULE 2 — DATA CONTAINERS (STRUCTS)
// =============================================================================
struct VehicleParams {
    double lf = 0.11, lr = 0.11;
    double m = 1.5;
    double Cm1 = 0.287, Cm2 = 0.054, Cr0 = 0.051, Cr2 = 0.0, Cr3 = 50.0;
    
    double dD_min = -5.0, dD_max = 5.0;
    double ddelta_min = -2.0, ddelta_max = 2.0;
};

struct NMPCParams {
    double Ts = 0.05; 
    double s_ref_N = 3.0; 
    double track_half_width = 0.5;

    VecX Q_diag  = (VecX() << 0.0, 1e-4, 1e-4, 1e-4, 1e-3, 5e-3).finished(); // s thường = 0 để path tracking
    VecU R_diag  = (VecU() << 1e-3, 5e-3).finished();
    VecX QN_diag = (VecX() << 5.0, 100.0, 1e-4, 1e-4, 1e-3, 5e-3).finished();
};

// =============================================================================
// MODULE 3 — MATH UTILITIES (NAMESPACE PURE FUNCTIONS)
// Rời rạc hóa hệ gốc (4x4) trước, sau đó mở rộng (6x6)
// =============================================================================
namespace math {

    inline void compute_jacobians(const VecX& x, double kappa, double dt, 
                                  const VehicleParams& vp, MatA& Ad_ext, MatB& Bd_ext) 
    {
        const double n = x(1), alpha = x(2), v = x(3), D = x(4), delta = x(5);
        const double lambda = vp.lr / (vp.lr + vp.lf);
        const double beta = lambda * delta;

        double denom = 1.0 - n * kappa;
        if (std::abs(denom) < 1e-5) denom = (denom > 0) ? 1e-5 : -1e-5;
        const double rho = 1.0 / denom;
        const double rho2 = rho * rho;

        const double cab = std::cos(alpha + beta);
        const double sab = std::sin(alpha + beta);
        const double cb  = std::cos(beta);
        const double sb  = std::sin(beta);

        const double tanh_cr3v = std::tanh(vp.Cr3 * v);
        const double sech2_cr3v = 1.0 - tanh_cr3v * tanh_cr3v;
        const double Fx = (vp.Cm1 - vp.Cm2 * v) * D - vp.Cr2 * v * v - vp.Cr0 * tanh_cr3v;
        
        const double dFx_dv = -vp.Cm2 * D - 2.0 * vp.Cr2 * v - vp.Cr0 * vp.Cr3 * sech2_cr3v;
        const double dFx_dD = vp.Cm1 - vp.Cm2 * v;

        // BƯỚC 1: Jacobian cho hệ lõi (4x4)
        Eigen::Matrix4d Ac = Eigen::Matrix4d::Zero();
        Eigen::Matrix<double, 4, 2> Bc = Eigen::Matrix<double, 4, 2>::Zero();

        double ds_dn = v * cab * kappa * rho2;
        double ds_dalpha = -v * sab * rho;
        double ds_dv = cab * rho;
        double ds_ddelta = -v * sab * lambda * rho;

        Ac(0, 1) = ds_dn;  Ac(0, 2) = ds_dalpha; Ac(0, 3) = ds_dv;
        Ac(1, 2) = v * cab; Ac(1, 3) = sab;
        Ac(2, 1) = -kappa * ds_dn; Ac(2, 2) = -kappa * ds_dalpha; Ac(2, 3) = sb / vp.lr - kappa * ds_dv;
        Ac(3, 3) = (dFx_dv * cb) / vp.m;

        Bc(0, 1) = ds_ddelta;
        Bc(1, 1) = v * cab * lambda;
        Bc(2, 1) = (v * lambda * cb) / vp.lr - kappa * ds_ddelta;
        Bc(3, 0) = (dFx_dD * cb) / vp.m; 
        Bc(3, 1) = (-Fx * sb * lambda) / vp.m;

        // BƯỚC 2: Rời rạc hóa Euler cho hệ lõi
        Eigen::Matrix4d Ad_core = Eigen::Matrix4d::Identity() + dt * Ac;
        Eigen::Matrix<double, 4, 2> Bd_core = dt * Bc;

        // BƯỚC 3: Mở rộng thành 6x6 (Lắp ghép Block Matrix)
        Ad_ext.setZero();
        Bd_ext.setZero();

        Ad_ext.block<4, 4>(0, 0) = Ad_core;
        Ad_ext.block<4, 2>(0, 4) = Bd_core; // D, delta ảnh hưởng tới [s, n, alpha, v]
        Ad_ext.block<2, 2>(4, 4) = Eigen::Matrix2d::Identity();

        Bd_ext.block<2, 2>(4, 0) = Eigen::Matrix2d::Identity() * dt;
    }

    inline VecX f_continuous(const VecX& x, const VecU& u, double kappa, const VehicleParams& vp) {
        const double n = x(1), alpha = x(2), v = x(3), D = x(4), delta = x(5);
        const double lambda = vp.lr / (vp.lr + vp.lf);
        const double beta = lambda * delta;

        double denom = 1.0 - n * kappa;
        if (std::abs(denom) < 1e-5) denom = (denom > 0) ? 1e-5 : -1e-5;
        const double Fx = (vp.Cm1 - vp.Cm2 * v) * D - vp.Cr2 * v * v - vp.Cr0 * std::tanh(vp.Cr3 * v);

        VecX dx;
        dx(0) = v * std::cos(alpha + beta) / denom;
        dx(1) = v * std::sin(alpha + beta);
        dx(2) = (v / vp.lr) * std::sin(beta) - kappa * dx(0);
        dx(3) = (Fx / vp.m) * std::cos(beta);
        dx(4) = u(0);
        dx(5) = u(1);
        return dx;
    }

    inline VecX rk4_step(const VecX& x, const VecU& u, double kappa, double dt, const VehicleParams& vp) {
        VecX k1 = f_continuous(x, u, kappa, vp);
        VecX k2 = f_continuous(x + 0.5 * dt * k1, u, kappa, vp);
        VecX k3 = f_continuous(x + 0.5 * dt * k2, u, kappa, vp);
        VecX k4 = f_continuous(x + dt * k3, u, kappa, vp);
        return x + (dt / 6.0) * (k1 + 2.0 * k2 + 2.0 * k3 + k4);
    }
} // namespace math

// =============================================================================
// MODULE 4 — CORE CONTROLLER CLASS
// =============================================================================
class SpatialNMPC {
public:
    SpatialNMPC(const VehicleParams& vp, const NMPCParams& mp) 
        : vp_(vp), mp_(mp), qp_solver_(N_QP_VARS, N_QP_INEQ) 
    {
        qpOASES::Options opts;
        opts.printLevel = qpOASES::PL_NONE;
        qp_solver_.setOptions(opts);

        x_bar_.fill(VecX::Zero());
        u_bar_.fill(VecU::Zero());

        // Precompute Block-Diagonal Q_bar and R_bar
        Q_bar_.setZero();
        R_bar_.setZero();
        Eigen::Matrix<double, Nx, Nx> Q = mp_.Q_diag.asDiagonal();
        Eigen::Matrix<double, Nx, Nx> QN = mp_.QN_diag.asDiagonal();
        Eigen::Matrix<double, Nu, Nu> R = mp_.R_diag.asDiagonal();

        for (int k = 0; k < N; ++k) {
            Q_bar_.block<Nx, Nx>(k * Nx, k * Nx) = (k == N - 1) ? QN : Q;
            R_bar_.block<Nu, Nu>(k * Nu, k * Nu) = R;
        }
    }

    // Main execution loop (Returns control velocities dD, ddelta)
    bool step(const VecX& x_current, const std::array<double, N>& kappa_seq, VecU& u_opt_out) {
        condensePhase(kappa_seq);
        bool success = solveQP(x_current, u_opt_out);
        if (success) {
            shiftTrajectory(kappa_seq.back());
        }
        return success;
    }

    // Hàm lấy giá trị D và delta thực tế (Đã tích phân)
    VecU getActualControlInput() const {
        return x_bar_[0].tail<2>(); // Trả về [D, delta]
    }

private:
    VehicleParams vp_;
    NMPCParams mp_;

    // Array tĩnh cho Zero Dynamic Allocation
    std::array<VecX, N + 1> x_bar_;
    std::array<VecU, N>     u_bar_;

    // Dữ liệu Condensed QP
    MatH H_;
    VecG g_partial_;
    MatC C_;
    Eigen::Matrix<double, N_QP_INEQ, 1> lbA_, ubA_;
    Eigen::Matrix<double, N_QP_VARS, 1> lb_, ub_;
    MatPhi Phi_;
    MatG G_;
    Eigen::Matrix<double, N * Nx, 1> d_hat_;

    Eigen::Matrix<double, N * Nx, N * Nx> Q_bar_;
    Eigen::Matrix<double, N * Nu, N * Nu> R_bar_;

    qpOASES::QProblem qp_solver_;
    bool is_qp_initialized_ = false;

    void condensePhase(const std::array<double, N>& kappa) {
        std::array<MatA, N> Ad;
        std::array<MatB, N> Bd;
        std::array<VecX, N> d;

        for (int k = 0; k < N; ++k) {
            math::compute_jacobians(x_bar_[k], kappa[k], mp_.Ts, vp_, Ad[k], Bd[k]);
            d[k] = x_bar_[k + 1] - math::rk4_step(x_bar_[k], u_bar_[k], kappa[k], mp_.Ts, vp_);
        }

        Phi_.setZero();
        G_.setZero();
        d_hat_.setZero();

        MatA Phi_k = MatA::Identity();
        VecX d_hat_k = VecX::Zero();

        for (int k = 0; k < N; ++k) {
            MatA Phi_next = Ad[k] * Phi_k;
            VecX d_hat_next = Ad[k] * d_hat_k + d[k];

            Phi_.block<Nx, Nx>(k * Nx, 0) = Phi_next;
            d_hat_.segment<Nx>(k * Nx) = d_hat_next;

            G_.block<Nx, Nu>(k * Nx, k * Nu) = Bd[k];
            for (int j = 0; j < k; ++j) {
                G_.block<Nx, Nu>(k * Nx, j * Nu) = Ad[k] * G_.block<Nx, Nu>((k - 1) * Nx, j * Nu);
            }

            Phi_k = Phi_next;
            d_hat_k = d_hat_next;
        }

        H_ = G_.transpose() * Q_bar_ * G_ + R_bar_;

        // Reference (x_ref)
        Eigen::Matrix<double, N * Nx, 1> e_bar;
        const double s0 = x_bar_[0](0);
        const double ds = mp_.s_ref_N / N;
        for (int k = 0; k < N; ++k) {
            VecX x_ref = VecX::Zero();
            x_ref(0) = s0 + ds * (k + 1);
            e_bar.segment<Nx>(k * Nx) = x_bar_[k + 1] - x_ref;
        }

        g_partial_ = G_.transpose() * (Q_bar_ * (d_hat_ + e_bar));

        // Constraints bounds
        C_.setZero();
        for (int k = 0; k < N; ++k) {
            C_.row(k) = G_.block<1, N_QP_VARS>(k * Nx + 1, 0); // Ràng buộc theo n (index 1)
            double n_bar_k = x_bar_[k + 1](1);
            double d_n_k   = d_hat_(k * Nx + 1);
            lbA_(k) = -mp_.track_half_width - n_bar_k - d_n_k;
            ubA_(k) =  mp_.track_half_width - n_bar_k - d_n_k;

            lb_(k * Nu + 0) = vp_.dD_min;
            ub_(k * Nu + 0) = vp_.dD_max;
            lb_(k * Nu + 1) = vp_.ddelta_min;
            ub_(k * Nu + 1) = vp_.ddelta_max;
        }
    }

    bool solveQP(const VecX& x_current, VecU& u_opt_out) {
        VecX dx0 = x_current - x_bar_[0];
        VecG g_full = g_partial_ + G_.transpose() * Q_bar_ * Phi_ * dx0;

        auto lbA_full = lbA_;
        auto ubA_full = ubA_;
        for (int k = 0; k < N; ++k) {
            double phi_n_dx0 = (Phi_.block<1, Nx>(k * Nx + 1, 0) * dx0)(0);
            lbA_full(k) -= phi_n_dx0;
            ubA_full(k) -= phi_n_dx0;
        }

        // Ép kiểu sang RowMajor cho qpOASES
        Eigen::Matrix<double, N_QP_VARS, N_QP_VARS, Eigen::RowMajor> H_rm = H_;
        Eigen::Matrix<double, N_QP_INEQ, N_QP_VARS, Eigen::RowMajor> C_rm = C_;
        Eigen::VectorXd du_opt(N_QP_VARS); // Phải dùng dạng này vì qpOASES yêu cầu array data
        
        qpOASES::int_t nWSR = 100;
        qpOASES::returnValue ret;

        // TẬN DỤNG CƠ CHẾ HOT-START CỦA SQP
        if (!is_qp_initialized_) {
            ret = qp_solver_.init(H_rm.data(), g_full.data(), C_rm.data(),
                                  lb_.data(), ub_.data(), lbA_full.data(), ubA_full.data(), nWSR);
            if (ret == qpOASES::SUCCESSFUL_RETURN) is_qp_initialized_ = true;
        } else {
            ret = qp_solver_.hotstart(H_rm.data(), g_full.data(), C_rm.data(),
                                      lb_.data(), ub_.data(), lbA_full.data(), ubA_full.data(), nWSR);
        }

        if (ret != qpOASES::SUCCESSFUL_RETURN) {
            is_qp_initialized_ = false; // Reset lại nếu không tìm được nghiệm
            return false;
        }

        qp_solver_.getPrimalSolution(du_opt.data());

        // SQP Update (Cộng dồn nghiệm vừa tìm được vào quỹ đạo danh định)
        for (int k = 0; k < N; ++k) {
            u_bar_[k] += du_opt.segment<Nu>(k * Nu);
        }
        
        u_opt_out = u_bar_[0]; // Lấy vận tốc dD, ddelta tối ưu nhất
        return true;
    }

    void shiftTrajectory(double kappa_last) {
        // Dịch chuyển trạng thái và tích phân mở rộng bước cuối
        for (int k = 0; k < N; ++k) x_bar_[k] = x_bar_[k + 1];
        x_bar_[N] = math::rk4_step(x_bar_[N - 1], u_bar_[N - 1], kappa_last, mp_.Ts, vp_);

        // Dịch chuyển đầu vào
        for (int k = 0; k < N - 1; ++k) u_bar_[k] = u_bar_[k + 1];
        // Giữ nguyên u_bar_[N-1] cho warm-start chu kỳ sau
    }
};

} // namespace nmpc