#pragma once
#include "nmpc_support.hpp"
#include <vector>

namespace nmpc {

// Ví dụ class NMPCSolver quản lý trạng thái, tham số, RTI-SQP
class NMPCSolver {
public:
    NMPCSolver(const Params& params, const CenterLine& cl)
        : params_(params), cl_(cl) {
        xbar_.resize(N, VecX::Zero());
        ubar_.resize(N, VecU::Zero());
    }

    // Đặt trạng thái ban đầu và tham chiếu
    void setInitialState(const VecX& x0) { x0_ = x0; }
    void setReference(const std::vector<VecX>& xref, const std::vector<VecU>& uref) {
        xref_ = xref;
        uref_ = uref;
    }

    // Một bước RTI-SQP
    bool step() {
        // 1. Shift trajectory
        for (int k = 0; k < N-1; ++k) {
            xbar_[k] = xbar_[k+1];
            ubar_[k] = ubar_[k+1];
        }
        xbar_[N-1] = xref_.back();
        ubar_[N-1] = uref_.back();

        // 2. Linearize
        std::vector<MatXX> A_seq(N);
        std::vector<MatXU> B_seq(N);
        for (int k = 0; k < N; ++k) {
            analytic_jacobians(xbar_[k], ubar_[k], params_.dt, params_, cl_, A_seq[k], B_seq[k]);
        }

        // 3. Condense QP
        Eigen::MatrixXd H;
        Eigen::VectorXd g;
        condense(xbar_, ubar_, x0_, params_, cl_, H, g, A_seq, B_seq);

        // 4. Solve QP (giả sử không có ràng buộc thêm)
        Eigen::VectorXd lb = Eigen::VectorXd::Constant(N*NU, -1.0);
        Eigen::VectorXd ub = Eigen::VectorXd::Constant(N*NU, 1.0);
        QPResult qp = solve_qp(H, g, lb, ub);
        if (!qp.success) return false;

        // 5. Apply control
        VecU du0 = qp.du.segment(0, NU);
        ubar_[0] += du0;
        // (Có thể tích phân trạng thái mới ở đây)
        return true;
    }

    // Lấy điều khiển hiện tại
    VecU currentControl() const { return ubar_[0]; }

private:
    Params params_;
    CenterLine cl_;
    VecX x0_;
    std::vector<VecX> xbar_, xref_;
    std::vector<VecU> ubar_, uref_;
};

} // namespace nmpc
