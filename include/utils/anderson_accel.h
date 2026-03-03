#pragma once
#include <Eigen/Dense>
#include <deque>

class AndersonAccel {
public:
  struct Options {
    int m;          // history size
    double lambda;  // regularization
    Options() : m(5), lambda(1e-6) {}
    Options(int m_, double lambda_) : m(m_), lambda(lambda_) {}
  };

  AndersonAccel() : opt_() {}
  explicit AndersonAccel(const Options& opt) : opt_(opt) {}

  void reset() { X_.clear(); }

  void init(const Eigen::VectorXd& x) {
    reset();
    X_.push_back(x);
  }

  Eigen::VectorXd compute(const Eigen::VectorXd& xk) {
    if (X_.empty()) { X_.push_back(xk); return xk; }
    X_.push_back(xk);
    if ((int)X_.size() > opt_.m + 1) X_.pop_front();

    const int n  = (int)X_.back().rows();
    const int mk = (int)X_.size() - 1;
    if (mk <= 1) return xk;

    Eigen::MatrixXd F(n, mk);
    for (int j = 0; j < mk; ++j) {
      F.col(j) = X_[j + 1] - X_[j];
    }

    Eigen::MatrixXd G = F.transpose() * F;
    G.diagonal().array() += opt_.lambda;

    Eigen::MatrixXd KKT(mk + 1, mk + 1);
    KKT.setZero();
    KKT.topLeftCorner(mk, mk) = G;
    KKT.topRightCorner(mk, 1).setOnes();
    KKT.bottomLeftCorner(1, mk).setOnes();

    Eigen::VectorXd rhs(mk + 1);
    rhs.setZero();
    rhs(mk) = 1.0;

    Eigen::VectorXd sol = KKT.ldlt().solve(rhs);
    Eigen::VectorXd alpha = sol.head(mk);

    Eigen::VectorXd x_acc = Eigen::VectorXd::Zero(n);
    for (int j = 0; j < mk; ++j) x_acc += alpha(j) * X_[j + 1];
    return x_acc;
  }

private:
  Options opt_;
  std::deque<Eigen::VectorXd> X_;
};
