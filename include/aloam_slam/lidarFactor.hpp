// Author:   Tong Qin               qintonguav@gmail.com
// 	         Shaozu Cao 		    saozu.cao@connect.ust.hk

#include <ceres/ceres.h>
#include <ceres/rotation.h>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl_conversions/pcl_conversions.h>


template <typename T>
Eigen::Matrix<T, 3, 3> skewSymmetric(const Eigen::Matrix<T, 3, 1> &vec) {

  Eigen::Matrix<T, 3, 3> ss;
  ss << 0.0, -vec[2], vec[1], vec[2], 0.0, -vec[0], -vec[1], vec[0], 0.0;

  return ss;
}

// | -------------------------- Edges ------------------------- |

/*//{ struct LidarEdgeFactor */
struct LidarEdgeFactor
{
  LidarEdgeFactor(const Eigen::Vector3d &p_, const Eigen::Vector3d &p_i_, const Eigen::Vector3d &p_j_, double s_) : p(p_), p_j(p_j_), s(s_) {
    v = (p_i_ - p_j_).normalized();
  }

  template <typename T>
  bool operator()(const T *q, const T *t, T *residual) const {
    getResidual(q, t, residual);
    return true;
  }

  template <typename T>
  void getResidual(const T *q, const T *t, T *residual) const {
    const Eigen::Matrix<T, 3, 1> p_{T(p.x()), T(p.y()), T(p.z())};
    const Eigen::Matrix<T, 3, 1> v_{T(v.x()), T(v.y()), T(v.z())};

    const Eigen::Matrix<T, 3, 1> trans = T(s) * Eigen::Matrix<T, 3, 1>{t[0], t[1], t[2]};
    const Eigen::Quaternion<T>   q_identity{T(1), T(0), T(0), T(0)};
    Eigen::Quaternion<T>         R{q[3], q[0], q[1], q[2]};
    R                                = q_identity.slerp(T(s), R);
    const Eigen::Matrix<T, 3, 1> p_w = R * p_ + trans;

    const Eigen::Matrix<T, 3, 1> j_to_p_w = p_w - p_j;
    const Eigen::Matrix<T, 3, 1> res      = j_to_p_w - (j_to_p_w.dot(v_)) * v_;

    residual[0] = res[0];
    residual[1] = res[1];
    residual[2] = res[2];
  }

  template <typename T>
  void getJacobian(const T *q, Eigen::Matrix<T, 3, 6> &jacobian) const {
    const Eigen::Matrix<T, 3, 1> p_{T(p.x()), T(p.y()), T(p.z())};
    const Eigen::Matrix<T, 3, 1> v_{T(v.x()), T(v.y()), T(v.z())};
    const Eigen::Matrix<T, 3, 3> I = Eigen::Matrix<T, 3, 3>::Identity();

    const Eigen::Quaternion<T> R{q[3], q[0], q[1], q[2]};

    const Eigen::Matrix<T, 3, 3> dr_dt = I - v_ * v_.transpose();
    const Eigen::Matrix<T, 3, 3> dr_dR = -dr_dt * R * skewSymmetric(p_);

    jacobian.block(0, 0, 3, 3) = dr_dt;
    jacobian.block(0, 3, 3, 3) = dr_dR;
  }

  static ceres::CostFunction *Create(LidarEdgeFactor *factor) {
    return (new ceres::AutoDiffCostFunction<LidarEdgeFactor, 3, 4, 3>(factor));
  }

  Eigen::Vector3d p, p_j;
  Eigen::Vector3d v;
  double          s;
};
/*//}*/

/*//{ struct LidarEdgeNormFactor */
struct LidarEdgeNormFactor
{
  LidarEdgeNormFactor(const Eigen::Vector3d &p_, const Eigen::Vector3d &p1_, const Eigen::Vector3d &p2_, double s_) : p(p_), p1(p1_), p2(p2_), s(s_) {
  }

  template <typename T>
  bool operator()(const T *q, const T *t, T *residual) const {
    getResidual(q, t, residual);
    return true;
  }

  template <typename T>
  void getResidual(const T *q, const T *t, T *residual) const {
    const Eigen::Matrix<T, 3, 1> p_{T(p.x()), T(p.y()), T(p.z())};
    const Eigen::Matrix<T, 3, 1> p1_{T(p1.x()), T(p1.y()), T(p1.z())};
    const Eigen::Matrix<T, 3, 1> p2_{T(p2.x()), T(p2.y()), T(p2.z())};

    const Eigen::Matrix<T, 3, 1> trans = T(s) * Eigen::Matrix<T, 3, 1>{t[0], t[1], t[2]};
    const Eigen::Quaternion<T>   q_identity{T(1), T(0), T(0), T(0)};
    Eigen::Quaternion<T>         R{q[3], q[0], q[1], q[2]};
    R = q_identity.slerp(T(s), R);

    const Eigen::Matrix<T, 3, 1> p_w = R * p_ + trans;
    residual[0]                      = ((p_w - p1_).cross(p_w - p2_)).norm() / (p1_ - p2_).norm();
  }

  template <typename T>
  void getJacobian(const T *q, const T *residual, Eigen::Matrix<T, 1, 6> &jacobian) const {
    const Eigen::Matrix<T, 3, 1> p1_{T(p1.x()), T(p1.y()), T(p1.z())};
    const Eigen::Matrix<T, 3, 1> p2_{T(p2.x()), T(p2.y()), T(p2.z())};
    const Eigen::Matrix<T, 3, 1> p1_p2_ = p1_ - p2_;

    const Eigen::Matrix<T, 3, 1> res{residual[0], residual[1], residual[2]};

    const Eigen::Matrix<T, 3, 3> R = Eigen::Quaternion<T>(q[3], q[0], q[1], q[2]).toRotationMatrix();

    const Eigen::Matrix<T, 1, 3> coeff = (1.0 / p1_p2_.norm()) * (res.transpose() / res.norm()) * skewSymmetric(p1_p2_);
    const Eigen::Matrix<T, 1, 3> dr_dt = -coeff;
    const Eigen::Matrix<T, 1, 3> dr_dq = coeff * R * skewSymmetric(p);

    jacobian << dr_dt[0], dr_dt[1], dr_dt[2], dr_dq[0], dr_dq[1], dr_dq[2];
  }

  static ceres::CostFunction *Create(LidarEdgeNormFactor *factor) {
    return (new ceres::AutoDiffCostFunction<LidarEdgeNormFactor, 1, 4, 3>(factor));
  }

  Eigen::Vector3d p, p1, p2;
  double          s;
};
/*//}*/

// | ------------------------- Planes ------------------------- |

/*//{ struct LidarPlaneFactor */
struct LidarPlaneFactor
{
  LidarPlaneFactor(const Eigen::Vector3d &curr_point_, const Eigen::Vector3d &n_, double d_) : p(curr_point_), n(n_), d(d_) {
  }

  template <typename T>
  bool operator()(const T *q, const T *t, T *residual) const {
    getResidual(q, t, residual);

    return true;
  }

  template <typename T>
  void getResidual(const T *q, const T *t, T *residual) const {

    const Eigen::Matrix<T, 3, 1> p_i{T(p.x()), T(p.y()), T(p.z())};
    const Eigen::Matrix<T, 3, 1> n_vec{T(n.x()), T(n.y()), T(n.z())};

    const Eigen::Quaternion<T>   R{q[3], q[0], q[1], q[2]};
    const Eigen::Matrix<T, 3, 1> translation = Eigen::Matrix<T, 3, 1>(t[0], t[1], t[2]);

    const Eigen::Matrix<T, 3, 1> p_i_w = R * p_i + translation;
    const Eigen::Matrix<T, 3, 1> res   = (n_vec.dot(p_i_w) + d) * n_vec;

    residual[0] = res[0];
    residual[1] = res[1];
    residual[2] = res[2];
  }

  template <typename T>
  void getJacobian(const T *q, Eigen::Matrix<T, 3, 6> &jacobian) const {

    const Eigen::Matrix<T, 3, 1> p_{T(p.x()), T(p.y()), T(p.z())};
    const Eigen::Matrix<T, 3, 1> n_{T(n.x()), T(n.y()), T(n.z())};

    const Eigen::Quaternion<T> R{q[3], q[0], q[1], q[2]};

    const Eigen::Matrix<T, 3, 3> dr_dt = n_ * n_.transpose();
    const Eigen::Matrix<T, 3, 3> dr_dR = -dr_dt * R * skewSymmetric(p_);

    jacobian.block(0, 0, 3, 3) = dr_dt;
    jacobian.block(0, 3, 3, 3) = dr_dR;
  }

  static ceres::CostFunction *Create(LidarPlaneFactor *factor) {
    return (new ceres::AutoDiffCostFunction<LidarPlaneFactor, 3, 4, 3>(factor));
  }

  Eigen::Vector3d p;
  Eigen::Vector3d n;
  double          d;
};
/*//}*/

/*//{ struct LidarPlaneNormFactorFromPoints */
struct LidarPlaneNormFactorFromPoints
{
  LidarPlaneNormFactorFromPoints(Eigen::Vector3d curr_point_, Eigen::Vector3d last_point_j_, Eigen::Vector3d last_point_l_, Eigen::Vector3d last_point_m_,
                                 double s_)
      : curr_point(curr_point_), last_point_j(last_point_j_), last_point_l(last_point_l_), last_point_m(last_point_m_), s(s_) {
    ljm_norm = (last_point_j - last_point_l).cross(last_point_j - last_point_m);
    ljm_norm.normalize();
  }

  template <typename T>
  bool operator()(const T *q, const T *t, T *residual) const {

    Eigen::Matrix<T, 3, 1> cp{T(curr_point.x()), T(curr_point.y()), T(curr_point.z())};
    Eigen::Matrix<T, 3, 1> lpj{T(last_point_j.x()), T(last_point_j.y()), T(last_point_j.z())};
    Eigen::Matrix<T, 3, 1> ljm{T(ljm_norm.x()), T(ljm_norm.y()), T(ljm_norm.z())};

    // Eigen::Quaternion<T> q_last_curr{q[3], T(s) * q[0], T(s) * q[1], T(s) * q[2]};
    Eigen::Quaternion<T> q_last_curr{q[3], q[0], q[1], q[2]};
    Eigen::Quaternion<T> q_identity{T(1), T(0), T(0), T(0)};
    q_last_curr = q_identity.slerp(T(s), q_last_curr);
    Eigen::Matrix<T, 3, 1> t_last_curr{T(s) * t[0], T(s) * t[1], T(s) * t[2]};

    Eigen::Matrix<T, 3, 1> lp;
    lp = q_last_curr * cp + t_last_curr;

    residual[0] = (lp - lpj).dot(ljm);

    return true;
  }

  static ceres::CostFunction *Create(LidarPlaneNormFactorFromPoints *factor) {
    return (new ceres::AutoDiffCostFunction<LidarPlaneNormFactorFromPoints, 1, 4, 3>(factor));
  }

  Eigen::Vector3d curr_point, last_point_j, last_point_l, last_point_m;
  Eigen::Vector3d ljm_norm;
  double          s;
};
/*//}*/

/*//{ struct LidarPlaneNormFactorFromNormal */
struct LidarPlaneNormFactorFromNormal
{

  LidarPlaneNormFactorFromNormal(const Eigen::Vector3d &curr_point_, const Eigen::Vector3d &plane_unit_norm_, double negative_OA_dot_norm_)
      : p(curr_point_), plane_unit_norm(plane_unit_norm_), negative_OA_dot_norm(negative_OA_dot_norm_) {
  }

  template <typename T>
  bool operator()(const T *q, const T *t, T *residual) const {
    getResidual(q, t, residual);
    return true;
  }

  template <typename T>
  void getResidual(const T *q, const T *t, T *residual) const {
    const Eigen::Quaternion<T>   Q_w{q[3], q[0], q[1], q[2]};
    const Eigen::Matrix<T, 3, 1> t_w{t[0], t[1], t[2]};

    const Eigen::Matrix<T, 3, 1> p_{T(p.x()), T(p.y()), T(p.z())};
    const Eigen::Matrix<T, 3, 1> p_w = Q_w * p_ + t_w;
    const Eigen::Matrix<T, 3, 1> unit_norm(T(plane_unit_norm.x()), T(plane_unit_norm.y()), T(plane_unit_norm.z()));

    residual[0] = unit_norm.dot(p_w) + T(negative_OA_dot_norm);
  }

  template <typename T>
  void getJacobian(const T *q, Eigen::Matrix<T, 1, 6> &jacobian) const {

    const Eigen::Matrix<T, 3, 3> R = Eigen::Quaternion<T>(q[3], q[0], q[1], q[2]).toRotationMatrix();
    const Eigen::Matrix<T, 1, 3> norm(plane_unit_norm.x(), plane_unit_norm.y(), plane_unit_norm.z());
    const Eigen::Matrix<T, 1, 3> dr_dq = -norm * R * skewSymmetric(p);

    /* jacobian << plane_unit_norm[0], dr_dq[0], plane_unit_norm[1], dr_dq[1], plane_unit_norm[2], dr_dq[2]; */
    jacobian << plane_unit_norm[0], plane_unit_norm[1], plane_unit_norm[2], dr_dq[0], dr_dq[1], dr_dq[2];
  }

  static ceres::CostFunction *Create(LidarPlaneNormFactorFromNormal *factor) {
    return (new ceres::AutoDiffCostFunction<LidarPlaneNormFactorFromNormal, 1, 4, 3>(factor));
  }

  Eigen::Vector3d p;
  Eigen::Vector3d plane_unit_norm;
  double          negative_OA_dot_norm;
};
/*//}*/
