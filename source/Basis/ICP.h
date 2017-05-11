///////////////////////////////////////////////////////////////////////////////
///   "Sparse Iterative Closest Point"
///   by Sofien Bouaziz, Andrea Tagliasacchi, Mark Pauly
///   Copyright (C) 2013  LGG, EPFL
///////////////////////////////////////////////////////////////////////////////
///   1) This file contains different implementations of the ICP algorithm.
///   2) This code requires EIGEN and NANOFLANN.
///   3) If OPENMP is activated some part of the code will be parallelized.
///   4) This code is for now designed for 3D registration
///   5) Two main input types are Eigen::Matrix3Xd or Eigen::Map<Eigen::Matrix3Xd>
///////////////////////////////////////////////////////////////////////////////
///   namespace nanoflann: NANOFLANN KD-tree adaptor for EIGEN
///   namespace RigidMotionEstimator: functions to compute the rigid motion
///   namespace SICP: sparse ICP implementation
///   namespace ICP: reweighted ICP implementation
///////////////////////////////////////////////////////////////////////////////
#ifndef ICP_H
#define ICP_H
#include <Eigen/Dense>
/// Compute the rigid motion for point-to-point and point-to-plane distances
namespace RigidMotionEstimator {
    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Confidence weights
    template <typename Derived1, typename Derived2, typename Derived3>
    Eigen::Affine3d point_to_point(Eigen::MatrixBase<Derived1>& X,
                                   Eigen::MatrixBase<Derived2>& Y,
                                   const Eigen::MatrixBase<Derived3>& w) {
        /// Normalize weight vector
        Eigen::VectorXd w_normalized = w/w.sum();
        /// De-mean
        Eigen::Vector3d X_mean, Y_mean;
        for(int i=0; i<3; ++i) {
            X_mean(i) = (X.row(i).array()*w_normalized.transpose().array()).sum();
            Y_mean(i) = (Y.row(i).array()*w_normalized.transpose().array()).sum();
        }
        X.colwise() -= X_mean;
        Y.colwise() -= Y_mean;
        /// Compute transformation
        Eigen::Affine3d transformation;
        Eigen::Matrix3d sigma = X * w_normalized.asDiagonal() * Y.transpose();
        Eigen::JacobiSVD<Eigen::Matrix3d> svd(sigma, Eigen::ComputeFullU | Eigen::ComputeFullV);
        if(svd.matrixU().determinant()*svd.matrixV().determinant() < 0.0) {
            Eigen::Vector3d S = Eigen::Vector3d::Ones(); S(2) = -1.0;
            transformation.linear().noalias() = svd.matrixV()*S.asDiagonal()*svd.matrixU().transpose();
        } else {
            transformation.linear().noalias() = svd.matrixV()*svd.matrixU().transpose();
        }
        transformation.translation().noalias() = Y_mean - transformation.linear()*X_mean;
        /// Apply transformation
        X = transformation*X;
        /// Re-apply mean
        X.colwise() += X_mean;
        Y.colwise() += Y_mean;
        /// Return transformation
        return transformation;
    }
    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    template <typename Derived1, typename Derived2>
    inline Eigen::Affine3d point_to_point(Eigen::MatrixBase<Derived1>& X,
                                          Eigen::MatrixBase<Derived2>& Y) {
        return point_to_point(X, Y, Eigen::VectorXd::Ones(X.cols()));
    }
    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Target normals (one 3D normal per column)
    /// @param Confidence weights
    /// @param Right hand side
    template <typename Derived1, typename Derived2, typename Derived3, typename Derived4, typename Derived5>
    Eigen::Affine3d point_to_plane(Eigen::MatrixBase<Derived1>& X,
                                   Eigen::MatrixBase<Derived2>& Y,
                                   Eigen::MatrixBase<Derived3>& N,
                                   const Eigen::MatrixBase<Derived4>& w,
                                   const Eigen::MatrixBase<Derived5>& u) {
        typedef Eigen::Matrix<double, 6, 6> Matrix66;
        typedef Eigen::Matrix<double, 6, 1> Vector6;
        typedef Eigen::Block<Matrix66, 3, 3> Block33;
        /// Normalize weight vector
        Eigen::VectorXd w_normalized = w/w.sum();
        /// De-mean
        Eigen::Vector3d X_mean;
        for(int i=0; i<3; ++i)
            X_mean(i) = (X.row(i).array()*w_normalized.transpose().array()).sum();
        X.colwise() -= X_mean;
        Y.colwise() -= X_mean;
        /// Prepare LHS and RHS
        Matrix66 LHS = Matrix66::Zero();
        Vector6 RHS = Vector6::Zero();
        Block33 TL = LHS.topLeftCorner<3,3>();
        Block33 TR = LHS.topRightCorner<3,3>();
        Block33 BR = LHS.bottomRightCorner<3,3>();
        Eigen::MatrixXd C = Eigen::MatrixXd::Zero(3,X.cols());
        #pragma omp parallel
        {
            #pragma omp for
            for(int i=0; i<X.cols(); i++) {
                C.col(i) = X.col(i).cross(N.col(i));
            }
            #pragma omp sections nowait
            {
                #pragma omp section
                for(int i=0; i<X.cols(); i++) TL.selfadjointView<Eigen::Upper>().rankUpdate(C.col(i), w(i));
                #pragma omp section
                for(int i=0; i<X.cols(); i++) TR += (C.col(i)*N.col(i).transpose())*w(i);
                #pragma omp section
                for(int i=0; i<X.cols(); i++) BR.selfadjointView<Eigen::Upper>().rankUpdate(N.col(i), w(i));
                #pragma omp section
                for(int i=0; i<C.cols(); i++) {
                    double dist_to_plane = -((X.col(i) - Y.col(i)).dot(N.col(i)) - u(i))*w(i);
                    RHS.head<3>() += C.col(i)*dist_to_plane;
                    RHS.tail<3>() += N.col(i)*dist_to_plane;
                }
            }
        }
        LHS = LHS.selfadjointView<Eigen::Upper>();
        /// Compute transformation
        Eigen::Affine3d transformation;
        Eigen::LDLT<Matrix66> ldlt(LHS);
        RHS = ldlt.solve(RHS);
        transformation  = Eigen::AngleAxisd(RHS(0), Eigen::Vector3d::UnitX()) *
                          Eigen::AngleAxisd(RHS(1), Eigen::Vector3d::UnitY()) *
                          Eigen::AngleAxisd(RHS(2), Eigen::Vector3d::UnitZ());
        transformation.translation() = RHS.tail<3>();
        /// Apply transformation
        X = transformation*X;
        /// Re-apply mean
        X.colwise() += X_mean;
        Y.colwise() += X_mean;
        /// Return transformation
        return transformation;
    }
    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Target normals (one 3D normal per column)
    /// @param Confidence weights
    template <typename Derived1, typename Derived2, typename Derived3, typename Derived4>
    inline Eigen::Affine3d point_to_plane(Eigen::MatrixBase<Derived1>& X,
                                          Eigen::MatrixBase<Derived2>& Yp,
                                          Eigen::MatrixBase<Derived3>& Yn,
                                          const Eigen::MatrixBase<Derived4>& w) {
        return point_to_plane(X, Yp, Yn, w, Eigen::VectorXd::Zero(X.cols()));
    }
}
///////////////////////////////////////////////////////////////////////////////
/// ICP implementation using ADMM/ALM/Penalty method
namespace SICP {
    struct Parameters {
        bool use_penalty = false; /// if use_penalty then penalty method else ADMM or ALM (see max_inner)
        double p = 1.0;           /// p norm
        double mu = 10.0;         /// penalty weight
        double alpha = 1.2;       /// penalty increase factor
        double max_mu = 1e5;      /// max penalty
        int max_icp = 100;        /// max ICP iteration
        int max_outer = 100;      /// max outer iteration
        int max_inner = 1;        /// max inner iteration. If max_inner=1 then ADMM else ALM
        double stop = 1e-5;       /// stopping criteria
        bool print_icpn = false;  /// (debug) print ICP iteration 
    };
    /// Shrinkage operator (Automatic loop unrolling using template)
    template<unsigned int I>
    inline double shrinkage(double mu, double n, double p, double s) {
        return shrinkage<I-1>(mu, n, p, 1.0 - (p/mu)*std::pow(n, p-2.0)*std::pow(s, p-1.0));
    }
    template<>
    inline double shrinkage<0>(double, double, double, double s) {return s;}
    /// 3D Shrinkage for point-to-point
    template<unsigned int I>
    inline void shrink(Eigen::Matrix3Xd& Q, double mu, double p) {
        double Ba = std::pow((2.0/mu)*(1.0-p), 1.0/(2.0-p));
        double ha = Ba + (p/mu)*std::pow(Ba, p-1.0);
        #pragma omp parallel for
        for(int i=0; i<Q.cols(); ++i) {
            double n = Q.col(i).norm();
            double w = 0.0;
            if(n > ha) w = shrinkage<I>(mu, n, p, (Ba/n + 1.0)/2.0);
            Q.col(i) *= w;
        }
    }
    /// 1D Shrinkage for point-to-plane
    template<unsigned int I>
    inline void shrink(Eigen::VectorXd& y, double mu, double p) {
        double Ba = std::pow((2.0/mu)*(1.0-p), 1.0/(2.0-p));
        double ha = Ba + (p/mu)*std::pow(Ba, p-1.0);
        #pragma omp parallel for
        for(int i=0; i<y.rows(); ++i) {
            double n = std::abs(y(i));
            double s = 0.0;
            if(n > ha) s = shrinkage<I>(mu, n, p, (Ba/n + 1.0)/2.0);
            y(i) *= s;
        }
    }

	/// Sparse ICP with point to point
	/// @param Source (one 3D point per column)
	/// @param Target (one 3D point per column)
	/// @param Parameters
	template <typename Derived1, typename Derived2>
	void point_to_point_fixed(Eigen::MatrixBase<Derived1>& X,
		Eigen::MatrixBase<Derived2>& Y,
		Parameters par = Parameters()) {

		/// Buffers
		Eigen::Matrix3Xd Q = Y;
		Eigen::Matrix3Xd Z = Eigen::Matrix3Xd::Zero(3, X.cols());
		Eigen::Matrix3Xd C = Eigen::Matrix3Xd::Zero(3, X.cols());
		Eigen::Matrix3Xd Xo1 = X;
		Eigen::Matrix3Xd Xo2 = X;
		/// ICP

		/// Computer rotation and translation
		double mu = par.mu;
		for (int outer = 0; outer < par.max_outer; ++outer) {
			double dual = 0.0;
			for (int inner = 0; inner < par.max_inner; ++inner) {
				/// Z update (shrinkage)
				Z = X - Q + C / mu;
				shrink<3>(Z, mu, par.p);
				/// Rotation and translation update
				Eigen::Matrix3Xd U = Q + Z - C / mu;
				RigidMotionEstimator::point_to_point(X, U);
				/// Stopping criteria
				dual = (X - Xo1).colwise().norm().maxCoeff();
				Xo1 = X;
				if (dual < par.stop) break;
			}
			/// C update (lagrange multipliers)
			Eigen::Matrix3Xd P = X - Q - Z;
			if (!par.use_penalty) C.noalias() += mu*P;
			/// mu update (penalty)
			if (mu < par.max_mu) mu *= par.alpha;
			/// Stopping criteria
			double primal = P.colwise().norm().maxCoeff();
			if (primal < par.stop && dual < par.stop) break;
		}

	}


    /// Sparse ICP with point to point
    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Parameters
    template <typename Derived1, typename Derived2>
    void point_to_point(Eigen::MatrixBase<Derived1>& X,
                        Eigen::MatrixBase<Derived2>& Y,
                        Parameters par = Parameters()) {
        /// Build kd-tree
        nanoflann::KDTreeAdaptor<Eigen::MatrixBase<Derived2>, 3, nanoflann::metric_L2_Simple> kdtree(Y);
        /// Buffers
        Eigen::Matrix3Xd Q = Eigen::Matrix3Xd::Zero(3, X.cols());
        Eigen::Matrix3Xd Z = Eigen::Matrix3Xd::Zero(3, X.cols());
        Eigen::Matrix3Xd C = Eigen::Matrix3Xd::Zero(3, X.cols());
        Eigen::Matrix3Xd Xo1 = X;
        Eigen::Matrix3Xd Xo2 = X;
        /// ICP
        for(int icp=0; icp<par.max_icp; ++icp) {
            if(par.print_icpn) std::cout << "Iteration #" << icp << "/" << par.max_icp << std::endl;
            /// Find closest point
            #pragma omp parallel for
            for(int i=0; i<X.cols(); ++i) {
                Q.col(i) = Y.col(kdtree.closest(X.col(i).data()));
            }
            /// Computer rotation and translation
            double mu = par.mu;
            for(int outer=0; outer<par.max_outer; ++outer) {
                double dual = 0.0;
                for(int inner=0; inner<par.max_inner; ++inner) {
                    /// Z update (shrinkage)
                    Z = X-Q+C/mu;
                    shrink<3>(Z, mu, par.p);
                    /// Rotation and translation update
                    Eigen::Matrix3Xd U = Q+Z-C/mu;
                    RigidMotionEstimator::point_to_point(X, U);
                    /// Stopping criteria
                    dual = (X-Xo1).colwise().norm().maxCoeff();
                    Xo1 = X;
                    if(dual < par.stop) break;
                }
                /// C update (lagrange multipliers)
                Eigen::Matrix3Xd P = X-Q-Z;
                if(!par.use_penalty) C.noalias() += mu*P;
                /// mu update (penalty)
                if(mu < par.max_mu) mu *= par.alpha;
                /// Stopping criteria
                double primal = P.colwise().norm().maxCoeff();
                if(primal < par.stop && dual < par.stop) break;
            }
            /// Stopping criteria
            double stop = (X-Xo2).colwise().norm().maxCoeff();
            Xo2 = X;
            if(stop < par.stop) break;
        }
    }
    /// Sparse ICP with point to plane
    /// @param Source (one 3D point per column)
    /// @param Target (one 3D point per column)
    /// @param Target normals (one 3D normal per column)
    /// @param Parameters
    template <typename Derived1, typename Derived2, typename Derived3>
    void point_to_plane(Eigen::MatrixBase<Derived1>& X,
                        Eigen::MatrixBase<Derived2>& Y,
                        Eigen::MatrixBase<Derived3>& N,
                        Parameters par = Parameters()) {
        /// Build kd-tree
        nanoflann::KDTreeAdaptor<Eigen::MatrixBase<Derived2>, 3, nanoflann::metric_L2_Simple> kdtree(Y);
        /// Buffers
        Eigen::Matrix3Xd Qp = Eigen::Matrix3Xd::Zero(3, X.cols());
        Eigen::Matrix3Xd Qn = Eigen::Matrix3Xd::Zero(3, X.cols());
        Eigen::VectorXd Z = Eigen::VectorXd::Zero(X.cols());
        Eigen::VectorXd C = Eigen::VectorXd::Zero(X.cols());
        Eigen::Matrix3Xd Xo1 = X;
        Eigen::Matrix3Xd Xo2 = X;
        /// ICP
        for(int icp=0; icp<par.max_icp; ++icp) {
            if(par.print_icpn) std::cout << "Iteration #" << icp << "/" << par.max_icp << std::endl;
            
            /// Find closest point
            #pragma omp parallel for
            for(int i=0; i<X.cols(); ++i) {
                int id = kdtree.closest(X.col(i).data());
                Qp.col(i) = Y.col(id);
                Qn.col(i) = N.col(id);
            }
            /// Computer rotation and translation
            double mu = par.mu;
            for(int outer=0; outer<par.max_outer; ++outer) {
                double dual = 0.0;
                for(int inner=0; inner<par.max_inner; ++inner) {
                    /// Z update (shrinkage)
                    Z = (Qn.array()*(X-Qp).array()).colwise().sum().transpose()+C.array()/mu;
                    shrink<3>(Z, mu, par.p);
                    /// Rotation and translation update
                    Eigen::VectorXd U = Z-C/mu;
                    RigidMotionEstimator::point_to_plane(X, Qp, Qn, Eigen::VectorXd::Ones(X.cols()), U);
                    /// Stopping criteria
                    dual = (X-Xo1).colwise().norm().maxCoeff();
                    Xo1 = X;
                    if(dual < par.stop) break;
                }
                /// C update (lagrange multipliers)
                Eigen::VectorXf P = (Qn.array()*(X-Qp).array()).colwise().sum().transpose()-Z.array();
                if(!par.use_penalty) C.noalias() += mu*P;
                /// mu update (penalty)
                if(mu < par.max_mu) mu *= par.alpha;
                /// Stopping criteria
                double primal = P.array().abs().maxCoeff();
                if(primal < par.stop && dual < par.stop) break;
            }
            /// Stopping criteria
            double stop = (X-Xo2).colwise().norm().maxCoeff();
            Xo2 = X;
            if(stop < par.stop) break;
        }
    }
}
///////////////////////////////////////////////////////////////////////////////
#endif
