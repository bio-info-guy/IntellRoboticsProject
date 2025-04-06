#include <InverseKinematics.hpp>

namespace IRlibrary {
	/** This function uses iterative Newton–Raphson to calculate the inverse
	 * kinematics given the list of joint screws B_i expressed in the
	 * end-effector frame, the end-effector home configuration M,
	 * the desired end-effector configuration T,
	 * an initial guess at the joint angles init_thetaList,
	 * and the tolerances eps_omega and eps_v on the final error.
	 * If a solution is not found within a maximum number of iterations, then success is false.
	 */
	bool IKinBody(std::vector <ScrewAxis> const & Blist, SE3Mat const & M, SE3Mat const & T, Eigen::VectorXd const & init_thetaList, Eigen::VectorXd & thetaList, double const eps_omega, double const eps_v, size_t maxIter) {
		// Copy data from initial list to the updated list of angles
		thetaList = init_thetaList;
		SE3Mat Tsb = SE3Mat::Identity();
		Twist Vs = Twist::Zero();
		for (size_t i = 0; i < maxIter; ++i) {
			std::vector <double> thetaVec(thetaList.data(), thetaList.data() + thetaList.size());
			Tsb = FKinBody(M, Blist, thetaVec);
			Vs = Adjoint(Tsb) * se3ToVec(MatrixLog6(TransInv(Tsb) * T));
			if( Vs.head(3).norm() < eps_omega and Vs.tail(3).norm() < eps_v) {
				return 0;
			}
			Eigen::MatrixXd Jb = JacobianBody(Blist, thetaVec);
			auto JbCOD = Jb.completeOrthogonalDecomposition();
			//Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd> JbCOD(Jb);
			if((unsigned)JbCOD.rank() < Blist.size()) {
				std::cout << "Jacobian not invertible!" << std::endl;
				return 1;
			}
			Eigen::MatrixXd pinv = JbCOD.pseudoInverse();
			thetaList += pinv * Vs;
		}
		return 1;
	}

/** This is similar to IKinBody, except that the joint screws S_i are expressed in the
 * space frame and the tolerances are interpreted in the space frame, also.
 */
	bool IKinSpace(std::vector <ScrewAxis> const & Slist, SE3Mat const & M, SE3Mat const & T, Eigen::VectorXd const & init_thetaList, Eigen::VectorXd & thetaList, double eps_omega, double eps_v, size_t maxIter) {
		// Copy data from initial list to the updated list of angles
		thetaList = init_thetaList;
		// Map eigen vector the thetalist.
		/* Eigen::Map <Eigen::VectorXd> thetaVector(&thetaList[0], init_thetaList.size()); */
		SE3Mat Tsb = SE3Mat::Identity();
		Twist Vs = Twist::Zero();
		for (size_t i = 0; i < maxIter; ++i) {
			std::vector <double> thetaVec(thetaList.data(), thetaList.data() + thetaList.size());
			Tsb = FKinSpace(M, Slist, thetaVec);
			Vs = Adjoint(Tsb) * se3ToVec(MatrixLog6(TransInv(Tsb) * T));
			if( Vs.head(3).norm() < eps_omega and Vs.tail(3).norm() < eps_v) {
				return 0;
			}
			Eigen::MatrixXd Js = JacobianSpace(Slist, thetaVec);
			//auto JsCOD = Js.completeOrthogonalDecomposition();
			Eigen::CompleteOrthogonalDecomposition<Eigen::MatrixXd>JsCOD(Js);
			if((unsigned)JsCOD.rank() < Slist.size()) {
				std::cout << "Jacobian not invertible!" << std::endl;
				return 1;
			}
			Eigen::MatrixXd pinv = JsCOD.pseudoInverse();
			thetaList += pinv * Vs;
		}
		return 1;
	}

	/** This function computes the inverse of the condition number.
	 * If this value is equal to 1 then the robot is in an isotropic configuration.
	 * If the value if close to zero then the robot is approaching singularity.
	 */

	double getReciprocalConditionNumber(std::vector <ScrewAxis> &Blist, Eigen::VectorXd &thetaList) {
		std::vector <double> thetaVec(thetaList.data(), thetaList.data() + thetaList.size());
		Eigen::MatrixXd Jb = JacobianBody (Blist, thetaVec);
		auto Jb_omega = Jb.topRows(3);
		auto Jb_v = Jb.bottomRows(3);
		auto Gomega = Jb_omega.transpose() * Jb_omega;
		auto Gv = Jb_v.transpose() * Jb_v;
		Eigen::EigenSolver <Eigen::MatrixXd> es;
		auto ev_omega = es.compute(Gomega, false).eigenvalues();
		auto ev_v = es.compute(Gv, false).eigenvalues();
		std::vector <double> ev_omegaV, ev_vV;
		for (int i = 0; i < ev_omega.size(); ++i) {
			auto ev = ev_omega(i);
			if (nearZero(ev.imag())) {
				ev_omegaV.push_back(ev.real());
			}
		}
		for (int i = 0; i < ev_v.size(); ++i) {
			auto ev = ev_v(i);
			if (nearZero(ev.imag())) {
				ev_vV.push_back(ev.real());
			}
		}
		auto lambda_min = std::min_element(begin(ev_omegaV), end(ev_omegaV));
		auto lambda_max = std::max_element(begin(ev_omegaV), end(ev_omegaV));
		if (nearZero(*lambda_max))
			return 0;
		double cond =  *lambda_min/(*lambda_max);

		lambda_min = std::min_element(begin(ev_vV), end(ev_vV));
		lambda_max = std::max_element(begin(ev_vV), end(ev_vV));
		if (nearZero(*lambda_max))
			return 0;
		double cond1 =  *lambda_min/(*lambda_max);
		return cond1 < cond ? cond1 : cond;
	}
} /* IRlibrary */
