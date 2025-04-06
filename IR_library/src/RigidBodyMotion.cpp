#include <RigidBodyMotion.hpp>
#include <iostream>
namespace IRlibrary {


	/** Returns true if a 3X3 matrix satisfies properties of SO(3) **/
	bool isSO3(SO3Mat mat){
	  SO3Mat eye;
	  eye << 1,0,0, 0,1,0, 0,0,1;
	  SO3Mat mat_prod = mat.transpose()*mat;
	  bool res = eye.isApprox(mat_prod) && nearZero(mat_prod.determinant() - 1);
	  return res;
	}

	/** Returns true if a 3x3 matrix is skew-symmetric **/
	bool isso3(so3Mat mat){
	  so3Mat trans_mat = -mat.transpose();
	    return trans_mat.isApprox(mat);
	}

	/** Returns true if a 4x4 matrix is SE(3) **/
	bool isSE3(SE3Mat mat){
	  Eigen::Matrix <double, 1, 4> last_row;
	  last_row << 0,0,0,1;
	  SO3Mat rot_mat = mat.block<3,3>(0,0);
	  bool rot_bool = isSO3(rot_mat);
	  bool last_row_bool = mat.row(3).isApprox(last_row);
	  return rot_bool && last_row_bool;
	}

	/** Returns true if a 4x4 matrix is se(3) **/
	bool isse3(se3Mat mat){
	so3Mat so3_mat = mat.block<3,3>(0,0);
	Eigen::Matrix <double, 1, 4> last_row;
	last_row << 0,0,0,0;
	bool rot_bool= isso3(so3_mat);
	bool last_row_bool = mat.row(3).isApprox(last_row);
	return rot_bool && last_row_bool;
	}

	/** Checks if the vector is unit **/
	bool isUnit(Vec2 vec){
	  return nearZero(std::abs(vec.norm()-1));
	}
	bool isUnit(Vec3 vec){
	  return nearZero(std::abs(vec.norm()-1));
	  
	}
	bool isUnit(Vec4 vec){
	  return nearZero(std::abs(vec.norm()-1));
	  
	}

	/** Computes the inverse of rotation matrix **/
	SO3Mat RotInv(SO3Mat mat){
	  return mat.transpose();
	}

	/** Computes the 3X3 skew-symmetric matrix corresponding to 3x1 omega vector **/
	so3Mat VecToso3(Vec3 omega){
		so3Mat mat;
		mat << 0, -omega(2), omega(1),
		  omega(2), 0, -omega(0), 
		  -omega(1), omega(0), 0;
		return mat;
	}

	/** Computes the 3-vector corresponding to 3x3 skew-symmetric matrix **/
	Vec3 so3ToVec(so3Mat mat){
		Vec3 omega;
		omega << mat(2,1), mat(0,2), mat(1,0);
		return omega;
	}

	/** Extracts the rotation axis, omega_hat, and the rotation amount, theta, from the 3-vector omega_hat theta of exponential coordinates for rotation, expc3 **/
	AxisAngle AxisAng3 (Vec3 expc3) {
		AxisAngle axisAngle;
		axisAngle.theta = expc3.norm();
		axisAngle.omega = expc3.normalized();
		if(nearZero(axisAngle.theta)){
		  axisAngle.omega=Vec3{0,0,1};
		}
		return axisAngle;
	}

	/** Computes the rotation matrix R \in SO(3) corresponding to the matrix exponential of so3Mat **/
	SO3Mat MatrixExp3 (so3Mat in_mat){
		SO3Mat mat;
		mat << 1, 0, 0,
					 0, 1, 0,
					 0, 0, 1;
		Vec3 omega = so3ToVec(in_mat);
		if(nearZero(in_mat.norm())){
		  return mat;
		}else{
		  AxisAngle omega_theta = (AxisAng3(omega));
		  double theta = omega_theta.theta;
		  so3Mat norm_mat = in_mat/theta;
		  return mat+std::sin(theta)*norm_mat+((1-std::cos(theta))*(norm_mat*norm_mat));
		    }
	}

	/** Computes the matrix logarithm so3mat \in so(3) of the rotation matrix R \in SO(3) **/
	so3Mat MatrixLog3(SO3Mat in_mat){
	  so3Mat mat;
	  mat << 1, 0, 0,
	         0, 1, 0,
	         0, 0, 1;
	  if (mat.isApprox(in_mat)){
	    return mat-mat;}
	  else if (nearZero(in_mat.trace()+1)){
	    Vec3 omega;
	    if(!nearZero(in_mat(2,2)+1))
	      {omega = (1.0/std::sqrt(2*(in_mat(2,2)+1)))*Vec3(in_mat(0,2),in_mat(1,2),1+in_mat(2,2));}

	    else if(!nearZero(in_mat(1,1)+1))
	      {omega = (1.0/std::sqrt(2*(in_mat(1,1)+1)))*Vec3(in_mat(0,1),in_mat(1,1)+1,in_mat(2,1));}

	    else{omega = (1.0/std::sqrt(2*(in_mat(0,0)+1)))*Vec3(in_mat(0,0)+1,in_mat(1,0),in_mat(2,0));}
	    mat = VecToso3(M_PI*omega);
	    return mat;
	  }
	  else{
	    double theta = std::acos((in_mat.trace()-1)/2.0);
	    theta = wrapTo2PI(theta);
	    mat = (theta/(std::sin(theta)*2))*(in_mat - in_mat.transpose());
	    return mat;
	  }
	}

	/** Compute the 4x4 transformation matrix **/
	SE3Mat RpToTrans(SO3Mat R, Vec3 p){
		SE3Mat T;
		T << R(0,0), R(0,1), R(0,2), p(0),
		     R(1,0), R(1,1), R(1,2), p(1),
		     R(2,0), R(2,1), R(2,2), p(2),
			0, 0, 0, 1;
		return T;
	}

	/** Extracts the rotation matrix and position vector from homogeneous transformation matrix **/
	void TransToRp(SE3Mat mat, SO3Mat &R, Vec3 &p){
		R = mat.block<3, 3>(0, 0);
		p << mat(0,3), mat(1,3), mat(2,3);
	}

	/** Inverse of transformation matrix **/
	SE3Mat TransInv(SE3Mat mat){
	  SO3Mat rot;
	  Vec3 p;
	  TransToRp(mat, rot, p);
	  return RpToTrans(rot.transpose(), - rot.transpose()*p);
	}

	/** Return the se(3) matrix corresponding to a 6-vector twist V **/
	se3Mat VecTose3(Twist V){
		se3Mat mat;
		Vec3 omega(V(0),V(1),V(2));
		Vec3 veloc(V(3),V(4),V(5));
		
		mat << VecToso3(omega), veloc,
		  0,0,0,0;
		return mat;
	}

	/** Returns Twist from se(3) matrix **/
	Twist se3ToVec(se3Mat mat){
		Twist V;
		V << mat(2,1), mat(0,2), mat(1,0), mat(0,3), mat(1,3), mat(2,3);
		return V;
	}

	/** Compute 6x6 adjoint matrix from T **/
	AdjMat Adjoint(SE3Mat mat) {
		AdjMat adj_mat;
		SO3Mat rot;
		Vec3 p;
		TransToRp(mat, rot, p);
		Eigen::MatrixXd zeros = Eigen::MatrixXd::Zero(3,3);
		adj_mat << rot, zeros, VecToso3(p)*rot, rot;
		return adj_mat;
	}

	/** Returns a normalized screw axis representation **/
	ScrewAxis ScrewToAxis(Vec3 q, Vec3 s, double h) {
		ScrewAxis S;
		S.segment(0,3) = s;
		S.segment(3,3) = q.cross(s) + (h*s);
		return S;
	}

	/** Extracts the normalized screw axis S and the distance traveled along the screw theta from the 6-vector of exponential coordinates Stheta **/
	void AxisAng(Twist STheta, ScrewAxis &S, double &theta){
	  theta = Vec3(STheta(0), STheta(1), STheta(2)).norm();
	  if (nearZero(theta))
	    theta = Vec3(STheta(3), STheta(4), STheta(5)).norm();
	  if(nearZero(theta))
	    {S = STheta;}
	  else{S = STheta/theta;}
	}

	/** Computes the homogeneous transformation matrix T \in SE(3) corresponding to matrix exponential of se3mat \in se(3) **/
	SE3Mat MatrixExp6(se3Mat in_mat){
		SE3Mat mat;
		Eigen::Matrix3d screw_mat = in_mat.block<3,3>(0,0);
		Eigen::Vector3d omega = so3ToVec(screw_mat);
		Vec3 veloc(in_mat(0,3), in_mat(1,3), in_mat(2,3));
		if (nearZero(omega.norm()))
		  {
		    mat << Eigen::MatrixXd::Identity(3,3), veloc,
		    0,0,0,1;
		    return mat;}
		else{
		  double theta = (AxisAng3(omega)).theta;
		  so3Mat omg_mat = screw_mat/theta;
		  Vec3 p = ((Eigen::MatrixXd::Identity(3,3)*theta+(1-std::cos(theta))*omg_mat+((theta-std::sin(theta))*(omg_mat*omg_mat)))*veloc)/theta;
		  mat << MatrixExp3(screw_mat), p,
		    0,0,0,1;
		  return mat;
		}
	}

	/** Computes the matrix logarithm se3mat \in se(3) of the homogeneous transformation matrix T \in SE(3) **/
	se3Mat MatrixLog6(SE3Mat T){
		se3Mat mat;
		SO3Mat rot;
		Vec3 p;
		TransToRp(T, rot, p);
		Eigen::Matrix3d omg_mat = MatrixLog3(rot);
		Eigen::Matrix3d zero_mat = Eigen::Matrix3d::Zero(3,3);
		if ( std::abs((omg_mat -zero_mat).norm()) < 1e-5 ) {
		  mat << zero_mat, p,
		    0, 0, 0, 0;
		}
		else {
		  double theta = std::acos((rot.trace() - 1) / 2.0);
		  Eigen::Matrix3d log_expand1 = Eigen::MatrixXd::Identity(3, 3) - omg_mat / 2.0;
		  Eigen::Matrix3d log_expand2 = (1.0 / theta - 1.0 / std::tan(theta / 2.0) / 2)*omg_mat*omg_mat / theta;
		  Eigen::Matrix3d log_expand = log_expand1 + log_expand2;
		  mat << omg_mat, log_expand*p,
		    0, 0, 0, 0;
		}
		return mat;
		
		return mat;
	}

} /* IRlibrary */ 

