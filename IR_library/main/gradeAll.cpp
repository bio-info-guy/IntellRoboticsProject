#include <cmath>
#include <gtest/gtest.h>
#include <iostream>
#include <TypeDefs.hpp>
#include <MathUtils.hpp>
#include <RigidBodyMotion.hpp>
#include <ForwardKinematics.hpp>
#include <Jacobian.hpp>

using namespace IRlibrary;

inline bool equalQ(double val1, double val2, double eps = 1e-12) { return fabs(val1 - val2) <= eps; }

TEST (nearZero, positiveInput) {
	EXPECT_TRUE(nearZero(0));
	EXPECT_TRUE(nearZero(0.0));
	EXPECT_FALSE(nearZero(1e-2));
	EXPECT_TRUE(nearZero(1e-12));
	EXPECT_FALSE(nearZero(1.0));
	EXPECT_FALSE(nearZero(1e10));
	EXPECT_FALSE(nearZero(1e-2, 0.0));
	EXPECT_TRUE(nearZero(1e-2, 1e-1));
}

TEST (nearZero, negativeInput) {
	EXPECT_TRUE(nearZero(-0));
	EXPECT_TRUE(nearZero(-0.0));
	EXPECT_FALSE(nearZero(-1e-2));
	EXPECT_TRUE(nearZero(-1e-12));
	EXPECT_FALSE(nearZero(-1.0));
	EXPECT_FALSE(nearZero(-1e10));
	EXPECT_FALSE(nearZero(-1e-2, 0.0));
	EXPECT_TRUE(nearZero(-1e-2, 1e-1));
}

TEST (wrapTo2PI, positiveInput) {
	EXPECT_TRUE(equalQ(wrapTo2PI(0), 0, 0) || equalQ(wrapTo2PI(0), 2 * M_PI));
	EXPECT_TRUE(equalQ(wrapTo2PI(M_PI), M_PI));
	EXPECT_TRUE(equalQ(wrapTo2PI(2 * M_PI), 0.0) || equalQ(wrapTo2PI(2 * M_PI), 2 * M_PI));
	EXPECT_TRUE(equalQ(wrapTo2PI(3 * M_PI), M_PI));
	EXPECT_TRUE(equalQ(wrapTo2PI(4 * M_PI), 0.0) || equalQ(wrapTo2PI(4 * M_PI), 2 * M_PI));
	EXPECT_TRUE(equalQ(wrapTo2PI(9 * M_PI), M_PI));
	EXPECT_TRUE(equalQ(wrapTo2PI(M_PI/3), M_PI/3));
	EXPECT_TRUE(equalQ(wrapTo2PI(13 * M_PI/3.), M_PI/3.));
}

TEST (wrapTo2PI, negativeInput) {
	EXPECT_TRUE(equalQ(wrapTo2PI(-0), 0, 0) || equalQ(wrapTo2PI(-0), 2 * M_PI));
	EXPECT_TRUE(equalQ(wrapTo2PI(-M_PI), M_PI));
	EXPECT_TRUE(equalQ(wrapTo2PI(-2 * M_PI), 0.0) || equalQ(wrapTo2PI(-2 * M_PI), 2 * M_PI));
	EXPECT_TRUE(equalQ(wrapTo2PI(-3 * M_PI), M_PI));
	EXPECT_TRUE(equalQ(wrapTo2PI(-4 * M_PI), 0.0) || equalQ(wrapTo2PI(-4 * M_PI), 2 * M_PI));
	EXPECT_TRUE(equalQ(wrapTo2PI(-9 * M_PI), M_PI));
	EXPECT_TRUE(equalQ(wrapTo2PI(-M_PI/3), 5 * M_PI/3));
	EXPECT_TRUE(equalQ(wrapTo2PI(-13 * M_PI/3.), 5 * M_PI/3.));
}

TEST (wrapToPI, positiveInput) {
	EXPECT_TRUE(equalQ(wrapToPI(0), 0, 0));
	EXPECT_TRUE(equalQ(wrapToPI(M_PI), -M_PI) || equalQ(wrapToPI(M_PI), M_PI));
	EXPECT_TRUE(equalQ(wrapToPI(2 * M_PI), 0.0));
	EXPECT_TRUE(equalQ(wrapToPI(3 * M_PI), -M_PI) || equalQ(wrapToPI(3 * M_PI), M_PI));
	EXPECT_TRUE(equalQ(wrapToPI(4 * M_PI), 0.0));
	EXPECT_TRUE(equalQ(wrapToPI(9 * M_PI), -M_PI) || equalQ(wrapToPI(9 * M_PI), M_PI));
	EXPECT_TRUE(equalQ(wrapToPI(M_PI/3), M_PI/3));
	EXPECT_TRUE(equalQ(wrapToPI(13 * M_PI/3.), M_PI/3.));
}

TEST (wrapToPI, negativeInput) {
	EXPECT_TRUE(equalQ(wrapToPI(-0), 0, 0));
	EXPECT_TRUE(equalQ(wrapToPI(-M_PI), -M_PI) || equalQ(wrapToPI(-M_PI), M_PI));
	EXPECT_TRUE(equalQ(wrapToPI(-2 * M_PI), 0.0));
	EXPECT_TRUE(equalQ(wrapToPI(-3 * M_PI), -M_PI) || equalQ(wrapToPI(-3 * M_PI), M_PI));
	EXPECT_TRUE(equalQ(wrapToPI(-4 * M_PI), 0.0));
	EXPECT_TRUE(equalQ(wrapToPI(-9 * M_PI), -M_PI) || equalQ(wrapToPI(-9 * M_PI), M_PI));
	EXPECT_TRUE(equalQ(wrapToPI(-M_PI/3), -M_PI/3));
	EXPECT_TRUE(equalQ(wrapToPI(-13 * M_PI/3.), -M_PI/3.));
}

TEST (isSO3, all) {
	EXPECT_TRUE(isSO3(SO3Mat::Identity()));
	SO3Mat mat;
	double tht = M_PI/3;
	mat << cos(tht), -sin(tht), 0, sin(tht), cos(tht), 0, 0, 0, 1;
	EXPECT_TRUE(isSO3(mat));
	mat << cos(tht), sin(tht), 0, -sin(tht), cos(tht), 0, 0, 0, 1;
	EXPECT_TRUE(isSO3(mat));
	mat << cos(tht), sin(tht), 0, sin(tht), cos(tht), 0, 0, 0, 1;
	EXPECT_FALSE(isSO3(mat));
	mat << 0.18478599181899182, -0.61237243569579452, 0.76867036968226601, 0.93366299471070121, 0.35355339059327376, 0.057213742298901648, -0.30680213417660024, 0.70710678118654752, 0.63708119613176732;
	EXPECT_TRUE(isSO3(mat));
	EXPECT_FALSE(isSO3(SO3Mat::Zero()));
}

TEST (isso3, all) {
	EXPECT_FALSE(isso3(so3Mat::Identity()));
	so3Mat mat;
	double x1 = rand();
	double x2 = rand();
	double x3 = rand();
	mat << 0, -x3, x2, x3, 0, -x1, -x2, x1, 0;
	EXPECT_TRUE(isso3(mat));
	EXPECT_TRUE(isso3(so3Mat::Zero()));
}

TEST (isSE3, all) {
	EXPECT_TRUE(isSE3(SE3Mat::Identity()));
	double x1 = rand();
	double x2 = rand();
	double x3 = rand();
	SO3Mat matR;
	SE3Mat matT;
	matT.bottomRows<1>() = Vec4{0, 0, 0, 1};
	double tht = M_PI/3;
	matR << cos(tht), -sin(tht), 0, sin(tht), cos(tht), 0, 0, 0, 1;
	matT.topLeftCorner<3,3>() = matR;
	matT.rightCols<1>() = Vec4{x1, x2, x3, 1};
	EXPECT_TRUE(isSE3(matT));

	matR << cos(tht), sin(tht), 0, -sin(tht), cos(tht), 0, 0, 0, 1;
	matT.topLeftCorner<3,3>() = matR;
	EXPECT_TRUE(isSE3(matT));

	matR << cos(tht), sin(tht), 0, sin(tht), cos(tht), 0, 0, 0, 1;
	matT.topLeftCorner<3,3>() = matR;
	EXPECT_FALSE(isSE3(matT));

	matR << 0.18478599181899182, -0.61237243569579452, 0.76867036968226601, 0.93366299471070121, 0.35355339059327376, 0.057213742298901648, -0.30680213417660024, 0.70710678118654752, 0.63708119613176732;
	matT.topLeftCorner<3,3>() = matR;
	EXPECT_TRUE(isSE3(matT));

	matT.rightCols<1>() = Vec4{x1, x2, x3, 2};
	EXPECT_FALSE(isSE3(matT));

	matT.rightCols<1>() = Vec4{x1, x2, x3, 1};
	matT.bottomRows<1>() = Vec4{1, 0, 0, 0};
	EXPECT_FALSE(isSE3(matT));

	matT.bottomRows<1>() = Vec4{0, 0, 0, 0};
	EXPECT_FALSE(isSE3(matT));
}

TEST (isse3, all) {
	EXPECT_FALSE(isse3(se3Mat::Identity()));
	so3Mat mat;
	double x1 = rand();
	double x2 = rand();
	double x3 = rand();
	mat << 0, -x3, x2, x3, 0, -x1, -x2, x1, 0;
	se3Mat matV;
	matV.bottomRows<1>() = Vec4::Zero();
	matV.topLeftCorner<3,3>() = mat;
	auto matV1 = matV;
	EXPECT_TRUE(isse3(matV1));
	EXPECT_TRUE(isse3(se3Mat::Zero()));
	matV.bottomRows<1>() = Vec4{0, 0, 0, 1};
	auto matV2 = matV;
	EXPECT_FALSE(isse3(matV2));
}

TEST (isUnit, Vec2) {
	double tht = ((double) rand() / (RAND_MAX)) + 2 * M_PI;
	EXPECT_TRUE(isUnit(Vec2{cos(tht), sin(tht)}));
	EXPECT_TRUE(isUnit(Vec2{1, 0}));
	EXPECT_TRUE(isUnit(Vec2{0, 1}));
	EXPECT_FALSE(isUnit(Vec2{0, 0}));
	EXPECT_FALSE(isUnit(Vec2{1, 1}));
}

TEST (isUnit, Vec3) {
	double tht = ((double) rand() / (RAND_MAX)) + 2 * M_PI;
	EXPECT_TRUE(isUnit(Vec3{cos(tht), sin(tht), 0}));
	EXPECT_TRUE(isUnit(Vec3{1, 0, 0}));
	EXPECT_TRUE(isUnit(Vec3{0, 1, 0}));
	EXPECT_FALSE(isUnit(Vec3{0, 0, 0}));
	EXPECT_FALSE(isUnit(Vec3{1, 1, 1}));
}

TEST (isUnit, Vec4) {
	double tht = ((double) rand() / (RAND_MAX)) + 2 * M_PI;
	EXPECT_TRUE(isUnit(Vec4{cos(tht), sin(tht), 0, 0}));
	EXPECT_TRUE(isUnit(Vec4{1, 0, 0, 0}));
	EXPECT_TRUE(isUnit(Vec4{0, 1, 0, 0}));
	EXPECT_FALSE(isUnit(Vec4{0, 0, 0, 0}));
	EXPECT_FALSE(isUnit(Vec4{1, 1, 1, 0}));
}

TEST (RotInv, all) {
	EXPECT_TRUE(isSO3(SO3Mat::Identity()));
	SO3Mat mat;
	double tht = M_PI/3;
	mat << cos(tht), -sin(tht), 0, sin(tht), cos(tht), 0, 0, 0, 1;
	EXPECT_EQ(mat.inverse(), RotInv(mat));
	mat << cos(tht), sin(tht), 0, -sin(tht), cos(tht), 0, 0, 0, 1;
	EXPECT_EQ(mat.inverse(), RotInv(mat));
	mat << cos(tht), sin(tht), 0, sin(tht), cos(tht), 0, 0, 0, 1;
	EXPECT_NE(mat.inverse(), RotInv(mat));
	mat << 0.18478599181899182, -0.61237243569579452, 0.76867036968226601, 0.93366299471070121, 0.35355339059327376, 0.057213742298901648, -0.30680213417660024, 0.70710678118654752, 0.63708119613176732;
	EXPECT_TRUE(mat.inverse().isApprox(RotInv(mat)));
}

TEST (VecToso3, all) {
	so3Mat mat;
	double x1, x2, x3;
	for (size_t i = 0; i < 10; ++i) {
		x1 = rand();
		x2 = rand();
		x3 = rand();
		mat << 0, -x3, x2, x3, 0, -x1, -x2, x1, 0;
		EXPECT_EQ(VecToso3(Vec3{x1, x2, x3}), mat);
	}
}

TEST (so3ToVec, all) {
	so3Mat mat;
	double x1, x2, x3;
	for (size_t i = 0; i < 10; ++i) {
		x1 = rand();
		x2 = rand();
		x3 = rand();
		mat << 0, -x3, x2, x3, 0, -x1, -x2, x1, 0;
		EXPECT_EQ(so3ToVec(mat), Vec3({x1, x2, x3}));
	}
}

TEST (AxisAng3, all) {
	AxisAngle aa;
	aa = AxisAng3(Vec3{0, 0, 0});
	EXPECT_TRUE(equalQ(aa.omega.norm(), 1));
	aa = AxisAng3(Vec3{0, 0, 1});
	EXPECT_TRUE(equalQ(aa.omega.norm(), 1));
	EXPECT_TRUE(equalQ(aa.theta, 1));
	double theta = M_PI/3;
	aa = AxisAng3(10 * Vec3{cos(theta), sin(theta), 0});
	EXPECT_TRUE(equalQ(aa.omega.norm(), 1));
	EXPECT_TRUE(equalQ(aa.theta, 10));
}

TEST (MatrixExp3, all) {
	SO3Mat mat;
	double tht = M_PI/3;
	mat << cos(tht), -sin(tht), 0, sin(tht), cos(tht), 0, 0, 0, 1;
	so3Mat mat_so3;
	mat_so3 << 0, -tht, 0, tht, 0, 0, 0, 0, 0;
	EXPECT_TRUE(mat.isApprox(MatrixExp3(mat_so3)));

	mat = SO3Mat::Identity();
	EXPECT_TRUE(mat.isApprox(MatrixExp3(so3Mat::Zero())));
}

TEST (MatrixLog3, all) {
	SO3Mat mat;
	double tht = M_PI/3;
	mat << cos(tht), -sin(tht), 0, sin(tht), cos(tht), 0, 0, 0, 1;
	so3Mat mat_so3;
	mat_so3 << 0, -tht, 0, tht, 0, 0, 0, 0, 0;
	EXPECT_TRUE(mat_so3.isApprox(MatrixLog3(mat)));

	tht = M_PI;
	mat << cos(tht), -sin(tht), 0, sin(tht), cos(tht), 0, 0, 0, 1;
	mat_so3 << 0, -tht, 0, tht, 0, 0, 0, 0, 0;
	EXPECT_TRUE(mat_so3.isApprox(MatrixLog3(mat)));

	tht = M_PI;
	mat << cos(tht), 0, sin(tht), 0, 1, 0, -sin(tht), 0, cos(tht);
	mat_so3 << 0, 0, tht, 0, 0, 0, -tht, 0, 0;

	EXPECT_TRUE(mat_so3.isApprox(MatrixLog3(mat)));
	mat = SO3Mat::Identity();
	EXPECT_TRUE(so3Mat::Zero().isApprox(MatrixLog3(mat)));
}

TEST (RpToTrans, all) {
	SO3Mat mat;
	double tht = M_PI/3;
	mat << cos(tht), -sin(tht), 0, sin(tht), cos(tht), 0, 0, 0, 1;
	Vec3 p(10.0, 12.0, -10.0);
	SE3Mat T = RpToTrans(mat, p);
	EXPECT_TRUE(isSE3(T));
	SE3Mat Tp;
	Tp.topLeftCorner<3,3>() = mat;
	Tp.block<3,1>(0,3) = p;
	Tp.bottomRows<1>() = Vec4({0, 0, 0, 1});
	EXPECT_TRUE(Tp.isApprox(T));
}

TEST (TransToRp, all) {
	SO3Mat mat, matp;
	double tht = M_PI/3;
	mat << cos(tht), -sin(tht), 0, sin(tht), cos(tht), 0, 0, 0, 1;
	Vec3 p(10.0, 12.0, -10.0);
	Vec3 pp;
	SE3Mat Tp;
	Tp.topLeftCorner<3,3>() = mat;
	Tp.block<3,1>(0,3) = p;
	Tp.bottomRows<1>() = Vec4({0, 0, 0, 1});
	TransToRp(Tp, matp, pp);
	EXPECT_TRUE(mat.isApprox(matp));
	EXPECT_TRUE(pp.isApprox(p));
}

TEST (TransInv, all) {
	SO3Mat mat, matp;
	double tht = M_PI/3;
	mat << cos(tht), -sin(tht), 0, sin(tht), cos(tht), 0, 0, 0, 1;
	Vec3 p(10.0, 12.0, -10.0);
	Vec3 pp;
	SE3Mat Tp;
	Tp.topLeftCorner<3,3>() = mat;
	Tp.block<3,1>(0,3) = p;
	Tp.bottomRows<1>() = Vec4({0, 0, 0, 1});
	EXPECT_TRUE(Tp.inverse().isApprox(TransInv(Tp)));
	EXPECT_TRUE(SE3Mat::Identity().isApprox(TransInv(SE3Mat::Identity())));
}

TEST (VecTose3, all) {
	SO3Mat mat;
	double tht = M_PI/3;
	so3Mat mat_so3;
	mat_so3 << 0, -tht, 0, tht, 0, 0, 0, 0, 0;
	Vec3 vb(10.0, 12.0, -10.0);
	Vec3 wb(0, 0, tht);
	se3Mat Tdot_check;
	Tdot_check = se3Mat::Zero();
	Tdot_check.topLeftCorner(3, 3) = mat_so3;
	Tdot_check.block<3,1>(0, 3) = vb;
	Twist V;
	V.head(3) = wb;
	V.tail(3) = vb;
	EXPECT_TRUE(Tdot_check.isApprox(VecTose3(V)));
}

TEST (se3ToVec, all) {
	SO3Mat mat;
	double tht = M_PI/3;
	so3Mat mat_so3;
	mat_so3 << 0, -tht, 0, tht, 0, 0, 0, 0, 0;
	Vec3 vb(10.0, 12.0, -10.0);
	Vec3 wb(0, 0, tht);
	se3Mat Tdot_check;
	Tdot_check = se3Mat::Zero();
	Tdot_check.topLeftCorner(3, 3) = mat_so3;
	Tdot_check.block<3,1>(0, 3) = vb;
	Twist V;
	V.head(3) = wb;
	V.tail(3) = vb;
	EXPECT_TRUE(V.isApprox(se3ToVec(Tdot_check)));
}

TEST(Adjoint, all) {
	double tht = M_PI/3;
	SO3Mat R;
	R << cos(tht), -sin(tht), 0, sin(tht), cos(tht), 0, 0, 0, 1;
	Vec3 p(1.5, 2.5, 3.5);
	SE3Mat T;
	T.topLeftCorner<3,3>() = R;
	T.block<3,1>(0,3) = p;
	T.bottomRows<1>() = Vec4({0, 0, 0, 1});
	AdjMat AdT;
	so3Mat mat_so3;
	mat_so3 << 0, -3.5, 2.5, 3.5, 0, -1.5, -2.5, 1.5, 0;
	AdT.block<3,3>(0,0) = R;
	AdT.block<3,3>(0,3) = so3Mat::Zero();
	AdT.block<3,3>(3,0) = mat_so3 * R;
	AdT.block<3,3>(3,3) = R;
	EXPECT_TRUE(AdT.isApprox(Adjoint(T)));
}

TEST (ScrewToAxis, all) {
	ScrewAxis S;
	S << 0, 0, 1, 0, -1, 2;
	Vec3 q(1.0, 0.0, 0.0);
	Vec3 s(0, 0, 1);
	double h = 2.0;
	EXPECT_TRUE(S.isApprox(ScrewToAxis(q, s, h)));
}

TEST (AxisAng, all) {
	Twist STheta; ScrewAxis S; double thetaDot;
	STheta << 0, 0, 0, 0, 0, 0;
	AxisAng(STheta, S, thetaDot);
	EXPECT_TRUE(STheta == S);

	STheta << 0, 0, 0, 3, 4, 0;
	AxisAng(STheta, S, thetaDot);
	EXPECT_TRUE(S.isApprox(STheta/5.));
	EXPECT_TRUE(nearZero(thetaDot - 5));

	STheta << 3, 4, 0, 5, 10, 15;
	AxisAng(STheta, S, thetaDot);
	EXPECT_TRUE(S.isApprox(STheta/5.));
	EXPECT_TRUE(nearZero(thetaDot - 5));
}

TEST (MatrixExp6, all) {

	se3Mat mat = se3Mat::Zero();
	SE3Mat T = SE3Mat::Identity();
	EXPECT_TRUE(T.isApprox(MatrixExp6(mat)));
	mat << 0,      0,       0,      0,
			0,      0, -M_PI/2., 3 * M_PI/4.,
			0, M_PI/2.,       0, 3 * M_PI/4.,
			0,      0,       0,      0;
	T << 1, 0, 0, 0,
		0, 0, -1, 0,
		0, 1, 0, 3,
		0, 0, 0, 1;
	EXPECT_TRUE(T.isApprox(MatrixExp6(mat)));
}

TEST (MatrixLog6, all) {

	se3Mat mat = se3Mat::Zero();
	SE3Mat T = SE3Mat::Identity();
	EXPECT_TRUE(mat.isApprox(MatrixLog6(T)));
	mat << 0,      0,       0,      0,
			0,      0, -M_PI/2., 3 * M_PI/4.,
			0, M_PI/2.,       0, 3 * M_PI/4.,
			0,      0,       0,      0;
	T << 1, 0, 0, 0,
		0, 0, -1, 0,
		0, 1, 0, 3,
		0, 0, 0, 1;
	EXPECT_TRUE(mat.isApprox(MatrixLog6(T)));
}

TEST(FKinBody, all){
	std::vector <ScrewAxis> Blist;
	std::vector <double> thetaList;
	SE3Mat M;
	M << -1, 0, 0, 0,
		0, 1, 0, 6,
		0, 0, -1, 2,
		0, 0, 0, 1;

	ScrewAxis S;
	S << 0,0,-1,2,0,0;
	Blist.push_back(S);
	S << 0,0,0,0,1,0;
	Blist.push_back(S);
	S << 0,0,1,0,0,1.0/M_PI;
	Blist.push_back(S);

	thetaList.push_back(M_PI/2.);
	thetaList.push_back(2.);
	thetaList.push_back(M_PI);

	SE3Mat T_endEffector;
	T_endEffector << 0, 1, 0, -4,
		1, 0, 0, 4,
		0, 0, -1, 1,
		0, 0, 0, 1;
	SE3Mat Tp = FKinBody(M, Blist, thetaList);

	EXPECT_TRUE(T_endEffector.isApprox(Tp));
}

TEST(FKinSpace, all){
	std::vector <ScrewAxis> Slist;
	std::vector <double> thetaList;
	SE3Mat M;
	M << -1, 0, 0, 0,
		0, 1, 0, 6,
		0, 0, -1, 2,
		0, 0, 0, 1;

	ScrewAxis S;
	S << 0,0,1,4,0,0;
	Slist.push_back(S);
	S << 0,0,0,0,1,0;
	Slist.push_back(S);
	S << 0,0,-1,-6,0,1.0/M_PI;
	Slist.push_back(S);

	thetaList.push_back(M_PI/2.);
	thetaList.push_back(2.);
	thetaList.push_back(M_PI);

	SE3Mat T_endEffector;
	T_endEffector << 0, 1, 0, -4,
		1, 0, 0, 4,
		0, 0, -1, 3,
		0, 0, 0, 1;
	SE3Mat Tp = FKinSpace(M, Slist, thetaList);

	EXPECT_TRUE(T_endEffector.isApprox(Tp));
}

TEST (JacobianBody, all)
{
	double q[] = {M_PI/13.0, M_PI/2.0, M_PI/7.0, 5.0};
	double l1 = 3.0;
	double l2 = 4.0;
	std::vector<ScrewAxis> Slist;
	std::vector<double> thetaList;
	ScrewAxis S;
	S << 0, 0, 1, 0, -l1-l2, 0;
	Slist.push_back(S);
	thetaList.push_back(q[0]);

	S << 0, 0, 1, 0, -l2, 0;
	Slist.push_back(S);
	thetaList.push_back(q[1]);

	S << 0, 0, 1, 0, 0, 0;
	Slist.push_back(S);
	thetaList.push_back(q[2]);

	S << 0, 0, 0, 0, 0, 1;
	Slist.push_back(S);
	thetaList.push_back(q[3]);

	auto Jb = JacobianBody(Slist, thetaList);

	JacobianMat Jbp(6, 4);
	Jbp <<  0, 0, 0, 0,
			0, 0, 0, 0,
			1, 1, 1, 0,
			-l2*sin(q[2])-l1*sin(q[2]+q[1]), -l2*sin(q[2]), 0, 0,
			-l2*cos(q[2])-l1*cos(q[2]+q[1]), -l2*cos(q[2]), 0, 0,
			0, 0, 0, 1;
	std::cout << Jbp << std::endl;
	std::cout << Jb << std::endl;
	EXPECT_TRUE(Jbp.isApprox(Jb));
}

TEST (JacobianSpace, all)
{
	double q[] = {M_PI/4.0, M_PI/6.0, M_PI/3.0, 3.0};
	double l1 = 3.0;
	double l2 = 4.0;
	std::vector<ScrewAxis> Slist;
	std::vector<double> thetaList;
	ScrewAxis S;
	S << 0, 0, 1, 0, 0, 0;
	Slist.push_back(S);
	thetaList.push_back(q[0]);

	S << 0, 0, 1, 0, -l1, 0;
	Slist.push_back(S);
	thetaList.push_back(q[1]);

	S << 0, 0, 1, 0, -l1-l2, 0;
	Slist.push_back(S);
	thetaList.push_back(q[2]);

	S << 0, 0, 0, 0, 0, 1;
	Slist.push_back(S);
	thetaList.push_back(q[3]);

	auto Js = JacobianSpace(Slist, thetaList);

	JacobianMat Jsp(6, 4);
	Jsp <<  0, 0, 0, 0,
			0, 0, 0, 0,
			1, 1, 1, 0,
			0, l1*sin(q[0]), l1*sin(q[0])+l2*sin(q[0]+q[1]), 0,
			0, -l1*cos(q[0]), -l1*cos(q[0])-l2*cos(q[0]+q[1]), 0,
			0, 0, 0, 1;
	//std::cout << Jsp << std::endl;
	//	std::cout << Js<< std::endl;
	EXPECT_TRUE(Jsp.isApprox(Js));
}

int main(int argc, char *argv[])
{
	testing::InitGoogleTest(&argc, argv);
	return RUN_ALL_TESTS();
}
