#include "KinForward.h"

void Kinematic::setMDH(const std::vector<Kinematic::MDH>& mdhs)
{
	_mdhs = mdhs;
}

std::vector<Eigen::Matrix4d> Kinematic::forward(const std::vector<double>& joints) const
{
	if (joints.size() < _mdhs.size())
		return {};

	std::vector<Eigen::Matrix4d> mats(7);
	Eigen::Matrix4d transMat = Eigen::Matrix4d::Identity();
	if (joints.size() == 9)
	{
		transMat(0, 3) = joints[6];
		transMat(1, 3) = joints[7];
		transMat(2, 3) = joints[8];
	}
	mats[0] = transMat;

	Eigen::Matrix4d lastLinkMat = mats[0];
	for (size_t i = 0; i < _mdhs.size(); i++)
	{
		mats[i + 1] = lastLinkMat * this->makeMDHMatrix(_mdhs[i], joints[i]);
		lastLinkMat = mats[i + 1];
	}
	return mats;
}

Eigen::Matrix4d Kinematic::makeMDHMatrix(const Kinematic::MDH& dh, double config) const
{
	Eigen::Matrix4d matAlpha = Eigen::Matrix4d::Identity();
	matAlpha(1, 1) = cos(dh.alpha);
	matAlpha(1, 2) = -sin(dh.alpha);
	matAlpha(2, 1) = sin(dh.alpha);
	matAlpha(2, 2) = cos(dh.alpha);
	Eigen::Matrix4d matA = Eigen::Matrix4d::Identity();
	matA(0, 3) = dh.a;
	Eigen::Matrix4d matTheta = Eigen::Matrix4d::Identity();
	matTheta(0, 0) = cos(dh.startTheta + config);
	matTheta(0, 1) = -sin(dh.startTheta + config);
	matTheta(1, 0) = sin(dh.startTheta + config);
	matTheta(1, 1) = cos(dh.startTheta + config);
	Eigen::Matrix4d matD = Eigen::Matrix4d::Identity();
	matD(2, 3) = dh.d;

	return matAlpha * matA * matTheta * matD;
}
