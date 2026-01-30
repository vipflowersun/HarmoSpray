#pragma once
#include "JMath.hpp"

class Kinematic
{
public:
	struct MDH
	{
		double startTheta = 0; // rad
		double alpha = 0;
		double a = 0;
		double d = 0;
		bool isTranslate = false;
	};
public:
	explicit Kinematic() = default;
	~Kinematic() = default;
	void setMDH(const std::vector<Kinematic::MDH>& mdhs);
	std::vector<Eigen::Matrix4d> forward(const std::vector<double>& joints) const;
private:
	Eigen::Matrix4d makeMDHMatrix(const Kinematic::MDH& dh, double config) const;
private:
	std::vector<Kinematic::MDH> _mdhs;
};