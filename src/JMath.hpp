#pragma once

#include "Eigen/Dense"
#include <Eigen/SVD>
#include <algorithm>
#include <list>
#include <memory>

namespace JMath
{
	constexpr double TOLERANCE = (1E-6);

	template <typename T>
	bool isRotationMatrix(const Eigen::Matrix3<T> &R)
	{
		Eigen::Matrix3<T> Rt = R.transpose();
		Eigen::Matrix3<T> shouldBeIdentity = Rt * R;
		Eigen::Matrix3<T> I = Eigen::Matrix3<T>::Identity();
		return (shouldBeIdentity - I).norm() < TOLERANCE && std::abs(R.determinant() - static_cast<T>(1)) < TOLERANCE;
	}

	/**
	 * @brief 对旋转矩阵进行数值修正
	 */
	template <typename T>
	inline Eigen::Matrix3<T> sanitizeRotationMatrix(const Eigen::Matrix3<T> &R, T eps = static_cast<T>(1e-6))
	{
		Eigen::Matrix3<T> M = R;

		// 1) clamp 元素到 [-1,1]（仅在非常接近边界时修正）
		for (int i = 0; i < 3; ++i)
		{
			for (int j = 0; j < 3; ++j)
			{
				if (M(i, j) > T(1) && (M(i, j) - T(1)) < eps)
					M(i, j) = T(1);
				else if (M(i, j) < T(-1) && (T(-1) - M(i, j)) < eps)
					M(i, j) = T(-1);
			}
		}

		// 2) 快速检测矩阵是否已经足够正交：
		//    计算 C = M^T * M，应为单位矩阵。若偏差和行列式偏离很小，则跳过昂贵的 SVD。
		const Eigen::Matrix3<T> I = Eigen::Matrix3<T>::Identity();
		Eigen::Matrix3<T> C = M.transpose() * M;
		T orthoError = (C - I).norm();
		T detDiff = std::abs(M.determinant() - T(1));

		// 容许的误差阈值（可根据应用场景调整；float 可设置大一点）
		T orthoTol = eps * static_cast<T>(10);
		T detTol = eps * static_cast<T>(10);

		if (orthoError <= orthoTol && detDiff <= detTol)
		{
			// 已经足够接近正交且 det 接近 +1，直接返回（避免 SVD 开销）
			return M;
		}

		// 3) 使用 SVD 做极分解得到最近的正交矩阵（仅在需要时执行）
		Eigen::JacobiSVD<Eigen::Matrix<T, 3, 3>> svd(M, Eigen::ComputeFullU | Eigen::ComputeFullV);
		Eigen::Matrix3<T> U = svd.matrixU();
		Eigen::Matrix3<T> V = svd.matrixV();
		Eigen::Matrix3<T> Rn = U * V.transpose();

		// 保证为右手坐标（det = +1）
		if (Rn.determinant() < T(0))
		{
			U.col(2) *= T(-1);
			Rn = U * V.transpose();
		}

		return Rn;
	}

	template <typename T>
	inline Eigen::Matrix4<T> sanitizeRotationMatrix(const Eigen::Matrix4<T> &R, T eps = static_cast<T>(1e-6))
	{
		Eigen::Matrix3<T> R3 = R.template block<3, 3>(0, 0);
		Eigen::Matrix3<T> R3n = sanitizeRotationMatrix(R3, eps);

		Eigen::Matrix4<T> Rn = R;
		Rn.template block<3, 3>(0, 0) = R3n;

		return Rn;
	}

	/**
	 * @brief 角度规范到[0,2pi]
	 * @param angle 角度
	 * @return 规范角度
	 */
	template <typename T>
	inline T angleMod2Pi(T angle)
	{
		T modAngle = std::fmod(angle, 2.0 * M_PI);

		// 如果结果为负数，加 2π 使其回到 [0, 2π)
		if (modAngle < 0)
		{
			modAngle += 2.0 * M_PI;
		}

		return modAngle;
	}

	/**
	 * @brief 角度规范到[-pi,pi]
	 * @param angle 角度
	 * @return 规范角度
	 */
	template <typename T>
	inline T angleModPi(T angle)
	{
		T modAngle = angleMod2Pi(angle);

		// 将结果调整到 [-π, π)
		if (modAngle >= M_PI)
		{
			modAngle -= M_PI * 2;
		}
		else if (modAngle < -M_PI)
		{
			modAngle += M_PI * 2;
		}

		return modAngle;
	}

	/**
	 * @brief 计算angleStart到angleStart的转动角(转动轴为Z+)
	 * @param vecStart 起始角
	 * @param vecFinal 终止角
	 * @return 转动角
	 * @note 转动角范围[-pi,pi) 正值为逆时针转动，负值为顺时针转动
	 */
	template <typename T>
	inline T angleTo(T angleStart, T angleFinal)
	{
		Eigen::Vector2<T> vecStart(cos(angleStart), sin(angleStart));
		Eigen::Vector2<T> vecFinal(cos(angleFinal), sin(angleFinal));

		// 计算点积和叉积
		T dot = vecStart.dot(vecFinal);
		T det = vecStart.x() * vecFinal.y() - vecStart.y() * vecFinal.x(); // 相当于二维向量的叉积

		// 计算角度 (atan2)
		return std::atan2(det, dot);
	}

	/**
	 * @brief 计算angleStart到angleStart的转动角(转动轴为Z+)
	 * @param vecStart 起始角
	 * @param vecFinal 终止角
	 * @param counterClockwise 逆时针方向
	 * @return 转动角
	 * @note 转动角范围[0,2pi)
	 */
	template <typename T>
	inline T angleTo(T angleStart, T angleFinal, bool counterClockwise)
	{
		const T TwoPi = static_cast<T>(2.0 * M_PI);
		T d = angleMod2Pi(angleFinal - angleStart); // [0, 2π)

		if (counterClockwise)
			return d;

		// 顺时针：如果刚好重合，返回 0，否则取 2π - d
		return (d == static_cast<T>(0)) ? static_cast<T>(0) : (TwoPi - d);
	}

	/**
	 * @brief 计算angle1到angle2的夹角
	 * @param angle1 角1
	 * @param angle2 角2
	 * @return 夹角
	 */
	template <typename T>
	inline T angleBetween(T angle1, T angle2)
	{
		angle1 = angleMod2Pi(angle1);
		angle2 = angleMod2Pi(angle2);

		T delta = std::abs(angle1 - angle2);

		return std::min(delta, 2 * M_PI - delta);
	}

	/**
	 * @brief 由rpy计算旋转矩阵(固定轴XYZ)
	 * @param roll 横滚角
	 * @param pitch 俯仰角
	 * @param yaw 偏航角
	 * @return 旋转矩阵
	 */
	template <typename T>
	inline Eigen::Matrix3<T> rpyToRotationMatrix(T roll, T pitch, T yaw)
	{
		// 创建绕 Z 轴旋转的矩阵 (yaw)
		Eigen::Matrix<T, 3, 3> yawMatrix = Eigen::AngleAxis<T>(yaw, Eigen::Vector3<T>::UnitZ()).toRotationMatrix();

		// 创建绕 Y 轴旋转的矩阵 (pitch)
		Eigen::Matrix<T, 3, 3> pitchMatrix = Eigen::AngleAxis<T>(pitch, Eigen::Vector3<T>::UnitY()).toRotationMatrix();

		// 创建绕 X 轴旋转的矩阵 (roll)
		Eigen::Matrix<T, 3, 3> rollMatrix = Eigen::AngleAxis<T>(roll, Eigen::Vector3<T>::UnitX()).toRotationMatrix();

		// 旋转矩阵为 yaw * pitch * roll
		Eigen::Matrix<T, 3, 3> rotationMatrix = yawMatrix * pitchMatrix * rollMatrix;

		return rotationMatrix;
	}

	/**
	 * @brief 由旋转矩阵计算rpy(固定轴XYZ)
	 * @param rotationMatrix 旋转矩阵
	 * @return rpy
	 */
	template <typename T>
	inline Eigen::Vector3<T> rotationMatrixToRPY(const Eigen::Matrix3<T> &rotationMatrix)
	{
		if (!isRotationMatrix<T>(rotationMatrix))
		{
			throw std::invalid_argument("Input matrix is not a valid rotation matrix.");
		}

		// 为避免 asin/acos 越界，先做数值修正（clamp + 极分解）
		Eigen::Matrix3<T> R = sanitizeRotationMatrix<T>(rotationMatrix);

		double sy = sqrt(R(0, 0) * R(0, 0) + R(1, 0) * R(1, 0));
		bool singular = sy < 1e-6;
		Eigen::Vector3<T> rpy = Eigen::Vector3<T>::Zero();
		if (!singular)
		{
			rpy(0) = atan2(R(2, 1), R(2, 2));
			rpy(1) = atan2(-R(2, 0), sy);
			rpy(2) = atan2(R(1, 0), R(0, 0));
		}
		else
		{
			rpy(0) = atan2(-R(1, 2), R(1, 1));
			rpy(1) = atan2(-R(2, 0), sy);
			rpy(2) = 0;
		}
		return rpy;
	}

	/**
	 * @brief 由旋转矩阵计算euler(浮动轴XYZ)
	 * @param rotationMatrix 旋转矩阵
	 * @return rpy
	 */
	template <typename T>
	inline Eigen::Vector3<T> rotationMatrixToEuler(const Eigen::Matrix3<T> &rotationMatrix)
	{
		if (!isRotationMatrix<>(rotationMatrix))
		{
			throw std::invalid_argument("Input matrix is not a valid rotation matrix.");
		}

		// 为避免 asin/acos 越界，先做数值修正（clamp + 极分解）
		Eigen::Matrix3<T> R = sanitizeRotationMatrix<T>(rotationMatrix);

		double sy = sqrt(R(0, 0) * R(0, 0) + R(0, 1) * R(0, 1)); // -c2 *****
		bool singular = sy < 1e-6;
		Eigen::Vector3<T> euler = Eigen::Vector3<T>::Zero();
		if (!singular)
		{
			euler(0) = atan2(-R(1, 2), R(2, 2));
			euler(1) = atan2(R(0, 2), sy);
			euler(2) = atan2(-R(0, 1), R(0, 0));
		}

		else
		{
			euler(0) = atan2(R(2, 1), R(1, 1));
			euler(1) = atan2(R(0, 2), sy);
			euler(2) = 0;
		}

		return euler;
	}

	/**
	 * @brief 由euler计算旋转矩阵(浮动轴rpy)
	 * @param roll 横滚角
	 * @param pitch 俯仰角
	 * @param yaw 偏航角
	 * @return 旋转矩阵
	 */
	template <typename T>
	inline Eigen::Matrix3<T> eulerToRotationMatrix(T roll, T pitch, T yaw)
	{
		// 创建绕 Z 轴旋转的矩阵 (yaw)
		Eigen::Matrix<T, 3, 3> yawMatrix = Eigen::AngleAxis<T>(yaw, Eigen::Vector3<T>::UnitZ()).toRotationMatrix();

		// 创建绕 Y 轴旋转的矩阵 (pitch)
		Eigen::Matrix<T, 3, 3> pitchMatrix = Eigen::AngleAxis<T>(pitch, Eigen::Vector3<T>::UnitY()).toRotationMatrix();

		// 创建绕 X 轴旋转的矩阵 (roll)
		Eigen::Matrix<T, 3, 3> rollMatrix = Eigen::AngleAxis<T>(roll, Eigen::Vector3<T>::UnitX()).toRotationMatrix();

		// 旋转矩阵为 roll * pitch * yaw
		Eigen::Matrix<T, 3, 3> rotationMatrix = rollMatrix * pitchMatrix * yawMatrix;

		return rotationMatrix;
	}

	/**
	 * @brief 由变换矩阵计算xyzrpy(固定轴rpy)
	 * @param transformMatrix 变换矩阵
	 * @return xyzrpy
	 */
	template <typename T>
	inline std::array<T, 6> transformMatrixToXyzrpy(const Eigen::Matrix4<T> &transformMatrix)
	{
		Eigen::Vector3<T> rpy = JMath::rotationMatrixToRPY<T>(transformMatrix.template block<3, 3>(0, 0));
		std::array<T, 6> xyzrpy;
		xyzrpy[0] = transformMatrix(0, 3);
		xyzrpy[1] = transformMatrix(1, 3);
		xyzrpy[2] = transformMatrix(2, 3);
		xyzrpy[3] = rpy[0];
		xyzrpy[4] = rpy[1];
		xyzrpy[5] = rpy[2];
		return xyzrpy;
	}

	/**
	 * @brief 由xyzrpy计算变换矩阵(固定轴rpy)
	 * @param xyzrpy
	 * @return 变换矩阵
	 */
	template <typename T>
	inline Eigen::Matrix4<T> xyzrpyToTransformMatrix(const std::array<T, 6> &xyzrpy)
	{
		Eigen::Matrix4<T> transformMatrix = Eigen::Matrix4<T>::Identity();
		transformMatrix(0, 3) = xyzrpy[0];
		transformMatrix(1, 3) = xyzrpy[1];
		transformMatrix(2, 3) = xyzrpy[2];
		transformMatrix.template block<3, 3>(0, 0) = JMath::rpyToRotationMatrix<T>(xyzrpy[3], xyzrpy[4], xyzrpy[5]);
		return transformMatrix;
	}

	/**
	 * @brief 规范化rpy角度到标准范围(pitch在[-pi/2,pi/2])
	 * @param roll 横滚角
	 * @param pitch 俯仰角
	 * @param yaw 偏航角
	 * @return 规范rpy数组
	 */
	template <typename T>
	inline std::array<T, 3> normalizeRPY(T roll, T pitch, T yaw)
	{
		Eigen::Matrix3<T> R = JMath::rpyToRotationMatrix(roll, pitch, yaw);
		Eigen::Vector3<T> rpy = JMath::rotationMatrixToRPY(R);
		return {rpy[0], rpy[1], rpy[2]};
	}

	/**
	 * @brief asinx+bcosx=c,解x
	 * @param a,b,c
	 * @return x解集
	 */
	template <typename T>
	inline std::vector<T> asinxAddbcosx(T a, T b, T c)
	{
		if (fabs(a) < TOLERANCE)
			a = 0;
		if (fabs(b) < TOLERANCE)
			b = 0;
		if (fabs(c) < TOLERANCE)
			c = 0;
		std::vector<T> x;
		T delta = c / sqrt(a * a + b * b);

		if (fabs(delta) > 1)
			return x;
		T phi = 0;
		if (a == 0) // if (a==0)
		{
			T x1;
			if (1 < c / b && c / b < 1.001)
				x1 = acos(1.00000);
			else if (-1.001 < c / b && c / b < -1)
				x1 = acos(-1.0000);
			else
			{
				x1 = acos(c / b);
			}
			T x2 = -x1;
			if (x1 == x2)
			{
				x.push_back(x1);
				return x;
			}
			x.push_back(x1);
			x.push_back(x2);
			return x;
		}
		else
		{
			phi = acos(a / sqrt(a * a + b * b));
			if (sin(phi) != b / sqrt(a * a + b * b))
			{
				phi = -phi;
			}
		}
		T x1 = asin(delta) - phi;
		T x2 = 0;
		if (c >= 0)
		{
			x2 = M_PI - asin(delta) - phi;
		}
		else
		{
			x2 = -M_PI - asin(delta) - phi;
		}
		if (x1 == x2)
		{
			x.push_back(x1);
			return x;
		}
		x.push_back(x1);
		x.push_back(x2);
		return x;
	}

	/**
	 * @brief 计算N的最低的非零位
	 * @param N
	 * @param maxDigit 最大位
	 * @return 最低的非零位
	 * @note 返回-1则不存在, 数位从0开始计数
	 */
	inline int lowestSetDigit(int N, int maxDigit)
	{
		int Ret = 0;
		while (Ret <= maxDigit)
		{
			if ((N & 0x01) == 1)
				return Ret;
			N >>= 1;
			Ret++;
		}
		return -1;
	}

	/**
	 * @brief 弧度转角度
	 * @param rad 弧度
	 * @return 角度
	 */
	inline double R2D(double rad)
	{
		return rad * 180.0 / M_PI;
	}

	/**
	 * @brief 角度转弧度
	 * @param degree 角度
	 * @return 弧度
	 */
	inline double D2R(double degree)
	{
		return degree * M_PI / 180.0;
	}

	/**
	 * @brief mm转inch
	 * @param mm
	 * @return inch
	 */
	inline double mm2Inch(double mm)
	{
		return 0.0393701 * mm;
	}

	/**
	 * @brief 解方程x^2 + ce[0] x + ce[1] =0
	 * @param ce 系数
	 * @return 方程解
	 */
	template <typename T>
	inline std::vector<T> quadraticSolver(const std::vector<T> &ce)
	{
		std::vector<T> ans(0, 0.);
		if (ce.size() != 2)
			return ans;
		using LDouble = long double;
		LDouble const b = -0.5L * ce[0];
		LDouble const c = ce[1];
		// x^2 -2 b x + c=0
		// (x - b)^2 = b^2 - c
		// b^2 >= fabs(c)
		// x = b \pm b sqrt(1. - c/(b^2))
		LDouble const b2 = b * b;
		LDouble const discriminant = b2 - c;
		LDouble const fc = std::abs(c);

		LDouble const TOL = 1e-24L;

		if (discriminant < 0.L)
			// negative discriminant, no real root
			return ans;

		// find the radical
		LDouble r;

		// given |p| >= |q|
		// sqrt(p^2 \pm q^2) = p sqrt(1 \pm q^2/p^2)
		if (b2 >= fc)
			r = std::abs(b) * std::sqrt(1.L - c / b2);
		else
			// c is negative, because b2 - c is non-negative
			r = std::sqrt(fc) * std::sqrt(1.L + b2 / fc);

		if (r >= TOL * std::abs(b))
		{
			// two roots
			if (b >= 0.L)
				// since both (b,r)>=0, avoid (b - r) loss of significance
				ans.push_back(static_cast<T>(b + r));
			else
				// since b<0, r>=0, avoid (b + r) loss of significance
				ans.push_back(static_cast<T>(b - r));

			// Vieta's formulas for the second root
			ans.push_back(static_cast<T>(c / ans.front()));
		}
		else
			// multiple roots
			ans.push_back(static_cast<T>(b));
		return ans;
	}

	/**
	 * @brief 解方程x^3 + ce[0] x^2 + ce[1] x + ce[2] = 0
	 * @param ce 系数
	 * @return 方程解
	 */
	template <typename T>
	inline std::vector<T> cubicSolver(const std::vector<T> &ce)
	{
		std::vector<T> ans(0, 0.);
		if (ce.size() != 3)
			return ans;
		// depressed cubic, Tschirnhaus transformation, x= t - b/(3a)
		// t^3 + p t +q =0
		T shift = (1. / 3) * ce[0];
		T p = ce[1] - shift * ce[0];
		T q = ce[0] * ((2. / 27) * ce[0] * ce[0] - (1. / 3) * ce[1]) + ce[2];
		// Cardano's method,
		//	t=u+v
		//	u^3 + v^3 + ( 3 uv + p ) (u+v) + q =0
		//	select 3uv + p =0, then,
		//	u^3 + v^3 = -q
		//	u^3 v^3 = - p^3/27
		//	so, u^3 and v^3 are roots of equation,
		//	z^2 + q z - p^3/27 = 0
		//	and u^3,v^3 are,
		//		-q/2 \pm sqrt(q^2/4 + p^3/27)
		//	discriminant= q^2/4 + p^3/27
		// std::cout<<"p="<<p<<"\tq="<<q<<std::endl;
		T discriminant = (1. / 27) * p * p * p + (1. / 4) * q * q;
		if (fabs(p) < 1.0e-75)
		{
			ans.push_back((q > 0) ? -pow(q, (1. / 3)) : pow(-q, (1. / 3)));
			ans[0] -= shift;
			//        DEBUG_HEADER
			//        std::cout<<"cubic: one root: "<<ans[0]<<std::endl;
			return ans;
		}
		// std::cout<<"discriminant="<<discriminant<<std::endl;
		if (discriminant > 0)
		{
			std::vector<T> ce2(2, 0.);
			ce2[0] = q;
			ce2[1] = -1. / 27 * p * p * p;
			std::vector<T> r = quadraticSolver(ce2);
			if (r.size() == 0)
			{ // should not happen
			}
			T u, v;
			u = (q <= 0) ? pow(r[0], 1. / 3) : -pow(-r[1], 1. / 3);
			// u=(q<=0)?pow(-0.5*q+sqrt(discriminant),1./3):-pow(0.5*q+sqrt(discriminant),1./3);
			v = (-1. / 3) * p / u;
			// std::cout<<"u="<<u<<"\tv="<<v<<std::endl;
			// std::cout<<"u^3="<<u*u*u<<"\tv^3="<<v*v*v<<std::endl;
			ans.push_back(u + v - shift);

			//        DEBUG_HEADER
			//        std::cout<<"cubic: one root: "<<ans[0]<<std::endl;
		}
		else
		{
			std::complex<T> u(q, 0), rt[3];
			u = std::pow(-0.5 * u - sqrt(0.25 * u * u + p * p * p / 27), 1. / 3);
			rt[0] = u - p / (3. * u) - shift;
			std::complex<T> w(-0.5, sqrt(3.) / 2);
			rt[1] = u * w - p / (3. * u * w) - shift;
			rt[2] = u / w - p * w / (3. * u) - shift;
			//        DEBUG_HEADER
			//        std::cout<<"Roots:\n";
			//        std::cout<<rt[0]<<std::endl;
			//        std::cout<<rt[1]<<std::endl;
			//        std::cout<<rt[2]<<std::endl;
			ans.push_back(rt[0].real());
			ans.push_back(rt[1].real());
			ans.push_back(rt[2].real());
		}
		// newton-raphson
		for (T &x0 : ans)
		{
			T dx = 0.;
			for (size_t i = 0; i < 20; ++i)
			{
				T f = ((x0 + ce[0]) * x0 + ce[1]) * x0 + ce[2];
				T df = (3. * x0 + 2. * ce[0]) * x0 + ce[1];
				if (fabs(df) > fabs(f) + TOLERANCE)
				{
					dx = f / df;
					x0 -= dx;
				}
				else
					break;
			}
		}

		return ans;
	}

	/**
	 * @brief 解方程x^4 + ce[0] x^3 + ce[1] x^2 + ce[2] x + ce[3] = 0
	 * @param ce 系数
	 * @return 方程解
	 */
	template <typename T>
	inline std::vector<T> quarticSolver(const std::vector<T> &ce)
	{
		std::vector<T> ans(0, 0.);
		if (ce.size() != 4)
			return ans;

		T shift = 0.25 * ce[0];
		T shift2 = shift * shift;
		T a2 = ce[0] * ce[0];
		T p = ce[1] - (3. / 8) * a2;
		T q = ce[2] + ce[0] * ((1. / 8) * a2 - 0.5 * ce[1]);
		T r = ce[3] - shift * ce[2] + (ce[1] - 3. * shift2) * shift2;

		if (q * q <= 1.e-4 * TOLERANCE * fabs(p * r))
		{ // Biquadratic equations
			T discriminant = 0.25 * p * p - r;
			if (discriminant < -1.e3 * TOLERANCE)
			{

				//            DEBUG_HEADER
				//            std::cout<<"discriminant="<<discriminant<<"\tno root"<<std::endl;
				return ans;
			}
			T t2[2];
			t2[0] = -0.5 * p - sqrt(fabs(discriminant));
			t2[1] = -p - t2[0];
			//        std::cout<<"t2[0]="<<t2[0]<<std::endl;
			//        std::cout<<"t2[1]="<<t2[1]<<std::endl;
			if (t2[1] >= 0.)
			{ // two real roots
				ans.push_back(sqrt(t2[1]) - shift);
				ans.push_back(-sqrt(t2[1]) - shift);
			}
			if (t2[0] >= 0.)
			{ // four real roots
				ans.push_back(sqrt(t2[0]) - shift);
				ans.push_back(-sqrt(t2[0]) - shift);
			}

			return ans;
		}
		if (fabs(r) < 1.0e-75)
		{
			std::vector<T> cubic(3, 0.);
			cubic[1] = p;
			cubic[2] = q;
			ans.push_back(0.);
			std::vector<T> r1 = cubicSolver(cubic);
			std::copy(r1.begin(), r1.end(), std::back_inserter(ans));
			for (size_t i = 0; i < ans.size(); i++)
				ans[i] -= shift;
			return ans;
		}

		std::vector<T> cubic(3, 0.);
		cubic[0] = 2. * p;
		cubic[1] = p * p - 4. * r;
		cubic[2] = -q * q;
		std::vector<T> r3 = cubicSolver(cubic);

		if (r3.size() == 1)
		{ // one real root from cubic
			if (r3[0] < 0.)
			{ // this should not happen
				return ans;
			}
			T sqrtz0 = sqrt(r3[0]);
			std::vector<T> ce2(2, 0.);
			ce2[0] = -sqrtz0;
			ce2[1] = 0.5 * (p + r3[0]) + 0.5 * q / sqrtz0;
			std::vector<T> r1 = quadraticSolver(ce2);
			if (r1.size() == 0)
			{
				ce2[0] = sqrtz0;
				ce2[1] = 0.5 * (p + r3[0]) - 0.5 * q / sqrtz0;
				r1 = quadraticSolver(ce2);
			}
			for (auto &x : r1)
			{
				x -= shift;
			}
			return r1;
		}
		if (r3[0] > 0. && r3[1] > 0.)
		{
			T sqrtz0 = sqrt(r3[0]);
			std::vector<T> ce2(2, 0.);
			ce2[0] = -sqrtz0;
			ce2[1] = 0.5 * (p + r3[0]) + 0.5 * q / sqrtz0;
			ans = quadraticSolver(ce2);
			ce2[0] = sqrtz0;
			ce2[1] = 0.5 * (p + r3[0]) - 0.5 * q / sqrtz0;
			std::vector<T> r1 = quadraticSolver(ce2);
			std::copy(r1.begin(), r1.end(), std::back_inserter(ans));
			for (auto &x : ans)
			{
				x -= shift;
			}
		}
		// newton-raphson
		for (T &x0 : ans)
		{
			T dx = 0.;
			for (size_t i = 0; i < 20; ++i)
			{
				T f = (((x0 + ce[0]) * x0 + ce[1]) * x0 + ce[2]) * x0 + ce[3];
				T df = ((4. * x0 + 3. * ce[0]) * x0 + 2. * ce[1]) * x0 + ce[2];
				//			DEBUG_HEADER
				//			qDebug()<<"i="<<i<<"\tx0="<<x0<<"\tf="<<f<<"\tdf="<<df;
				if (fabs(df) > 1E-20)
				{
					dx = f / df;
					x0 -= dx;
				}
				else
					break;
			}
		}

		return ans;
	}

	/**
	 * @brief 构造平移矩阵
	 * @param xyz 平移量
	 * @return 平移矩阵
	 */
	template <typename T>
	inline Eigen::Matrix4<T> makeTranslate(const Eigen::Vector3<T> &xyz)
	{
		Eigen::Matrix4<T> mat = Eigen::Matrix4<T>::Identity();
		mat(0, 3) = xyz(0);
		mat(1, 3) = xyz(1);
		mat(2, 3) = xyz(2);

		return mat;
	}

	/**
	 * @brief 构造旋转矩阵
	 * @param axis 旋转轴
	 * @param angle 旋转量
	 * @return 旋转矩阵
	 */
	template <typename T>
	inline Eigen::Matrix4<T> makeRotate(const Eigen::Vector3<T> &axis, T angle)
	{
		// 将旋转轴归一化
		Eigen::Vector3<T> _axis = axis;
		_axis.normalize();

		// 使用 AngleAxis 创建旋转矩阵
		Eigen::AngleAxis<T> rotation(angle, _axis);

		// 转换为旋转矩阵
		Eigen::Matrix3<T> rotationMatrix = rotation.toRotationMatrix();

		Eigen::Matrix4<T> mat = Eigen::Matrix4<T>::Identity();
		mat.template block<3, 3>(0, 0) = rotationMatrix;

		return mat;
	}

	/**
	 * @brief 构造旋转矩阵
	 * @param axis 旋转轴
	 * * @param axisPoint 旋转轴上点
	 * @param angle 旋转量
	 * @return 旋转矩阵
	 */
	template <typename T>
	inline Eigen::Matrix4<T> makeRotate(const Eigen::Vector3<T> &axis, const Eigen::Vector3<T> &axisPoint, T angle)
	{
		// 将旋转轴归一化
		Eigen::Vector3<T> _axis = axis.normalized();

		// 平移到原点的矩阵
		Eigen::Matrix4<T> T_to_origin = Eigen::Matrix4<T>::Identity();
		T_to_origin(0, 3) = -axisPoint[0];
		T_to_origin(1, 3) = -axisPoint[1];
		T_to_origin(2, 3) = -axisPoint[2];

		// 使用 AngleAxis 生成旋转矩阵
		Eigen::AngleAxis<T> rotation(angle, _axis);
		Eigen::Matrix3<T> R = rotation.toRotationMatrix();

		// 嵌入到 4x4 矩阵
		Eigen::Matrix4<T> R_homogeneous = Eigen::Matrix4<T>::Identity();
		R_homogeneous.template block<3, 3>(0, 0) = R;

		// 平移回原位置的矩阵
		Eigen::Matrix4<T> T_back = Eigen::Matrix4<T>::Identity();
		T_back(0, 3) = axisPoint[0];
		T_back(1, 3) = axisPoint[1];
		T_back(2, 3) = axisPoint[2];

		// 最终变换矩阵
		Eigen::Matrix4<T> M = T_back * R_homogeneous * T_to_origin;

		return M;
	}

	/**
	 * @brief 构造定轴旋转矩阵
	 * @param axis 旋转轴
	 * @param axisPt 旋转轴上点
	 * @param angle 旋转量
	 * @return 定轴旋转矩阵
	 */
	template <typename T>
	inline Eigen::Matrix4<T> makeFixedAxisRotate(const Eigen::Vector3<T> &axis, const Eigen::Vector3<T> &axisPt, T angle)
	{
		// 将旋转轴归一化
		Eigen::Vector3<T> _axis = axis;
		_axis.normalize();

		// 平移到原点的矩阵
		Eigen::Matrix4<T> T_to_origin = Eigen::Matrix4<T>::Identity();
		T_to_origin(0, 3) = -axisPt[0];
		T_to_origin(1, 3) = -axisPt[1];
		T_to_origin(2, 3) = -axisPt[2];

		// 使用 AngleAxis 生成旋转矩阵
		Eigen::AngleAxis<T> rotation(angle, _axis);
		Eigen::Matrix3<T> R = rotation.toRotationMatrix();

		// 嵌入到 4x4 矩阵
		Eigen::Matrix4<T> R_homogeneous = Eigen::Matrix4<T>::Identity();
		R_homogeneous.template block<3, 3>(0, 0) = R;

		// 平移回原位置的矩阵
		Eigen::Matrix4<T> T_back = Eigen::Matrix4<T>::Identity();
		T_back(0, 3) = axisPt[0];
		T_back(1, 3) = axisPt[1];
		T_back(2, 3) = axisPt[2];

		// 最终变换矩阵
		Eigen::Matrix4<T> M = T_back * R_homogeneous * T_to_origin;

		return M;
	}

	/**
	 * @brief 构造向量r1旋转到向量r2的旋转矩阵
	 * @param r1 旋转前向量
	 * @param r2 旋转后向量
	 * @return 定轴旋转矩阵
	 */
	template <typename T>
	inline Eigen::Matrix4<T> makeRotate(const Eigen::Vector3<T> &r1, const Eigen::Vector3<T> &r2)
	{
		Eigen::Vector3<T> u1 = r1.normalized();
		Eigen::Vector3<T> u2 = r2.normalized();
		double cosTheta = u1.dot(u2);

		// 若方向相同，返回单位阵
		if (cosTheta > 1.0 - 1e-10)
			return Eigen::Matrix4<T>::Identity();

		// 若方向相反，选择任意垂直向量为旋转轴
		if (cosTheta < -1.0 + 1e-10)
		{
			Eigen::Vector3d axis = Eigen::Vector3d::UnitX().cross(u1);
			if (axis.norm() < 1e-6)
				axis = Eigen::Vector3d::UnitY().cross(u1);
			axis.normalize();
			Eigen::Matrix4<T> homoMatrix = Eigen::Matrix4<T>::Identity();
			homoMatrix.template block<3, 3>(0, 0) = Eigen::AngleAxisd(M_PI, axis).toRotationMatrix();
			return homoMatrix;
		}

		Eigen::Vector3d axis = u1.cross(u2);
		axis.normalize();
		double angle = std::acos(cosTheta);
		Eigen::Matrix4<T> homoMatrix = Eigen::Matrix4<T>::Identity();
		homoMatrix.template block<3, 3>(0, 0) = Eigen::AngleAxisd(angle, axis).toRotationMatrix();
		return homoMatrix;
	}

	template <typename T>
	inline Eigen::Matrix4<T> makeXYZRPY(const Eigen::Vector3<T> &xyz, const Eigen::Vector3<T> &rpy)
	{
		Eigen::Matrix4<T> xyzMat = JMath::makeTranslate(xyz);
		Eigen::Matrix3<T> rpyMat3 = JMath::rpyToRotationMatrix(rpy(0), rpy(1), rpy(2));
		Eigen::Matrix4<T> rpyMat = Eigen::Matrix4<T>::Identity();
		rpyMat.template block<3, 3>(0, 0) = rpyMat3;
		return xyzMat * rpyMat;
	}

	/**
	 * @brief 计算点集XY投影凸包
	 * @param points 点集(Nx3)
	 * @return 凸包点集
	 */
	template <typename T>
	inline Eigen::MatrixX<T> computeConvexHullXY(const Eigen::MatrixX<T> &points)
	{
		auto cross2D = [](const Eigen::Vector3<T> &a, const Eigen::Vector3<T> &b) -> T
		{
			return a.x() * b.y() - a.y() * b.x();
		};

		std::vector<Eigen::Vector3<T>> hull;
		size_t n = points.rows();
		if (!n)
			return {};

		// 排序点集
		std::vector<Eigen::Vector3<T>> sortedPoints(points.rows());
		int i = 0;
		for (const auto &point : points.rowwise())
		{
			sortedPoints[i++] = point;
		}
		std::sort(sortedPoints.begin(), sortedPoints.end(),
				  [](const Eigen::Vector3<T> &a, const Eigen::Vector3<T> &b)
				  {
					  return (a.x() < b.x()) || (a.x() == b.x() && a.y() < b.y());
				  });

		// 下凸壳
		for (size_t j = 0; j < n; ++j)
		{
			while (hull.size() >= 2 &&
				   cross2D(hull.back() - *(hull.rbegin() + 1), sortedPoints[j] - hull.back()) <= 0)
			{
				hull.pop_back();
			}
			hull.push_back(sortedPoints[j]);
		}

		// 上凸壳
		size_t t = hull.size();
		for (size_t j = n - 1; j > 0; --j)
		{
			while (hull.size() > t &&
				   cross2D(hull.back() - *(hull.rbegin() + 1), sortedPoints[j - 1] - hull.back()) <= 0)
			{
				hull.pop_back();
			}
			hull.push_back(sortedPoints[j - 1]);
		}

		hull.pop_back(); // 最后一个点重复，删除

		Eigen::MatrixX<T> ret(hull.size(), 3);
		i = 0;
		for (const auto &pt : hull)
		{
			ret.row(i++) = pt;
		}
		return ret;
	}

	/**
	 * @brief 计算点集XY投影OBB
	 * @param points 点集(Nx3)
	 * @param minPoint 正矩形Min
	 * @param maxPoint 正矩形Max
	 * @param angle 正矩形angle，旋转中心为XY坐标系原点
	 */
	template <typename T>
	inline void computeOBBXY(const Eigen::MatrixX<T> &points, Eigen::Vector3<T> &minPoint, Eigen::Vector3<T> &maxPoint, T &angle)
	{
		if (points.rows() == 1)
		{
			minPoint = points.row(0);
			maxPoint = points.row(0);
			angle = 0.0;
			return;
		}
		// 计算凸包
		Eigen::MatrixX<T> hull = computeConvexHullXY(points);

		size_t n = hull.rows();
		T minArea = std::numeric_limits<T>::max();

		// 遍历凸包边
		for (size_t i = 0; i < n; ++i)
		{
			Eigen::Vector3<T> edge = hull.row((i + 1) % n) - hull.row(i);
			T theta = std::atan2(edge.y(), edge.x());

			// 构造旋转矩阵
			Eigen::Matrix3<T> R;
			R << std::cos(theta), -std::sin(theta), 0,
				std::sin(theta), std::cos(theta), 0,
				0, 0, 1;

			// 将点旋转到与边平行
			std::vector<Eigen::Vector3<T>> rotated;
			for (const auto &point : hull.rowwise())
			{
				rotated.push_back(R * point.transpose());
			}

			// 找到最小和最大坐标
			Eigen::Vector3<T> minCoord = rotated[0], maxCoord = rotated[0];
			for (const auto &p : rotated)
			{
				minCoord = minCoord.cwiseMin(p);
				maxCoord = maxCoord.cwiseMax(p);
			}

			// 计算面积
			T area = (maxCoord.x() - minCoord.x()) * (maxCoord.y() - minCoord.y());
			if (area < minArea)
			{
				minArea = area;
				minPoint = minCoord;
				maxPoint = maxCoord;
				angle = theta;
			}
		}
		angle = -angle;
	}

	/**
	 * @brief 计算经过mat变换后的点坐标
	 * @param mat 变换矩阵
	 * @param pt 点坐标
	 * @return 变换后的坐标
	 */
	template <typename T>
	inline Eigen::Vector3<T> transformPoint(const Eigen::Matrix4<T> &mat, const Eigen::Vector3<T> &pt)
	{
		Eigen::Vector4<T> homoPt(pt[0], pt[1], pt[2], 1);
		return (mat * homoPt).head(3);
	}

	/**
	 * @brief 计算经过mat变换后的位姿
	 * @param mat 变换矩阵
	 * @param pose 位姿(xyzrpy)
	 * @return 变换后的坐标
	 */
	template <typename T>
	inline std::array<T, 6> transformPose(const Eigen::Matrix4<T> &mat, const std::array<T, 6> &pose)
	{
		Eigen::Vector3<T> pt(pose[0], pose[1], pose[2]);
		Eigen::Matrix3<T> rotMat = JMath::rpyToRotationMatrix(pose[3], pose[4], pose[5]);
		Eigen::Vector3<T> newPt = JMath::transformPoint(mat, pt);
		Eigen::Vector3<T> newRpy = JMath::rotationMatrixToRPY<T>(mat.template block<3, 3>(0, 0) * rotMat);
		return std::array<T, 6>{newPt.x(), newPt.y(), newPt.z(), newRpy.x(), newRpy.y(), newRpy.z()};
	}

	/**
	 * @brief 计算两个四元素的距离
	 * @param q1 四元数1
	 * @param q2 四元数2
	 * @return 四元素的距离(弧度)
	 */
	template <typename T>
	T quaternionDistance(const Eigen::Quaternion<T> &q1, const Eigen::Quaternion<T> &q2)
	{
		// 假设 q1 和 q2 是单位四元数；否则需要先归一化
		T dot = q1.w() * q2.w() + q1.x() * q2.x() + q1.y() * q2.y() + q1.z() * q2.z();

		// 取绝对值以获得最短角度；并钳位到 [-1,1] 以防数值误差
		dot = std::clamp(std::abs(dot), T(-1), T(1));

		return T(2) * std::acos(dot); // 返回弧度
	}

	/**
	 * @brief 四元数球面线性插值
	 * @param start_q 起点四元数
	 * @param end_q 终点四元数
	 * @param t 系数[0, 1]
	 * @return 插值点四元数
	 */
	template <typename T>
	inline Eigen::Quaternion<T> quaternion_Slerp(Eigen::Quaternion<T> &start_q, Eigen::Quaternion<T> &end_q, T t)
	{
		Eigen::Quaternion<T> lerp_q;

		T cos_angle = start_q.x() * end_q.x() + start_q.y() * end_q.y() + start_q.z() * end_q.z() + start_q.w() * end_q.w();

		// If the dot product is negative, the quaternions have opposite handed-ness and slerp won't take
		// the shorter path. Fix by reversing one quaternion.
		if (cos_angle < 0)
		{
			end_q.x() = -end_q.x();
			end_q.y() = -end_q.y();
			end_q.z() = -end_q.z();
			end_q.w() = -end_q.w();
			cos_angle = -cos_angle;
		}

		T ratio_A, ratio_B;
		// If the inputs are too close for comfort, linearly interpolate
		if (cos_angle > 0.99995f)
		{
			ratio_A = 1.0f - t;
			ratio_B = t;
		}
		else
		{
			T sin_angle = sqrt(1.0f - cos_angle * cos_angle);
			T angle = atan2(sin_angle, cos_angle);
			ratio_A = sin((1.0f - t) * angle) / sin_angle;
			ratio_B = sin(t * angle) / sin_angle;
		}

		lerp_q.x() = ratio_A * start_q.x() + ratio_B * end_q.x();
		lerp_q.y() = ratio_A * start_q.y() + ratio_B * end_q.y();
		lerp_q.z() = ratio_A * start_q.z() + ratio_B * end_q.z();
		lerp_q.w() = ratio_A * start_q.w() + ratio_B * end_q.w();

		return lerp_q.normalized();
	}

	/**
	 * @brief 计算矩阵距离
	 * @param T1 矩阵1
	 * @param T2 矩阵2
	 * @return 矩阵距离
	 */
	template <typename T>
	inline T matrixDistance(const Eigen::Matrix4<T> &T1, const Eigen::Matrix4<T> &T2)
	{
		Eigen::Matrix3<T> R1 = T1.template block<3, 3>(0, 0);
		Eigen::Matrix3<T> R2 = T2.template block<3, 3>(0, 0);
		// 对旋转子矩阵进行数值修正以避免 asin/acos 越界或非正交带来的误差
		Eigen::Matrix3<T> R1s = sanitizeRotationMatrix<T>(R1);
		Eigen::Matrix3<T> R2s = sanitizeRotationMatrix<T>(R2);
		T trace = (R1s.transpose() * R2s).trace();
		T cos_theta = (trace - static_cast<T>(1.0)) / static_cast<T>(2.0);
		cos_theta = std::clamp(cos_theta, static_cast<T>(-1), static_cast<T>(1));
		T rotDist = JMath::R2D(std::acos(cos_theta));

		Eigen::Vector3<T> t1 = T1.template block<3, 1>(0, 3);
		Eigen::Vector3<T> t2 = T2.template block<3, 1>(0, 3);
		T transDist = (t1 - t2).norm();

		return rotDist * 0.5 + transDist * 0.5;
	}
}
