#pragma once
#include "AIS_Shape_WithFrame.hpp"

#include "Eigen/Dense"

#include "sigslot\signal.hpp"

#include <list>
#include <mutex>
class OccView;

class Model
{
public:
	// parts的第一个为root
	explicit Model(const std::list<Handle(AIS_Shape_WithFrame)> &parts);
	explicit Model(const std::list<Handle(AIS_InteractiveObject)> &parts);

public:
	inline void setData(const std::string &key, const std::any &value)
	{
		_data[key] = value;
	}
	inline void removeData(const std::string &key)
	{
		auto it = _data.find(key);
		if (it != _data.end())
			_data.erase(it);
	}
	template <typename T>
	inline bool data(const std::string &key, T &value) const
	{
		auto find = _data.find(key);
		if (find == _data.end() || find->second.type() != typeid(T))
			return false;

		value = std::any_cast<T>(find->second);
		return true;
	}
	void setModelName(const std::string &name);
	std::string modelName() const;
	std::shared_ptr<Model> clone() const;
	bool setMatrix(const std::vector<Eigen::Matrix4d> &mats);
	void getMatrix(std::vector<Eigen::Matrix4d> &mats);
	void setRootMatrix(const Eigen::Matrix4d &mat);
	void setColor(int r, int g, int b);
	void displayInOcc(OccView *occ);
	void removeFromOcc(OccView *occ);
	void setVisible(OccView *occ, bool visible);
	bool setPartCoordMatrix(int index, const Eigen::Matrix4d &mat);
	void setSelectable(bool selectable, TopAbs_ShapeEnum mode = TopAbs_COMPOUND);
	bool selectable() const;

private:
	static gp_Trsf convertEigenToGpTrsf(const Eigen::Matrix4d &matrix);
	static Eigen::Matrix4d convertGpTrsfToEigen(const gp_Trsf &matrix);
	static bool isTrsfEqual(const gp_Trsf &t1, const gp_Trsf &t2, double tol = 1E-6);

private:
	std::list<Handle(AIS_InteractiveObject)> _parts; // 模型部件列表
	std::mutex _mutex;
	std::vector<Eigen::Matrix4d> _matrices;
	std::map<std::string, std::any> _data;
	std::array<bool, TopAbs_SHAPE> _selectMode = {false};
	std::string _modelName;

public:
	sigslot::signal<const std::vector<Eigen::Matrix4d> &> matrixChanged;
};