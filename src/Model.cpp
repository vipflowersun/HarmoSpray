#include "Model.h"
#include "OccView.h"

Model::Model(const std::list<Handle(AIS_Shape_WithFrame)> &parts)
{
	_parts.insert(_parts.end(), parts.begin(), parts.end());
	_matrices.resize(_parts.size(), Eigen::Matrix4d::Identity());
}

Model::Model(const std::list<Handle(AIS_InteractiveObject)> &parts) : _parts(parts)
{
	_matrices.resize(_parts.size(), Eigen::Matrix4d::Identity());
}

void Model::setModelName(const std::string &name)
{
	_modelName = name;
}

std::string Model::modelName() const
{
	return _modelName;
}

std::shared_ptr<Model> Model::clone() const
{
	std::list<Handle(AIS_InteractiveObject)> cloneParts;
	for (const auto &part : _parts)
	{
		if (Handle(AIS_Shape_WithFrame) frameShape = Handle(AIS_Shape_WithFrame)::DownCast(part))
		{
			Handle(AIS_Shape_WithFrame) clonedPart = frameShape->clone();
			cloneParts.push_back(clonedPart);
		}
		else if (Handle(AIS_Shape) shape = Handle(AIS_Shape_WithFrame)::DownCast(part))
		{
			Handle(AIS_Shape) clonePart = new AIS_Shape(shape->Shape());
			cloneParts.push_back(clonePart);
		}
		//......
	}

	std::shared_ptr<Model> cloneModel = std::make_shared<Model>(cloneParts);
	return cloneModel;
}

bool Model::setMatrix(const std::vector<Eigen::Matrix4d> &mats)
{
	std::unique_lock<std::mutex> lck(_mutex);
	if (mats.size() != _parts.size())
		return false;
	if (_matrices == mats)
		return true;
	int i = 0;
	for (const auto &part : _parts)
	{
		if (Handle(AIS_Shape_WithFrame) frameShape = Handle(AIS_Shape_WithFrame)::DownCast(part))
			frameShape->SetLocalTransformation(Model::convertEigenToGpTrsf(mats[i++]));
		else
			part->SetLocalTransformation(Model::convertEigenToGpTrsf(mats[i++]));
	}
	_matrices = mats;
	this->matrixChanged(mats);
	return true;
}

void Model::getMatrix(std::vector<Eigen::Matrix4d> &mats)
{
	std::unique_lock<std::mutex> lck(_mutex);
	mats.clear();
	for (const auto &part : _parts)
	{
		mats.push_back(Model::convertGpTrsfToEigen(part->LocalTransformation()));
	}
}

void Model::setRootMatrix(const Eigen::Matrix4d &mat)
{
	std::unique_lock<std::mutex> lck(_mutex);
	if (_parts.empty())
		return;
	_matrices.clear();
	Eigen::Matrix4d dMat = convertGpTrsfToEigen(_parts.front()->LocalTransformation()).inverse() * mat;
	for (auto it = _parts.begin(); it != _parts.end(); it++)
	{
		Eigen::Matrix4d currentMat = convertGpTrsfToEigen((*it)->LocalTransformation());
		Eigen::Matrix4d newMat = currentMat * dMat; // 应用根部件的变换
		if (Handle(AIS_Shape_WithFrame) frameShape = Handle(AIS_Shape_WithFrame)::DownCast((*it)))
			frameShape->SetLocalTransformation(convertEigenToGpTrsf(newMat));
		else
			(*it)->SetLocalTransformation(convertEigenToGpTrsf(newMat));
		_matrices.push_back(newMat);
	}
	this->matrixChanged(_matrices);
}

void Model::setColor(int r, int g, int b)
{
	std::unique_lock<std::mutex> lck(_mutex);
	for (const auto &part : _parts)
	{
		part->SetColor(Quantity_Color(r / 255.0, g / 255.0, b / 255.0, Quantity_TOC_RGB));
	}
}

void Model::setTransparency(double transparency)
{
	std::unique_lock<std::mutex> lck(_mutex);
	for (const auto &part : _parts)
	{
		part->SetTransparency(transparency);
	}
}

void Model::displayInOcc(OccView *occ)
{
	std::unique_lock<std::mutex> lck(_mutex);
	for (const auto &part : _parts)
	{
		if (Handle(AIS_Shape_WithFrame) frameShape = Handle(AIS_Shape_WithFrame)::DownCast(part))
			occ->displayAIS(frameShape, true);
		else
			occ->getContext()->Display(part, Standard_False);

		occ->getContext()->Deactivate(part);
		for (size_t i = 0; i < _selectMode.size(); i++)
		{
			if (_selectMode[i])
			{
				occ->getContext()->Activate(part, AIS_Shape::SelectionMode(static_cast<TopAbs_ShapeEnum>(i)));
			}
		}
	}
}

void Model::removeFromOcc(OccView *occ)
{
	std::unique_lock<std::mutex> lck(_mutex);
	for (const auto &part : _parts)
	{
		occ->getContext()->Remove(part, Standard_False);
		if (Handle(AIS_Shape_WithFrame) frameShape = Handle(AIS_Shape_WithFrame)::DownCast(part))
		{
			occ->getContext()->Remove(frameShape->frameShape(), Standard_False);
		}
	}
}

void Model::setVisible(OccView *occ, bool visible)
{
	std::unique_lock<std::mutex> lck(_mutex);
	for (const auto &part : _parts)
	{
		if (visible)
		{
			occ->getContext()->Display(part, Standard_False);
			if (Handle(AIS_Shape_WithFrame) frameShape = Handle(AIS_Shape_WithFrame)::DownCast(part))
				occ->getContext()->Display(frameShape->frameShape(), Standard_False);
		}
		else
		{
			occ->getContext()->Erase(part, Standard_False);
			if (Handle(AIS_Shape_WithFrame) frameShape = Handle(AIS_Shape_WithFrame)::DownCast(part))
				occ->getContext()->Erase(frameShape->frameShape(), Standard_False);
		}
	}
}

bool Model::setPartCoordMatrix(int index, const Eigen::Matrix4d &mat)
{
	std::unique_lock<std::mutex> lck(_mutex);
	if (index < 0 || index >= static_cast<int>(_parts.size()))
		return false;
	auto it = _parts.begin();
	std::advance(it, index);
	if (Handle(AIS_Shape_WithCoord) shapeCoord = Handle(AIS_Shape_WithCoord)::DownCast(*it))
	{
		shapeCoord->setCoordLocate(mat);
		if (auto contex = shapeCoord->GetContext())
			contex->Redisplay(shapeCoord, Standard_True);
		return true;
	}
	return false;
}

void Model::setSelectable(bool selectable, TopAbs_ShapeEnum mode /* = TopAbs_COMPOUND*/)
{
	std::unique_lock<std::mutex> lck(_mutex);
	if (!selectable)
		_selectMode.fill(false);
	else
		_selectMode[mode] = true;
	for (const auto &part : _parts)
	{
		auto context = part->GetContext();
		if (!context)
			continue;
		if (selectable)
			context->Activate(part, AIS_Shape::SelectionMode(mode));
		else
			context->Deactivate(part);
	}
}

bool Model::selectable() const
{
	return std::any_of(_selectMode.begin(), _selectMode.end(), [](bool v)
					   { return v; });
}

gp_Trsf Model::convertEigenToGpTrsf(const Eigen::Matrix4d &matrix)
{
	// 提取旋转部分
	gp_Mat rotation(
		matrix(0, 0), matrix(0, 1), matrix(0, 2),
		matrix(1, 0), matrix(1, 1), matrix(1, 2),
		matrix(2, 0), matrix(2, 1), matrix(2, 2));

	// 提取平移部分
	gp_XYZ translation(matrix(0, 3), matrix(1, 3), matrix(2, 3));

	// 构造gp_Trsf
	gp_Trsf trsf;
	trsf.SetValues(
		rotation(1, 1), rotation(1, 2), rotation(1, 3), translation.X(),
		rotation(2, 1), rotation(2, 2), rotation(2, 3), translation.Y(),
		rotation(3, 1), rotation(3, 2), rotation(3, 3), translation.Z()); // 设置公差

	return trsf;
}

Eigen::Matrix4d Model::convertGpTrsfToEigen(const gp_Trsf &matrix)
{
	Eigen::Matrix4d mat;
	mat << matrix.Value(1, 1), matrix.Value(1, 2), matrix.Value(1, 3), matrix.Value(1, 4),
		matrix.Value(2, 1), matrix.Value(2, 2), matrix.Value(2, 3), matrix.Value(2, 4),
		matrix.Value(3, 1), matrix.Value(3, 2), matrix.Value(3, 3), matrix.Value(3, 4),
		0.0, 0.0, 0.0, 1.0;
	return mat;
}

bool Model::isTrsfEqual(const gp_Trsf &t1, const gp_Trsf &t2, double tol)
{
	// 比较 3x4 矩阵的每个元素
	for (int row = 1; row <= 3; ++row)
	{
		for (int col = 1; col <= 4; ++col)
		{
			if (std::abs(t1.Value(row, col) - t2.Value(row, col)) > tol)
				return false;
		}
	}
	// 可选：比较变换类型和缩放因子
	if (t1.Form() != t2.Form())
		return false;
	if (std::abs(t1.ScaleFactor() - t2.ScaleFactor()) > tol)
		return false;
	return true;
}
