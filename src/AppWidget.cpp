#include "AppWidget.h"
#include "JMath.hpp"
#include "OccExtension.h"
#include "Scene.h"

#include "STEPControl_Reader.hxx"
#include "Geom_CartesianPoint.hxx"
#include "AIS_Point.hxx"
#include "AIS_Line.hxx"
#include "Geom_Line.hxx"
#include "BRepBuilderAPI_MakeEdge.hxx"
#include "TopoDS_Edge.hxx"

#include <QDebug>
#include <QMessageBox>
#include <QMenu>
#include <QGridLayout>
#include <QTimer>
#include <QDragEnterEvent>
#include <QDropEvent>
#include <QMimeData>
#include <QUrl>

#include <iostream>
#include <filesystem>

#define DEVICE_NAME "default"

static gp_Trsf convertEigenToGpTrsf(const Eigen::Matrix4d &matrix)
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

AppWidget::AppWidget(QWidget *parent)
	: QWidget(parent)
{
	_ui = std::make_shared<Ui::AppWid>();
	_ui->setupUi(this);

	SCENE.setLogLevel(0);
	SCENE.run();

	int err = _client.connect("127.0.0.1");
	if (err)
	{
		_ui->info->error(QStringLiteral("连接到服务器失败, 错误码: %1").arg(err));
		return;
	}
	std::array<double, 7> linkLengths;
	_client.getManipulatorLinkLength(DEVICE_NAME, linkLengths);
	if (err)
	{
		_ui->info->error(QStringLiteral("getManipulatorLinkLength失败, 错误码: %1").arg(err));
		return;
	}
	std::vector<Kinematic::MDH> mdhs(6);
	mdhs[1].a = linkLengths[0];
	mdhs[0].d = linkLengths[1];
	mdhs[2].a = linkLengths[2];
	mdhs[3].a = linkLengths[3];
	mdhs[3].d = linkLengths[4];
	mdhs[5].d = linkLengths[5];
	_kinematic.setMDH(mdhs);

	connect(_ui->btnFitAll, &QPushButton::clicked, this, [this]() { _ui->widOcc->fitAll(); });
	_client.onJointPositionsUpdated(std::bind(&AppWidget::onJointPositionsUpdated, this, std::placeholders::_1, std::placeholders::_2));
}

void AppWidget::showEvent(QShowEvent *event)
{
	QWidget::showEvent(event);
	QTimer::singleShot(1000, this, &AppWidget::loadScene);
}

void AppWidget::onJointPositionsUpdatedGUI()
{
	if (!_model)
		return;

	_model->setMatrix(_matVec);
	_coordShape->SetLocalTransformation(convertEigenToGpTrsf(_matVec[6]));
	_coordShapeTool->SetLocalTransformation(convertEigenToGpTrsf(_matVec[6] * JMath::xyzrpyToTransformMatrix(_toolPoseToEndFlange)));
	_ui->widOcc->update();
}

//void AppWidget::onLinkPosesUpdated(const std::map<std::string, std::vector<std::array<double, 16>>> &linkPoses,
//								   uint64_t clock)
//{
//	auto it = linkPoses.find("default");
//	if (it == linkPoses.end() || it->second.size() < 6)
//	{
//		qDebug() << "no device named default or default is not robot";
//		return;
//	}
//	if (!_model || !_coordShape || !_coordShapeTool)
//		return;
//
//	_matVec.resize(7);
//	_matVec[0] = Eigen::Matrix4d::Identity();
//	if (it->second.size() > 6)
//	{
//		std::array<double, 16> linkPose = it->second[it->second.size() - 7];
//		for (int r = 0; r < 4; r++)
//		{
//			for (int c = 0; c < 4; c++)
//			{
//				_matVec[0](r, c) = linkPose[r * 4 + c];
//			}
//		}
//		_matVec[0] = JMath::sanitizeRotationMatrix(_matVec[0]);
//	}
//	for (int i = 1; i < 7; i++)
//	{
//		std::array<double, 16> linkPose = it->second[i - 1 + it->second.size() - 6];
//		for (int r = 0; r < 4; r++)
//		{
//			for (int c = 0; c < 4; c++)
//			{
//				_matVec[i](r, c) = linkPose[r * 4 + c];
//			}
//		}
//		_matVec[i] = JMath::sanitizeRotationMatrix(_matVec[i]);
//	}
//
//	QMetaObject::invokeMethod(this, "onLinkPosesUpdatedGUI", Qt::QueuedConnection);
//}

void AppWidget::closeEvent(QCloseEvent *event)
{
	_client.close();
	SCENE.shut();
	QWidget::closeEvent(event);
}

bool AppWidget::loadScene()
{
	std::string r6FileName[] = { "base.STEP", "link1.STEP", "link2.STEP", "link3.STEP", "link4.STEP", "link5.STEP", "link6.STEP" };
	std::list<std::string> modelFiles;
	// 检查文件是否存在
	for (const auto& file : r6FileName)
	{
		std::string filePath = "models/" + file;
		if (!std::filesystem::exists(filePath))
		{
			_ui->info->error(QStringLiteral("模型 %1 不存在").arg(QString::fromStdString(file)));
			return false;
		}
		modelFiles.push_back(filePath);
	}

	// 加载设备模型
	std::list<Handle(AIS_Shape_WithFrame)> parts;
	for (const auto& modelFile : modelFiles)
	{
		TopoDS_Shape topoShape;
		STEPControl_Reader reader;
		if (reader.ReadFile(modelFile.c_str()) == IFSelect_RetDone)
		{
			reader.TransferRoots();
			topoShape = reader.OneShape();
		}
		if (topoShape.IsNull())
		{
			_ui->info->error(QStringLiteral("模型读入失败, 检查文件: %1").arg(QString::fromStdString(modelFile)));
			return false;
		}
		Handle(AIS_Shape_WithFrame) shape = new AIS_Shape_WithFrame(topoShape);
		parts.push_back(shape);
	}
	_model = std::make_shared<Model>(parts);
	_model->setColor(200, 200, 30);
	_model->displayInOcc(_ui->widOcc);

	gp_Ax2 ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1), gp_Dir(1, 0, 0));
	_coordShape = OccExtension::drawCoordinate(ax2, 100);
	_ui->widOcc->getContext()->Display(_coordShape, false);
	_ui->widOcc->getContext()->Deactivate(_coordShape);

	_coordShapeTool = OccExtension::drawCoordinate(ax2, 100);
	_ui->widOcc->getContext()->Display(_coordShapeTool, false);
	_ui->widOcc->getContext()->Deactivate(_coordShapeTool);

	_ui->widOcc->fitAll();

	_ui->info->log("加载场景成功");
	return true;
}

void AppWidget::onJointPositionsUpdated(const std::map<std::string, std::vector<double>>& positions, uint64_t clock)
{
	auto it = positions.find(DEVICE_NAME);
	if (it == positions.end())
	{
		_ui->info->error(QStringLiteral("设备%1不存在").arg(QString(DEVICE_NAME)));
		return;
	}

	std::vector<double> joints = it->second;
	for (int i=0; i<6; i++)
		joints[i] = JMath::D2R(joints[i]);
	_matVec = _kinematic.forward(joints);

	QMetaObject::invokeMethod(this, "onJointPositionsUpdatedGUI", Qt::QueuedConnection);
}
