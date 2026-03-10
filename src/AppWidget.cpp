#include "AppWidget.h"
#include "JMath.hpp"
#include "OccExtension.h"
#include "Scene.h"

#include "STEPControl_Reader.hxx"
#include "Geom_CartesianPoint.hxx"
#include "AIS_Point.hxx"
#include "AIS_Line.hxx"
#include "Geom_Line.hxx"
#include "TopoDS_Edge.hxx"
#include "BRepBuilderAPI_MakeEdge.hxx"
#include "BRepPrimAPI_MakeCylinder.hxx"
#include "BRepPrimAPI_MakeCone.hxx"
#include "TopoDS.hxx"

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
#include <format>
#include <fstream>
#include <filesystem>

#define DEVICE_NAME "default"

#define TOOL_NAME "sprayer"

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

	SCENE.setLogLevel(0xff);
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
	mdhs[0] = { 0, 0, 0, 0 };
	mdhs[1] = { -M_PI_2, -M_PI_2, 0, 0 };
	mdhs[2] = { 0, 0, 0, 0 };
	mdhs[3] = { 0, -M_PI_2, 0, 0 };
	mdhs[4] = { 0, M_PI_2, 0, 0 };
	mdhs[5] = { M_PI, -M_PI_2, 0, };
	mdhs[1].a = linkLengths[0];
	mdhs[0].d = linkLengths[1];
	mdhs[2].a = linkLengths[2];
	mdhs[3].a = linkLengths[3];
	mdhs[3].d = linkLengths[4];
	mdhs[5].d = linkLengths[5];
	_kinematic.setMDH(mdhs);
	double sprayDist = _ui->editSprayDist->text().toDouble();
	_client.addTool(DEVICE_NAME, TOOL_NAME, { 400 + sprayDist, 0, 0, 0, 0, 0 });

	connect(_ui->btnFitAll, &QPushButton::clicked, this, [this]() { _ui->widOcc->fitAll(); });
	connect(_ui->btnStartSim, &QPushButton::clicked, this, &AppWidget::onBtnStartSim);
	connect(_ui->btnStopSim, &QPushButton::clicked, this, &AppWidget::onBtnStopSim);
	connect(_ui->btnSwitchFixture, &QPushButton::clicked, this, &AppWidget::onBtnSwitchFixture);
	connect(_ui->editOffsetX, &QLineEdit::editingFinished, this, &AppWidget::onEditOffetEditingFinished);
	connect(_ui->editOffsetY, &QLineEdit::editingFinished, this, &AppWidget::onEditOffetEditingFinished);
	connect(_ui->editOffsetZ, &QLineEdit::editingFinished, this, &AppWidget::onEditOffetEditingFinished);
	connect(_ui->editSprayDist, &QLineEdit::editingFinished, this, &AppWidget::onEditSprayDistEditingFinished);
	connect(_ui->editCylinderD, &QLineEdit::editingFinished, this, &AppWidget::onCylinderShapeEditingFinished);
	connect(_ui->editCylinderH, &QLineEdit::editingFinished, this, &AppWidget::onCylinderShapeEditingFinished);
	connect(_ui->editConeTopD, &QLineEdit::editingFinished, this, &AppWidget::onConeShapeEditingFinished);
	connect(_ui->editConeBottomD, &QLineEdit::editingFinished, this, &AppWidget::onConeShapeEditingFinished);
	connect(_ui->editConeH, &QLineEdit::editingFinished, this, &AppWidget::onConeShapeEditingFinished);
	connect(_ui->widOcc, &OccView::selectionChanged, this, &AppWidget::onSelectionChanged);

	_client.onJointPositionsUpdated(std::bind(&AppWidget::onJointPositionsUpdated, this, std::placeholders::_1, std::placeholders::_2));
	_client.onScriptStatusUpdated(std::bind(&AppWidget::onScriptStatusUpdated, this, std::placeholders::_1));
	_client.onErrorUpdated(std::bind(&AppWidget::onErrorUpdated, this, std::placeholders::_1));
}

void AppWidget::showEvent(QShowEvent *event)
{
	QWidget::showEvent(event);
	if (!_model)
		QTimer::singleShot(1000, this, &AppWidget::loadScene);
}

void AppWidget::onJointPositionsUpdatedGUI()
{
	if (!_model)
		return;

	_model->setMatrix(_matVec);
	_coordShape->SetLocalTransformation(convertEigenToGpTrsf(_matVec.back()));
	double sprayDist = _ui->editSprayDist->text().toDouble();
	std::array<double, 6> endFlangePose;
	endFlangePose.fill(0);
	endFlangePose[0] = 400 + sprayDist;
	endFlangePose[2] = 0;
	_coordShapeTool->SetLocalTransformation(convertEigenToGpTrsf(_matVec.back() * JMath::xyzrpyToTransformMatrix(endFlangePose)));
	_ui->widOcc->update();
}

void AppWidget::closeEvent(QCloseEvent *event)
{
	_client.close();
	SCENE.shut();
	QWidget::closeEvent(event);
}

bool AppWidget::loadScene()
{
	std::string r6FileName[] = { "base.STEP", "y.STEP", "link1.STEP", "link2.STEP", "link3.STEP", "link4.STEP", "link5.STEP", "link6.STEP" };
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
	_model->setSelectable(true, TopAbs_VERTEX);
	_ui->info->log("加载设备模型成功");

	//工装
	std::string fixtureCylinderFile = "models/fixtureCylinder.STEP";
	if (!std::filesystem::exists(fixtureCylinderFile))
	{
		_ui->info->error(QStringLiteral("模型 %1 不存在").arg(QString::fromStdString(fixtureCylinderFile)));
		return false;
	}
	TopoDS_Shape topoShape;
	STEPControl_Reader reader;
	if (reader.ReadFile(fixtureCylinderFile.c_str()) == IFSelect_RetDone)
	{
		reader.TransferRoots();
		topoShape = reader.OneShape();
	}
	if (topoShape.IsNull())
	{
		_ui->info->error(QStringLiteral("模型读入失败, 检查文件: %1").arg(QString::fromStdString(fixtureCylinderFile)));
		return false;
	}
	Handle(AIS_Shape_WithFrame) shape = new AIS_Shape_WithFrame(topoShape);
	_fixCylinder = std::make_shared<Model>(std::list<Handle(AIS_Shape_WithFrame)>{ shape });
	_fixCylinder->setColor(150, 150, 150);
	_fixCylinder->displayInOcc(_ui->widOcc);
	_fixCylinder->setSelectable(true, TopAbs_VERTEX);
	Eigen::Vector3d offset(0, 0, 0);
	offset(0) = _ui->editOffsetX->text().toDouble();
	offset(1) = _ui->editOffsetY->text().toDouble();
	offset(2) = _ui->editOffsetZ->text().toDouble();
	_fixCylinder->setMatrix({ JMath::makeTranslate(offset) });
	_ui->info->log("加载圆柱工装模型成功");

	std::string fixtureConeFile = "models/fixtureCone.STEP";
	if (!std::filesystem::exists(fixtureConeFile))
	{
		_ui->info->error(QStringLiteral("模型 %1 不存在").arg(QString::fromStdString(fixtureConeFile)));
		return false;
	}
	reader = STEPControl_Reader();
	if (reader.ReadFile(fixtureConeFile.c_str()) == IFSelect_RetDone)
	{
		reader.TransferRoots();
		topoShape = reader.OneShape();
	}
	if (topoShape.IsNull())
	{
		_ui->info->error(QStringLiteral("模型读入失败, 检查文件: %1").arg(QString::fromStdString(fixtureConeFile)));
		return false;
	}
	shape = new AIS_Shape_WithFrame(topoShape);
	_fixCone = std::make_shared<Model>(std::list<Handle(AIS_Shape_WithFrame)>{ shape });
	_fixCone->setColor(150, 150, 150);
	_fixCone->displayInOcc(_ui->widOcc);
	offset(0) = _ui->editOffsetX->text().toDouble();
	offset(1) = _ui->editOffsetY->text().toDouble();
	offset(2) = _ui->editOffsetZ->text().toDouble();
	_fixCone->setMatrix({ JMath::makeTranslate(offset) });
	_ui->info->log("加载圆锥工装模型成功");
	_fixCone->setVisible(_ui->widOcc, false);


	//圆柱工件
	gp_Ax2 axis(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0));
	Standard_Real radius = _ui->editCylinderD->text().toDouble() / 2;
	Standard_Real height = _ui->editCylinderH->text().toDouble();
	TopoDS_Shape cylShape = BRepPrimAPI_MakeCylinder(axis, radius, height).Shape();
	Handle(AIS_Shape_WithFrame) cyl = new AIS_Shape_WithFrame(cylShape);
	_wpCylinder = std::make_shared<Model>(std::list<Handle(AIS_Shape_WithFrame)>{ cyl });
	_wpCylinder->setColor(50, 100, 150);
	_wpCylinder->setTransparency();
	_wpCylinder->displayInOcc(_ui->widOcc);
	_wpCylinder->setMatrix({ JMath::makeTranslate(offset) });

	//圆锥工件
	axis = gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
	Standard_Real rBottom = _ui->editConeBottomD->text().toDouble() / 2;
	Standard_Real rTop = _ui->editConeTopD->text().toDouble() / 2;
	Standard_Real coneHeight = _ui->editConeH->text().toDouble();
	TopoDS_Shape coneShape = BRepPrimAPI_MakeCone(axis, rBottom, rTop, coneHeight).Shape();
	Handle(AIS_Shape_WithFrame) cone = new AIS_Shape_WithFrame(coneShape);
	_wpCone = std::make_shared<Model>(std::list<Handle(AIS_Shape_WithFrame)>{ cone });
	_wpCone->setColor(50, 100, 150);
	_wpCone->setTransparency();
	_wpCone->displayInOcc(_ui->widOcc);
	_wpCone->setMatrix({ JMath::makeTranslate(offset) });
	_wpCone->setVisible(_ui->widOcc, false);
	

	//基坐标系
	gp_Ax2 ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1), gp_Dir(1, 0, 0));
	_coordShape = OccExtension::drawCoordinate(ax2, 100);
	_ui->widOcc->getContext()->Display(_coordShape, false);
	_ui->widOcc->getContext()->Deactivate(_coordShape);

	//工具坐标系
	_coordShapeTool = OccExtension::drawCoordinate(ax2, 100);
	_ui->widOcc->getContext()->Display(_coordShapeTool, false);
	_ui->widOcc->getContext()->Deactivate(_coordShapeTool);

	_ui->widOcc->fitAll();

	_ui->info->log("加载场景成功");
	return true;
}

void AppWidget::onEditOffetEditingFinished()
{
	Eigen::Vector3d offset(0, 0, 0);
	offset(0) = _ui->editOffsetX->text().toDouble();
	offset(1) = _ui->editOffsetY->text().toDouble();
	offset(2) = _ui->editOffsetZ->text().toDouble();
	if (_fixCylinder)
		_fixCylinder->setMatrix({ JMath::makeTranslate(offset) });
	if (_wpCylinder)
		_wpCylinder->setMatrix({ JMath::makeTranslate(offset) });
	if (_fixCone)
		_fixCone->setMatrix({ JMath::makeTranslate(offset) });
	if (_wpCone)
		_wpCone->setMatrix({ JMath::makeTranslate(offset) });

	if (_useCylinder && _fixCylinder)
		_fixCylinder->redisplayInOcc(_ui->widOcc);
	else if (!_useCylinder && _fixCone)
		_fixCone->redisplayInOcc(_ui->widOcc);

	_ui->widOcc->update();
}

void AppWidget::onEditSprayDistEditingFinished()
{
	double sprayDist = _ui->editSprayDist->text().toDouble();
	if (int err = _client.setToolPose(DEVICE_NAME, TOOL_NAME, { 400 + sprayDist, 0, 0, 0, 0, 0 }))
	{
		_ui->info->error(QStringLiteral("setToolPose失败, 错误码: %1").arg(err));
	}
	this->onJointPositionsUpdatedGUI();
}

void AppWidget::onCylinderShapeEditingFinished()
{
	//圆柱工件
	if (_wpCylinder)
		_wpCylinder->removeFromOcc(_ui->widOcc);

	Eigen::Vector3d offset(0, 0, 0);
	offset(0) = _ui->editOffsetX->text().toDouble();
	offset(1) = _ui->editOffsetY->text().toDouble();
	offset(2) = _ui->editOffsetZ->text().toDouble();

	gp_Ax2 axis(gp_Pnt(0, 0, 0), gp_Dir(0, 1, 0));
	Standard_Real radius = _ui->editCylinderD->text().toDouble() / 2;
	Standard_Real height = _ui->editCylinderH->text().toDouble();
	TopoDS_Shape cylShape = BRepPrimAPI_MakeCylinder(axis, radius, height).Shape();
	Handle(AIS_Shape_WithFrame) cyl = new AIS_Shape_WithFrame(cylShape);
	_wpCylinder = std::make_shared<Model>(std::list<Handle(AIS_Shape_WithFrame)>{ cyl });
	_wpCylinder->setColor(50, 100, 150);
	_wpCylinder->setTransparency();
	_wpCylinder->displayInOcc(_ui->widOcc);
	_wpCylinder->setMatrix({ JMath::makeTranslate(offset) });
	if (_useCylinder)
		_wpCylinder->setVisible(_ui->widOcc, true);
	else
		_wpCylinder->setVisible(_ui->widOcc, false);

	_ui->widOcc->update();
}

void AppWidget::onConeShapeEditingFinished()
{
	//圆柱工件
	if (_wpCone)
		_wpCone->removeFromOcc(_ui->widOcc);

	Eigen::Vector3d offset(0, 0, 0);
	offset(0) = _ui->editOffsetX->text().toDouble();
	offset(1) = _ui->editOffsetY->text().toDouble();
	offset(2) = _ui->editOffsetZ->text().toDouble();

	//圆锥工件
	gp_Ax2 axis = gp_Ax2(gp_Pnt(0, 0, 0), gp_Dir(0, 0, 1));
	Standard_Real rBottom = _ui->editConeBottomD->text().toDouble() / 2;
	Standard_Real rTop = _ui->editConeTopD->text().toDouble() / 2;
	Standard_Real coneHeight = _ui->editConeH->text().toDouble();
	TopoDS_Shape coneShape = BRepPrimAPI_MakeCone(axis, rBottom, rTop, coneHeight).Shape();
	Handle(AIS_Shape_WithFrame) cone = new AIS_Shape_WithFrame(coneShape);
	_wpCone = std::make_shared<Model>(std::list<Handle(AIS_Shape_WithFrame)>{ cone });
	_wpCone->setColor(50, 100, 150);
	_wpCone->setTransparency();
	_wpCone->displayInOcc(_ui->widOcc);
	_wpCone->setMatrix({ JMath::makeTranslate(offset) });
	if (_useCylinder)
		_wpCone->setVisible(_ui->widOcc, false);
	else
		_wpCone->setVisible(_ui->widOcc, true);

	_ui->widOcc->update();
}

void AppWidget::onBtnStartSim()
{
	std::array<double, 6> toolPoseStart = { 0 };
	std::array<double, 3> externalStart = { 0 };
	double offsetX = _ui->editOffsetX->text().toDouble();
	if (_useCylinder)
	{
		if (offsetX > 0)
			toolPoseStart[0] = offsetX - _ui->editCylinderD->text().toDouble() / 2;
		else
			toolPoseStart[0] = offsetX + _ui->editCylinderD->text().toDouble() / 2;
		toolPoseStart[1] = _ui->editOffsetY->text().toDouble();
		toolPoseStart[2] = _ui->editOffsetZ->text().toDouble();
		toolPoseStart[3] = 0;
		toolPoseStart[4] = 0;
		if (offsetX < 0)
			toolPoseStart[5] = 180;
		externalStart[1] = _ui->editOffsetY->text().toDouble() + 1000;
	}
	else
	{
		double coneAngle = atan((_ui->editConeBottomD->text().toDouble() / 2 - _ui->editConeTopD->text().toDouble() / 2) / _ui->editConeH->text().toDouble());
		if (offsetX > 0)
			toolPoseStart[0] = offsetX - _ui->editConeBottomD->text().toDouble() / 2;
		else
			toolPoseStart[0] = offsetX + _ui->editConeBottomD->text().toDouble() / 2;
		toolPoseStart[1] = _ui->editOffsetY->text().toDouble();
		toolPoseStart[2] = _ui->editOffsetZ->text().toDouble();
		toolPoseStart[3] = 0;
		if (offsetX > 0)
			toolPoseStart[4] = JMath::R2D(coneAngle);
		else
		{
			toolPoseStart[4] = -JMath::R2D(coneAngle);
			toolPoseStart[5] = 180;
		}
		externalStart[1] = _ui->editOffsetY->text().toDouble() + 1000;
	}
	
	std::array<double, 6> toolPoseEnd = toolPoseStart;
	std::array<double, 3> externalEnd = externalStart;
	if (_useCylinder)
	{
		toolPoseEnd[1] += _ui->editCylinderH->text().toDouble();
		//externalEnd[1] = WP_CYLINDER_END_Y;
		externalEnd[1] = externalStart[1] + _ui->editCylinderH->text().toDouble() - 1500;
	}
	else
	{
		if (offsetX > 0)
			toolPoseEnd[0] = offsetX - _ui->editConeTopD->text().toDouble() / 2;
		else
			toolPoseEnd[0] = offsetX + _ui->editConeTopD->text().toDouble() / 2;
		toolPoseEnd[2] = _ui->editOffsetZ->text().toDouble() + _ui->editConeH->text().toDouble();
	}

	double sprayVel = _ui->editYVel->text().toDouble();

	std::string script;
	script += std::format(
		"moveJ(\"default\", {{{}, 0, 0, 0, 0, 0, 0, 0, 0}}, 100, 100, -1)\n",
		offsetX > 0 ? 0 : -150
	);

	script += std::format(
		"moveP(\"default\", {{{}, {}, {}, {}, {}, {}, {}, {}, {}}}, 100, 100, -1, \"sprayer\")\n",
		toolPoseStart[0], toolPoseStart[1], toolPoseStart[2],
		toolPoseStart[3], toolPoseStart[4], toolPoseStart[5],
		externalStart[0], externalStart[1], externalStart[2]
	);
	script += std::format(
		"moveL(\"default\", {{{}, {}, {}, {}, {}, {}, {}, {}, {}}}, {}, 100, -1, \"sprayer\")",
		toolPoseEnd[0], toolPoseEnd[1], toolPoseEnd[2],
		toolPoseEnd[3], toolPoseEnd[4], toolPoseEnd[5],
		externalEnd[0], externalEnd[1], externalEnd[2],
		sprayVel
	);

	_ui->tbScript->setPlainText(QString::fromStdString(script));

	int err = _client.clearErrors();
	if (err)
	{
		_ui->info->error(QStringLiteral("clearErrors失败, 错误码: %1").arg(err));
		return;
	}

	err = _client.setMode(JMC::OperationMode::MODE_AUTO);
	if (err)
	{
		_ui->info->error(QStringLiteral("setMode失败, 错误码: %1").arg(err));
		return;
	}
	JMC::OperationMode mode;
	int attempts = 0;
	while (true)
	{
		err = _client.getMode(mode);
		if (err)
		{
			_ui->info->error(QStringLiteral("getMode失败, 错误码: %1").arg(err));
			return;
		}
		if (mode == JMC::OperationMode::MODE_AUTO || attempts > 10000)
			break;
		QApplication::processEvents();
		attempts++;
	}
	if (mode != JMC::OperationMode::MODE_AUTO)
	{
		_ui->info->error(QStringLiteral("无法切换到自动模式"));
		return;
	}
	
	//使能
	err = _client.setEnable(true);
	if (err)
	{
		_ui->info->error(QStringLiteral("setEnable失败, 错误码: %1").arg(err));
		return;
	}
	bool enable = false;
	while (true)
	{
		_client.getEnable(enable);
		if (err)
		{
			_ui->info->error(QStringLiteral("getEnable失败, 错误码: %1").arg(err));
			return;
		}
		if (enable || attempts > 10000)
			break;
		QApplication::processEvents();
		attempts++;
	}
	if (!enable)
	{
		_ui->info->error(QStringLiteral("无法切换到使能状态"));
		return;
	}

	// 写入 ./scripts/main.lua
	std::filesystem::create_directories("scripts");
	const std::string path = "scripts/main.lua";
	std::ofstream ofs(path, std::ios::trunc);
	if (!ofs)
	{
		_ui->info->error(QStringLiteral("无法打开脚本文件: %1").arg(QString::fromStdString(path)));
		return;
	}
	ofs << script;
	ofs.close();
	err = _client.setAutoRunScript("main.lua");
	if (err)
	{
		_ui->info->error(QStringLiteral("setAutoRunScript失败, 错误码: %1").arg(err));
		return;
	}
	std::this_thread::sleep_for(std::chrono::milliseconds(100));
	err = _client.runScript();
	if (err)
	{
		_ui->info->error(QStringLiteral("runScript失败, 错误码: %1").arg(err));
		return;
	}
}

void AppWidget::onBtnStopSim()
{
	int err = _client.setMode(JMC::OperationMode::MODE_STOP);
	if (err)
	{
		_ui->info->error(QStringLiteral("setMode失败, 错误码: %1").arg(err));
		return;
	}
}

void AppWidget::onBtnSwitchFixture()
{
	if (_useCylinder)
	{
		_fixCylinder->setVisible(_ui->widOcc, false);
		_wpCylinder->setVisible(_ui->widOcc, false);
		_fixCone->setVisible(_ui->widOcc, true);
		_wpCone->setVisible(_ui->widOcc, true);
	}
	else
	{
		_fixCylinder->setVisible(_ui->widOcc, true);
		_wpCylinder->setVisible(_ui->widOcc, true);
		_fixCone->setVisible(_ui->widOcc, false);
		_wpCone->setVisible(_ui->widOcc, false);
	}
	_useCylinder = !_useCylinder;

	_ui->widOcc->update();
}

void AppWidget::onSelectionChanged()
{
	std::vector<Handle(AIS_InteractiveObject)> shapes;
	std::vector<TopoDS_Shape> selectedTopos = _ui->widOcc->querySelection(shapes);
	if (selectedTopos.size() != 1)
		return;
	TopoDS_Shape targetTopo = selectedTopos[0];
	if (targetTopo.ShapeType() != TopAbs_VERTEX)
		return;
	TopoDS_Vertex vertex = TopoDS::Vertex(targetTopo);
	gp_Pnt p = BRep_Tool::Pnt(vertex);
	_ui->info->log(QStringLiteral("选中顶点坐标: (%1, %2, %3)").arg(p.X()).arg(p.Y()).arg(p.Z()));
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
	_matVec.insert(_matVec.begin(), Eigen::Matrix4d::Identity());

	QMetaObject::invokeMethod(this, "onJointPositionsUpdatedGUI", Qt::QueuedConnection);
}

void AppWidget::onScriptStatusUpdated(const JMC::ScriptStatus& scriptStatus)
{
	QWidget* wids[] = { _ui->btnSwitchFixture,  _ui->btnStartSim , _ui->editOffsetX, _ui->editOffsetY, _ui->editOffsetZ,
	_ui->editSprayDist, _ui->editWpVel, _ui->editYVel, _ui->editCylinderD, _ui->editCylinderH, _ui->editConeTopD, _ui->editConeBottomD,
	_ui->editConeH };

	bool enable = (scriptStatus != JMC::ScriptStatus::SCRIPT_RUNNING);

	for (const auto& wid : wids)
	{
		QMetaObject::invokeMethod(wid, "setEnabled", Q_ARG(bool, enable));
	}

	if (scriptStatus == JMC::ScriptStatus::SCRIPT_RUNNING)
	{
		QMetaObject::invokeMethod(this, "startWpRotate");
	}
	else
	{
		QMetaObject::invokeMethod(this, "stopWpRotate");
	}
}

void AppWidget::onErrorUpdated(const std::string& error)
{
	if (_error == error)
		return;
	_error = error;
	QStringList listStr = QString::fromStdString(error).split("\n");
	for (const auto& str : listStr)
	{
		_ui->info->error(str);
	}
}

void AppWidget::startWpRotate()
{
	double rotateRPM = _ui->editWpVel->text().toDouble();

	QTimer* timer = this->findChild<QTimer*>("wpRotateTimer");
	if (timer)
		return;

	timer = new QTimer(this);
	connect(timer, &QTimer::timeout, this, [this, rotateRPM]() {
		std::vector<Eigen::Matrix4d> mats;
		if (_useCylinder && _wpCylinder)
		{
			_wpCylinder->getMatrix(mats);
			mats[0] *= JMath::makeRotate(Eigen::Vector3d(0, 1, 0), JMath::D2R(rotateRPM / 60.0 * 360.0 * 0.1));
			_wpCylinder->setMatrix(mats);
		}
		else if (!_useCylinder && _wpCone)
		{
			_wpCone->getMatrix(mats);
			mats[0] *= JMath::makeRotate(Eigen::Vector3d(0, 0, 1), JMath::D2R(rotateRPM / 60.0 * 360.0 * 0.1));
			_wpCone->setMatrix(mats);
		}
		_ui->widOcc->update();
	});
	timer->setObjectName("wpRotateTimer");
	timer->setInterval(100);
	timer->start();
}

void AppWidget::stopWpRotate()
{
	QTimer* timer = this->findChild<QTimer*>("wpRotateTimer");
	if (timer)
	{
		timer->stop();
		delete timer;
	}
}
