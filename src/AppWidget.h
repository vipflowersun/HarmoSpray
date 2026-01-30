#pragma once
#include <QWidget>
#include <memory>
#include "ui_AppUi.h"
#include <optional>
#include <Eigen/Dense>

#include "AIS_Trihedron.hxx"

#include "JMCClient.h"
#include "Model.h"
#include "KinForward.h"

class AppWidget : public QWidget
{
	Q_OBJECT
public:
	explicit AppWidget(QWidget *parent = nullptr);

protected:
	virtual void showEvent(QShowEvent *event) override;
	virtual void closeEvent(QCloseEvent *event) override;

private slots:
	void onJointPositionsUpdatedGUI();
	bool loadScene();
private:
	void onJointPositionsUpdated(const std::map<std::string, std::vector<double>>& positions, uint64_t clock);

private:
	std::shared_ptr<Ui::AppWid> _ui;
	JMC::Client _client;
	std::shared_ptr<Model> _model;
	Handle(AIS_Trihedron) _coordShape;
	Handle(AIS_Trihedron) _coordShapeTool;
	std::array<double, 6> _toolPoseToEndFlange = {100.5, 0, 637, 0, M_PI_4, 0};
	std::vector<Eigen::Matrix4d> _matVec;
	Kinematic _kinematic;
};
