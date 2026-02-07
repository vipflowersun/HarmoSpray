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
	void onEditOffetEditingFinished();
	void onEditSprayDistEditingFinished();
	void onBtnStartSim();
	void onBtnStopSim();
	void onBtnSwitchFixture();
	void startWpRotate();
	void stopWpRotate();
private:
	void onJointPositionsUpdated(const std::map<std::string, std::vector<double>>& positions, uint64_t clock);
	void onScriptStatusUpdated(const JMC::ScriptStatus& scriptStatus);
	void onErrorUpdated(const std::string& error);
private:
	std::shared_ptr<Ui::AppWid> _ui;
	JMC::Client _client;
	std::shared_ptr<Model> _model;
	std::shared_ptr<Model> _fixCylinder;
	std::shared_ptr<Model> _fixCone;
	std::shared_ptr<Model> _wpCylinder;
	std::shared_ptr<Model> _wpCone;
	Handle(AIS_Trihedron) _coordShape;
	Handle(AIS_Trihedron) _coordShapeTool;
	std::vector<Eigen::Matrix4d> _matVec;
	Kinematic _kinematic;
	bool _useCylinder = true;
	std::string _error;
};
