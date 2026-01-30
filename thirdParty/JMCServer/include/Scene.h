#pragma once
#include "sigslot/signal.hpp"

#include <memory>
#include <array>
#include <map>

#if defined(_WIN32) || defined(__CYGWIN__)
#ifdef JMCSERVER_LIB
#define JMCSERVERAPI_EXPORT __declspec(dllexport)
#else
#define JMCSERVERAPI_EXPORT __declspec(dllimport)
#endif
#else
#if defined(__GNUC__) && __GNUC__ >= 4
#define JMCSERVERAPI_EXPORT __attribute__((visibility("default")))
#else
#define JMCSERVERAPI_EXPORT
#endif
#endif

#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif // !M_PI

#define SCENE JMC::Scene::getInstance()

namespace JMC
{
	class ScenePrivate;
	class JMCSERVERAPI_EXPORT Scene
	{
	public:
		struct MDH
		{
			double startTheta = 0; // rad
			double alpha = 0;
			double a = 0;
			double d = 0;
			double min = -std::numeric_limits<double>::infinity();
			double max = std::numeric_limits<double>::infinity();
			bool isTranslate = false;
			bool isBindWP = false; // 是否绑定到工件(五轴系统用于区分链)
		};
		struct TPDO
		{
			std::vector<uint32_t> status;
			std::vector<double> actPos;
			std::vector<double> actVel;
			uint32_t DI = 0;
			uint64_t clock = 0; // us
		};

	private:
		explicit Scene() noexcept;

	public:
		static Scene &getInstance();

	public:
		void setLogLevel(int level);
		std::vector<std::string> luaRegFunctions() const;
		std::string scenePrettyString() const;
		bool run(int timeoutSec = 10) const;
		void shut();
		void setIp(const std::string &ip);
		std::string ip() const;
		void etcSlotCount(int &realSlot, int &virtualSlot) const;
		double etcIntervalSec() const;
		std::string etcState() const;
		std::string graph();
		std::string runScript(const std::string &scrip);
		std::string ternimateScript();
		std::string switchManual();
		std::string switchAuto();
		std::string switchStop();
		std::string addR6(const std::string &parentNode, const std::string &name, const std::array<Scene::MDH, 6> &mdhs, int startEtcSlot);
		std::string setEthercatSlotLimits(int slot, double minPos, double maxPos, double maxVel);
		std::string queryEtcTPDO(TPDO &tpdo);
		std::vector<TPDO> fetchScopeData();

	public:
		sigslot::signal<const std::string &, const std::string &> stateTransfered;
		sigslot::signal<int> scriptLineUpdated; // line
		sigslot::signal<> scriptStarted;
		sigslot::signal<const std::string &> scriptFinished;										 // errorMsg
		sigslot::signal<const std::string &> errorUpdated;											 // errors
		sigslot::signal<const std::string &, const std::string &> deviceAdded;						 // parentNode, deviceName
		sigslot::signal<const TPDO &> pdoUpdated;													 // only for ethercat master simMode
		sigslot::signal<const std::string &, const std::array<double, 6> &> toolPoseChangedPDOCycle; // only for ethercat master simMode

	private:
		std::shared_ptr<ScenePrivate> _prv;
	};
}