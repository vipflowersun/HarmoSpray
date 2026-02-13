#pragma once
#include <array>
#include <list>
#include <string>
#include <map>
#include <vector>
#include <functional>
#include <stdint.h>
class ClientPrv;

#if defined(_WIN32) || defined(__CYGWIN__)
#ifdef JMCCLIENT_LIB
#define JMCCLIENTAPI_EXPORT __declspec(dllexport)
#else
#define JMCCLIENTAPI_EXPORT __declspec(dllimport)
#endif
#else
#if defined(__GNUC__) && __GNUC__ >= 4
#define JMCCLIENTAPI_EXPORT __attribute__((visibility("default")))
#else
#define JMCCLIENTAPI_EXPORT
#endif
#endif

// Error codes
#define ERR_SUCCEED (0)
#define ERR_OFFSET (100)
// 连接服务器失败
#define ERR_CONNECT_SERVER_FAILED (ERR_OFFSET + 1)
// 未连接到服务器
#define ERR_SERVER_NOT_CONNECTED (ERR_OFFSET + 2)
// 不合法的输入
#define ERR_INVALID_INPUT (ERR_OFFSET + 3)
// 等待响应超时
#define ERR_WAIT_RESPONSE_TIMEOUT (ERR_OFFSET + 4)
// 响应格式无效
#define ERR_RESPONSE_FORMAT_INVALID (ERR_OFFSET + 5)
// FTP通讯失败
#define ERR_FTP_COMMUNICATION_FAILED (ERR_OFFSET + 6)
// 文件操作失败
#define ERR_FILE_OPERATION_FAILED (ERR_OFFSET + 7)

namespace JMC
{
    enum class DeviceType
    {
        DEVICE_UNKNOWN = -1,
        DEVICE_XYZ = 0,
        DEVICE_ROBOT,
        DEVICE_ROBOTEA3,
        DEVICE_WELDER,
        DEVICE_CAMERA,
        DEVICE_SCANNER,
        DEVICE_MAX,
    };
    enum class OperationMode
    {
        MODE_STOP = 0,
        MODE_MANUAL,
        MODE_AUTO,
        MODE_ERROR, // read only
    };
    enum class ScriptStatus
    {
        SCRIPT_RUNNING = 0,
        SCRIPT_PAUSED,
        SCRIPT_FINISHED,
        SCRIPT_ERROR
    };
    enum class PeripheralType
    {
        PERIPHERAL_WELDER = 0,  // 焊机
        PERIPHERAL_CAMERA,      // 2D相机
        PERIPHERAL_SCANNER,     // 3D相机
        PERIPHERAL_SEAMTRACKER, // 缝隙跟踪仪
        PERIPHERAL_GRIPPER,     // 夹爪
        PERIPHERAL_BELT,        // 传送带
        PERIPHERAL_VACUUM,      // 吸盘
    };
    struct FileInfo
    {
        std::string name;
        bool isDir{false};
        size_t size{0};
        std::string lastModifyTime;
    };
    class JMCCLIENTAPI_EXPORT Client
    {
    public:
        explicit Client() noexcept;
        virtual ~Client();
        Client(const Client &) = delete;
        Client(Client &&) = delete;
        Client &operator=(const Client &) = delete;
        Client &operator=(Client &&) = delete;

    public:
        /// @brief 设置超时时间
        /// @param timeoutMs
        /// @return 错误码
        int setTimeout(int timeoutMs);
        /// @brief 连接到服务器
        /// @param ip 服务器IP地址
		/// @param flag 连接标志 bit0-命令服务器 bit1-广播服务器 bit2-事件通知服务器
        /// @param port 服务器端口
        /// @return 错误码
        int connect(const std::string &ip, int flag = 0b111, int port = 8888);
        /// @brief 关闭连接
        /// @return 错误码
        int close();
        /// @brief 注册连接成功回调
        /// @param callback
        /// @return 错误码
        int onConnected(std::function<void()> callback);
        /// @brief 注册断开连接回调
        /// @param callback
        /// @return 错误码
        int onDisconnected(std::function<void()> callback);
        /// @brief 注册脚本打印回调
        /// @param callback
        /// @return 错误码
        int onScriptPrintCalled(std::function<void(const std::string &content)> callback);
        /// @brief 设置订阅标志
        /// @param flag 订阅标志
        /// @return 错误码
        int setSubscriptionFlag(uint32_t flag);
        /// @brief 注册状态更新回调
        /// @param callback
        /// @return 错误码
        int onStateUpdated(std::function<void(const std::string &state)> callback);
        /// @brief 注册错误更新回调
        /// @param callback
        /// @return 错误码
        int onErrorUpdated(std::function<void(const std::string &error)> callback);
        /// @brief 注册使能状态更新回调
        /// @param callback
        /// @return 错误码
        int onEnableUpdated(std::function<void(bool enable)> callback);
        /// @brief 注册紧急标志更新回调
        /// @param callback
        /// @return 错误码
        int onEmergencyFlagUpdated(std::function<void(bool emergencyFlag)> callback);
        /// @brief 注册脚本名称更新回调
        /// @param callback
        /// @return 错误码
        int onScriptNameUpdated(std::function<void(const std::string &scriptName)> callback);
        /// @brief 注册脚本状态更新回调
        /// @param callback
        /// @return 错误码
        int onScriptStatusUpdated(std::function<void(const JMC::ScriptStatus &scriptStatus)> callback);
        /// @brief 注册脚本当前行号更新回调
        /// @param callback
        /// @return 错误码
        int onScriptCurrentLineUpdated(std::function<void(int currentLine)> callback);
        /// @brief 注册运动状态更新回调
        /// @param callback
        /// @return 错误码
        int onMovingStatusUpdated(std::function<void(const std::map<std::string, int> &status)> callback);
        /// @brief 注册关节位置更新回调
        /// @param callback
        /// @return 错误码
        int onJointPositionsUpdated(std::function<void(const std::map<std::string, std::vector<double>> &positions, uint64_t clock)> callback);
        /// @brief 注册关节速度更新回调
        /// @param callback
        /// @return 错误码
        int onJointVelocitiesUpdated(std::function<void(const std::map<std::string, std::vector<double>> &velocities, uint64_t clock)> callback);
        /// @brief 注册关节力矩更新回调
        /// @param callback
        /// @return 错误码
        int onJointTorquesUpdated(std::function<void(const std::map<std::string, std::vector<double>> &torques, uint64_t clock)> callback);
        /// @brief 注册工具末端位姿更新回调
        /// @param callback
        /// @return 错误码
        int onTcpPosesUpdated(std::function<void(const std::map<std::string, std::array<double, 6>> &poses, uint64_t clock)> callback);
        /// @brief 注册设备连杆位姿更新回调
        /// @param callback
        /// @return 错误码
        int onLinkPosesUpdated(std::function<void(const std::map<std::string, std::vector<std::array<double, 16>>> &linkPoses, uint64_t clock)> callback);
        /// @brief 注册IO状态更新回调
        /// @param callback
        /// @return 错误码
        int onIOStatesUpdated(std::function<void(uint32_t ioStates)> callback);
        /// @brief 注册模拟量更新回调
        /// @param callback
        /// @return 错误码
        int onAnalogValuesUpdated(std::function<void(const std::vector<double> &analogValues)> callback);
        /// @brief 注册外设状态更新回调
        /// @param callback
        /// @return 错误码
        int onPeripheralUpdated(std::function<void(const std::map<std::string, std::map<std::string, std::string>> &peripheralStates)> callback);
		/// @brief 注册点云数据更新回调
        /// @param callback
        /// @return 错误码
		int onPointCloudDataUpdated(std::function<void(const std::string& deviceName)> callback);
        /// @brief 清空场景设备树
        /// @return 错误码
        int clearScene();
        /// @brief 在场景中增加一个工业机械臂设备
        /// @param parentDevice 父节点设备名称，空字符串表示添加到根节点
        /// @param name 设备名称
        /// @param ethercatStartSlot EtherCAT起始槽位
        /// @return 错误码
        int addIndustrialManipulator(const std::string &name, int ethercatStartSlot, const std::string &parentDevice = "");
        /// @brief 在场景中增加一个带XYZ外部轴工业机械臂设备
        /// @param parentDevice 父节点设备名称，空字符串表示添加到根节点
        /// @param name 设备名称
		/// @param xyzEnableMask xyz轴使能掩码，位0表示X轴使能，位1表示Y轴使能，位2表示Z轴使能
        /// @param ethercatStartSlot EtherCAT起始槽位
        /// @return 错误码
        int addIndustrialManipulatorEA3(const std::string &name, int xyzEnableMask, int ethercatStartSlot, const std::string &parentDevice = "");
        /// @brief 在场景中增加一个协作机械臂设备
        /// @param parentDevice 父节点设备名称，空字符串表示添加到根节点
        /// @param name 设备名称
        /// @param ethercatStartSlot EtherCAT起始槽位
        /// @return 错误码
        int addCollaborativeManipulator(const std::string &name, int ethercatStartSlot, const std::string &parentDevice = "");
        /// @brief 在场景中增加一个三坐标设备
        /// @param parentDevice 父节点设备名称，空字符串表示添加到根节点
        /// @param name 设备名称
        /// @param xyzEnableMask xyz轴使能掩码，位0表示X轴使能，位1表示Y轴使能，位2表示Z轴使能
        /// @param ethercatStartSlot EtherCAT起始槽位
        /// @return 错误码
        int addTranslatorXYZ(const std::string &name, int xyzEnableMask, int ethercatStartSlot, const std::string &parentDevice = "");
		/// @brief 在场景中增加一个虚拟焊机设备
        /// @param parentDevice 父节点设备名称，空字符串表示添加到根节点
        /// @param name 设备名称
        /// @return 错误码
        int addWelderVirtual(const std::string& name, const std::string& parentDevice = "");
        /// @brief 在场景中增加一个Ethercat焊机设备
        /// @param parentDevice 父节点设备名称，空字符串表示添加到根节点
        /// @param name 设备名称
        /// @param vendor 设备厂商标识
		/// @param ethercaSlot slot号
        /// @return 错误码
        int addWelderEthercat(const std::string& name, const std::string& vendor, int ethercaSlot, const std::string& parentDevice = "");
        /// @brief 在场景中增加一个扫描设备
        /// @param name 设备名称
        /// @param parentDevice 父节点设备名称，空字符串表示添加到根节点
        /// @param vendor 设备厂商标识
        /// @return 错误码
        int addScanner(const std::string& name, const std::string& vendor, const std::string& parentDevice = "");
        /// @brief 在场景中移除一个设备
        /// @param name 设备名称
        /// @note 移除设备会同时移除其全部子树设备
        /// @return 错误码
        int removeDevice(const std::string &name);
        /// @brief 获取场景图的JSON描述
        /// @param sceneGraphJson 场景图的JSON字符串
        /// @return 错误码
        int getSceneGraph(std::string &sceneGraphJson);
        /// @brief 获取场景中所有设备名称
        /// @param devices 设备名称与类型的键值对列表
        /// @return 错误码
        int getSceneDevices(std::vector<std::pair<std::string, DeviceType>> & devices);
		/// @brief 下载控制器的配置文件到本地
        /// @param filePath 本地文件路径
		/// @note 文件格式为jcf
        /// @return 错误码
        int downloadConfigFile(const std::string& filePath);
        /// @brief 上传本地的控制器的配置文件到控制器
        /// @param filePath 本地文件路径
        /// @note 文件格式为jcf
        /// @note 操作后需重启控制器生效
        /// @return 错误码
		int uploadConfigFile(const std::string& filePath);
        /// @brief 重启控制器
        /// @return 错误码
        int restartController();
        /// @brief 关闭控制器
        /// @return 错误码
		int poweroffController();
        /// @brief 获取设备所有工具
        /// @param device 设备名称
        /// @param tools 工具名称与安装位姿的键值对
        /// @return 错误码
        int getTools(const std::string &device, std::map<std::string, std::array<double, 6>> &tools);
        /// @brief 在设备上增加一个工具
        /// @param device 设备名称
        /// @param toolName 工具名称
        /// @param pose 安装位姿
        /// @return 错误码
        int addTool(const std::string &device, const std::string &toolName, const std::array<double, 6> &pose);
        /// @brief 从设备上移除一个工具
        /// @param device 设备名称
        /// @param toolName 工具名称
        /// @return 错误码
        int removeTool(const std::string &device, const std::string &toolName);
        /// @brief 设置工具的安装位姿
        /// @param device 设备名称
        /// @param toolName 工具名称
        /// @param pose 安装位姿
        /// @return 错误码
        int setToolPose(const std::string &device, const std::string &toolName, const std::array<double, 6> &pose);
        /// @brief 设置设备的当前工具
        /// @param device 设备名称
        /// @param toolName 工具名称
        /// @return 错误码
        int setCurrentTool(const std::string &device, const std::string &toolName);
        /// @brief 获取设备的当前工具
        /// @param device 设备名称
        /// @param toolName 工具名称
        /// @return 错误码
        int getCurrentTool(const std::string &device, std::string &toolName);
        /// @brief 获取设备所有工件
        /// @param device 设备名称
        /// @param wcses 工件名称与工件位姿的键值对
        /// @return 错误码
        int getWcses(const std::string &device, std::map<std::string, std::array<double, 6>> &wcses);
        /// @brief 在设备上增加一个工件
        /// @param device 设备名称
        /// @param wcsName 工件名称
        /// @param pose 安装位姿
        /// @return 错误码
        int addWcs(const std::string &device, const std::string &wcsName, const std::array<double, 6> &pose);
        /// @brief 从设备上移除一个工件
        /// @param device 设备名称
        /// @param wcsName 工件名称
        /// @return 错误码
        int removeWcs(const std::string &device, const std::string &wcsName);
        /// @brief 设置工件的安装位姿
        /// @param device 设备名称
        /// @param wcsName 工件名称
        /// @param pose 安装位姿
        /// @return 错误码
        int setWcsPose(const std::string &device, const std::string &wcsName, const std::array<double, 6> &pose);
        /// @brief 设置设备的当前工件
        /// @param device 设备名称
        /// @param wcsName 工件名称
        /// @return 错误码
        int setCurrentWcs(const std::string &device, const std::string &wcsName);
        /// @brief 获取设备的当前工件
        /// @param device 设备名称
        /// @param wcsName 工件名称
        /// @return 错误码
        int getCurrentWcs(const std::string &device, std::string &wcsName);
        /// @brief 获取设备所有示教点
        /// @param device 设备名称
        /// @param teachPoints 示教点名称与示教点位姿的键值对
        /// @note teachPoints存储格式为N维为关节位置(与设备关节数对应)
        /// @return 错误码
        int getTeachPoints(const std::string &device, std::map<std::string, std::vector<double>> &teachPoints);
        /// @brief 在设备上增加一个示教点
        /// @param device 设备名称
        /// @param teachPointName 示教点名称
        /// @param teachPoint 示教点关节位置
        /// @note teachPoints存储格式为N维为关节位置(与设备关节数对应)
        /// @return 错误码
        int addTeachPoint(const std::string &device, const std::string &teachPointName, const std::vector<double> &teachPoint);
        /// @brief 从设备上移除一个示教点
        /// @param device 设备名称
        /// @param teachPointName 示教点名称
        /// @return 错误码
        int removeTeachPoint(const std::string &device, const std::string &teachPointName);
        /// @brief 修改示教点的关节位置
        /// @param device 设备名称
        /// @param teachPointName 示教点名称
        /// @param teachPoint 示教点关节位置
        /// @note teachPoints存储格式为N维为关节位置(与设备关节数对应)
        /// @return 错误码
        int setTeachPoint(const std::string &device, const std::string &teachPointName, const std::vector<double> &teachPoint);
        /// @brief 修改示教点的名称
        /// @param device 设备名称
        /// @param teachPointNameOld 示教点旧名称
        /// @param teachPointNew 示教点新名称
        /// @return 错误码
        int setTeachPointName(const std::string& device, const std::string& teachPointNameOld, const std::string& teachPointNameNew);
        /// @brief 计算运动学正解
        /// @param device 设备名称
        /// @param joints 输入关节角度
        /// @param poseSolution 输出位姿
        /// @param tcp 工具名称 默认("")为当前工作坐标系
        /// @param wcs 工件名称 默认("")为当前工件
        /// @return 错误码
        int forward(const std::string &device,
                    const std::vector<double> &joints,
                    std::array<double, 6> &poseSolution,
                    const std::string &tool = "",
                    const std::string &wcs = "");
        /// @brief 计算运动学正解2
        /// @param device 设备名称
        /// @param joints 输入关节角度
        /// @param linksPose 输出全部连杆位姿
        /// @return 错误码
        int forward2(const std::string &device,
                     const std::vector<double> &joints,
                     std::vector<std::array<double, 6>> &linksPose);
        /// @brief 计算运动学逆解
        /// @param device 设备名称
        /// @param pose 输入位姿
        /// @param jointsSolutions 输出关节
        /// @param tcp 工具名称 默认("")为当前工作坐标系
        /// @param wcs 工件名称 默认("")为当前工件
        /// @return 错误码
        int inverse(const std::string &device,
                    const std::array<double, 6> &pose,
                    std::vector<std::vector<double>> &jointsSolutions,
                    const std::string &tool = "",
                    const std::string &wcs = "");
        /// @brief 关节运动
        /// @param device 设备名称
        /// @param joints 目标关节位置 大小应与设备关节数一致
        /// @param vel  速度百分比(0,100]
        /// @param acc  加速度百分比(0,100]
        /// @param blendT 平滑时间[0-1000]，0为不平滑，到此位置停止 [ms]
        /// @return 错误码
        int moveJ(const std::string &device,
                  const std::vector<double> &joints,
                  double vel,
                  double acc,
                  double blendT);
        /// @brief 关节运动
        /// @param device 设备名称
        /// @param teachPointName 示教点名称
        /// @param vel  速度百分比(0,100]
        /// @param acc  加速度百分比(0,100]
        /// @param blendT 平滑时间[0-1000]，0为不平滑，到此位置停止 [ms]
        /// @return 错误码
        int moveJ(const std::string &device,
                  const std::string &teachPointName,
                  double vel,
                  double acc,
                  double blendT);
        /// @brief 位姿运动
        /// @param device 设备名称
        /// @param pose 目标位姿 [x,y,z,rx,ry,rz] or [x,y,z,rx,ry,rz,ex,ey,ez]
        /// @param vel 速度百分比(0,100]
        /// @param acc 加速度百分比(0,100]
        /// @param blendT 平滑时间[0-1000]，0为不平滑，到此位置停止 [ms]
        /// @param enableEA 当目标位置格式为[x,y,z,rx,ry,rz]指定是否使能外部轴联动
        /// @param tcp 工具名称 默认("")为末端法兰坐标系
        /// @param wcs 工件名称 默认("")为基座 
        /// @return 错误码
        int moveP(const std::string &device,
                  const std::vector<double> &pose,
                  double vel,
                  double acc,
                  double blendT,
                  bool enableEA,
                  const std::string &tcp = "",
                  const std::string &wcs = "");
        /// @brief 直线运动
        /// @param device 设备名称
        /// @param pose 目标位姿 [x,y,z,rx,ry,rz] or [x,y,z,rx,ry,rz,ex,ey,ez]
        /// @param vel 速度百分比(0,100]
        /// @param acc 加速度百分比(0,100]
        /// @param blendR 平滑半径[0-1000]，0为不平滑，到此位置停止 [mm]
		/// @param enableEA 当目标位置格式为[x,y,z,rx,ry,rz]指定是否使能外部轴联动
        /// @param tcp 工具名称 默认("")为末端法兰坐标系
        /// @param wcs 工件名称 默认("")为基座
        /// @param process 工艺名称 默认("")为不配置工艺
        /// @return 错误码
        int moveL(const std::string &device,
                  const std::vector<double> &pose,
                  double vel,
                  double acc,
                  double blendR,
                  bool enableEA,
                  const std::string &tcp = "",
                  const std::string &wcs = "",
                  const std::string &process = "");
        /// @brief 直线运动
        /// @param device 设备名称
        /// @param teachPointName 示教点名称
        /// @param vel 速度百分比(0,100]
        /// @param acc 加速度百分比(0,100]
        /// @param blendR 平滑半径[0-1000]，0为不平滑，到此位置停止 [mm]
        /// @param process 工艺名称 默认("")为不配置工艺
        /// @return 错误码
        int moveL(const std::string &device,
                  const std::string &teachPointName,
                  double vel,
                  double acc,
                  double blendR,
                  const std::string& process = "");
        /// @brief 圆弧运动
        /// @param device 设备名称
        /// @param viaPose 途经点位姿
        /// @param toPose 目标位姿
        /// @param vel 速度百分比(0,100]
        /// @param acc 加速度百分比(0,100]
        /// @param circle 是否为整圆运动
        /// @param blendR 平滑半径[0-1000]，0为不平滑，到此位置停止 [mm]
        /// @param joints 目标关节位置 (可选，用于多逆解系统消除歧义) 大小应与设备关节数一致
        /// @param tcp 工具名称 默认("")为末端法兰坐标系
        /// @param wcs 工件名称 默认("")为基座
        /// @return 错误码
        int moveC(const std::string &device,
                  const std::array<double, 6> &viaPose,
                  const std::array<double, 6> &toPose,
                  double vel,
                  double acc,
                  bool circle,
                  double blendR,
                  std::vector<double> *joints = nullptr,
                  const std::string &tcp = "",
                  const std::string &wcs = "");
        /// @brief 圆弧运动
        /// @param device 设备名称
        /// @param teachPointViaName 途经示教点
        /// @param teachPointToName 目标示教点
        /// @param vel 速度百分比(0,100]
        /// @param acc 加速度百分比(0,100]
        /// @param blendR 平滑半径[0-1000]，0为不平滑，到此位置停止 [mm]
        /// @return 错误码
        int moveC(const std::string &device,
                  const std::string &teachPointViaName,
                  const std::string &teachPointToName,
                  double vel,
                  double acc,
                  double blendR);
        /// @brief 关节点动
        /// @param device 设备名称
        /// @param velocities 关节速度百分比(0,100] 正负代表方向 大小应与设备关节数一致
        /// @param acc 加速度百分比(0,100]
        /// @note 维持jog运动需要周期性调用该接口，调用周期不小于200ms
        /// @return 错误码
        int jogJ(const std::string &device,
                 const std::vector<double> &velocities,
                 double acc);
        /// @brief 笛卡尔点动
        /// @param device 设备名称
		/// @param velocities 速度百分比(0,100] 正负代表方向 [0,5]:笛卡尔速度 [6,8]:外部轴速度
        /// @param acc 加速度百分比(0,100]
        /// @param tcp 工具名称 默认("")为末端法兰坐标系
        /// @param wcs 工件名称 默认("")为基座
        /// @note 维持jog运动需要周期性调用该接口，调用周期不小于200ms
        /// @return 错误码
        int jogCartesian(const std::string &device,
                         const std::array<double, 9> &velocities,
                         double acc,
                         const std::string &tcp = "",
                         const std::string &wcs = "");
        /// @brief 终止所有的运动
        /// @return 错误码
        /// @note 该指令也会终止脚本的运行
        int moveCancel();
        /// @brief 获取设备轴数
        /// @param device 设备名称
        /// @param axisCount 轴数
        /// @return 轴数
        int getAxisCount(const std::string &device, int &axisCount);
        /// @brief 获取设备当前形位的奇异度
        /// @param device 设备名称
		/// @param singular 奇异度[0, 100] 数值越大表示越接近奇异
        /// @return 错误码
        int getSingular(const std::string& device, double& singular);
        /// @brief 使能奇异保护
        /// @param device 设备名称
        /// @param enable 使能状态
        /// @return 错误码
        int setSingularProtectionEnable(const std::string &device, bool enable);
        /// @brief 设置DO
        /// @param value
        /// @param mask 掩码
        /// @return 错误码
        int setDO(int value, int mask);
        /// @brief 读取DO
        /// @param value
        /// @return 错误码
        int getDO(int &value);
        /// @brief 读取DI
        /// @param value
        /// @return 错误码
        int getDI(int &value);
        /// @brief 设置DI
        /// @param value
        /// @param mask 掩码
        /// @note 该命令只在IO为虚拟时有效
        /// @return 错误码
        int setDI(int value, int mask);
        /// @brief 设置为虚拟IO
        /// @param virtualDO
        /// @param virtualDI
        /// @return
        int setVirtualIO(bool virtualDO, bool virtualDI);
        /// @brief 设置AO
        /// @param values
        /// @param mask 掩码
        /// @return 错误码
        int setAO(const std::vector<double> &values, int mask);
        /// @brief 读取AO
        /// @param values
        /// @return 错误码
        int getAO(std::vector<double> &values);
        /// @brief 读取AI
        /// @param values
        /// @return 错误码
        int getAI(std::vector<double> &values);
        /// @brief 设置AI
        /// @param values
        /// @param mask 掩码
        /// @note 该命令只在AIO为虚拟时有效
        /// @return 错误码
        int setAI(const std::vector<double> &values, int mask);
        /// @brief 设置为虚拟AIO
        /// @param virtualAO
        /// @param virtualAI
        /// @return 错误码
        int setVirtualAIO(bool virtualAO, bool virtualAI);
        /// @brief 设置DI滤波系数
        /// @param filter [0,100]
        /// @return 错误码
        int setDIFilter(double filter);
        /// @brief 设置AI滤波系数
        /// @param filter [0,100]
        /// @return 错误码
        int setAIFilter(double filter);
        /// @brief 设置服务器ip
        /// @param ip
        /// @return 错误码
        int setServerIP(const std::string &ip);
        /// @brief 设置运行模式
        /// @param mode 运行模式
        /// @return 错误码
        int setMode(JMC::OperationMode mode);
        /// @brief 获取运行模式
        /// @param mode 运行模式
        /// @return 错误码
        int getMode(JMC::OperationMode &mode);
        /// @brief 设置所有设备使能
        /// @param enable
        /// @return 错误码
        int setEnable(bool enable);
        /// @brief 获取所有设备使能
        /// @param enable
        /// @return 错误码
        int getEnable(bool &enable);
        /// @brief 设置手动模式速度百分比
        /// @param speed (0,100]
        /// @return 错误码
        int setManualSpeed(double speed);
        /// @brief 获取手动模式速度百分比
        /// @param speed (0,100]
        /// @return 错误码
        int getManualSpeed(double &speed);
        /// @brief 设置自动模式速度百分比
        /// @param speed (0,100]
        /// @return 错误码
        int setAutoSpeed(double speed);
        /// @brief 获取自动模式速度百分比
        /// @param speed (0,100]
        /// @return 错误码
        int getAutoSpeed(double &speed);
        /// @brief 设置速度百分比
        /// @param device 设备名称
        /// @param vel [0,100]
        /// @return 错误码
        int setVelocity(const std::string &device, double vel);
        /// @brief 获取速度百分比
        /// @param device 设备名称
        /// @param vel [0,100]
        /// @return 错误码
        int getVelocity(const std::string &device, double &vel);
        /// @brief 设置加速度百分比
        /// @param device 设备名称
        /// @param acc [0,100]
        /// @return 错误码
        int setAcc(const std::string &device, double acc);
        /// @brief 获取加速度百分比
        /// @param device 设备名称
        /// @param acc [0,100]
        /// @return 错误码
        int getAcc(const std::string &device, double &acc);
        /// @brief 设置加加速度百分比
        /// @param device 设备名称
        /// @param jerk [0,100]
        /// @return 错误码
        int setJerk(const std::string &device, double jerk);
        /// @brief 获取加加速度百分比
        /// @param device 设备名称
        /// @param jerk [0,100]
        /// @return 错误码
        int getJerk(const std::string &device, double &jerk);
        /// @brief 设置机器人杆长
        /// @param device 设备名称
        /// @param linkLength
        /// @note
        /// @return 错误码
        int setManipulatorLinkLength(const std::string &device, const std::array<double, 7> &linkLength);
        /// @brief 获取机器人杆长
        /// @param device 设备名称
        /// @param linkLength
        /// @return 错误码
        int getManipulatorLinkLength(const std::string &device, std::array<double, 7> &linkLength);
        /// @brief 设置设备各轴脉冲当量(pusle/mm or pusle/rad)
        /// @param device 设备名称
        /// @param ppu
        /// @return 错误码
        int setPuslePerUnit(const std::string &device, const std::vector<double> &ppu);
        /// @brief 读取设备各轴脉冲当量(pusle/mm or pusle/rad)
        /// @param device 设备名称
        /// @param ppu
        /// @return 错误码
        int getPuslePerUnit(const std::string &device, std::vector<double> &ppu);
        /// @brief 设置设备零位编码器值
        /// @param device 设备名称
        /// @param values
        /// @return 错误码
        int setEncodeZero(const std::string &device, const std::vector<double> &values);
        /// @brief 获取设备零位编码器值
        /// @param device 设备名称
        /// @param values
        /// @return 错误码
        int getEncodeZero(const std::string &device, std::vector<double> &values);
        /// @brief 设置带外部轴机械臂的外部轴方向
        /// @param device 设备名称
        /// @param axisIndex 外部轴索引，6~8
        /// @param dir 方向向量
        /// @return 错误码
        int setExternalAxisDirection(const std::string &device, int axisIndex, const std::array<double, 3> &dir);
        /// @brief 获取带外部轴机械臂的外部轴方向
        /// @param device 设备名称
        /// @param axisIndex 外部轴索引，6~8
        /// @param dir 方向向量
        /// @return 错误码
        int getExternalAxisDirection(const std::string &device, int axisIndex, std::array<double, 3> &dir);
		/// @brief 获取带外部轴机械臂的外部轴使能掩码
        /// @param device 设备名称
        /// @param enableMask 使能掩码
        /// @return 错误码
        int getExternalAxisEnableMask(const std::string& device, int& enableMask);
        /// @brief 设置驱动器报错后是否去使能
        /// @param on
        /// @return 错误码
        int setDriverErrorHoldEnable(bool on);
        /// @brief 读取驱动器报错后是否去使能
        /// @param on
        /// @return 错误码
        int getDriverErrorHoldEnable(bool &on);
        /// @brief 设置开机自动使能
        /// @param on
        /// @return 错误码
        int setPowerOnEnable(bool on);
        /// @brief 读取开机自动使能
        /// @param on
        /// @return 错误码
        int getPowerOnEnable(bool &on);
        /// @brief 设置设备限位数据
        /// @param device 设备名称
        /// @param negtiveLimits 负限位
        /// @param positiveLimits 正限位
        /// @param maxVelocities 最大速度
        /// @return 错误码
        int setSoftLimits(const std::string &device,
                          const std::vector<double> negtiveLimits,
                          const std::vector<double> positiveLimits,
                          const std::vector<double> maxVelocities);
        /// @brief 读取设备限位数据
        /// @param device 设备名称
        /// @param negtiveLimits 负限位
        /// @param positiveLimits 正限位
        /// @param maxVelocities 最大速度
        /// @return 错误码
        int getSoftLimits(const std::string &device,
                          std::vector<double> &negtiveLimits,
                          std::vector<double> &positiveLimits,
                          std::vector<double> &maxVelocities);
        /// @brief 获取控制器软件版本号
        /// @param version
        /// @return 错误码
        int getMCVersion(std::string &version);
        /// @brief 获取控制器UUID
        /// @param uuid
        /// @return 错误码
        int getMCUUID(std::string &uuid);
        /// @brief 获取Ethercat实际和虚拟槽位数
		/// @param realCount 实际槽位数
		/// @param realCount 虚拟槽位数
        /// @return 错误码
        int getEthercatSlotCount(int &realCount, int& virtualCount);
		/// @brief 获取Ethercat槽位的设备类型列表
        /// @param types
        /// @return 错误码
        int getEthercatSlotTypes(std::vector<int>& types);
        /// @brief 获取控制器的错误信息
        /// @param errors
        /// @return 错误码
        int getErrors(std::string &errors);
        /// @brief 清除控制器的错误信息
        /// @return 错误码
        int clearErrors();
        /// @brief 获取急停标志
        /// @param flag
        /// @return 错误码
        int getEmergencyFlag(bool &flag);
        /// @brief 获取设备关节位置
        /// @param deivce 设备名称
        /// @param joints 关节位置
        /// @return 错误码
        int getJointPosition(const std::string &deivce, std::vector<double> &joints);
        /// @brief 获取设备关节速度
        /// @param deivce 设备名称
        /// @param velocity 关节速度
        /// @return 错误码
        int getJointVelocity(const std::string &deivce, std::vector<double> &velocity);
        /// @brief 获取设备关节力矩
        /// @param deivce 设备名称
        /// @param torque 关节力矩
        /// @return 错误码
        int getJointTorque(const std::string &deivce, std::vector<double> &torque);
        /// @brief 获取设备工具位姿
        /// @param deivce 设备名称
        /// @param pose 工具位置
        /// @param tool 工具名称，默认为法兰末端
        /// @param wcs 工具坐标系，默认为设备基坐标系
        /// @return 错误码
        int getToolPose(const std::string &device,
                        std::array<double, 6> &pose,
                        const std::string &tool = "",
                        const std::string &wcs = "");
        /// @brief 获取设备当前工具在当前工件坐标系下的位姿
        /// @param deivce 设备名称
        /// @param pose 工具位置
        /// @param tool 工具名称，默认为法兰末端
        /// @param wcs 工具坐标系，默认为设备基坐标系
        /// @return 错误码
        int getCurrentToolPose(const std::string &device, std::array<double, 6> &pose);
        /// @brief 获取设备运动状态
        /// @param device 设备名臣
        /// @param moving 运动中
        /// @return 错误码
        int getMovingStatus(const std::string &device, bool &moving);
        /// @brief 获取脚本执行状态
        /// @param status
        /// @return 错误码
        int getScriptStatus(JMC::ScriptStatus &status);
        /// @brief 获取脚本当前的执行行号
        /// @param line
        /// @return 错误码
        int getScriptCurrentLine(int &line);
        /// @brief 清空控制器上所有的脚本
        /// @param succeed 操作结果
        /// @return 错误码
        int clearScripts(bool& succeed);
        /// @brief 上传脚本到控制器
        /// @param dir 服务器目标路径
        /// @param file 脚本文件的本地绝对路径
        /// @note dir默认为根目录, 非根目录需以"/"开头
        /// @return 错误码
        int uploadScript(const std::string &file, const std::string &dir = "");
        /// @brief 下载脚本到本地
        /// @param dir 服务器文件路径
        /// @param file 脚本文件的本地绝对路径
        /// @note dir以"/"开头且带文件名
        /// @return 错误码
        int downloadScript(const std::string &dir, const std::string &file);
        /// @brief 删除服务器端文件或目录
        /// @param filePath 服务器文件或目录
        /// @note filePath以"/"开头，若为目录则以"/"结尾
        /// @return 错误码
        int removeFile(const std::string &filePath);
        /// @brief 枚举服务器端目标目录的所有文件信息
        /// @param files 文件信息
        /// @param dir 目标目录
        /// @note dir默认为根目录, 非根目录需以"/"开头
        /// @return 错误码
        int listFiles(std::list<JMC::FileInfo> &files, const std::string &dir = "");
        /// @brief 设置控制器自动运行的脚本名称
        /// @param file 脚本名称
        /// @return 错误码
        int setAutoRunScript(const std::string &file);
        /// @brief 获取控制器自动运行的脚本名称
        /// @param file 脚本名称
        /// @return 错误码
        int getAutoRunScript(std::string &file);
        /// @brief 启动脚本
        /// @return 错误码
        /// @note 启动的目标脚本为自动运行设置的脚本
        int runScript();
        /// @brief 终止脚本
        /// @return 错误码
        int stopScript();
        /// @brief 暂停脚本
        /// @return 错误码
        int pauseScript();
        /// @brief 恢复脚本
        /// @return 错误码
        int resumeScript();
        /// @brief 添加外围设备
        /// @param peripheralName 外围设备名称
        /// @param peripheralType 外围设备类型
        /// @return 错误码
        int peripheralAdd(const std::string &peripheralName,
                          JMC::PeripheralType peripheralType);
        /// @brief 删除外围设备
        /// @param peripheralName 外围设备名称
        /// @return 错误码
        int peripheralRemove(const std::string &peripheralName);
        /// @brief 外围设备控制
        /// @param peripheralName 外围设备名称
        /// @param commandName 命令名称
        /// @param commandParametorsJson 命令参数，JSON格式
        /// @return 错误码
        int peripheralRunCommand(const std::string &peripheralName,
                                 const std::string &commandName,
                                 const std::string &commandParametorsJson);
        /// @brief 外围设备状态读取
        /// @param peripheralName 外围设备名称
        /// @param stateName 状态名称
        /// @param stateJson 状态值，JSON格式
        /// @return 错误码
        int peripheralGetState(const std::string &peripheralName,
                               const std::string &stateName,
                               std::string &stateJson);
        /// @brief 创建一个新的TCP连接
        /// @param ip IP地址
        /// @param port 端口号
        /// @param tcpName 连接名称
        /// @param port 超时时间，单位毫秒，默认1000ms
        /// @return 错误码
        int newTcpConnection(const std::string &ip, int port, std::string &tcpName, int timeoutMs, int &status);
        /// @brief 关闭一个TCP连接
        /// @param tcpName 连接名称
        /// @note 关闭后该连接名称将失效
        /// @return 错误码
        int closeTcpConnection(const std::string &tcpName);
        /// @brief 发送数据到TCP连接
        /// @param tcpName 连接名称
        /// @param data 发送的数据
        /// @param status 发送状态，1表示成功，0表示失败
        /// @return 错误码
        int tcpSendData(const std::string &tcpName,
                        const std::vector<uint8_t> &data,
                        int &status);
        /// @brief 从TCP连接接收数据
        /// @param tcpName 连接名称
        /// @param data 接收的数据
        /// @param timeoutMs 超时时间，单位毫秒
        /// @param status 接收状态，1表示成功，0表示失败
        /// @return 错误码
        int tcpReceiveData(const std::string &tcpName,
                           std::vector<uint8_t> &data,
                           int timeoutMs,
                           int &status);
        /// @brief 创建一个新的ModebusTcp连接
        /// @param ip 主机ip
        /// @param port 主机端口号
        /// @param modbusName 连接名称
        /// @param timeoutMs 超时时间，单位毫秒，默认1000ms
        /// @return 错误码
        int newModbusConnection(const std::string &ip,
                                int port,
                                const std::string &modbusName,
                                int timeoutMs = 1000);
        /// @brief 关闭一个ModebusTcp连接
        /// @param modbusName 连接名称
        /// @return 错误码
        int closeModbusConnection(const std::string &modbusName);
        /// @brief 读取多个线圈状态
        /// @param modbusName 连接名称
        /// @param address 起始地址
        /// @param quantity 读取数量
        /// @param values 读取到的值
        /// @return 错误码
        int modbusReadBits(const std::string &modbusName,
                           int address,
                           int quantity,
                           std::vector<uint8_t> &values);
        /// @brief 读取多个只读输入线圈状态
        /// @param modbusName 连接名称
        /// @param address 起始地址
        /// @param quantity 读取数量
        /// @param values 读取到的值
        /// @return 错误码
        int modbusReadInputBits(const std::string &modbusName,
                                int address,
                                int quantity,
                                std::vector<uint8_t> &values);
        /// @brief 读取多个保持寄存器
        /// @param modbusName 连接名称
        /// @param address 起始地址
        /// @param quantity 读取数量
        /// @param values 读取到的值
        /// @return 错误码
        int modbusReadRegisters(const std::string &modbusName,
                                int address,
                                int quantity,
                                std::vector<uint16_t> &values);
        /// @brief 读取多个只读输入寄存器
        /// @param modbusName 连接名称
        /// @param address 起始地址
        /// @param quantity 读取数量
        /// @param values 读取到的值
        /// @return 错误码
        int modbusReadInputRegisters(const std::string &modbusName,
                                     int address,
                                     int quantity,
                                     std::vector<uint16_t> &values);
        /// @brief 写入多个线圈状态
        /// @param modbusName 连接名称
        /// @param address 起始地址
        /// @param values 写入的值
        /// @return 错误码
        int modbusWriteBits(const std::string &modbusName,
                            int address,
                            const std::vector<uint8_t> &values);
        /// @brief 写入多个保持寄存器
        /// @param modbusName 连接名称
        /// @param address 起始地址
        /// @param values 写入的值
        /// @return 错误码
        int modbusWriteRegisters(const std::string &modbusName,
                                 int address,
                                 const std::vector<uint16_t> &values);
        /// @brief 写入多个保持寄存器并读取多个保持寄存器
        /// @param modbusName 连接名称
        /// @param writeAddress 写入起始地址
        /// @param values 写入的值
        /// @param readAddress 读取起始地址
        /// @param readQuantity 读取数量
        /// @param readValues 读取到的值
        /// @return 错误码
        int modbusWriteReadRegisters(const std::string &modbusName,
                                     int writeAddress,
                                     const std::vector<uint16_t> &values,
                                     int readAddress,
                                     int readQuantity,
                                     std::vector<uint16_t> &readValues);
        /// @brief 计算六轴机械臂的tcp标定
        /// @param device 设备名称
        /// @param tipJointSamples 4个对尖点的关节位置样本
        /// @param orientationTcpSamples 3个对齐方向的tcp姿态样本(原点，X方向，Z方向)
        /// @param tcpPose 计算得到的tcp位姿
		/// @param precision 标定精度
        /// @return 错误码
        int calibrationTcp(const std::string &device,
                           const std::array<std::array<double, 6>, 4> &tipJointSamples,
                           const std::array<std::array<double, 6>, 3> &orientationTcpSamples,
                           std::array<double, 6> &tcpPose,
                           double* precision = nullptr);
        /// @brief 计算外部轴标定
        /// @param device 设备名称
        /// @param externalAxisIndex 外部轴索引，6~8代表第7~9轴
        /// @param measuredData 2个测量数据样本，每个样本包含9个关节位置
        /// @param externalAxisCaliData 计算得到的外部轴标定数据(directionX, directionY, directionZ, ppu)
        /// @param precision 标定精度
        /// @return 错误码
        int calibrationExternalAxis(const std::string &device,
                                    int externalAxisIndex,
                                    const std::array<std::vector<double>, 2> &measuredData,
                                    std::array<double, 4> &externalAxisCaliData,
                                    double* precision = nullptr);
        /// @brief 获取脚本函数名称列表
        /// @param functionNames 函数名称列表
        /// @return 错误码
        int getScriptFunctionNames(std::vector<std::string> &functionNames);
        /// @brief 获取焊机型号
        /// @param device 设备名称
        /// @param vendor 焊机型号
        /// @return 错误码
        int getWelderVendor(const std::string& device, std::string& vendor);
		/// @brief 开启/关闭焊接
        /// @param device 设备名称
        /// @param enable 焊接使能
        /// @return 错误码
        int setWeldEnable(const std::string& device, bool enable);
        /// @brief 开启/关闭焊接(带缓冲)
        /// @param device 设备名称
		/// @param motionDevice 运动设备名称
        /// @param enable 焊接使能
        /// @return 错误码
        int setWeldEnableFifo(const std::string& device, const std::string& motionDevice, bool enable);
        /// @brief 设置焊接工作模式
        /// @param device 设备名称
        /// @param mode 工作模式
        /// @return 错误码
        int setWelderMode(const std::string& device, int mode);
        /// @brief 开启/关闭焊接送气
        /// @param device 设备名称
        /// @param enable 送气使能
        /// @return 错误码
        int setWelderGasEnable(const std::string& device, bool enable);
        /// @brief 开启/关闭焊接送丝
        /// @param device 设备名称
        /// @param enable 送丝使能
        /// @return 错误码
        int setWelderWireFeedEnable(const std::string& device, bool enable);
        /// @brief 开启/关闭焊接退丝
        /// @param device 设备名称
        /// @param enable 退丝使能
        /// @return 错误码
        int setWelderWireFeedbackEnable(const std::string& device, bool enable);
        /// @brief 焊接错误复位
        /// @param device 设备名称
        /// @return 错误码
        int resetWelderError(const std::string& device);
        /// @brief 设置焊机接触传感使能
        /// @param device 设备名称
        /// @param enable 接触传感使能
        /// @return 错误码
        int setWelderTouchSensingEnable(const std::string& device, bool enable);
        /// @brief 设置焊机高压吹气使能
        /// @param device 设备名称
        /// @param enable 高压吹气使能
        /// @return 错误码
        int setWelderBlowThroughEnable(const std::string& device, bool enable);
        /// @brief 设置焊机焊接电流[0-100]%
        /// @param device 设备名称
        /// @param enable 焊接电流[0-100]%
        /// @return 错误码
        int setWelderCurrent(const std::string& device, double persent);
        /// @brief 设置焊机弧长校正[0-100]%
        /// @param device 设备名称
        /// @param enable 弧长校正[0-100]%
        /// @return 错误码
        int setWelderArcLengthCorrection(const std::string& device, double persent);
        /// @brief 设置焊机脉冲峰值[0-100]%
        /// @param device 设备名称
        /// @param enable 脉冲峰值[0-100]%
        /// @return 错误码
        int setWelderPulseCorrection(const std::string& device, double persent);
        /// @brief 设置焊机烧穿回烧时间[0-100]%
        /// @param device 设备名称
        /// @param enable 烧穿回烧时间[0-100]%
        /// @return 错误码
        int setWelderBurnBack(const std::string& device, double persent);
        /// @brief 获取焊机是否引弧成功(焊丝碰触保护)
        /// @param device 设备名称
        /// @param res 是否引弧成功
        /// @return 错误码
        int getWelderArcStable(const std::string& device, bool& res);
        /// @brief 获取焊机电源准备就绪
        /// @param device 设备名称
        /// @param ready 电源准备就绪
        /// @return 错误码
        int getWelderPowerSourceReady(const std::string& device, bool& ready);
        /// @brief 获取焊机电流有无(是否起弧成功)
        /// @param device 设备名称
        /// @param res 电流有无
        /// @return 错误码
        int getWelderMainCurrentSignal(const std::string& device, bool& res);
        /// @brief 获取焊机错误码
        /// @param device 设备名称
        /// @param errorCode 焊机错误码
        /// @return 错误码
        int getWelderErrorCode(const std::string& device, int& errorCode);
        /// @brief 获取焊机焊接电压
        /// @param device 设备名称
        /// @param persent 焊接电压百分比
        /// @return 错误码
        int getWelderVoltage(const std::string& device, double& persent);
        /// @brief 获取焊机焊接电流
        /// @param device 设备名称
        /// @param persent 焊接电流百分比
        /// @return 错误码
        int getWelderCurrent(const std::string& device, double& persent);
        /// @brief 获取焊机送丝速度
        /// @param device 设备名称
        /// @param persent 送丝速度百分比
        /// @return 错误码
        int getWelderWireFeedSpeed(const std::string& device, double& persent);
        /// @brief 获取扫描相机可用硬件数量
        /// @param device 设备名称
        /// @param vendor 扫描相机型号
        /// @param count 硬件数量
        /// @return 错误码
		int getScannerAvailableCount(const std::string& device, const std::string& vendor, int& count);
        /// @brief 获取扫描相机型号
        /// @param device 设备名称
        /// @param vendor 扫描相机型号
        /// @return 错误码
        int getScannerVendor(const std::string& device, std::string& vendor);
        /// @brief 连接扫描相机
        /// @param device 设备名称
        /// @param ip 相机ip
        /// @param index 相机编号
        /// @return 错误码
		int scannerConnect(const std::string& device, const std::string& ip, int index);
        /// @brief 断开扫描相机
        /// @param device 设备名称
        /// @return 错误码
        int scannerDisconnect(const std::string& device);
        /// @brief 启动扫描
        /// @param device 设备名称
        /// @return 错误码
        int scanStart(const std::string& device);
        /// @brief 终止扫描
        /// @param device 设备名称
        /// @return 错误码
		int scanStop(const std::string& device);
		/// @brief 下载扫描点云数据到本地文件
        /// @param device 设备名称
        /// @param path 本地文件路径
        /// @return 错误码
        int scannerDownloadPcdData(const std::string& device, const std::string& path);
        /// @brief 设置扫描校正系数
        /// @param device 设备名称
		/// @param k 校正系数数组[kx, ky, kz]
        /// @return 错误码
        int scannerSetCorrectK(const std::string& device, const std::array<double, 3>& k);
        /// @brief 获取扫描校正系数
        /// @param device 设备名称
        /// @param k 校正系数数组[kx, ky, kz]
        /// @return 错误码
		int scannerGetCorrectK(const std::string& device, std::array<double, 3>& k);
		/// @brief 设置扫描ROIX范围
        /// @param device 设备名称
        /// @param range ROIX范围
        /// @return 错误码
        int scannerSetRoiX(const std::string& device, const std::array<double, 2>& range);
        /// @brief 获取扫描ROIX范围
        /// @param device 设备名称
        /// @param range ROIX范围
        /// @return 错误码
        int scannerGetRoiX(const std::string& device, std::array<double, 2>& range);
        /// @brief 设置扫描ROIY范围
        /// @param device 设备名称
        /// @param range ROIY范围
        /// @return 错误码
        int scannerSetRoiY(const std::string& device, const std::array<double, 2>& range);
        /// @brief 获取扫描ROIY范围
        /// @param device 设备名称
        /// @param range ROIY范围
        /// @return 错误码
        int scannerGetRoiY(const std::string& device, std::array<double, 2>& range);
        /// @brief 设置扫描ROIZ范围
        /// @param device 设备名称
        /// @param range ROIZ范围
        /// @return 错误码
        int scannerSetRoiZ(const std::string& device, const std::array<double, 2>& range);
        /// @brief 获取扫描ROIZ范围
        /// @param device 设备名称
        /// @param range ROIZ范围
        /// @return 错误码
        int scannerGetRoiZ(const std::string& device, std::array<double, 2>& range);
        /// @brief 删除一个工艺
        /// @param name 工艺名称
        /// @return 错误码
        int removeProcessor(const std::string& name);
        /// @brief 新增一个焊接工艺
        /// @param name 工艺名称
        /// @return 错误码
        int addProcessorWeld(const std::string& name);
        /// @brief 运动队列休眠
        /// @param device 设备名称
		/// @param millisecond 休眠时间，单位毫秒
        /// @return 错误码
        int waitFifo(const std::string& device, double millisecond);
    private:
        ClientPrv *_prv;
    };
}