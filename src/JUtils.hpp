#ifndef JUTILS_H
#define JUTILS_H

#include <string>
#include <chrono>
#include <random>
#include <algorithm>
#include <sstream>
#include <iostream>
#include <cctype>
#include <optional>
#include <iomanip>
#include <ctime>
#include <filesystem>

#ifdef _WIN32
#include <windows.h>
#include <dbghelp.h>
#include <comdef.h>
#include <Wbemidl.h>
#pragma comment(lib, "dbghelp.lib")
#pragma comment(lib, "wbemuuid.lib")
#pragma comment(lib, "ole32.lib")
#pragma comment(lib, "oleaut32.lib")
#else
#include <time.h>
#include <unistd.h>
#include <fstream>
#endif

#include "Eigen/Dense"

namespace JUtils
{
	namespace Format
	{
		template <typename T>
		std::string vectorFormat(const std::vector<T> &arr, int precision = 3)
		{
			std::ostringstream oss;
			oss << "[";
			for (size_t i = 0; i < arr.size(); ++i)
			{
				oss << std::fixed << std::setprecision(precision) << arr[i];
				if (i != arr.size() - 1)
					oss << ", ";
			}
			oss << "]";
			return oss.str();
		}
		template <typename T, int N>
		std::string arrayFormat(const std::array<T, N> &arr, int precision = 3)
		{
			std::ostringstream oss;
			oss << "[";
			for (size_t i = 0; i < N; ++i)
			{
				oss << std::fixed << std::setprecision(precision) << arr[i];
				if (i != N - 1)
					oss << ", ";
			}
			oss << "]";
			return oss.str();
		}

		inline bool checkIpFormat(const std::string &ip)
		{
			if (ip.empty())
				return false;
			// 禁止前后空白或包含空白字符
			for (char c : ip)
			{
				if (std::isspace(static_cast<unsigned char>(c)))
					return false;
			}

			std::istringstream iss(ip);
			std::string token;
			int parts = 0;
			while (std::getline(iss, token, '.'))
			{
				++parts;
				// 每段非空且长度不超过3
				if (token.empty() || token.size() > 3)
					return false;
				// 必须全为数字
				for (char c : token)
				{
					if (!std::isdigit(static_cast<unsigned char>(c)))
						return false;
				}
				// 解析为整数，且 0..255
				int val = 0;
				for (char c : token)
				{
					val = val * 10 + (c - '0');
					if (val > 255)
						return false;
				}
			}
			return parts == 4;
		}

		inline std::string fileTimeToString(std::filesystem::file_time_type ftime)
		{
			using namespace std::chrono;

			// C++17 手动把 file_time_type 对齐到 system_clock
			const auto sctp = time_point_cast<system_clock::duration>(
				ftime - std::filesystem::file_time_type::clock::now() + system_clock::now());

			std::time_t tt = system_clock::to_time_t(sctp);
			std::tm tmBuf{};
#if defined(_WIN32)
			localtime_s(&tmBuf, &tt);
#else
			localtime_r(&tt, &tmBuf);
#endif

			char buffer[20];
			if (std::strftime(buffer, sizeof(buffer), "%Y-%m-%d %H:%M:%S", &tmBuf) == 0)
				return {};
			return buffer;
		}
	}

	namespace Random
	{
		/**
		 * @brief 生成一个整型随机数
		 * @param min 随机数下限
		 * @param max 随机数上限
		 * @return 随机数
		 */
		template <typename T>
		inline T generateIntRandom(const T &min, const T &max)
		{
			std::mt19937_64 device(std::random_device{}());
			std::uniform_int_distribution<T> num(min, max);
			return num(device);
		}

		/**
		 * @brief 生成一个浮点型随机数
		 * @param min 随机数下限
		 * @param max 随机数上限
		 * @return 随机数
		 */
		template <typename T>
		inline T generateRealRandom(const T &min, const T &max)
		{
			std::mt19937_64 device(std::random_device{}());
			std::uniform_real_distribution<T> num(min, max);
			return num(device);
		}

		/**
		 * @brief 生成一个UUID
		 * @return UUID
		 */
		inline std::string generateUUID()
		{
			std::random_device rd;
			std::mt19937 gen(rd());
			std::uniform_int_distribution<uint64_t> dis(0, UINT64_MAX);
			uint64_t uuid = (dis(gen) << 32 | dis(gen));
			return std::to_string(uuid);
		}
	}

	namespace Time
	{
		enum timeStep
		{
			second = 0,
			milliseconds,
			microseconds,
			nanoseconds
		};

		/**
		 * @brief 获取系统时间(相对于1970年1月1日0时0分，精度取决于系统精度)
		 * @param step 时间单位
		 * @return 系统时间
		 */
		inline uint64_t getSystemTime(timeStep step = timeStep::milliseconds)
		{
			auto now = std::chrono::steady_clock::now().time_since_epoch();
			uint64_t timeStamp = 0;
			switch (step)
			{
			case JUtils::Time::second:
				timeStamp = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::seconds>(now).count());
				break;
			case JUtils::Time::milliseconds:
				timeStamp = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::milliseconds>(now).count());
				break;
			case JUtils::Time::microseconds:
				timeStamp = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::microseconds>(now).count());
				break;
			case JUtils::Time::nanoseconds:
				timeStamp = static_cast<uint64_t>(std::chrono::duration_cast<std::chrono::nanoseconds>(now).count());
				break;
			default:
				break;
			}
			return timeStamp;
		}

		/**
		 * @brief 高精度Sleep
		 * @param microsecond 微妙睡眠时间
		 */
		inline void high_precision_sleep(int microsecond)
		{
#ifdef WIN32
			LARGE_INTEGER perfFreq = {0};
			LARGE_INTEGER start = {0};
			LARGE_INTEGER now = {0};

			QueryPerformanceFrequency(&perfFreq); // 获取性能计数器的频率
			QueryPerformanceCounter(&start);	  // 获取起始时间

			const LONGLONG targetTicks = (LONGLONG)microsecond * perfFreq.QuadPart / 1000000LL; // 目标计数值

			do
			{
				QueryPerformanceCounter(&now);
			} while ((now.QuadPart - start.QuadPart) < targetTicks); // 判断是否达到目标时间
#else
			struct timespec ts;
			ts.tv_sec = microsecond / 1000000;			 // 秒部分
			ts.tv_nsec = (microsecond % 1000000) * 1000; // 纳秒部分

			while (nanosleep(&ts, &ts) == -1 && errno == EINTR)
			{
				// 如果被信号中断，继续休眠
			}
#endif
		}
	}

	namespace BitOperation
	{
		/**
		 * @brief 取位操作
		 * @param value 目标
		 * @param index 位号 0起始
		 * @return 置位
		 */
		inline bool bit(int value, int index)
		{
			return (value >> index) & 0x01;
		}
		/**
		 * @brief 置位操作
		 * @param value 目标
		 * @param index 位号 0起始
		 * @return 置位
		 */
		inline int setBit(int value, int index)
		{
			return value | (0x01 << index);
		}
		/**
		 * @brief 重置位操作
		 * @param value 目标
		 * @param index 位号 0起始
		 * @return 重置位
		 */
		inline int resetBit(int value, int index)
		{
			return value & (~(0x01 << index));
		}
		/**
		 * @brief toggle位操作
		 * @param value 目标
		 * @param index 位号 0起始
		 * @return toggle位
		 */
		inline int toggleBit(int value, int index)
		{
			return value ^ (0x01 << index);
		}
		/**
		 * @brief 写多位操作
		 * @param value 目标
		 * @param startBit 起始位号 0起始
		 * @param bitCount 位数
		 * @param writeValue 写入值
		 * @return 新值
		 */
		inline int writeBits(int value, int startBit, int bitCount, int writeValue)
		{
			if (bitCount <= 0 || startBit < 0)
				return value;

			const unsigned WIDTH = sizeof(unsigned) * CHAR_BIT;
			if ((unsigned)startBit >= WIDTH)
				return value;

			// 限制要写的位数不超过剩余位宽
			unsigned bits = static_cast<unsigned>(bitCount);
			if (bits > WIDTH - static_cast<unsigned>(startBit))
				bits = WIDTH - static_cast<unsigned>(startBit);

			// 生成 bitCount 个 1 的掩码，避免 (1u<<WIDTH) 未定义
			unsigned lowMask = (bits == WIDTH) ? ~0u : ((1u << bits) - 1u);
			unsigned mask = lowMask << static_cast<unsigned>(startBit);

			unsigned uvalue = static_cast<unsigned>(value);
			unsigned uwrite = static_cast<unsigned>(writeValue) & lowMask; // 只取低 bits 位

			uvalue = (uvalue & ~mask) | ((uwrite << static_cast<unsigned>(startBit)) & mask);
			return static_cast<int>(uvalue);
		}
	}

	namespace StringOperation
	{
		inline std::vector<std::string> split(const std::string &str, char delimiter)
		{
			std::vector<std::string> tokens;
			std::stringstream ss(str);
			std::string token;
			while (std::getline(ss, token, delimiter))
			{
				tokens.push_back(token);
			}
			return tokens;
		}
	}

	namespace Trace
	{
#ifdef _WIN32
		inline void printStackTrace()
		{
			HANDLE process = GetCurrentProcess();
			HANDLE thread = GetCurrentThread();

			CONTEXT context;
			RtlCaptureContext(&context);

			SymInitialize(process, NULL, TRUE);

			STACKFRAME64 stackFrame;
			memset(&stackFrame, 0, sizeof(STACKFRAME64));

			DWORD machineType = IMAGE_FILE_MACHINE_AMD64;

			stackFrame.AddrPC.Offset = context.Rip;
			stackFrame.AddrPC.Mode = AddrModeFlat;
			stackFrame.AddrFrame.Offset = context.Rbp;
			stackFrame.AddrFrame.Mode = AddrModeFlat;
			stackFrame.AddrStack.Offset = context.Rsp;
			stackFrame.AddrStack.Mode = AddrModeFlat;

			while (StackWalk64(machineType, process, thread, &stackFrame, &context, NULL, SymFunctionTableAccess64, SymGetModuleBase64, NULL))
			{
				DWORD64 address = stackFrame.AddrPC.Offset;

				// 获取函数名
				char symbolBuffer[sizeof(SYMBOL_INFO) + 256];
				SYMBOL_INFO *symbol = (SYMBOL_INFO *)symbolBuffer;
				symbol->MaxNameLen = 255;
				symbol->SizeOfStruct = sizeof(SYMBOL_INFO);

				if (SymFromAddr(process, address, 0, symbol))
				{
					// 过滤掉系统函数（例如包含 "ntdll" 或 "kernel32" 的符号）
					if (strstr(symbol->Name, "ntdll") ||
						strstr(symbol->Name, "kernel32") ||
						strstr(symbol->Name, "std"))
						continue;

					std::cout << "Function: " << symbol->Name << " at 0x" << std::hex << symbol->Address;

					// 获取文件名和行号
					IMAGEHLP_LINE64 line;
					DWORD displacement = 0;
					memset(&line, 0, sizeof(IMAGEHLP_LINE64));
					line.SizeOfStruct = sizeof(IMAGEHLP_LINE64);

					if (SymGetLineFromAddr64(process, address, &displacement, &line))
					{
						std::cout << " in " << line.FileName << " at line " << line.LineNumber;
					}
					else
					{
						std::cout << " (no source info)";
					}

					std::cout << std::endl;
				}
			}

			SymCleanup(process);
		}
#endif
	}

	namespace Serialize
	{
		inline std::string serializeMatrix(const Eigen::Matrix4d &mat)
		{
			std::ostringstream oss;
			for (int i = 0; i < mat.rows(); ++i)
			{
				for (int j = 0; j < mat.cols(); ++j)
				{
					oss << mat(i, j);
					if (i != mat.rows() - 1 || j != mat.cols() - 1)
						oss << ",";
				}
			}
			return oss.str();
		}

		inline std::optional<Eigen::Matrix4d> deserializeMatrix(const std::string &str)
		{
			Eigen::Matrix4d mat;
			std::istringstream iss(str);
			std::string token;
			int idx = 0;
			while (std::getline(iss, token, ',') && idx < 16)
			{
				try
				{
					mat(idx / 4, idx % 4) = std::stod(token);
				}
				catch (...)
				{
					return std::nullopt;
				}
				++idx;
			}
			// 检查是否正好16个元素且没有多余数据
			if (idx != 16 || std::getline(iss, token, ','))
			{
				return std::nullopt;
			}
			return mat;
		}
	}

	namespace Hardware
	{
		inline void setNetworkInterfaceIP(const std::string &interfaceName, const std::string &ipAddress)
		{
#ifdef _WIN32
			std::string command = "netsh interface ip set address name=\"" + interfaceName + "\" static " + ipAddress + " " + "255.255.255.0" + " " + "none";
			system(command.c_str());
#else
			// Linux 下的实现（需要 root 权限）
			std::string command = "sudo ifconfig " + interfaceName + " " + ipAddress + " netmask " + "255.255.255.0";
			system(command.c_str());
#endif
		}

		inline std::string getCPUUUID()
		{
#ifdef _WIN32
			// Windows: 使用 WMI 查询 CPU ProcessorId
			std::string uuid;
			HRESULT hres;

			// 初始化 COM
			hres = CoInitializeEx(0, COINIT_MULTITHREADED);
			if (FAILED(hres) && hres != RPC_E_CHANGED_MODE)
			{
				// 如果返回 RPC_E_CHANGED_MODE，表示 COM 已经初始化过，可以继续
				return "COM_INIT_FAILED_0x" + std::to_string(hres);
			}

			bool needUninit = SUCCEEDED(hres); // 记录是否需要反初始化

			// 设置 COM 安全级别
			hres = CoInitializeSecurity(
				NULL,
				-1,
				NULL,
				NULL,
				RPC_C_AUTHN_LEVEL_DEFAULT,
				RPC_C_IMP_LEVEL_IMPERSONATE,
				NULL,
				EOAC_NONE,
				NULL);

			// RPC_E_TOO_LATE 表示安全级别已经设置过，可以忽略
			if (FAILED(hres) && hres != RPC_E_TOO_LATE)
			{
				if (needUninit)
					CoUninitialize();
				return "SECURITY_INIT_FAILED_0x" + std::to_string(hres);
			}

			// 获取 WMI locator
			IWbemLocator *pLoc = NULL;
			hres = CoCreateInstance(
				CLSID_WbemLocator,
				0,
				CLSCTX_INPROC_SERVER,
				IID_IWbemLocator, (LPVOID *)&pLoc);

			if (FAILED(hres))
			{
				if (needUninit)
					CoUninitialize();
				return "LOCATOR_CREATE_FAILED_0x" + std::to_string(hres);
			}

			// 连接到 WMI
			IWbemServices *pSvc = NULL;
			hres = pLoc->ConnectServer(
				_bstr_t(L"ROOT\\CIMV2"),
				NULL,
				NULL,
				0,
				NULL,
				0,
				0,
				&pSvc);

			if (FAILED(hres))
			{
				pLoc->Release();
				if (needUninit)
					CoUninitialize();
				return "WMI_CONNECT_FAILED_0x" + std::to_string(hres);
			}

			// 设置代理安全级别
			hres = CoSetProxyBlanket(
				pSvc,
				RPC_C_AUTHN_WINNT,
				RPC_C_AUTHZ_NONE,
				NULL,
				RPC_C_AUTHN_LEVEL_CALL,
				RPC_C_IMP_LEVEL_IMPERSONATE,
				NULL,
				EOAC_NONE);

			if (FAILED(hres))
			{
				pSvc->Release();
				pLoc->Release();
				if (needUninit)
					CoUninitialize();
				return "PROXY_BLANKET_FAILED_0x" + std::to_string(hres);
			}

			// 执行 WQL 查询
			IEnumWbemClassObject *pEnumerator = NULL;
			hres = pSvc->ExecQuery(
				bstr_t("WQL"),
				bstr_t("SELECT ProcessorId FROM Win32_Processor"),
				WBEM_FLAG_FORWARD_ONLY | WBEM_FLAG_RETURN_IMMEDIATELY,
				NULL,
				&pEnumerator);

			if (FAILED(hres))
			{
				pSvc->Release();
				pLoc->Release();
				if (needUninit)
					CoUninitialize();
				return "QUERY_FAILED_0x" + std::to_string(hres);
			}

			// 获取查询结果
			IWbemClassObject *pclsObj = NULL;
			ULONG uReturn = 0;

			if (pEnumerator)
			{
				HRESULT hr = pEnumerator->Next(WBEM_INFINITE, 1, &pclsObj, &uReturn);
				if (uReturn > 0)
				{
					VARIANT vtProp;
					VariantInit(&vtProp);

					hr = pclsObj->Get(L"ProcessorId", 0, &vtProp, 0, 0);
					if (SUCCEEDED(hr))
					{
						if (vtProp.vt == VT_BSTR && vtProp.bstrVal != NULL)
						{
							// 转换 BSTR 到 std::string
							_bstr_t b(vtProp.bstrVal);
							uuid = (const char *)b;
						}
						else if (vtProp.vt == VT_NULL)
						{
							uuid = "NULL_PROCESSOR_ID";
						}
					}

					VariantClear(&vtProp);
					pclsObj->Release();
				}
				pEnumerator->Release();
			}

			// 清理
			pSvc->Release();
			pLoc->Release();
			if (needUninit)
				CoUninitialize();

			return uuid.empty() ? "EMPTY_PROCESSOR_ID" : uuid;

#else
			// Linux: 读取 /proc/cpuinfo 或使用 dmidecode
			std::string uuid;

			// 方法1: 尝试使用 dmidecode (需要 root 权限)
			FILE *pipe = popen("dmidecode -t processor 2>/dev/null | grep ID | head -n 1 | awk '{print $2}'", "r");
			if (pipe)
			{
				char buffer[128];
				if (fgets(buffer, sizeof(buffer), pipe) != nullptr)
				{
					uuid = buffer;
					// 移除换行符和空白字符
					uuid.erase(std::remove_if(uuid.begin(), uuid.end(),
											   [](unsigned char c)
											   { return std::isspace(c); }),
							   uuid.end());
				}
				pclose(pipe);
			}

			// 方法2: 如果 dmidecode 失败，尝试读取 /proc/cpuinfo
			if (uuid.empty())
			{
				std::ifstream cpuinfo("/proc/cpuinfo");
				if (cpuinfo.is_open())
				{
					std::string line;
					while (std::getline(cpuinfo, line))
					{
						// 查找 "serial" 字段
						if (line.find("serial") != std::string::npos)
						{
							size_t pos = line.find(":");
							if (pos != std::string::npos)
							{
								uuid = line.substr(pos + 1);
								// 移除前后空格
								uuid.erase(0, uuid.find_first_not_of(" \t"));
								uuid.erase(uuid.find_last_not_of(" \t") + 1);
								break;
							}
						}
					}
					cpuinfo.close();
				}
			}

			// 方法3: 尝试读取机器 ID（更可靠的替代方案）
			if (uuid.empty())
			{
				std::ifstream machineId("/etc/machine-id");
				if (machineId.is_open())
				{
					std::getline(machineId, uuid);
					machineId.close();
					if (!uuid.empty())
					{
						return uuid; // machine-id 作为后备方案
					}
				}
			}

			// 方法4: 使用 CPU 信息的哈希作为最后的替代
			if (uuid.empty())
			{
				std::ifstream cpuinfo("/proc/cpuinfo");
				if (cpuinfo.is_open())
				{
					std::string line;
					std::string cpuInfo;
					while (std::getline(cpuinfo, line))
					{
						if (line.find("model name") != std::string::npos ||
							line.find("cpu MHz") != std::string::npos ||
							line.find("cache size") != std::string::npos)
						{
							cpuInfo += line;
						}
					}
					cpuinfo.close();

					if (!cpuInfo.empty())
					{
						// 使用哈希生成唯一标识
						std::hash<std::string> hasher;
						size_t hash = hasher(cpuInfo);
						std::ostringstream oss;
						oss << std::hex << std::setw(16) << std::setfill('0') << hash;
						uuid = "HASH_" + oss.str();
					}
				}
			}

			return uuid.empty() ? "UNKNOWN" : uuid;
#endif
		}
	}
}

#endif // JUTILS_H
