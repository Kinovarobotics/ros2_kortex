#ifndef __DEVICECONFIGCLIENT_H__
#define __DEVICECONFIGCLIENT_H__

#include <string>
#include <future>
#include <functional>
#include <exception>

#include "Frame.pb.h"
#include "DeviceConfig.pb.h"

#include "Frame.h"
#include "IRouterClient.h"
#include "NotificationHandler.h"

#if __cplusplus >= 201402L
#define DEPRECATED [[ deprecated ]]
#define DEPRECATED_MSG(msg) [[ deprecated(msg) ]]
#elif defined(__GNUC__)
#define DEPRECATED __attribute__ ((deprecated))
#define DEPRECATED_MSG(msg) __attribute__ ((deprecated(msg)))
#elif defined(_MSC_VER)
#define DEPRECATED __declspec(deprecated)
#define DEPRECATED_MSG(msg) __declspec(deprecated(msg))
#else
#define DEPRECATED 
#define DEPRECATED_MSG 
#endif
namespace Kinova
{
namespace Api
{
	namespace DeviceConfig
	{
		// todogr move somewhere else
		const std::string   none = "";
		
		enum FunctionUids
		{
			eUidGetRunMode = 0x90001,
			eUidSetRunMode = 0x90002,
			eUidGetDeviceType = 0x90003,
			eUidGetFirmwareVersion = 0x90004,
			eUidGetBootloaderVersion = 0x90005,
			eUidGetModelNumber = 0x90006,
			eUidGetPartNumber = 0x90007,
			eUidGetSerialNumber = 0x90008,
			eUidGetMACAddress = 0x90009,
			eUidGetIPv4Settings = 0x9000a,
			eUidSetIPv4Settings = 0x9000b,
			eUidGetPartNumberRevision = 0x9000c,
			eUidRebootRequest = 0x9000e,
			eUidSetSafetyEnable = 0x9000f,
			eUidSetSafetyErrorThreshold = 0x90010,
			eUidSetSafetyWarningThreshold = 0x90011,
			eUidSetSafetyConfiguration = 0x90012,
			eUidGetSafetyConfiguration = 0x90013,
			eUidGetSafetyInformation = 0x90014,
			eUidGetSafetyEnable = 0x90015,
			eUidGetSafetyStatus = 0x90016,
			eUidClearAllSafetyStatus = 0x90017,
			eUidClearSafetyStatus = 0x90018,
			eUidGetAllSafetyConfiguration = 0x90019,
			eUidGetAllSafetyInformation = 0x9001a,
			eUidResetSafetyDefaults = 0x9001b,
			eUidSafetyTopic = 0x9001c,
			eUidExecuteCalibration = 0x90022,
			eUidGetCalibrationResult = 0x90023,
			eUidStopCalibration = 0x90024,
			eUidSetCapSenseConfig = 0x90025,
			eUidGetCapSenseConfig = 0x90026,
			eUidReadCapSenseRegister = 0x90027,
			eUidWriteCapSenseRegister = 0x90028,
		};
		
		class DeviceConfigClient
		{
			static const uint32_t m_serviceVersion = 1;
			static const uint32_t m_serviceId = eIdDeviceConfig;
			NotificationHandler m_notificationHandler;

		protected:
			IRouterClient* const m_clientRouter;

		public:
			DeviceConfigClient(IRouterClient* clientRouter);
			static uint32_t getUniqueFctId(uint16_t fctId);

			RunMode GetRunMode(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetRunMode_callback(std::function< void (const Error&, const RunMode&) > callback, uint32_t deviceId = 0);
			std::future<RunMode> GetRunMode_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetRunMode(const RunMode& runmode, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetRunMode_callback(const RunMode& runmode, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetRunMode_async(const RunMode& runmode, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			DeviceType GetDeviceType(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetDeviceType_callback(std::function< void (const Error&, const DeviceType&) > callback, uint32_t deviceId = 0);
			std::future<DeviceType> GetDeviceType_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			FirmwareVersion GetFirmwareVersion(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetFirmwareVersion_callback(std::function< void (const Error&, const FirmwareVersion&) > callback, uint32_t deviceId = 0);
			std::future<FirmwareVersion> GetFirmwareVersion_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			BootloaderVersion GetBootloaderVersion(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetBootloaderVersion_callback(std::function< void (const Error&, const BootloaderVersion&) > callback, uint32_t deviceId = 0);
			std::future<BootloaderVersion> GetBootloaderVersion_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			ModelNumber GetModelNumber(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetModelNumber_callback(std::function< void (const Error&, const ModelNumber&) > callback, uint32_t deviceId = 0);
			std::future<ModelNumber> GetModelNumber_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			PartNumber GetPartNumber(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetPartNumber_callback(std::function< void (const Error&, const PartNumber&) > callback, uint32_t deviceId = 0);
			std::future<PartNumber> GetPartNumber_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			SerialNumber GetSerialNumber(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetSerialNumber_callback(std::function< void (const Error&, const SerialNumber&) > callback, uint32_t deviceId = 0);
			std::future<SerialNumber> GetSerialNumber_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			MACAddress GetMACAddress(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetMACAddress_callback(std::function< void (const Error&, const MACAddress&) > callback, uint32_t deviceId = 0);
			std::future<MACAddress> GetMACAddress_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			IPv4Settings GetIPv4Settings(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetIPv4Settings_callback(std::function< void (const Error&, const IPv4Settings&) > callback, uint32_t deviceId = 0);
			std::future<IPv4Settings> GetIPv4Settings_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetIPv4Settings(const IPv4Settings& ipv4settings, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetIPv4Settings_callback(const IPv4Settings& ipv4settings, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetIPv4Settings_async(const IPv4Settings& ipv4settings, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			PartNumberRevision GetPartNumberRevision(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetPartNumberRevision_callback(std::function< void (const Error&, const PartNumberRevision&) > callback, uint32_t deviceId = 0);
			std::future<PartNumberRevision> GetPartNumberRevision_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void RebootRequest(const RebootRqst& rebootrqst, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void RebootRequest_callback(const RebootRqst& rebootrqst, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> RebootRequest_async(const RebootRqst& rebootrqst, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetSafetyEnable(const SafetyEnable& safetyenable, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetSafetyEnable_callback(const SafetyEnable& safetyenable, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetSafetyEnable_async(const SafetyEnable& safetyenable, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetSafetyErrorThreshold(const SafetyThreshold& safetythreshold, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetSafetyErrorThreshold_callback(const SafetyThreshold& safetythreshold, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetSafetyErrorThreshold_async(const SafetyThreshold& safetythreshold, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetSafetyWarningThreshold(const SafetyThreshold& safetythreshold, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetSafetyWarningThreshold_callback(const SafetyThreshold& safetythreshold, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetSafetyWarningThreshold_async(const SafetyThreshold& safetythreshold, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetSafetyConfiguration(const SafetyConfiguration& safetyconfiguration, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetSafetyConfiguration_callback(const SafetyConfiguration& safetyconfiguration, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetSafetyConfiguration_async(const SafetyConfiguration& safetyconfiguration, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			SafetyConfiguration GetSafetyConfiguration(const Kinova::Api::Common::SafetyHandle& safetyhandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetSafetyConfiguration_callback(const Kinova::Api::Common::SafetyHandle& safetyhandle, std::function< void (const Error&, const SafetyConfiguration&) > callback, uint32_t deviceId = 0);
			std::future<SafetyConfiguration> GetSafetyConfiguration_async(const Kinova::Api::Common::SafetyHandle& safetyhandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			SafetyInformation GetSafetyInformation(const Kinova::Api::Common::SafetyHandle& safetyhandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetSafetyInformation_callback(const Kinova::Api::Common::SafetyHandle& safetyhandle, std::function< void (const Error&, const SafetyInformation&) > callback, uint32_t deviceId = 0);
			std::future<SafetyInformation> GetSafetyInformation_async(const Kinova::Api::Common::SafetyHandle& safetyhandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			SafetyEnable GetSafetyEnable(const Kinova::Api::Common::SafetyHandle& safetyhandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetSafetyEnable_callback(const Kinova::Api::Common::SafetyHandle& safetyhandle, std::function< void (const Error&, const SafetyEnable&) > callback, uint32_t deviceId = 0);
			std::future<SafetyEnable> GetSafetyEnable_async(const Kinova::Api::Common::SafetyHandle& safetyhandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			SafetyStatus GetSafetyStatus(const Kinova::Api::Common::SafetyHandle& safetyhandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetSafetyStatus_callback(const Kinova::Api::Common::SafetyHandle& safetyhandle, std::function< void (const Error&, const SafetyStatus&) > callback, uint32_t deviceId = 0);
			std::future<SafetyStatus> GetSafetyStatus_async(const Kinova::Api::Common::SafetyHandle& safetyhandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void ClearAllSafetyStatus(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ClearAllSafetyStatus_callback(std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> ClearAllSafetyStatus_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void ClearSafetyStatus(const Kinova::Api::Common::SafetyHandle& safetyhandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ClearSafetyStatus_callback(const Kinova::Api::Common::SafetyHandle& safetyhandle, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> ClearSafetyStatus_async(const Kinova::Api::Common::SafetyHandle& safetyhandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			SafetyConfigurationList GetAllSafetyConfiguration(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetAllSafetyConfiguration_callback(std::function< void (const Error&, const SafetyConfigurationList&) > callback, uint32_t deviceId = 0);
			std::future<SafetyConfigurationList> GetAllSafetyConfiguration_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			SafetyInformationList GetAllSafetyInformation(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetAllSafetyInformation_callback(std::function< void (const Error&, const SafetyInformationList&) > callback, uint32_t deviceId = 0);
			std::future<SafetyInformationList> GetAllSafetyInformation_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void ResetSafetyDefaults(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ResetSafetyDefaults_callback(std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> ResetSafetyDefaults_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			Kinova::Api::Common::NotificationHandle OnNotificationSafetyTopic(std::function< void (Kinova::Api::Common::SafetyNotification) > callback, const Kinova::Api::Common::NotificationOptions& notificationoptions, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void ExecuteCalibration(const Calibration& calibration, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ExecuteCalibration_callback(const Calibration& calibration, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> ExecuteCalibration_async(const Calibration& calibration, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			CalibrationResult GetCalibrationResult(const CalibrationElement& calibrationelement, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetCalibrationResult_callback(const CalibrationElement& calibrationelement, std::function< void (const Error&, const CalibrationResult&) > callback, uint32_t deviceId = 0);
			std::future<CalibrationResult> GetCalibrationResult_async(const CalibrationElement& calibrationelement, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			CalibrationResult StopCalibration(const Calibration& calibration, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void StopCalibration_callback(const Calibration& calibration, std::function< void (const Error&, const CalibrationResult&) > callback, uint32_t deviceId = 0);
			std::future<CalibrationResult> StopCalibration_async(const Calibration& calibration, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetCapSenseConfig(const CapSenseConfig& capsenseconfig, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetCapSenseConfig_callback(const CapSenseConfig& capsenseconfig, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetCapSenseConfig_async(const CapSenseConfig& capsenseconfig, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			CapSenseConfig GetCapSenseConfig(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetCapSenseConfig_callback(std::function< void (const Error&, const CapSenseConfig&) > callback, uint32_t deviceId = 0);
			std::future<CapSenseConfig> GetCapSenseConfig_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			CapSenseRegister ReadCapSenseRegister(const CapSenseRegister& capsenseregister, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ReadCapSenseRegister_callback(const CapSenseRegister& capsenseregister, std::function< void (const Error&, const CapSenseRegister&) > callback, uint32_t deviceId = 0);
			std::future<CapSenseRegister> ReadCapSenseRegister_async(const CapSenseRegister& capsenseregister, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void WriteCapSenseRegister(const CapSenseRegister& capsenseregister, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void WriteCapSenseRegister_callback(const CapSenseRegister& capsenseregister, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> WriteCapSenseRegister_async(const CapSenseRegister& capsenseregister, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});


		private:
			void messageHeaderValidation(const Frame& msgFrame){ /* todogr ... */ }
		};
	}
}
}

#endif