#ifndef __VISIONCONFIGCLIENT_H__
#define __VISIONCONFIGCLIENT_H__

#include <string>
#include <future>
#include <functional>
#include <exception>

#include "Frame.pb.h"
#include "VisionConfig.pb.h"

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
	namespace VisionConfig
	{
		// todogr move somewhere else
		const std::string   none = "";
		
		enum FunctionUids
		{
			eUidSetSensorSettings = 0x50001,
			eUidGetSensorSettings = 0x50002,
			eUidGetOptionValue = 0x50003,
			eUidSetOptionValue = 0x50004,
			eUidGetOptionInformation = 0x50005,
			eUidVisionTopic = 0x50006,
			eUidDoSensorFocusAction = 0x50007,
			eUidGetIntrinsicParameters = 0x50008,
			eUidGetIntrinsicParametersProfile = 0x50009,
			eUidSetIntrinsicParameters = 0x5000a,
			eUidGetExtrinsicParameters = 0x5000b,
			eUidSetExtrinsicParameters = 0x5000c,
		};
		
		class VisionConfigClient
		{
			static const uint32_t m_serviceVersion = 1;
			static const uint32_t m_serviceId = eIdVisionConfig;
			NotificationHandler m_notificationHandler;

		protected:
			IRouterClient* const m_clientRouter;

		public:
			VisionConfigClient(IRouterClient* clientRouter);
			static uint32_t getUniqueFctId(uint16_t fctId);

			void SetSensorSettings(const SensorSettings& sensorsettings, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetSensorSettings_callback(const SensorSettings& sensorsettings, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetSensorSettings_async(const SensorSettings& sensorsettings, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			SensorSettings GetSensorSettings(const SensorIdentifier& sensoridentifier, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetSensorSettings_callback(const SensorIdentifier& sensoridentifier, std::function< void (const Error&, const SensorSettings&) > callback, uint32_t deviceId = 0);
			std::future<SensorSettings> GetSensorSettings_async(const SensorIdentifier& sensoridentifier, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			OptionValue GetOptionValue(const OptionIdentifier& optionidentifier, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetOptionValue_callback(const OptionIdentifier& optionidentifier, std::function< void (const Error&, const OptionValue&) > callback, uint32_t deviceId = 0);
			std::future<OptionValue> GetOptionValue_async(const OptionIdentifier& optionidentifier, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetOptionValue(const OptionValue& optionvalue, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetOptionValue_callback(const OptionValue& optionvalue, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetOptionValue_async(const OptionValue& optionvalue, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			OptionInformation GetOptionInformation(const OptionIdentifier& optionidentifier, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetOptionInformation_callback(const OptionIdentifier& optionidentifier, std::function< void (const Error&, const OptionInformation&) > callback, uint32_t deviceId = 0);
			std::future<OptionInformation> GetOptionInformation_async(const OptionIdentifier& optionidentifier, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			Kinova::Api::Common::NotificationHandle OnNotificationVisionTopic(std::function< void (VisionNotification) > callback, const Kinova::Api::Common::NotificationOptions& notificationoptions, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void DoSensorFocusAction(const SensorFocusAction& sensorfocusaction, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void DoSensorFocusAction_callback(const SensorFocusAction& sensorfocusaction, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> DoSensorFocusAction_async(const SensorFocusAction& sensorfocusaction, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			IntrinsicParameters GetIntrinsicParameters(const SensorIdentifier& sensoridentifier, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetIntrinsicParameters_callback(const SensorIdentifier& sensoridentifier, std::function< void (const Error&, const IntrinsicParameters&) > callback, uint32_t deviceId = 0);
			std::future<IntrinsicParameters> GetIntrinsicParameters_async(const SensorIdentifier& sensoridentifier, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			IntrinsicParameters GetIntrinsicParametersProfile(const IntrinsicProfileIdentifier& intrinsicprofileidentifier, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetIntrinsicParametersProfile_callback(const IntrinsicProfileIdentifier& intrinsicprofileidentifier, std::function< void (const Error&, const IntrinsicParameters&) > callback, uint32_t deviceId = 0);
			std::future<IntrinsicParameters> GetIntrinsicParametersProfile_async(const IntrinsicProfileIdentifier& intrinsicprofileidentifier, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetIntrinsicParameters(const IntrinsicParameters& intrinsicparameters, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetIntrinsicParameters_callback(const IntrinsicParameters& intrinsicparameters, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetIntrinsicParameters_async(const IntrinsicParameters& intrinsicparameters, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			ExtrinsicParameters GetExtrinsicParameters(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetExtrinsicParameters_callback(std::function< void (const Error&, const ExtrinsicParameters&) > callback, uint32_t deviceId = 0);
			std::future<ExtrinsicParameters> GetExtrinsicParameters_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetExtrinsicParameters(const ExtrinsicParameters& extrinsicparameters, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetExtrinsicParameters_callback(const ExtrinsicParameters& extrinsicparameters, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetExtrinsicParameters_async(const ExtrinsicParameters& extrinsicparameters, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});


		private:
			void messageHeaderValidation(const Frame& msgFrame){ /* todogr ... */ }
		};
	}
}
}

#endif