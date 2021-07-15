#ifndef __CONTROLCONFIGCLIENT_H__
#define __CONTROLCONFIGCLIENT_H__

#include <string>
#include <future>
#include <functional>
#include <exception>

#include "Frame.pb.h"
#include "ControlConfig.pb.h"

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
	namespace ControlConfig
	{
		// todogr move somewhere else
		const std::string   none = "";
		
		enum FunctionUids
		{
			eUidSetGravityVector = 0x100001,
			eUidGetGravityVector = 0x100002,
			eUidSetPayloadInformation = 0x100003,
			eUidGetPayloadInformation = 0x100004,
			eUidSetToolConfiguration = 0x100005,
			eUidGetToolConfiguration = 0x100006,
			eUidControlConfigurationTopic = 0x100007,
			eUidUnsubscribe = 0x100008,
			eUidSetCartesianReferenceFrame = 0x100009,
			eUidGetCartesianReferenceFrame = 0x10000a,
			eUidGetControlMode = 0x10000d,
			eUidSetJointSpeedSoftLimits = 0x10000e,
			eUidSetTwistLinearSoftLimit = 0x10000f,
			eUidSetTwistAngularSoftLimit = 0x100010,
			eUidSetJointAccelerationSoftLimits = 0x100011,
			eUidGetKinematicHardLimits = 0x100012,
			eUidGetKinematicSoftLimits = 0x100013,
			eUidGetAllKinematicSoftLimits = 0x100014,
			eUidSetDesiredLinearTwist = 0x100015,
			eUidSetDesiredAngularTwist = 0x100016,
			eUidSetDesiredJointSpeeds = 0x100017,
			eUidGetDesiredSpeeds = 0x100018,
			eUidResetGravityVector = 0x100019,
			eUidResetPayloadInformation = 0x10001a,
			eUidResetToolConfiguration = 0x10001b,
			eUidResetJointSpeedSoftLimits = 0x10001c,
			eUidResetTwistLinearSoftLimit = 0x10001d,
			eUidResetTwistAngularSoftLimit = 0x10001e,
			eUidResetJointAccelerationSoftLimits = 0x10001f,
			eUidControlModeTopic = 0x100020,
		};
		
		class ControlConfigClient
		{
			static const uint32_t m_serviceVersion = 1;
			static const uint32_t m_serviceId = eIdControlConfig;
			NotificationHandler m_notificationHandler;

		protected:
			IRouterClient* const m_clientRouter;

		public:
			ControlConfigClient(IRouterClient* clientRouter);
			static uint32_t getUniqueFctId(uint16_t fctId);

			void SetGravityVector(const GravityVector& gravityvector, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetGravityVector_callback(const GravityVector& gravityvector, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetGravityVector_async(const GravityVector& gravityvector, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			GravityVector GetGravityVector(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetGravityVector_callback(std::function< void (const Error&, const GravityVector&) > callback, uint32_t deviceId = 0);
			std::future<GravityVector> GetGravityVector_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetPayloadInformation(const PayloadInformation& payloadinformation, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetPayloadInformation_callback(const PayloadInformation& payloadinformation, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetPayloadInformation_async(const PayloadInformation& payloadinformation, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			PayloadInformation GetPayloadInformation(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetPayloadInformation_callback(std::function< void (const Error&, const PayloadInformation&) > callback, uint32_t deviceId = 0);
			std::future<PayloadInformation> GetPayloadInformation_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetToolConfiguration(const ToolConfiguration& toolconfiguration, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetToolConfiguration_callback(const ToolConfiguration& toolconfiguration, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetToolConfiguration_async(const ToolConfiguration& toolconfiguration, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			ToolConfiguration GetToolConfiguration(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetToolConfiguration_callback(std::function< void (const Error&, const ToolConfiguration&) > callback, uint32_t deviceId = 0);
			std::future<ToolConfiguration> GetToolConfiguration_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			Kinova::Api::Common::NotificationHandle OnNotificationControlConfigurationTopic(std::function< void (ControlConfigurationNotification) > callback, const Kinova::Api::Common::NotificationOptions& notificationoptions, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void Unsubscribe(const Kinova::Api::Common::NotificationHandle& notificationhandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetCartesianReferenceFrame(const CartesianReferenceFrameInfo& cartesianreferenceframeinfo, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetCartesianReferenceFrame_callback(const CartesianReferenceFrameInfo& cartesianreferenceframeinfo, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetCartesianReferenceFrame_async(const CartesianReferenceFrameInfo& cartesianreferenceframeinfo, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			CartesianReferenceFrameInfo GetCartesianReferenceFrame(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetCartesianReferenceFrame_callback(std::function< void (const Error&, const CartesianReferenceFrameInfo&) > callback, uint32_t deviceId = 0);
			std::future<CartesianReferenceFrameInfo> GetCartesianReferenceFrame_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			ControlModeInformation GetControlMode(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetControlMode_callback(std::function< void (const Error&, const ControlModeInformation&) > callback, uint32_t deviceId = 0);
			std::future<ControlModeInformation> GetControlMode_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetJointSpeedSoftLimits(const JointSpeedSoftLimits& jointspeedsoftlimits, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetJointSpeedSoftLimits_callback(const JointSpeedSoftLimits& jointspeedsoftlimits, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetJointSpeedSoftLimits_async(const JointSpeedSoftLimits& jointspeedsoftlimits, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetTwistLinearSoftLimit(const TwistLinearSoftLimit& twistlinearsoftlimit, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetTwistLinearSoftLimit_callback(const TwistLinearSoftLimit& twistlinearsoftlimit, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetTwistLinearSoftLimit_async(const TwistLinearSoftLimit& twistlinearsoftlimit, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetTwistAngularSoftLimit(const TwistAngularSoftLimit& twistangularsoftlimit, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetTwistAngularSoftLimit_callback(const TwistAngularSoftLimit& twistangularsoftlimit, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetTwistAngularSoftLimit_async(const TwistAngularSoftLimit& twistangularsoftlimit, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetJointAccelerationSoftLimits(const JointAccelerationSoftLimits& jointaccelerationsoftlimits, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetJointAccelerationSoftLimits_callback(const JointAccelerationSoftLimits& jointaccelerationsoftlimits, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetJointAccelerationSoftLimits_async(const JointAccelerationSoftLimits& jointaccelerationsoftlimits, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			KinematicLimits GetKinematicHardLimits(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetKinematicHardLimits_callback(std::function< void (const Error&, const KinematicLimits&) > callback, uint32_t deviceId = 0);
			std::future<KinematicLimits> GetKinematicHardLimits_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			KinematicLimits GetKinematicSoftLimits(const ControlModeInformation& controlmodeinformation, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetKinematicSoftLimits_callback(const ControlModeInformation& controlmodeinformation, std::function< void (const Error&, const KinematicLimits&) > callback, uint32_t deviceId = 0);
			std::future<KinematicLimits> GetKinematicSoftLimits_async(const ControlModeInformation& controlmodeinformation, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			KinematicLimitsList GetAllKinematicSoftLimits(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetAllKinematicSoftLimits_callback(std::function< void (const Error&, const KinematicLimitsList&) > callback, uint32_t deviceId = 0);
			std::future<KinematicLimitsList> GetAllKinematicSoftLimits_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetDesiredLinearTwist(const LinearTwist& lineartwist, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetDesiredLinearTwist_callback(const LinearTwist& lineartwist, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetDesiredLinearTwist_async(const LinearTwist& lineartwist, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetDesiredAngularTwist(const AngularTwist& angulartwist, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetDesiredAngularTwist_callback(const AngularTwist& angulartwist, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetDesiredAngularTwist_async(const AngularTwist& angulartwist, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetDesiredJointSpeeds(const JointSpeeds& jointspeeds, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetDesiredJointSpeeds_callback(const JointSpeeds& jointspeeds, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetDesiredJointSpeeds_async(const JointSpeeds& jointspeeds, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			DesiredSpeeds GetDesiredSpeeds(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetDesiredSpeeds_callback(std::function< void (const Error&, const DesiredSpeeds&) > callback, uint32_t deviceId = 0);
			std::future<DesiredSpeeds> GetDesiredSpeeds_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			GravityVector ResetGravityVector(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ResetGravityVector_callback(std::function< void (const Error&, const GravityVector&) > callback, uint32_t deviceId = 0);
			std::future<GravityVector> ResetGravityVector_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			PayloadInformation ResetPayloadInformation(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ResetPayloadInformation_callback(std::function< void (const Error&, const PayloadInformation&) > callback, uint32_t deviceId = 0);
			std::future<PayloadInformation> ResetPayloadInformation_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			ToolConfiguration ResetToolConfiguration(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ResetToolConfiguration_callback(std::function< void (const Error&, const ToolConfiguration&) > callback, uint32_t deviceId = 0);
			std::future<ToolConfiguration> ResetToolConfiguration_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			JointSpeedSoftLimits ResetJointSpeedSoftLimits(const ControlModeInformation& controlmodeinformation, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ResetJointSpeedSoftLimits_callback(const ControlModeInformation& controlmodeinformation, std::function< void (const Error&, const JointSpeedSoftLimits&) > callback, uint32_t deviceId = 0);
			std::future<JointSpeedSoftLimits> ResetJointSpeedSoftLimits_async(const ControlModeInformation& controlmodeinformation, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			TwistLinearSoftLimit ResetTwistLinearSoftLimit(const ControlModeInformation& controlmodeinformation, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ResetTwistLinearSoftLimit_callback(const ControlModeInformation& controlmodeinformation, std::function< void (const Error&, const TwistLinearSoftLimit&) > callback, uint32_t deviceId = 0);
			std::future<TwistLinearSoftLimit> ResetTwistLinearSoftLimit_async(const ControlModeInformation& controlmodeinformation, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			TwistAngularSoftLimit ResetTwistAngularSoftLimit(const ControlModeInformation& controlmodeinformation, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ResetTwistAngularSoftLimit_callback(const ControlModeInformation& controlmodeinformation, std::function< void (const Error&, const TwistAngularSoftLimit&) > callback, uint32_t deviceId = 0);
			std::future<TwistAngularSoftLimit> ResetTwistAngularSoftLimit_async(const ControlModeInformation& controlmodeinformation, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			JointAccelerationSoftLimits ResetJointAccelerationSoftLimits(const ControlModeInformation& controlmodeinformation, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ResetJointAccelerationSoftLimits_callback(const ControlModeInformation& controlmodeinformation, std::function< void (const Error&, const JointAccelerationSoftLimits&) > callback, uint32_t deviceId = 0);
			std::future<JointAccelerationSoftLimits> ResetJointAccelerationSoftLimits_async(const ControlModeInformation& controlmodeinformation, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			Kinova::Api::Common::NotificationHandle OnNotificationControlModeTopic(std::function< void (ControlModeNotification) > callback, const Kinova::Api::Common::NotificationOptions& notificationoptions, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});


		private:
			void messageHeaderValidation(const Frame& msgFrame){ /* todogr ... */ }
		};
	}
}
}

#endif