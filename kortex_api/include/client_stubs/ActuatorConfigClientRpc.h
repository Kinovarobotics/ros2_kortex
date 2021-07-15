#ifndef __ACTUATORCONFIGCLIENT_H__
#define __ACTUATORCONFIGCLIENT_H__

#include <string>
#include <future>
#include <functional>
#include <exception>

#include "Frame.pb.h"
#include "ActuatorConfig.pb.h"

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
	namespace ActuatorConfig
	{
		// todogr move somewhere else
		const std::string   none = "";
		
		enum FunctionUids
		{
			eUidGetAxisOffsets = 0xa0001,
			eUidSetAxisOffsets = 0xa0002,
			eUidReadTorqueCalibration = 0xa0003,
			eUidWriteTorqueCalibration = 0xa0004,
			eUidSetTorqueOffset = 0xa0005,
			eUidGetControlMode = 0xa0006,
			eUidSetControlMode = 0xa0007,
			eUidGetActivatedControlLoop = 0xa0008,
			eUidSetActivatedControlLoop = 0xa0009,
			eUidGetVectorDriveParameters = 0xa000a,
			eUidSetVectorDriveParameters = 0xa000b,
			eUidGetEncoderDerivativeParameters = 0xa000c,
			eUidSetEncoderDerivativeParameters = 0xa000d,
			eUidGetControlLoopParameters = 0xa000e,
			eUidSetControlLoopParameters = 0xa000f,
			eUidStartFrequencyResponse = 0xa0010,
			eUidStopFrequencyResponse = 0xa0011,
			eUidStartStepResponse = 0xa0012,
			eUidStopStepResponse = 0xa0013,
			eUidStartRampResponse = 0xa0014,
			eUidStopRampResponse = 0xa0015,
			eUidSelectCustomData = 0xa0016,
			eUidGetSelectedCustomData = 0xa0017,
			eUidSetCommandMode = 0xa0018,
			eUidClearFaults = 0xa0019,
			eUidSetServoing = 0xa001a,
			eUidMoveToPosition = 0xa001b,
			eUidGetCommandMode = 0xa001c,
			eUidGetServoing = 0xa001d,
			eUidGetTorqueOffset = 0xa001e,
			eUidSetCoggingFeedforwardMode = 0xa001f,
			eUidGetCoggingFeedforwardMode = 0xa0020,
		};
		
		class ActuatorConfigClient
		{
			static const uint32_t m_serviceVersion = 1;
			static const uint32_t m_serviceId = eIdActuatorConfig;
			NotificationHandler m_notificationHandler;

		protected:
			IRouterClient* const m_clientRouter;

		public:
			ActuatorConfigClient(IRouterClient* clientRouter);
			static uint32_t getUniqueFctId(uint16_t fctId);

			AxisOffsets GetAxisOffsets(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetAxisOffsets_callback(std::function< void (const Error&, const AxisOffsets&) > callback, uint32_t deviceId = 0);
			std::future<AxisOffsets> GetAxisOffsets_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetAxisOffsets(const AxisPosition& axisposition, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetAxisOffsets_callback(const AxisPosition& axisposition, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetAxisOffsets_async(const AxisPosition& axisposition, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			TorqueCalibration ReadTorqueCalibration(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ReadTorqueCalibration_callback(std::function< void (const Error&, const TorqueCalibration&) > callback, uint32_t deviceId = 0);
			std::future<TorqueCalibration> ReadTorqueCalibration_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void WriteTorqueCalibration(const TorqueCalibration& torquecalibration, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void WriteTorqueCalibration_callback(const TorqueCalibration& torquecalibration, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> WriteTorqueCalibration_async(const TorqueCalibration& torquecalibration, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetTorqueOffset(const TorqueOffset& torqueoffset, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetTorqueOffset_callback(const TorqueOffset& torqueoffset, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetTorqueOffset_async(const TorqueOffset& torqueoffset, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			ControlModeInformation GetControlMode(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetControlMode_callback(std::function< void (const Error&, const ControlModeInformation&) > callback, uint32_t deviceId = 0);
			std::future<ControlModeInformation> GetControlMode_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetControlMode(const ControlModeInformation& controlmodeinformation, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetControlMode_callback(const ControlModeInformation& controlmodeinformation, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetControlMode_async(const ControlModeInformation& controlmodeinformation, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			ControlLoop GetActivatedControlLoop(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetActivatedControlLoop_callback(std::function< void (const Error&, const ControlLoop&) > callback, uint32_t deviceId = 0);
			std::future<ControlLoop> GetActivatedControlLoop_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetActivatedControlLoop(const ControlLoop& controlloop, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetActivatedControlLoop_callback(const ControlLoop& controlloop, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetActivatedControlLoop_async(const ControlLoop& controlloop, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			VectorDriveParameters GetVectorDriveParameters(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetVectorDriveParameters_callback(std::function< void (const Error&, const VectorDriveParameters&) > callback, uint32_t deviceId = 0);
			std::future<VectorDriveParameters> GetVectorDriveParameters_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetVectorDriveParameters(const VectorDriveParameters& vectordriveparameters, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetVectorDriveParameters_callback(const VectorDriveParameters& vectordriveparameters, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetVectorDriveParameters_async(const VectorDriveParameters& vectordriveparameters, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			EncoderDerivativeParameters GetEncoderDerivativeParameters(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetEncoderDerivativeParameters_callback(std::function< void (const Error&, const EncoderDerivativeParameters&) > callback, uint32_t deviceId = 0);
			std::future<EncoderDerivativeParameters> GetEncoderDerivativeParameters_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetEncoderDerivativeParameters(const EncoderDerivativeParameters& encoderderivativeparameters, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetEncoderDerivativeParameters_callback(const EncoderDerivativeParameters& encoderderivativeparameters, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetEncoderDerivativeParameters_async(const EncoderDerivativeParameters& encoderderivativeparameters, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			ControlLoopParameters GetControlLoopParameters(const LoopSelection& loopselection, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetControlLoopParameters_callback(const LoopSelection& loopselection, std::function< void (const Error&, const ControlLoopParameters&) > callback, uint32_t deviceId = 0);
			std::future<ControlLoopParameters> GetControlLoopParameters_async(const LoopSelection& loopselection, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetControlLoopParameters(const ControlLoopParameters& controlloopparameters, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetControlLoopParameters_callback(const ControlLoopParameters& controlloopparameters, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetControlLoopParameters_async(const ControlLoopParameters& controlloopparameters, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void StartFrequencyResponse(const FrequencyResponse& frequencyresponse, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void StartFrequencyResponse_callback(const FrequencyResponse& frequencyresponse, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> StartFrequencyResponse_async(const FrequencyResponse& frequencyresponse, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void StopFrequencyResponse(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void StopFrequencyResponse_callback(std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> StopFrequencyResponse_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void StartStepResponse(const StepResponse& stepresponse, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void StartStepResponse_callback(const StepResponse& stepresponse, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> StartStepResponse_async(const StepResponse& stepresponse, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void StopStepResponse(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void StopStepResponse_callback(std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> StopStepResponse_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void StartRampResponse(const RampResponse& rampresponse, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void StartRampResponse_callback(const RampResponse& rampresponse, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> StartRampResponse_async(const RampResponse& rampresponse, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void StopRampResponse(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void StopRampResponse_callback(std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> StopRampResponse_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SelectCustomData(const CustomDataSelection& customdataselection, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SelectCustomData_callback(const CustomDataSelection& customdataselection, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SelectCustomData_async(const CustomDataSelection& customdataselection, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			CustomDataSelection GetSelectedCustomData(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetSelectedCustomData_callback(std::function< void (const Error&, const CustomDataSelection&) > callback, uint32_t deviceId = 0);
			std::future<CustomDataSelection> GetSelectedCustomData_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetCommandMode(const CommandModeInformation& commandmodeinformation, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetCommandMode_callback(const CommandModeInformation& commandmodeinformation, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetCommandMode_async(const CommandModeInformation& commandmodeinformation, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void ClearFaults(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ClearFaults_callback(std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> ClearFaults_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetServoing(const Servoing& servoing, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetServoing_callback(const Servoing& servoing, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetServoing_async(const Servoing& servoing, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void MoveToPosition(const PositionCommand& positioncommand, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void MoveToPosition_callback(const PositionCommand& positioncommand, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> MoveToPosition_async(const PositionCommand& positioncommand, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			CommandModeInformation GetCommandMode(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetCommandMode_callback(std::function< void (const Error&, const CommandModeInformation&) > callback, uint32_t deviceId = 0);
			std::future<CommandModeInformation> GetCommandMode_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			Servoing GetServoing(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetServoing_callback(std::function< void (const Error&, const Servoing&) > callback, uint32_t deviceId = 0);
			std::future<Servoing> GetServoing_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			TorqueOffset GetTorqueOffset(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetTorqueOffset_callback(std::function< void (const Error&, const TorqueOffset&) > callback, uint32_t deviceId = 0);
			std::future<TorqueOffset> GetTorqueOffset_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetCoggingFeedforwardMode(const CoggingFeedforwardModeInformation& coggingfeedforwardmodeinformation, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetCoggingFeedforwardMode_callback(const CoggingFeedforwardModeInformation& coggingfeedforwardmodeinformation, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetCoggingFeedforwardMode_async(const CoggingFeedforwardModeInformation& coggingfeedforwardmodeinformation, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			CoggingFeedforwardModeInformation GetCoggingFeedforwardMode(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetCoggingFeedforwardMode_callback(std::function< void (const Error&, const CoggingFeedforwardModeInformation&) > callback, uint32_t deviceId = 0);
			std::future<CoggingFeedforwardModeInformation> GetCoggingFeedforwardMode_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});


		private:
			void messageHeaderValidation(const Frame& msgFrame){ /* todogr ... */ }
		};
	}
}
}

#endif