#ifndef __INTERCONNECTCONFIGCLIENT_H__
#define __INTERCONNECTCONFIGCLIENT_H__

#include <string>
#include <future>
#include <functional>
#include <exception>

#include "Frame.pb.h"
#include "InterconnectConfig.pb.h"

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
	namespace InterconnectConfig
	{
		// todogr move somewhere else
		const std::string   none = "";
		
		enum FunctionUids
		{
			eUidGetUARTConfiguration = 0xe0001,
			eUidSetUARTConfiguration = 0xe0002,
			eUidGetEthernetConfiguration = 0xe0003,
			eUidSetEthernetConfiguration = 0xe0004,
			eUidGetGPIOConfiguration = 0xe0005,
			eUidSetGPIOConfiguration = 0xe0006,
			eUidGetGPIOState = 0xe0007,
			eUidSetGPIOState = 0xe0008,
			eUidGetI2CConfiguration = 0xe0009,
			eUidSetI2CConfiguration = 0xe000a,
			eUidI2CRead = 0xe000b,
			eUidI2CReadRegister = 0xe000c,
			eUidI2CWrite = 0xe000d,
			eUidI2CWriteRegister = 0xe000e,
		};
		
		class InterconnectConfigClient
		{
			static const uint32_t m_serviceVersion = 1;
			static const uint32_t m_serviceId = eIdInterconnectConfig;
			NotificationHandler m_notificationHandler;

		protected:
			IRouterClient* const m_clientRouter;

		public:
			InterconnectConfigClient(IRouterClient* clientRouter);
			static uint32_t getUniqueFctId(uint16_t fctId);

			Kinova::Api::Common::UARTConfiguration GetUARTConfiguration(const Kinova::Api::Common::UARTDeviceIdentification& uartdeviceidentification, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetUARTConfiguration_callback(const Kinova::Api::Common::UARTDeviceIdentification& uartdeviceidentification, std::function< void (const Error&, const Kinova::Api::Common::UARTConfiguration&) > callback, uint32_t deviceId = 0);
			std::future<Kinova::Api::Common::UARTConfiguration> GetUARTConfiguration_async(const Kinova::Api::Common::UARTDeviceIdentification& uartdeviceidentification, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetUARTConfiguration(const Kinova::Api::Common::UARTConfiguration& uartconfiguration, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetUARTConfiguration_callback(const Kinova::Api::Common::UARTConfiguration& uartconfiguration, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetUARTConfiguration_async(const Kinova::Api::Common::UARTConfiguration& uartconfiguration, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			EthernetConfiguration GetEthernetConfiguration(const EthernetDeviceIdentification& ethernetdeviceidentification, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetEthernetConfiguration_callback(const EthernetDeviceIdentification& ethernetdeviceidentification, std::function< void (const Error&, const EthernetConfiguration&) > callback, uint32_t deviceId = 0);
			std::future<EthernetConfiguration> GetEthernetConfiguration_async(const EthernetDeviceIdentification& ethernetdeviceidentification, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetEthernetConfiguration(const EthernetConfiguration& ethernetconfiguration, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetEthernetConfiguration_callback(const EthernetConfiguration& ethernetconfiguration, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetEthernetConfiguration_async(const EthernetConfiguration& ethernetconfiguration, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			GPIOConfiguration GetGPIOConfiguration(const GPIOIdentification& gpioidentification, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetGPIOConfiguration_callback(const GPIOIdentification& gpioidentification, std::function< void (const Error&, const GPIOConfiguration&) > callback, uint32_t deviceId = 0);
			std::future<GPIOConfiguration> GetGPIOConfiguration_async(const GPIOIdentification& gpioidentification, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetGPIOConfiguration(const GPIOConfiguration& gpioconfiguration, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetGPIOConfiguration_callback(const GPIOConfiguration& gpioconfiguration, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetGPIOConfiguration_async(const GPIOConfiguration& gpioconfiguration, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			GPIOState GetGPIOState(const GPIOIdentification& gpioidentification, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetGPIOState_callback(const GPIOIdentification& gpioidentification, std::function< void (const Error&, const GPIOState&) > callback, uint32_t deviceId = 0);
			std::future<GPIOState> GetGPIOState_async(const GPIOIdentification& gpioidentification, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetGPIOState(const GPIOState& gpiostate, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetGPIOState_callback(const GPIOState& gpiostate, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetGPIOState_async(const GPIOState& gpiostate, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			I2CConfiguration GetI2CConfiguration(const I2CDeviceIdentification& i2cdeviceidentification, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetI2CConfiguration_callback(const I2CDeviceIdentification& i2cdeviceidentification, std::function< void (const Error&, const I2CConfiguration&) > callback, uint32_t deviceId = 0);
			std::future<I2CConfiguration> GetI2CConfiguration_async(const I2CDeviceIdentification& i2cdeviceidentification, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void SetI2CConfiguration(const I2CConfiguration& i2cconfiguration, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetI2CConfiguration_callback(const I2CConfiguration& i2cconfiguration, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetI2CConfiguration_async(const I2CConfiguration& i2cconfiguration, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			I2CData I2CRead(const I2CReadParameter& i2creadparameter, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void I2CRead_callback(const I2CReadParameter& i2creadparameter, std::function< void (const Error&, const I2CData&) > callback, uint32_t deviceId = 0);
			std::future<I2CData> I2CRead_async(const I2CReadParameter& i2creadparameter, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			I2CData I2CReadRegister(const I2CReadRegisterParameter& i2creadregisterparameter, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void I2CReadRegister_callback(const I2CReadRegisterParameter& i2creadregisterparameter, std::function< void (const Error&, const I2CData&) > callback, uint32_t deviceId = 0);
			std::future<I2CData> I2CReadRegister_async(const I2CReadRegisterParameter& i2creadregisterparameter, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void I2CWrite(const I2CWriteParameter& i2cwriteparameter, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void I2CWrite_callback(const I2CWriteParameter& i2cwriteparameter, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> I2CWrite_async(const I2CWriteParameter& i2cwriteparameter, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void I2CWriteRegister(const I2CWriteRegisterParameter& i2cwriteregisterparameter, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void I2CWriteRegister_callback(const I2CWriteRegisterParameter& i2cwriteregisterparameter, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> I2CWriteRegister_async(const I2CWriteRegisterParameter& i2cwriteregisterparameter, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});


		private:
			void messageHeaderValidation(const Frame& msgFrame){ /* todogr ... */ }
		};
	}
}
}

#endif