#ifndef __DEVICEMANAGERCLIENT_H__
#define __DEVICEMANAGERCLIENT_H__

#include <string>
#include <future>
#include <functional>
#include <exception>

#include "Frame.pb.h"
#include "DeviceManager.pb.h"

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
	namespace DeviceManager
	{
		// todogr move somewhere else
		const std::string   none = "";
		
		enum FunctionUids
		{
			eUidReadAllDevices = 0x170001,
		};
		
		class DeviceManagerClient
		{
			static const uint32_t m_serviceVersion = 1;
			static const uint32_t m_serviceId = eIdDeviceManager;
			NotificationHandler m_notificationHandler;

		protected:
			IRouterClient* const m_clientRouter;

		public:
			DeviceManagerClient(IRouterClient* clientRouter);
			static uint32_t getUniqueFctId(uint16_t fctId);

			DeviceHandles ReadAllDevices(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ReadAllDevices_callback(std::function< void (const Error&, const DeviceHandles&) > callback, uint32_t deviceId = 0);
			std::future<DeviceHandles> ReadAllDevices_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});


		private:
			void messageHeaderValidation(const Frame& msgFrame){ /* todogr ... */ }
		};
	}
}
}

#endif