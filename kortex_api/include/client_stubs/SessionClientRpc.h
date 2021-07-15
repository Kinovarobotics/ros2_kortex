#ifndef __SESSIONCLIENT_H__
#define __SESSIONCLIENT_H__

#include <string>
#include <future>
#include <functional>
#include <exception>

#include "Frame.pb.h"
#include "Session.pb.h"

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
	namespace Session
	{
		// todogr move somewhere else
		const std::string   none = "";
		
		enum FunctionUids
		{
			eUidCreateSession = 0x10001,
			eUidCloseSession = 0x10002,
			eUidKeepAlive = 0x10003,
			eUidGetConnections = 0x10004,
		};
		
		class SessionClient
		{
			static const uint32_t m_serviceVersion = 1;
			static const uint32_t m_serviceId = eIdSession;
			NotificationHandler m_notificationHandler;

		protected:
			IRouterClient* const m_clientRouter;

		public:
			SessionClient(IRouterClient* clientRouter);
			static uint32_t getUniqueFctId(uint16_t fctId);

			void CreateSession(const CreateSessionInfo& createsessioninfo, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void CreateSession_callback(const CreateSessionInfo& createsessioninfo, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> CreateSession_async(const CreateSessionInfo& createsessioninfo, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void CloseSession(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void CloseSession_callback(std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> CloseSession_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void KeepAlive(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void KeepAlive_callback(std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> KeepAlive_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			ConnectionList GetConnections(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void GetConnections_callback(std::function< void (const Error&, const ConnectionList&) > callback, uint32_t deviceId = 0);
			std::future<ConnectionList> GetConnections_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});


		private:
			void messageHeaderValidation(const Frame& msgFrame){ /* todogr ... */ }
		};
	}
}
}

#endif