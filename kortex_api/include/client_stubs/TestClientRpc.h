#ifndef __TESTCLIENT_H__
#define __TESTCLIENT_H__

#include <string>
#include <future>
#include <functional>
#include <exception>

#include "Frame.pb.h"
#include "Test.pb.h"

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
	namespace Test
	{
		// todogr move somewhere else
		const std::string   none = "";
		
		enum FunctionUids
		{
			eUidSetMockValidationStruct = 0xff00001,
			eUidTestParamAndReturn = 0xff00002,
			eUidTestParamOnly = 0xff00003,
			eUidTestReturnOnly = 0xff00004,
			eUidTestTimeout = 0xff00005,
			eUidTestNotif = 0xff00006,
			eUidTestNotifUnsubscribe = 0xff00007,
			eUidTestAsync = 0xff00008,
			eUidTestConcurrence = 0xff00009,
			eUidTestTriggerNotif = 0xff0000a,
			eUidTestNotImplemented = 0xff0000b,
			eUidServerError = 0xff0000c,
			eUidUnsubscribe = 0xff0001e,
			eUidSomethingChangeTopic = 0xff0001f,
			eUidTriggerSomethingChangeTopic = 0xff00020,
			eUidWait = 0xff00021,
			eUidThrow = 0xff00022,
			eUidDisconnect = 0xff00023,
			eUidForget = 0xff00024,
			eUidNotImplemented = 0xff00025,
			eUidDeprecated = 0xff00027,
			eUidDeprecatedWithMessage = 0xff00028,
		};
		
		class TestClient
		{
			static const uint32_t m_serviceVersion = 1;
			static const uint32_t m_serviceId = eIdTest;
			NotificationHandler m_notificationHandler;

		protected:
			IRouterClient* const m_clientRouter;

		public:
			TestClient(IRouterClient* clientRouter);
			static uint32_t getUniqueFctId(uint16_t fctId);

			void SetMockValidationStruct(const validateStruct& validatestruct, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void SetMockValidationStruct_callback(const validateStruct& validatestruct, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> SetMockValidationStruct_async(const validateStruct& validatestruct, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			RcvStruct TestParamAndReturn(const SendStruct& sendstruct, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void TestParamAndReturn_callback(const SendStruct& sendstruct, std::function< void (const Error&, const RcvStruct&) > callback, uint32_t deviceId = 0);
			std::future<RcvStruct> TestParamAndReturn_async(const SendStruct& sendstruct, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void TestParamOnly(const SendStruct& sendstruct, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void TestParamOnly_callback(const SendStruct& sendstruct, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> TestParamOnly_async(const SendStruct& sendstruct, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			RcvStruct TestReturnOnly(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void TestReturnOnly_callback(std::function< void (const Error&, const RcvStruct&) > callback, uint32_t deviceId = 0);
			std::future<RcvStruct> TestReturnOnly_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void TestTimeout(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void TestTimeout_callback(std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> TestTimeout_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			Kinova::Api::Common::NotificationHandle OnNotificationTestNotif(std::function< void (TestNotification) > callback, const NotifOption& notifoption, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void TestNotifUnsubscribe(const Kinova::Api::Common::NotificationHandle& notificationhandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void TestAsync(const timeToResponse& timetoresponse, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void TestAsync_callback(const timeToResponse& timetoresponse, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> TestAsync_async(const timeToResponse& timetoresponse, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void TestConcurrence(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void TestConcurrence_callback(std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> TestConcurrence_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void TestTriggerNotif(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void TestTriggerNotif_callback(std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> TestTriggerNotif_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void TestNotImplemented(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void TestNotImplemented_callback(std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> TestNotImplemented_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			TestError ServerError(const TestError& testerror, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void ServerError_callback(const TestError& testerror, std::function< void (const Error&, const TestError&) > callback, uint32_t deviceId = 0);
			std::future<TestError> ServerError_async(const TestError& testerror, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void Unsubscribe(const Kinova::Api::Common::NotificationHandle& notificationhandle, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			Kinova::Api::Common::NotificationHandle OnNotificationSomethingChangeTopic(std::function< void (SomethingChanged) > callback, const Kinova::Api::Common::NotificationOptions& notificationoptions, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void TriggerSomethingChangeTopic(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void TriggerSomethingChangeTopic_callback(std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> TriggerSomethingChangeTopic_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void Wait(const Delay& delay, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void Wait_callback(const Delay& delay, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> Wait_async(const Delay& delay, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void Throw(const Kinova::Api::Error& error, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void Throw_callback(const Kinova::Api::Error& error, std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> Throw_async(const Kinova::Api::Error& error, uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void Disconnect(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void Disconnect_callback(std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> Disconnect_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void Forget(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void Forget_callback(std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> Forget_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			void NotImplemented(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			void NotImplemented_callback(std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			std::future<void> NotImplemented_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			DEPRECATED void Deprecated(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			DEPRECATED void Deprecated_callback(std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			DEPRECATED std::future<void> Deprecated_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});

			DEPRECATED_MSG("Purposely deprecated for test") void DeprecatedWithMessage(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});
			DEPRECATED_MSG("Purposely deprecated for test") void DeprecatedWithMessage_callback(std::function< void (const Error&) > callback, uint32_t deviceId = 0);
			DEPRECATED_MSG("Purposely deprecated for test") std::future<void> DeprecatedWithMessage_async(uint32_t deviceId = 0, const RouterClientSendOptions& options = {false, 0, 3000});


		private:
			void messageHeaderValidation(const Frame& msgFrame){ /* todogr ... */ }
		};
	}
}
}

#endif