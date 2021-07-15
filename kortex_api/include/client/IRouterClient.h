#ifndef _I_ROUTER_CLIENT_H_
#define _I_ROUTER_CLIENT_H_


#include <string>
#include <future>
#include <functional>
#include <exception>

#include "Frame.pb.h"

#include "ITransportClient.h"
#include "KError.h"

using namespace std;

namespace Kinova
{
namespace Api
{
    typedef std::function<void (const Kinova::Api::Frame&)> MessageCallback;

    typedef struct
    {
        bool andForget;
        uint32_t delay_ms;
        uint32_t timeout_ms;
    
    } RouterClientSendOptions;

    class IRouterClient
    {
    public:
        virtual ~IRouterClient() {}

        virtual void reset() = 0;
        virtual void registerBridgingCallback(std::function<void (Frame &)> bridgingCallback) = 0;
        virtual void registerNotificationCallback( uint32_t serviceId, function<Error (Frame&)> ) = 0;
        virtual void registerErrorCallback( function<void(KError)> callback ) = 0;
        virtual void registerHitCallback(std::function<void (FrameTypes)> hitSessionCallback) = 0;

        virtual future<Frame> send(const std::string& txPayload, uint32_t serviceVersion, uint32_t funcId, uint32_t deviceId, const RouterClientSendOptions& options ) = 0;
        virtual Error sendWithCallback(const std::string& txPayload, uint32_t serviceVersion, uint32_t funcId, uint32_t deviceId, MessageCallback callback) = 0;
        virtual Error sendMsgFrame(const Frame& msgFrame) = 0;

        virtual uint16_t getConnectionId() = 0;
        virtual void SetActivationStatus(bool isActive) = 0;

        virtual ITransportClient* getTransport() = 0;
    };

} // namespace Api
} // namespace Kinova

#endif // _I_ROUTER_CLIENT_H_
