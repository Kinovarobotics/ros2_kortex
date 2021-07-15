#ifndef _ROUTER_CLIENT_H_
#define _ROUTER_CLIENT_H_


#include <string>
#include <future>
#include <functional>
#include <exception>

#include "Frame.pb.h"

#include "ITransportClient.h"
#include "IRouterClient.h"

#include "FrameHandler.h"


using namespace std;

namespace Kinova
{
namespace Api
{

    typedef map< uint32_t, function<Error (Frame&)> > NotificationServices;

    class RouterClient : public IRouterClient
    {
	private:
        ITransportClient* const m_transport;
    
        NotificationServices        m_notificationServices;
        function<void(KError)>      m_errorCallback;
        function<void(FrameTypes)>  m_hitSessionCallback;
        function<void(Frame&)>      m_bridgingCallback;

        FrameHandler            m_frameHandler;
        uint16_t                m_msgId;
        uint16_t                m_sessionId;

        //A flag indicating if the instance can be used.
        bool                    m_isActive;

        std::mutex              m_send_mutex;

    public:
        RouterClient(ITransportClient* transport, function<void (KError)> errorCallback);
        virtual ~RouterClient();

        virtual void reset() override;
        virtual void registerBridgingCallback(std::function<void (Frame &)> bridgingCallback) override;
        virtual void registerNotificationCallback( uint32_t serviceId, function<Error (Frame&)> ) override;
        virtual void registerErrorCallback( function<void(KError)> callback ) override;
        virtual void registerHitCallback(std::function<void (FrameTypes)> hitSessionCallback) override;

        virtual future<Frame> send(const std::string& txPayload, uint32_t serviceVersion, uint32_t funcId, uint32_t deviceId, const RouterClientSendOptions& options ) override;
        virtual Error sendWithCallback(const std::string& txPayload, uint32_t serviceVersion, uint32_t funcId, uint32_t deviceId, MessageCallback callback) override;
        virtual Error sendMsgFrame(const Frame& msgFrame) override;

        virtual uint16_t getConnectionId() override;
        virtual void SetActivationStatus(bool isActive) override;

        virtual ITransportClient* getTransport() override {return m_transport;}

    private:
        uint16_t generateNewMsgId();
        void frameHandler(const void *rxBuffer, uint32_t rxSize);
    };

} // namespace Api
} // namespace Kinova

#endif // _ROUTER_CLIENT_H_
