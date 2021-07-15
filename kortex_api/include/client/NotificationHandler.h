#ifndef _NOTIFICATION_HANDLER_H_
#define _NOTIFICATION_HANDLER_H_


#include <memory>
#include <map>
#include <vector>
#include <functional>
#include <future>

#include "KBasicException.h"
#include "KDetailedException.h"

#include "Frame.pb.h"

#include "ITransportClient.h"
#include "IRouterClient.h"

using namespace std;

namespace Kinova
{
namespace Api
{

    struct AbstractCallbackFunction
    {
        AbstractCallbackFunction() = default;
        virtual ~AbstractCallbackFunction() = default;

        virtual Error call(Frame& msgFrameNotif) = 0;
    };

    template <class DataType>
    struct CallbackFunction : public AbstractCallbackFunction
    {
        static constexpr bool isOk = std::is_base_of<::google::protobuf::Message, DataType>::value;
        static_assert(isOk, "DataType must inherit from ::google::protobuf::Message");

        std::function< void (DataType) > m_callbackFct;

        CallbackFunction(std::function< void(DataType) > callback) : AbstractCallbackFunction() { m_callbackFct = callback; }
        virtual ~CallbackFunction() override {}

        virtual Error call(Frame& msgFrameNotif) override
        {
            Error error;
            error.set_error_code(ErrorCodes::ERROR_NONE);

            DataType decodedMsgNotif;
            if( !decodedMsgNotif.ParseFromString(msgFrameNotif.payload()) )
            {
                HeaderInfo headerInfo( msgFrameNotif.header() );

                error.set_error_code(ERROR_PROTOCOL_CLIENT);
                error.set_error_sub_code(PAYLOAD_DECODING_ERR);
                error.set_error_sub_string(string("The data payload could not be deserialized : notification for serviceId=") + to_string(headerInfo.m_serviceInfo.serviceId) + " \n");
            }
            else
            {
                // todo ???  thread(m_callbackFct, move(decodedMsgNotif)).detach();
                thread(m_callbackFct, decodedMsgNotif).detach();
            }

            return error;
        }
    };

    // todoErr add the validation information beside the callback
    typedef std::unordered_map< uint32_t, vector< shared_ptr<AbstractCallbackFunction> > > CallbackMap;

    class NotificationHandler
    {
        CallbackMap     m_callbackMap;
        std::mutex      m_mutex;

    public:
        NotificationHandler();
        ~NotificationHandler();

        template <class DataType>
        void addCallback( uint32_t idKey, std::function<void(DataType)> callback )
        {
            std::lock_guard<std::mutex> w_scoped(m_mutex);
            std::shared_ptr<AbstractCallbackFunction> fct = std::make_shared<CallbackFunction<DataType>>(callback);
            m_callbackMap[idKey].push_back( fct );
        }

        void clearIdKeyCallbacks( uint32_t idKey );
        void clearAll();

        Error call(Frame& msgFrameNotif);
    };

}
}

#endif // _NOTIFICATION_HANDLER_H_
