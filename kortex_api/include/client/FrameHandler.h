#ifndef _MESSAGE_MANAGER_H_
#define _MESSAGE_MANAGER_H_

#include <iostream>
#include <list>
#include <future>
#include <memory>
#include <queue>
#include <chrono>

#include "HeaderInfo.h"
#include "Frame.pb.h"

using namespace std::chrono;

namespace Kinova
{
namespace Api
{
    struct CallbackTimeoutStruct
    {

        uint32_t m_timeout;
        steady_clock::time_point m_registeredTimePoint;

        explicit CallbackTimeoutStruct(uint32_t timeout)
            : m_timeout(timeout)
        {
            m_registeredTimePoint = time_point_cast<milliseconds>(steady_clock::now());
        }

        bool checkExpiry()
        {
            uint32_t duration = duration_cast<milliseconds>(steady_clock::now() - m_registeredTimePoint).count();
            return (duration >= m_timeout);
        }
    };

    typedef std::function<void (const Kinova::Api::Frame&)> MessageCallback;
    typedef std::queue<std::pair<uint32_t, CallbackTimeoutStruct> > TimeoutClockByMeesageCallbackQueue;

    class FrameHandler
    {
    public:
        explicit FrameHandler(uint32_t maxCallbackTimeout = 60000);
        ~FrameHandler() = default;

        std::future<Frame> registerMessage(uint32_t msgId);
        // TODO sfforget 2018-05-30 Add error callback
        void registerMessageCallback(uint32_t msgId, const MessageCallback& callback);

        Error manageReceivedMessage(Frame &msgFrame);
        void setMessageException(HeaderInfo& headerInfo, Error& error);

    private:
        void cleanDanglingCallback();

        std::mutex      m_mutex;
        uint32_t        m_maxCallbackTimeout;
        // IdGenerator<uint32_t> m_apiIdGenerator;

        // TODO cleanup messages without response from server to avoid memory leak.
        // todoErr add the validation information beside the message promise
        std::unordered_map< uint32_t, std::shared_ptr<std::promise<Frame>> > m_messagePromises;
        TimeoutClockByMeesageCallbackQueue m_timeoutClockByCallback;
        std::unordered_map< uint32_t, MessageCallback > m_messageCallbacks;
    };

}
}

#endif // _MESSAGE_MANAGER_H_
