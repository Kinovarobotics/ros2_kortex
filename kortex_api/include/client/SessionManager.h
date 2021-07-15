#ifndef _SESSION_MANAGER_H_
#define _SESSION_MANAGER_H_

#include <iostream>
#include <list>
#include <future>
#include <memory>
#include <functional>

#include "IRouterClient.h"
#include "Frame.pb.h"
#include "Session.pb.h"
#include "SessionClientRpc.h"

namespace Kinova
{
namespace Api
{
    class SessionManager : public Session::SessionClient
    {
    public:
        SessionManager() = delete;
        explicit SessionManager(IRouterClient* router, std::function<void()> connectionTimeoutCallback = nullptr);
        virtual ~SessionManager();

        void CreateSession(const Session::CreateSessionInfo& info);
        void CloseSession();
        Session::ConnectionList GetConnections();

    private:
        void Hit(FrameTypes hitType);
        void ThreadSessionValidation();
    
        std::function<void()>           m_connectionTimeoutCallback;
        
        std::thread                     m_thread;
        std::atomic<bool>               m_hasBeenSent { false };
        std::atomic<bool>               m_hasBeenReceived { false };
        std::atomic<bool>               m_threadRunning { false };
        Session::CreateSessionInfo      m_sessionInfo;

        void checkTransport();
    };

}
}

#endif // _MESSAGE_MANAGER_H_
