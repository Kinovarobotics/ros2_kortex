/* ***************************************************************************
 * Kinova inc.
 *
 * Copyright (c) 2006-2018 Kinova Incorporated. All rights reserved.
 ****************************************************************************/

#ifndef __TRANSPORT_CLIENT_TCP_H__
#define __TRANSPORT_CLIENT_TCP_H__

#if defined(_OS_WINDOWS)
// ---- win ----
#include <stdio.h>
#include <winsock2.h>
#elif defined(_OS_UNIX)
// --- linux ---
#include <sys/types.h>
#include <sys/socket.h>
#include <netinet/in.h>
#include <arpa/inet.h>
#include <netdb.h>
#include <sys/select.h>
#include <sys/fcntl.h>

#include <iostream>
#include <unistd.h>
#include <ctime>
#include <stdio.h>
#include <unistd.h>
#include <cerrno>
#include <cstdio>
#include <cstring>
#include <cstdlib>
#else
#warning Unknown OS type!
#endif

#include <atomic>
#include <thread>
#include <mutex>

#include <string>
#include <functional>
#include <exception>

#include <iostream>
#include <chrono>

#include "ITransportClient.h"
#include "KinovaTcpUtilities.h"

#if defined(_OS_WINDOWS)
typedef int32_t socklen_t;
#endif

namespace Kinova
{
namespace Api
{
    class TransportClientTcp : public ITransportClient
    {
    private:
        const uint32_t kApiPort					= 10000;
        // Configuration        m_config;
        bool                    m_isInitialized;
        struct sockaddr_in      m_socketAddr{};
        socklen_t               m_socketAddrSize{};
        int32_t                 m_socketFd{};
        #if defined(_OS_WINDOWS)
        WSADATA                 m_wsa{};
        #endif

        bool                    m_isUsingRcvThread;
        std::atomic<bool>       m_isRunning { true };
        std::mutex              m_sendMutex;

        // ---- non-blocking ----
        fd_set          m_original_rx{};
        fd_set          m_readfds{};

        int             numfd{};
        struct hostent  *m_host{};
        struct timeval  m_tv{};
        // ----------------------

        // 65535 - 20 (ip header) - 20 (tcp header) = 65495 bytes
        static constexpr uint32_t kMaxTxBufferSize = 65495;
        static constexpr uint32_t kMaxRxBufferSize = 65495;
        static constexpr uint32_t kMaxBufferSize = 16777216;

        bool                 m_bIsReceiving { false };
        uint32_t             m_nTotalBytesRead {0};
        uint32_t             m_nTotalBytesToRead {0};
        uint8_t*             m_tx_buffer;
        uint8_t*             m_rx_buffer;

        uint32_t             m_current_buffer_size_rx = { kMaxRxBufferSize };
        uint32_t             m_current_buffer_size_tx = { kMaxTxBufferSize };


        KinovaTcpUtilities m_utilities_object;

        std::function<void (const char*, uint32_t) > m_onMessageCallback;


    public:
        TransportReadyStateEnum readyState;
        std::thread             m_receiveThread;

        explicit TransportClientTcp(bool isUsingRcvThread = true);
        ~TransportClientTcp() override;

        bool connect(std::string host = "127.0.0.1", uint32_t port = 10000) override;
        void disconnect() override;

        void send(const char* txBuffer, uint32_t txSize) override;
        void onMessage(std::function<void (const char*, uint32_t)> callback) override;

        char* getTxBuffer(uint32_t const& allocation_size) override;
        size_t getMaxTxBufferSize() override { return kMaxBufferSize; }

        int processReceive(long rcvTimeout_usec);
        int processReceive(struct timeval rcvTimeout_tv);

        virtual void getHostAddress(std::string &host, uint32_t &port) override {
            host = mHostAddress;
            port = mHostPort;
        };

    private:

        std::string mHostAddress;
        uint32_t mHostPort;

        void receiveThread(std::atomic<bool> &program_is_running);
        
        int callReceiveFrom();
    };

} // namespace Api
} // namespace Kinova

#endif // __TRANSPORT_CLIENT_H__
