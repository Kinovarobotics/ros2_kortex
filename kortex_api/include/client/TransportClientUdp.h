/* ***************************************************************************
 * Kinova inc.
 * Project :
 *
 * Copyright (c) 2006-2018 Kinova Incorporated. All rights reserved.
 ****************************************************************************/

#ifndef __TRANSPORT_CLIENT_UDP_H__
#define __TRANSPORT_CLIENT_UDP_H__

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
#include <netdb.h>      // host struct
#include <sys/select.h> // use select() for multiplexing
#include <sys/fcntl.h>  // for non-blocking

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

#if defined(_OS_WINDOWS) // todogr
typedef int32_t socklen_t;
#endif

namespace Kinova
{
namespace Api
{
    const uint32_t kApiPort					= 10000;

    class TransportClientUdp : public ITransportClient
    {
        // Configuration        m_config;
        bool                   	m_isInitialized;
        struct sockaddr_in      m_socketAddr;
        socklen_t               m_socketAddrSize;
        int32_t                 m_socketFd;
        #if defined(_OS_WINDOWS)
        WSADATA                 m_wsa;
        #endif

        bool                    m_isUsingRcvThread;
        std::atomic<bool>       m_isRunning { true };
        std::mutex              m_sendMutex;

        // ---- non-blocking ----
        fd_set          m_original_rx;
        fd_set          m_readfds;

        int             numfd;
        struct hostent  *m_host;
        struct timeval  m_tv;
        // ----------------------

        // 65535 - 20 (ip header) - 8 (udp header) = 65507 bytes
        static constexpr uint32_t kMaxTxBufferSize = 65507;
        static constexpr uint32_t kMaxRxBufferSize = 65507;

        char m_txBuffer[kMaxTxBufferSize];
        char m_rxBuffer[kMaxRxBufferSize];

        std::function<void (const char*, uint32_t) > m_onMessageCallback;

        // int32_t m_countTx;
        // int32_t m_countRx;
        // ChronoClock m_start;
        // ChronoClock m_elapsed;

    public:
        TransportReadyStateEnum readyState;
        std::thread             m_receiveThread;

        TransportClientUdp(bool isUsingRcvThread = true);
        virtual ~TransportClientUdp();

        virtual bool connect(std::string host = "127.0.0.1", uint32_t port = Kinova::Api::kApiPort) override;
        virtual void disconnect() override;

        virtual void send(const char* txBuffer, uint32_t txSize) override;
        virtual void onMessage(std::function<void (const char*, uint32_t)> callback) override;

        // virtual void onClose(ConnectionClose eventClose)> callback) = 0;
        // virtual void onError(ConnectionError error)> callback) = 0;

        // virtual bool isClosed() = 0;
        // virtual bool isClosing() = 0;
        // virtual bool isConnecting() = 0;
        // virtual bool isOpen() = 0;
        // virtual bool isUninitialized() = 0;

        virtual char* getTxBuffer(uint32_t const& allocation_size) override { return m_txBuffer; }
        virtual size_t getMaxTxBufferSize() override { return kMaxTxBufferSize; }

        // double getTimeComm();
    
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
        
        // return value: <0 means error (-errorCode); =0 means timeout nothing received; >0 means nbr of handles ready to recvFrom
        int callReceiveFrom();
    };

} // namespace Api
} // namespace Kinova

#endif // __TRANSPORT_CLIENT_H__
