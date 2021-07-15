/* ***************************************************************************
 * Kinova inc.
 * Project : 
 *
 * Copyright (c) 2006-2018 Kinova Incorporated. All rights reserved.
 ****************************************************************************/

#ifndef _TRANSPORT_BRIDGE_DUMMY_H_
#define _TRANSPORT_BRIDGE_DUMMY_H_

#include <string>
#include <functional>

#include "ITransportClient.h"
#include "ITransportServer.h"

namespace Kinova
{
    using namespace Api;
    
    namespace Helper
    {
        class TransportBridgeDummy
        {
            class TransportBridgeClientDummy : public ITransportClient
            {
                TransportBridgeDummy *m_transportBridgeDummy;
            
            public:
                TransportBridgeClientDummy(TransportBridgeDummy *transportBridgeDummy)
                {
                    m_transportBridgeDummy = transportBridgeDummy;
                }
                
                virtual ~TransportBridgeClientDummy();
                
                virtual bool connect(std::string host = "127.0.0.1", uint32_t port = kApiPort) override { return true; }
                
                virtual void disconnect() override {}
                
                virtual void send(const char *txBuffer, uint32_t txSize) override
                {
                    m_transportBridgeDummy->m_onMessageCallbackServer(m_transportBridgeDummy->m_connectionHandle, (uint8_t *) txBuffer, (int) txSize);
                }
                
                virtual void onMessage(std::function<void(const char *, uint32_t)> callbackClient) override
                {
                    m_transportBridgeDummy->m_onMessageCallbackClient = callbackClient;
                }
                
                virtual char *getTxBuffer() override { return m_transportBridgeDummy->m_bufferClientToServer; }
                
                virtual size_t getMaxTxBufferSize() override { return m_transportBridgeDummy->m_maxBufferSizeClientToServer; }
            };
            
            class TransportBridgeServerDummy : public ITransportServer
            {
                TransportBridgeDummy *m_transportBridgeDummy;
            
            public:
                TransportBridgeServerDummy(TransportBridgeDummy *transportBridgeDummy)
                {
                    m_transportBridgeDummy = transportBridgeDummy;
                }
                
                virtual ~TransportBridgeServerDummy() override {}
                
                virtual void Process() override {}
                
                
                virtual bool Send(const ClientConnectionHandle &connectionHandle, char *TXBuffer, int txLen) override
                {
                    m_transportBridgeDummy->m_onMessageCallbackClient((const char *) TXBuffer, (uint32_t) txLen);
                }
                
                virtual void OnMessage(std::function<void(const ClientConnectionHandle &, uint8_t *, int)> callbackServer) override
                {
                    m_transportBridgeDummy->m_onMessageCallbackServer = callbackServer;
                }
                
                virtual char *GetTxBuffer(size_t *pTxLen) override
                {
                    *pTxLen = m_transportBridgeDummy->m_maxBufferSizeServerToClient;
                    
                    return m_transportBridgeDummy->m_bufferServerToClient;
                }
            };
            
            ITransportServer *m_server;
            ITransportClient *m_client;
        
        public:
            static constexpr uint32_t kMaxBufferSizeServerToClient = 65507;
            static constexpr uint32_t kMaxBufferSizeClientToServer = 65507;
            
            const uint32_t m_maxBufferSizeServerToClient = kMaxBufferSizeServerToClient;
            const uint32_t m_maxBufferSizeClientToServer = kMaxBufferSizeClientToServer;
            
            char m_bufferServerToClient[kMaxBufferSizeServerToClient];
            char m_bufferClientToServer[kMaxBufferSizeClientToServer];
            
            std::function<void(const char *, uint32_t)> m_onMessageCallbackClient;
            std::function<void(const ClientConnectionHandle &, uint8_t *, int)> m_onMessageCallbackServer;
            
            struct ClientConnectionHandle m_connectionHandle;
            
            TransportBridgeDummy()
            {
                m_connectionHandle.clientId = 1;
                auto m_server = new TransportBridgeServerDummy(this);
                auto m_client = new TransportBridgeClientDummy(this);
            }
            
            virtual ~TransportBridgeDummy()
            {
                delete m_server;
                delete m_client;
            }
            
            ITransportServer *getServerTransport() { return m_server; }
            
            ITransportClient *getClientTransport() { return m_client; }
        };
        
    }
}
#endif
