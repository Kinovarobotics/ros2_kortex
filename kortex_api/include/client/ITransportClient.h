#ifndef _I_TRANSPORT_CLIENT_H_
#define _I_TRANSPORT_CLIENT_H_


#include <string>
#include <functional>
#include <exception>

#include <cmath>

using namespace std;

namespace Kinova
{
namespace Api
{
    // enums
    enum TransportReadyStateEnum 
    {
        CONNECTING = 0,
        OPEN = 1,
        CLOSING = 2,
        CLOSED = 3,
        UNINITIALIZED = 4,
        RECONNECTING = 5,
    };

    // interface
    class ITransportClient
    {
    public:
        TransportReadyStateEnum readyState;

        virtual ~ITransportClient() {}

        virtual bool connect(std::string host, uint32_t port) = 0;
        virtual void disconnect() = 0;

        virtual void send(const char* txBuffer, uint32_t txSize) = 0;
        virtual void onMessage(std::function<void (const char*, uint32_t)> callback) = 0;

        // virtual void onClose(ConnectionClose eventClose)> callback) = 0;
        // virtual void onError(ConnectionError error)> callback) = 0;

        // virtual bool isClosed() = 0;
        // virtual bool isClosing() = 0;
        // virtual bool isConnecting() = 0;
        // virtual bool isOpen() = 0;
        // virtual bool isUninitialized() = 0;

        virtual char* getTxBuffer(uint32_t const& allocation_size) = 0;
        virtual size_t getMaxTxBufferSize() = 0;

        virtual void getHostAddress(std::string &host, uint32_t &port) = 0;
    };


} // namespace Api
} // namespace Kinova

#endif // _I_TRANSPORT_CLIENT_H_
