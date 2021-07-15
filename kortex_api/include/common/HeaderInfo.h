#ifndef KINOVAUTILAPI_H
#define KINOVAUTILAPI_H

#include "Frame.pb.h"

namespace Kinova
{
namespace Api
{

    typedef union
    {
        uint32_t frame_info;
        struct
        {
            uint32_t errorSubCode: 12;
            uint32_t errorCode: 4;
            uint32_t deviceId: 8;

            uint32_t frameType: 4;
            uint32_t headerVersion : 4;
        };
    } FrameInfo;

    typedef union
    {
        uint32_t message_info;
        struct
        {
            uint32_t messageId: 16;
            uint32_t sessionId: 16;
        };
    } MessageInfo;
    
    typedef union
    {
        uint32_t service_info;
        struct
        {
            uint32_t functionId: 16;
            uint32_t serviceId: 12;
            uint32_t serviceVersion: 4;
        };
        
        struct
        {
            uint32_t functionUid: 28;
        };
    } ServiceInfo;
    
    typedef union
    {
        uint32_t payload_info;
        struct
        {
            uint32_t payloadLength: 24;
            uint32_t reserved: 8;
        };
    } PayloadInfo;
    
    class HeaderInfo
    {
    public:
        HeaderInfo();
        HeaderInfo(const Header& headerProto);

        Header createHeader();

        void fillHeader(Header* pHeader);

        FrameInfo        m_frameInfo;
        MessageInfo      m_messageInfo;
        ServiceInfo      m_serviceInfo;
        PayloadInfo      m_payloadInfo;
    };


} // namespace Api
} // namespace Kinova

#endif
