#ifndef KINOVAERROR_H
#define KINOVAERROR_H

#include <string>
#include <sstream>

#include "Frame.pb.h"

#include "HeaderInfo.h"

namespace Kinova
{
namespace Api
{
    class KError
    {
    public:
        KError(Kinova::Api::ErrorCodes errorCode, Kinova::Api::SubErrorCodes errorSubCode, std::string errorDescription);
        KError(const HeaderInfo& header, Kinova::Api::ErrorCodes errorCode, Kinova::Api::SubErrorCodes errorSubCode, std::string errorDescription);
        KError(const Error& error);
        KError(const HeaderInfo& header, const Error& error);

        static Error fillError(Kinova::Api::ErrorCodes errorCode, Kinova::Api::SubErrorCodes errorSubCode, std::string errorDescription);

        std::string toString() const;

        bool            isThereHeaderInfo();
        HeaderInfo      getHeader();
        Error           getError();

        KError& operator =(const KError& other) = default;

    private:
        bool            m_isThereHeaderInfo;
        HeaderInfo      m_header;
        Error           m_error;
    };

} // namespace Api
} // namespace Kinova

#endif
