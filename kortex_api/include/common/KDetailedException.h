#ifndef KINOVASERVEREXCEPTION_H
#define KINOVASERVEREXCEPTION_H

#include <string>
#include <sstream>

#include "KBasicException.h"

#include <Frame.pb.h>
#include "KError.h"

#include "HeaderInfo.h"

namespace Kinova
{
namespace Api
{
    class KDetailedException : public KBasicException
    {
    public:
        KDetailedException(const KError& error);
        KDetailedException(const KDetailedException &other);

        virtual const char* what() const throw() override;
        virtual std::string toString() override;

        KError&      getErrorInfo() { return m_error; }

    private:
        void init(const HeaderInfo& header, const Error& error);
    
        KError       m_error;
        std::string  m_errorStr;
    };

} // namespace Api
} // namespace Kinova

#endif
