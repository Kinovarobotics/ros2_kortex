#ifndef KINOVAEXCEPTION_H
#define KINOVAEXCEPTION_H

#include <string>
#include <sstream>
#include <exception>

namespace Kinova
{
namespace Api
{

    class KBasicException : public std::runtime_error
    {
    public:
        KBasicException(const std::string& msgStr);
        KBasicException(const KBasicException &other);

        virtual const char* what() const throw() override;
        virtual std::string toString();
    };

    // todogr add a new kinova exception: KProgException
    typedef KBasicException KProgException;

} // namespace Api
} // namespace Kinova

#endif
