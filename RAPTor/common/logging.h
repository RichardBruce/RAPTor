#ifndef __LOGGING_H__
#define __LOGGING_H__

/* Standard headers */
#include <numeric>
#include <sstream>
#include <string>

/* Boost headers */
#include "boost/log/core.hpp"
#include "boost/log/trivial.hpp"
#include "boost/log/expressions.hpp"


namespace raptor_physics
{
class init_logger
{
    public :
        init_logger()
        {
            boost::log::core::get()->set_filter
            (
                boost::log::trivial::severity >= BOOST_LOG_LEVEL
            );
        }
};


/* Method entry and exit logging RAII class */
class method_logger
{
    public :
        method_logger(const std::string &method) : _method(method)
        {
            BOOST_LOG_TRIVIAL(info) << "Begin " << _method;
        }

        ~method_logger()
        {
            BOOST_LOG_TRIVIAL(info) << "End " << _method;
        }

    private :
        const std::string _method;
};

#ifdef __GNUC__ 
#define METHOD_LOG method_logger __method_logger(__PRETTY_FUNCTION__);
#else
    #error Method log statement not defined for this compiler
#endif /* #ifdef __GNUC__ */


/* Utility functions to convert arrays to stream for logging */
template<class InputIterator, class Formatter>
std::string array_to_stream(InputIterator b, InputIterator e, const Formatter &f)
{
    typedef std::iterator_traits<InputIterator> traits;

    std::stringstream ss;
    std::accumulate(b, e, &ss, [&f](std::stringstream *ss, typename traits::value_type i) { return f(ss, i); });

    return ss.str();
}

template<class InputIterator, class Formatter, class PostFix>
std::string array_to_stream(InputIterator b, InputIterator e, const Formatter &f, PostFix post)
{
    typedef std::iterator_traits<InputIterator> traits;
    
    std::stringstream ss;
    std::accumulate(b, e, &ss, [&f](std::stringstream *ss, typename traits::value_type i) { return f(ss, i); });

    ss << post;
    return ss.str();
}

template<class T, class Formatter, class PostFix>
std::string array_to_stream(const T *const t, const Formatter &f, const int size, const int step, PostFix post)
{
    std::stringstream ss;
    for (int i = 0; i < size; i += step)
    {
        f(&ss, t[i]);
    }

    ss << post;
    return ss.str();
}
}; /* namespace raptor_physics */

#endif /* #ifndef __LOGGING_H__ */
