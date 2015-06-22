#pragma once

/* Standard headers */
#include <type_traits>
#include <vector>

/* Networking headers */
#include "serialisation.h"


namespace raptor_networking
{
/* Class send an object as a response */
template<class T>
class object_response
{
    /* Must be default constructible or serialisation wont work */
    static_assert(std::is_default_constructible<T>::value, "object_response::Type must be default constructible");

    public :
        object_response(const T obj) : _obj(obj) { };

        std::vector<char>* operator()(const std::unique_ptr<std::istream> &req) const
        {
            std::vector<char> *serial_obj = new std::vector<char>();
            return serialise(serial_obj, _obj);
        }

    private :
        const T _obj;
};
}; /* namespace raptor_networking */
