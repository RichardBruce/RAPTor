#ifndef __STACK_CONTROLLER_H__
#define __STACK_CONTROLLER_H__

/* Standard headers */
#include <cstdint>
#include <string>
#include <thread>
#include <vector>

/* Boost headers */
#include "boost/uuid/uuid.hpp"
#include "boost/uuid/uuid_io.hpp"
#include "boost/asio/io_service.hpp"

/* Tbb headers */
#include "tbb/concurrent_hash_map.h"

/* Networking headers */
#include "stack_component.h"
#include "message_delivery.h"
#include "stack_map_element.h"
#include "stack_accessor.h"


namespace raptor_networking
{
using boost::uuids::uuid;

/* Data structure to hold a stack that may be executed in parallel */
class stack_controller
{
    public :
        stack_controller(boost::asio::io_service &io_service)
            : _io_service(io_service), _clean_stack_top(nullptr), _clean_stack_bottom(nullptr) {  };

        ~stack_controller()
        {
            /* Stop io service */
            _io_service.stop();

            /* Wait for threads to complete */
            for (auto *const thread : _threads)
            {
                thread->join();
                delete thread;
            }

            /* Clean the base stack */
            if (_clean_stack_bottom != nullptr)
            {
                delete _clean_stack_bottom;
            }

            /* Clear all the stack strands */
            for (auto &s : _stacks)
            {
                delete s.second;
            }
        }

        stack_controller& clean_stack(message_delivery *const clean_stack_top, stack_component *const clean_stack_bottom)
        {
            _clean_stack_top = clean_stack_top;
            _clean_stack_bottom = clean_stack_bottom;
            return *this;
        }

        stack_controller& start(const size_t nr_threads)
        {
            /* Start the io service thread pool */
            _threads.reserve(nr_threads);
            for (size_t i = 0; i < nr_threads; ++i)
            {
                std::thread *const thread = new std::thread(std::bind(
                    (std::size_t(boost::asio::io_service::*)())&boost::asio::io_service::run, &_io_service));
                _threads.push_back(thread);
            }

            return *this;
        }

        /* Member to take control of the stack if you chose to go it alone */
        stack_controller& lock_stack(stack_accessor &acc, const uuid &addr)
        {
            /* Insert a new stack if one isnt present and lock whatever is found */
            if (_stacks.insert(acc, addr))
            {
                auto new_stack = _clean_stack_top->clean_clone();
                acc.create_element(new stack_map_element(new_stack));
            }

            return *this;
        }

        /* Thread safe access to the stack */
        /* Send the lambda to an io service thread to run, but dont run it within this function */
        template<class Lambda>
        stack_controller& post(const uuid &addr, const Lambda &func)
        {
            _io_service.post([this, addr, func]()
            {
                run(addr, func);
            });

            return *this;
        }

        /* Send the lambda to an io service thread to run. If this thread is an io service thread run it within this funtcion */
        template<class Lambda>
        stack_controller& run(const uuid &addr, const Lambda &func)
        {
            stack_accessor acc;
            lock_stack(acc, addr);
            func(acc);

            return *this;
        }

        /* Access functions */
        boost::asio::io_service&    io_service()                { return _io_service;       }
        size_t                      number_of_stacks() const    { return _stacks.size();    }

    private :
        typedef tbb::concurrent_hash_map<uuid, stack_map_element*, hash_compare<uuid>> stack_map;

        boost::asio::io_service &   _io_service;
        std::vector<std::thread *>  _threads;
        stack_map                   _stacks;
        message_delivery *          _clean_stack_top;
        stack_component *           _clean_stack_bottom;
};
}; /* namespace raptor_networking */
#endif /* #ifndef __STACK_CONTROLLER_H__ */
