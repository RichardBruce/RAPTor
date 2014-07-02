#ifndef __STACK_MAP_ELEMENT_H__
#define __STACK_MAP_ELEMENT_H__

/* Standard headers */
#include <cstdint>

/* Networking headers */
#include "stack_component.h"


namespace raptor_networking
{
/* Forward declarations */
class message_delivery;

/* Class to hold all the information you made need when access in stack */
class stack_map_element
{
    public :
        stack_map_element(const stack_component::copied_pair &stack)
            : _top(stack.top()), _bot(stack.bottom()), _seq(0) {  };

        ~stack_map_element()
        {
            delete _bot;
        }

        /* Get the top of the protocol stack */
        message_delivery *const stack_top()     { return _top;      }

        /* Get the bottom of the protocol stack */
        stack_component *const  stack_bottom()  { return _bot;      }

        /* Get the next sequence id for this stack. Sequence is allowed to wrap */
        std::uint32_t           next_sequence() { return _seq++;    }

    private :
        message_delivery  *const    _top;
        stack_component   *const    _bot;
        std::uint32_t               _seq;
};
}; /* namespace raptor_networking */
#endif /* #ifndef __STACK_MAP_ELEMENT_H__ */
