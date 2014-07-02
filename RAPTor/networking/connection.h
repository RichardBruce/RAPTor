#ifndef __CONNECTION_H__
#define __CONNECTION_H__

/* Networking headers */
#include "stack_component.h"
#include "stack_accessor.h"


namespace raptor_networking
{
/* Forward declarations */
class group;
class stack_controller;

/* Pure virtual class to represent a connection */
template<class UpNode>
class connection : public stack_component
{
    public :
        connection(stack_controller *const ctrl, group *const grp)
            : _ctrl(ctrl), _grp(grp) {  };

        /* Access functions */
        group *const stack_group() const { return _grp; }

        /* Virtual function to start listening to connections */
        virtual void start_receiving() = 0;

        stack_component& link_up_node(UpNode *const up_node)
        {
            return *this;
        }


        /* Virtual function to clone as was constructed */
        virtual stack_component::copied_pair clean_clone() override
        {
            /* This is the bottom level of the stack and not cloned */
            return copied_pair(this);
        }

        /* Virtual function to connect the upward links of the stack */
        virtual stack_component& build_up_links(void *const sc) override
        {
            /* No up link is directly held */
            /* Instead we go through the stack controller to find a link per address */
            return *this;
        }

    protected :
        stack_controller *const _ctrl;
        group *const            _grp;
};
}; /* namespace raptor_networking */

#endif /* #ifndef __CONNECTION_H__ */