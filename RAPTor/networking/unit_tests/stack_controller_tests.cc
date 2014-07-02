#ifdef STAND_ALONE
#define BOOST_TEST_MODULE stack_controller test
#endif /* #ifdef STAND_ALONE */


/* Standard headers */
#include <atomic>

/* Boost headers */
#include "boost/uuid/uuid.hpp"
#include "boost/test/unit_test.hpp"
#include "boost/asio/io_service.hpp"

/* Tbb headers */
#include "tbb/concurrent_hash_map.h"

/* Networking headers */
#include "stack_map_element.h"
#include "stack_accessor.h"
#include "stack_controller.h"

/* Test headers */
#include "stack_component_fixture.h"
#include "mock_data_receiver.h"
#include "mock_stack_component.h"


using boost::uuids::uuid;
using namespace raptor_networking;


/* Test data */
struct stack_controller_fixture : public stack_component_fixture<mock_stack_component>
{
    stack_controller_fixture()
        : ran_gener(),
          uuid_gener(&ran_gener),
          helloworld("hello world")
    {
        ran_gener.seed(time(NULL));
        link_uut(new mock_stack_component(bottom_node));
    };

    stack_component::copied_pair new_clone_result()
    {
        auto clone_result = delivery_node->clean_clone();
        clone_result.top()->build_up_links();
        return clone_result;
    }
    
    typedef boost::uuids::basic_random_generator<boost::mt19937> uuid_gen;
    typedef tbb::concurrent_hash_map<uuid, stack_map_element*, hash_compare<uuid>> stack_map;

    stack_map                   stacks;
    boost::mt19937              ran_gener;  /* Random number generator to drive the uuid generator  */
    uuid_gen                    uuid_gener; /* Uuid generator for process ids                       */
    std::string                 helloworld;
};

BOOST_FIXTURE_TEST_SUITE( stack_controller_tests, stack_controller_fixture )

/* stack_map_element tests */
/* Test constructors */
BOOST_AUTO_TEST_CASE( stack_map_element_ctor_test )
{
    auto clone_result = new_clone_result();
    stack_map_element ele(clone_result);

    BOOST_CHECK(ele.stack_top() == clone_result.top());
    /* This shold be mid not bottom */
    /* A connection is always at the bottom, but isnt counted as part of the stack in this context */
    BOOST_CHECK(ele.stack_bottom() == clone_result.bottom());
    BOOST_CHECK(ele.next_sequence() == 0);
}


BOOST_AUTO_TEST_CASE( stack_map_element_seq_test )
{
    auto clone_result = new_clone_result();
    stack_map_element ele(clone_result);

    BOOST_CHECK(ele.next_sequence() == 0);
    BOOST_CHECK(ele.next_sequence() == 1);
    BOOST_CHECK(ele.next_sequence() == 2);
    BOOST_CHECK(ele.next_sequence() == 3);
    BOOST_CHECK(ele.next_sequence() == 4);
}


/* Test hash compare */
BOOST_AUTO_TEST_CASE( hash_compare_test )
{
    auto clone_result = new_clone_result();
    stack_map_element ele(clone_result);
    auto nil_uuid = boost::uuids::nil_uuid();
    BOOST_CHECK(stacks.size() == 0);

    /* Test any it even works with concurrent_hash_map */
    {

        stack_map::accessor acc;
        BOOST_CHECK(stacks.insert(acc, nil_uuid));
        BOOST_CHECK(acc->first == nil_uuid);
        BOOST_CHECK(acc->second == nullptr);
        acc->second = &ele;
        BOOST_CHECK(stacks.size() == 1);
    }

    /* Test the first key can be matched */
    {
        stack_map::accessor acc;
        BOOST_CHECK(!stacks.insert(acc, nil_uuid));
        BOOST_CHECK(acc->first == nil_uuid);
        BOOST_CHECK(acc->second == &ele);
        BOOST_CHECK(stacks.size() == 1);
    }

    /* Test it can be not matched */
    {
        auto uuid0 = uuid_gener();
        stack_map::accessor acc;
        BOOST_CHECK(stacks.insert(acc, uuid0));
        BOOST_CHECK(acc->first == uuid0);
        BOOST_CHECK(acc->second == nullptr);
        BOOST_CHECK(stacks.size() == 2);
    }
}


/* stack_accessor */
BOOST_AUTO_TEST_CASE( stack_accessor_implicit_conversion_test )
{
    stack_accessor acc;
    auto nil_uuid = boost::uuids::nil_uuid();
    BOOST_CHECK(stacks.insert(acc, nil_uuid));
    BOOST_CHECK(acc.address() == nil_uuid);
}


BOOST_AUTO_TEST_CASE( stack_accessor_create_element_test )
{
    auto clone_result = new_clone_result();
    stack_map_element ele(clone_result);
    auto nil_uuid = boost::uuids::nil_uuid();

    stack_accessor acc;
    BOOST_CHECK(stacks.insert(acc, nil_uuid));
    BOOST_CHECK(acc.address() == nil_uuid);
    BOOST_CHECK(stacks.size() == 1);
    
    acc.create_element(&ele);
    BOOST_CHECK(acc.stack_top() == ele.stack_top());
    BOOST_CHECK(acc.stack_bottom() == ele.stack_bottom());
    BOOST_CHECK(acc.next_sequence() == 0);
    BOOST_CHECK(acc.next_sequence() == 1);
    BOOST_CHECK(acc.next_sequence() == 2);
    BOOST_CHECK(acc.next_sequence() == 3);
}


BOOST_AUTO_TEST_CASE( stack_accessor_release_test )
{
    auto nil_uuid = boost::uuids::nil_uuid();

    stack_accessor acc0;
    BOOST_CHECK(stacks.insert(acc0, nil_uuid));
    BOOST_CHECK(acc0.address() == nil_uuid);

    acc0.release();

    stack_accessor acc1;
    BOOST_CHECK(!stacks.insert(acc1, nil_uuid));
    BOOST_CHECK(acc1.address() == nil_uuid);
}


/* stack_controller */
BOOST_AUTO_TEST_CASE( stack_controller_ctor_test )
{
    stack_controller ctrl(ioservice);
    BOOST_CHECK(&ctrl.io_service() == &ioservice);
}


BOOST_AUTO_TEST_CASE( stack_controller_create_stack_test )
{
    /* Create the stack controller */
    auto clone_result = new_clone_result();
    stack_controller ctrl(ioservice);
    ctrl.clean_stack(clone_result.top(), clone_result.bottom());
    BOOST_CHECK(ctrl.number_of_stacks() == 0);

    /* Lock a stack */
    stack_accessor acc;
    auto nil_uuid = boost::uuids::nil_uuid();
    ctrl.lock_stack(acc, nil_uuid);
    BOOST_CHECK(ctrl.number_of_stacks() == 1);
    acc.release();

    /* and again */
    ctrl.lock_stack(acc, nil_uuid);
    BOOST_CHECK(ctrl.number_of_stacks() == 1);
    acc.release();

    /* and just incase a different one */
    ctrl.lock_stack(acc, uuid_gener());
    BOOST_CHECK(ctrl.number_of_stacks() == 2);
    acc.release();
}


BOOST_AUTO_TEST_CASE( stack_controller_run_test )
{
    /* Create the stack controller */
    stack_controller ctrl(ioservice);
    auto clone_result = new_clone_result();
    ctrl.clean_stack(clone_result.top(), clone_result.bottom());
    BOOST_CHECK(ctrl.number_of_stacks() == 0);

    /* Run some work */
    std::string output;
    auto nil_uuid = boost::uuids::nil_uuid();
    ctrl.run(nil_uuid, [&clone_result, &output, this](const stack_accessor &acc)
        {
            output = helloworld;
        });
    BOOST_CHECK(ctrl.number_of_stacks() == 1);
    BOOST_CHECK(output == helloworld);
}


BOOST_AUTO_TEST_CASE( stack_controller_post_test )
{
    /* Create the stack controller */
    stack_controller ctrl(ioservice);
    auto clone_result = new_clone_result();
    ctrl.clean_stack(clone_result.top(), clone_result.bottom());
    BOOST_CHECK(ctrl.number_of_stacks() == 0);

    /* Create some work */
    std::atomic<int> output;
    auto nil_uuid = boost::uuids::nil_uuid();

    output.store(0);
    ctrl.post(nil_uuid, [&output, this](const stack_accessor &acc)
        {
            output.store(1);
        });
    BOOST_CHECK(ctrl.number_of_stacks() == 0);
    BOOST_CHECK(output.load() == 0);

    /* Start some threads to process it and wait */
    ctrl.start(2);
    while (!ctrl.io_service().stopped())
    {
        std::this_thread::sleep_for(std::chrono::milliseconds(1));
    }
    BOOST_CHECK(ctrl.number_of_stacks() == 1);
    BOOST_CHECK(output.load() == 1);
}


BOOST_AUTO_TEST_SUITE_END()
