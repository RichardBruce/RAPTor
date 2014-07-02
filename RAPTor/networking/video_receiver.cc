/* Standard headers */
#include <memory>

/* Networking headers */
#include "serialisation.h"
#include "msg_client.h"

/* Display headers */
#include "sdl_event_handler.h"

/* Raytracer headers */
#include "camera.h"
#include "perlin_noise_3d_mapper.h"

void my_terminate()
{
    std::cout << "program about to terminate" << std::endl;
    abort();
}

namespace
{
    /* Invoke set_terminate as part of global constant initialization */
    static const bool SET_TERMINATE = std::set_terminate(my_terminate);
}


/* Main routine */
int main(int argc, char **argv)
{
    /* Check number of arguements */
    if (argc != 4)
    {
        std::cout << "Usage: " << std::endl;
        std::cout << "./video_receiver <server address> <send port> <recv port>" << std::endl;
        std::cout << "Example: ./video_receiver 0.0.0.0 7000 7001" << std::endl;
        std::cout << std::endl;
        return 1;
    }

    /* Construct udp client */
    const std::string addr(argv[1]);
    const short send_port = boost::lexical_cast<short>(argv[2]);
    const short recv_port = boost::lexical_cast<short>(argv[3]);
    std::unique_ptr<raptor_networking::msg_client> client (new raptor_networking::msg_client("0.0.0.0", send_port, recv_port, 0));

    /* Attempt to connect to server */
    std::cout << "Attempting to connect to server: " << addr << " " << send_port << " " << recv_port << std::endl;
    if (!client->start("videocst", addr, 10000))
    {
        std::cout << "Connection with sever could not be established" << std::endl;
        return 2;
    }

    /* Drawing loop */
    int do_next = 0;
    std::unique_ptr<sdl_event_handler_base> cam_event_handler(client->event_handler());
    while (do_next != 1)
    {
        /* Process user input */
        do_next = cam_event_handler->wait_for_event();

        /* TODO -- Send events back to the server */
    }

    /* msg_client DTOR will let the server know the client is exitting */
    return 0;
}