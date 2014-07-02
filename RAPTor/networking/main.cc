/* Standard headers */
#include <string>

/* Networking headers */
#include "udp_server.h"
#include "udp_client.h"
#include "subscription_response.h"


int main(int argc, char **argv)
{
    bool run_as_client = false;
    std::string host = "0.0.0.0";

    /* Parse input arguements */
    if (argc > 1)
    {
        for (int i=1; i<argc; i++)
        {
            if ((argv[i][0] == '-') && (argv[i][1] == '-'))
            {
                argv[i]++;
            }

            if (strcmp(argv[i], "-client") == 0)
            {
                run_as_client = true;
            }
        }
    }

    if (run_as_client)
    {
        raptor_networking::udp_client client(host, raptor_networking::CONTROL_PORT);
        client.try_subscribe_to_server();

        std::string recv;
        client.start_listening_for_refresh(&recv);
        std::cout << "Received: " << recv << std::endl;

        client.unsubscribe_from_server();
    }
    else
    {
        raptor_networking::udp_server server(raptor_networking::MULTI_CAST_ADDR, raptor_networking::CONTROL_PORT, raptor_networking::MULTI_CAST_PORT);
        server.start_listening_for_client(raptor_networking::console_ask_subscription_response());
        server.start_listening_for_client(raptor_networking::console_ask_subscription_response());
        std::cout << "sending multi cast" << std::endl;
        sleep(1);
        server.publish_unconfirmed("Multi cast hello world");
        server.start_listening_for_client(raptor_networking::console_ask_subscription_response());
        server.start_listening_for_client(raptor_networking::console_ask_subscription_response());
    }

    // raptor_networking::udp_connection conn;
    // if (run_as_client)
    // {
    //     conn.send_and_confirm(host, port, "Hello world!", 50, 1000);
    // }
    // else
    // {
    //     conn.listen_and_confirm(raptor_networking::UDP_ACKNOWLEDGE, boost::lexical_cast<int>(port), 50);
    // }

    return 0;
}