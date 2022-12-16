#include <iostream>

#include <iostream>
#include <string>
#include <boost/asio.hpp>
#include "Udp_core/Udp_server.h"
#include "Udp_core/Udp_client.h"

// this function exist only for unit test for udp_core
int main() {

    const string localhost="127.0.0.1";

    Udp_server udp_server(12344);
    Udp_client udp_client(12345,localhost,12344);

    while(true)
    {
        //make sure that all other threads have been fired
        if(!udp_client.is_init_done()||!udp_server.is_init_done())
        {
            this_thread::sleep_for(chrono::milliseconds(1));
            continue;
        }
        this_thread::sleep_for(chrono::milliseconds(1));
        udp_client.add_snd_request("123456789");
    }
    return 0;
}


