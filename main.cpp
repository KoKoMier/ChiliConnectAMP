#include "src/ChiliAPM.hpp"

void SignalCatch(int Signal);

int main(int argc, char **argv)
{
    int argvs;

    while ((argvs = getopt(argc, argv, "h::a::s::g::m::c::f::R::j::")) != -1)
    {
        switch (argvs)
        {
        case 'h':
        {
            std::cout << "./ChiliAPM -a detect start"
                      << "\r\n";
            std::cout << "./ChiliAPM -s UartKey_test"
                      << "\r\n";
            std::cout << "./ChiliAPM -g Uartsend_test"
                      << "\r\n";
            std::cout << "./ChiliAPM -j Json_test"
                      << "\r\n";
            std::cout << "./ChiliAPM -m mtf-02_test"
                      << "\r\n";
        }
        break;
        case 'a':
        {
            ChiliAPM APM_Settle;
            signal(SIGINT, SignalCatch);
            signal(SIGTERM, SignalCatch);
            APM_Settle.ChiliAPMStartUp();
            APM_Settle.TaskThreadPrint();
        }
        break;
        case 's':
        {
            ChiliAPM APM_Settle;
            APM_Settle.UartKey_test();
        }
        break;
        case 'g':
        {
            ChiliAPM APM_Settle;    
            APM_Settle.Uartsend_test();

        }
        break;
        case 'j':
        {
            ChiliAPM APM_Settle;
            APM_Settle.Json_test();
        }
        break;
        case 'm':
        {
            ChiliAPM APM_Settle;
            APM_Settle.MTF02_test();
        }
        break;
        }
    }
}

void SignalCatch(int Signal)
{
    SystemSignal = Signal;
};