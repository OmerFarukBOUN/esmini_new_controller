#include "esminiLib.hpp"
#include <zenoh.hxx>
#include <iostream>
#include <string>
#include <vector>
#include <optional>

// Parse and strip --osi_subscriber_keyexpr argument
std::pair<std::optional<std::string>, std::vector<char*>> parseArgs(int argc, char** argv)
{
    std::optional<std::string> keyexprOpt;
    std::vector<char*>         newArgv;
    newArgv.push_back(argv[0]);  // keep program name

    for (int i = 1; i < argc; ++i)
    {
        std::string arg = argv[i];
        if (arg == "--osi_subscriber_keyexpr")
        {
            if (i + 1 < argc)
            {
                std::string arg_str = argv[i + 1];
                if (arg_str[0] != '-')
                {
                    keyexprOpt = arg_str;
                    i++;  // skip the value
                }
                else
                {
                    keyexprOpt = "esmini/gt";
                }
            }
            else
            {
                // if flag given without value, use default
                keyexprOpt = "esmini/gt";
            }
        }
        else
        {
            newArgv.push_back(argv[i]);
        }
    }

    return {keyexprOpt, newArgv};
}

int main(int argc, char** argv)
{
    // Parse arguments and strip custom flag
    auto [keyexprOpt, newArgv] = parseArgs(argc, argv);
    int newArgc                = static_cast<int>(newArgv.size());
std::cout << "newArgv (" << newArgc << " args):" << std::endl;
for (int i = 0; i < newArgc; ++i)
{
    std::cout << "  [" << i << "]: " << newArgv[i] << std::endl;
}
    // Init esmini
    if (SE_InitWithArgs(newArgc, const_cast<const char**>(newArgv.data())) != 0)
    {
        std::cerr << "Failed to initialize esmini" << std::endl;
        return -1;
    }

    // Optional Zenoh setup
    std::optional<zenoh::Session>   pub_session;
    std::optional<zenoh::Publisher> publisher;

    if (keyexprOpt.has_value())
    {
        std::string keyexpr = keyexprOpt.value().empty() ? "esmini/gt" : keyexprOpt.value();

        auto config = zenoh::Config::create_default();
        pub_session = zenoh::Session::open(std::move(config));
        publisher   = pub_session->declare_publisher(zenoh::KeyExpr(keyexpr));

        std::cout << "Zenoh publisher set on keyexpr: " << keyexpr << std::endl;
    }
    else
    {
        std::cout << "No --osi_subscriber_keyexpr provided. Skipping Zenoh publish." << std::endl;
    }

    // Run simulation loop
    while (SE_GetQuitFlag() == 0)
    {
        SE_Step();

        // Get OSI groundtruth (adjust API to your needs)
        int         size   = 0;
        const void* buffer = SE_GetOSIGroundTruth(&size);
        if (buffer && size > 0 && publisher.has_value())
        {
            publisher->put(std::string_view((const char*)buffer, size));
        }
    }

    SE_Close();
    return 0;
}