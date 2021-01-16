#include "../include/logger.h"
#include <uhd/utils/safe_main.hpp>
#include <boost/program_options.hpp>
#include <csignal>


// set namespace to shorten code
namespace po = boost::program_options;


// prototypes
void sig_int_handler(int);


// global vars
static bool stop_signal_called = false;
Logger main_logger ("Main", "../logs/main.log");


// safe main
int UHD_SAFE_MAIN(int argc, char* argv[]) {
    
    // variables for program options
    std::string name;
    int age;

    // set up program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "help message")
        ("name,n", po::value<std::string>(&name)->default_value("N/A"), "Person's name")
        ("age,a", po::value<int>(&age)->default_value(-1), "Person's age")
        ("nice-print", "Print out info nicely");

    po::variables_map vm;
    po::store(po::parse_command_line(argc, argv, desc), vm);
    po::notify(vm);

    // print the help message
    if (vm.count("help") or vm.count("h")) {
        std::cout << desc << std::endl;
        return EXIT_SUCCESS;
    }

    // instantiate CTRL-C interrupt handler
    std::signal(SIGINT, &sig_int_handler);

    // while loop to keep running
    while (not stop_signal_called) {
        if (vm.count("nice-print")) {
            // print out nicely using logger
            main_logger.log("Name is " + name);
            if (age < 0) {
                main_logger.log("Age is unknown");
            } else {
                main_logger.log("Age is " + std::to_string(age));
            }
        } else {
            // print out not so nicely
            main_logger.log("Name: " + name);
            main_logger.log("Age: " + std::to_string(age));
        }
        // pause for 1 second so result can be read
        sleep(1);
    }
    
    return EXIT_SUCCESS;
}


// Functions
// Interrupt handler
void sig_int_handler(int) {
    stop_signal_called = true;
}
