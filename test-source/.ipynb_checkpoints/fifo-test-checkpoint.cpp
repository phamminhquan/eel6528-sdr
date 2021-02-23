#include "../include/logger.h"
#include "../include/fifo.h"
#include <thread>
#include <uhd/utils/safe_main.hpp>
#include <uhd/utils/thread.hpp>
#include <boost/program_options.hpp>
#include <sstream>
#include <csignal>


// set namespace to shorten code
namespace po = boost::program_options;


// prototypes
void sig_int_handler(int);
void print_age(std::string name, int age, tsFIFO<std::string>& fifo);


// global vars
static bool stop_signal_called = false;
Logger main_logger ("Main", "../logs/main.log");


// safe main
int UHD_SAFE_MAIN(int argc, char* argv[]) {
    
    // set main thread prio to highest
    uhd::set_thread_priority_safe(1.0, true);

    // variables for program options
    std::string name;
    int age;

    // set up program options
    po::options_description desc("Allowed options");
    desc.add_options()
        ("help,h", "help message")
        ("name,n", po::value<std::string>(&name)->default_value("N/A"), "Person's name")
        ("age,a", po::value<int>(&age)->default_value(-1), "Person's age");

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

    // construct fifo to hold all messages
    tsFIFO<std::string> fifo;

    // spawn threads
    int num_threads = 3;
    std::thread thread_worker[num_threads];
    for (int i=0; i<num_threads; i++) {
        thread_worker[i] = std::thread(&print_age, name, age, std::ref(fifo));
    }

    std::string message;

    // while loop to keep running
    while (not stop_signal_called) {
        if (fifo.pop(message))
            main_logger.log(message + " (" + std::to_string(fifo.size()) +
                    " messages left in FIFO)");
        else
            main_logger.log(" FIFO is empty");
        std::this_thread::sleep_for(std::chrono::milliseconds(100));
    }
    
    // join threads
    for (int i=0; i<num_threads; i++) {
        if (thread_worker[i].joinable())
            thread_worker[i].join();
    }

    return EXIT_SUCCESS;
}


// Functions
// Interrupt handler
void sig_int_handler(int) {
    stop_signal_called = true;
}

// thread function
void print_age(std::string name, int age, tsFIFO<std::string>& fifo) {
    // set thread prio
    uhd::set_thread_priority_safe(1.0, true);
    // construct string to push to fifo
    std::ostringstream message;
    message << "Main thread popped message from thread #"
        << std::this_thread::get_id()
        << ": " + name + " is " + std::to_string(age)
        << " years old";
    // check ctrl-c
    while (not stop_signal_called) {
        fifo.push(message.str());
        std::this_thread::sleep_for(std::chrono::seconds(1));
    }
}
