#include "../include/logger.h"
#include "../include/config.h"


using namespace std;


// prototypes


// global vars
Logger main_logger ("Main", "../logs/main.log");
Config config ("../config/test.json", "../logs/config.log");


int main(int argc, char* argv[]) {
    main_logger.log(true, "Hello World!");

    return 0;
}
