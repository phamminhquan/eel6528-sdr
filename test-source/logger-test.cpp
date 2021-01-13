#include "../include/logger.h"


using namespace std;


// prototypes


// global vars
logger main_logger ("Main", "../logs/main.log");


int main(int argc, char* argv[]) {
    main_logger.log(true, "Hello World!");

    return 0;
}
