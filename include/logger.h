#ifndef LOGGER_H
#define LOGGER_H

#include <iostream>
#include <cstdio>
#include <cstdlib>
#include <stdexcept>
#include <cstdarg>
#include <mutex>


using namespace std;


// Logger to file
class Logger {
    private:
        // log file handler
        FILE * file;
        
        // log header (can be describing a function or entity, ex: main, ...)
        std::string header;

        // filepath
        std::string filepath;

        // mutex locking the log file write privilege
        std::mutex mutex;

    public:
        // constructor
        Logger(std::string _header, std::string _filepath) {
            // putting input argument to object attribute
            filepath = _filepath;
            header = _header;

            // open file in append mode
            file = std::fopen(_filepath.c_str(), "a");

            // check file open error
            if (file == NULL) {
                // notify user of error by printing to cout
                std::cout << "Error: Can't open " << _filepath << " for logging" << std::endl;

                // throw error to stop program
                throw std::invalid_argument("Can't open log file");
            }

            // file open successfully, notify the user by logging to file and cout
            this->log("Status: Started new logger at %s", _filepath.c_str());
        }

        // destructor
        ~Logger() {
            // notify use that the object is closing
            this->log("Status: Closing the logger");

            // close the file
            std::fclose(file);
        }

        // log message with a timestamp
        void log(std::string s, ...) {
            // determine arguments
            va_list args;   // get list of arguments
            va_start(args, s);  // put additional arguments to s

            // get the time for timestamp
            std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
            auto now_resolved = std::chrono::time_point_cast<std::chrono::microseconds>(now); // cast timepoint to microseconds
            auto value = now_resolved.time_since_epoch(); // microseconds since epoch
            long duration = value.count(); // count of microseconds since epoch
            std::string dur_str = std::to_string(duration); // turn count to string

            // build message string
            std::string format ("[" + dur_str + "] [" + header + "] " + s + "\n"); // create print format
            char * buffer = new char[1024 + format.length()];
            vsprintf(buffer, format.c_str(), args);
            
            // close variable arguments
            va_end(args);

            // print out message if to_cout is true
            std::fputs(buffer, stdout);
            
            // lock mutex before write to file
            // wrap with try-catch for error handling
            try {
                // print message to file
                std::fputs(buffer, file);

                // flush write buffer
                std::fflush(file);
            } catch (...) {
                // open log file
                file  = std::fopen(filepath.c_str(), "a");

                // check if file is open
                if (file == NULL) {
                    // notify user of error by printing to cout
                    std::cout << "Error: Can't open " << filepath << " for logging" << std::endl;

                    // throw error to stop program
                    throw std::invalid_argument("Can't open log file");
                }
            }
        }

        // log message with a timestamp
        void logf(std::string s, ...) {
            // determine arguments
            va_list args;   // get list of arguments
            va_start(args, s);  // put additional arguments to s

            // get the time for timestamp
            std::chrono::time_point<std::chrono::system_clock> now = std::chrono::system_clock::now();
            auto now_resolved = std::chrono::time_point_cast<std::chrono::microseconds>(now); // cast timepoint to microseconds
            auto value = now_resolved.time_since_epoch(); // microseconds since epoch
            long duration = value.count(); // count of microseconds since epoch
            std::string dur_str = std::to_string(duration); // turn count to string

            // build message string
            std::string format ("[" + dur_str + "] [" + header + "] " + s + "\n"); // create print format
            char * buffer = new char[1024 + format.length()];
            vsprintf(buffer, format.c_str(), args);
            
            // close variable arguments
            va_end(args);
          
            // lock mutex before write to file
            // wrap with try-catch for error handling
            try {
                // print message to file
                std::fputs(buffer, file);

                // flush write buffer
                std::fflush(file);
            } catch (...) {
                // open log file
                file  = std::fopen(filepath.c_str(), "a");

                // check if file is open
                if (file == NULL) {
                    // notify user of error by printing to cout
                    std::cout << "Error: Can't open " << filepath << " for logging" << std::endl;

                    // throw error to stop program
                    throw std::invalid_argument("Can't open log file");
                }
            }
        }
};

#endif //LOGGER_H
