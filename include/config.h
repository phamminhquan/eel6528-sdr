#ifndef CONFIG_H
#define CONFIG_H


#include <string>
#include "logger.h"
#include <exception>
#include <fstream>
#include <jsoncpp/json/json.h>

using namespace std;


// Config class with json
class Config {
    public:
        // logger
        Logger *logger;

        // config vars TODO: update config variables to whatever needed
        // e.g. float center_freq;

        // constructor
        inline Config (std::string config_filepath, std::string _logFilepath) {
            // set up logger
            logger = new Logger("Config", _logFilepath);
            try {
                // json parsing
                std::ifstream file(config_filepath);
                Json::Value root;   // will contain root value after parsing
                Json::Reader reader;
                bool parsingSuccessful = reader.parse(file, root);
                if (!parsingSuccessful) {
                    // throw parsing error
                    throw "Failed to parse config file:\n" + reader.getFormattedErrorMessages();
                }

                // TODO: extract key and values from file
                // e.g. center_freq = root["center_freq"].asFloat();
                
                // TODO: log config to check at run time

            } catch (exception& e) {
                // log error
                logger->log(e.what());

                // throw error
                throw "Failed to construct Config object, exception is in log file";
            }
        };
};


#endif //CONFIG_H
