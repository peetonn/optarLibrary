//
// logging.cpp
//
//  Created by Peter Gusev on 12 November 2019.
//  Copyright 2013-2019 Regents of the University of California
//
#include "logging.hpp"

#include <spdlog/async.h>
#include <spdlog/sinks/basic_file_sink.h>
#include <spdlog/sinks/stdout_color_sinks.h>

using namespace std;
using namespace optar;

#define DEFAULT_FORMAT "%E.%f [%12n] [%^%-8l%$] [thread %t] %!() : %v"

// logging level
// could be either:
// - "trace"
// - "debug"
// - "info"
// - "warn"
// - "err"
// - "critical"
#define LOG_LEVEL_ENV "OPTARLIB_LOG_LEVEL"
#define LOG_FORMAT_ENV "OPTARLIB_LOG_FMT"
#define LOG_FILE_ENV "OPTARLIB_LOG_FILE"

shared_ptr<spdlog::logger> mainLogger;
string logFile = "";
string logLevel = "";
once_flag onceFlag;

void initMainLogger();
void initLogger(shared_ptr<helpers::logger>);

// init logging upon library loading
struct _LibInitializer {
    _LibInitializer() {
        call_once(onceFlag, bind(initMainLogger));
    }
} libInitializer = {};

void initMainLogger()
{
    logLevel = getenv(LOG_LEVEL_ENV) ? string(getenv(LOG_LEVEL_ENV)) : "";
    logFile = getenv(LOG_FILE_ENV) ? string(getenv(LOG_FILE_ENV)) : "";

    if (logFile != "")
        mainLogger = spdlog::basic_logger_mt<spdlog::async_factory>("optar", logFile);
    else
        mainLogger = spdlog::stdout_color_mt("optar");

    spdlog::set_pattern(getenv(LOG_FORMAT_ENV) ? getenv(LOG_FORMAT_ENV) : DEFAULT_FORMAT);
    initLogger(mainLogger);
}

void initLogger(shared_ptr<helpers::logger> logger)
{
    logger->flush_on(spdlog::level::err);
    if (logLevel == "")
        logLevel = "info";

    logger->set_level(spdlog::level::from_str(logLevel));
    logger->info("Initialized logger {}: level {} file {}",  logger->name(),
        spdlog::level::to_short_c_str(logger->level()), logFile);
    logger->flush();
}

void newLogger(std::string loggerName)
{
    shared_ptr<spdlog::logger> logger;

    if (logFile != "")
        logger = spdlog::basic_logger_mt<spdlog::async_factory>(loggerName, logFile);
    else
        logger = spdlog::stdout_color_mt(loggerName);

    initLogger(logger);
}

shared_ptr<spdlog::logger> getLogger(string loggerName)
{
    auto logger = spdlog::get(loggerName);
    return logger;
}

void flushLogger(std::string loggerName)
{
    spdlog::get(loggerName)->flush();
}
