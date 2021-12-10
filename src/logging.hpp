//
// logging.hpp
//
//  Created by Peter Gusev on 12 November 2019.
//  Copyright 2013-2019 Regents of the University of California
//

#ifndef __logging_hpp__
#define __logging_hpp__

#include "config.hpp"

#if HAVE_SPDLOG

#if DEBUG
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#else
#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_WARN
#endif

#include <spdlog/spdlog.h>

#define OPTAR_LOGGER_NAME "optar"
#define OPTAR_LOGGER (optar::getLogger(OPTAR_LOGGER_NAME))

#define OLOG_LOGGER_TRACE SPDLOG_LOGGER_TRACE
#define OLOG_LOGGER_DEBUG SPDLOG_LOGGER_DEBUG
#define OLOG_LOGGER_INFO SPDLOG_LOGGER_INFO
#define OLOG_LOGGER_WARN SPDLOG_LOGGER_WARN
#define OLOG_LOGGER_ERROR SPDLOG_LOGGER_ERROR
#define OLOG_LOGGER_CRITICAL SPDLOG_LOGGER_CRITICAL

#define OLOG_TRACE_TAG(tag, ...) (OLOG_TRACE(tag##__VA_ARGS__))

namespace optar {
    namespace helpers {
        typedef spdlog::logger logger;
        typedef spdlog::level::level_enum log_level;
        typedef std::function<void(const std::string&)> LogCallback;
    }

    void newLogger(std::string loggerName);
    std::shared_ptr<helpers::logger> getLogger(std::string loggerName);
    void flushLogger(std::string loggerName);
    void registerCallback(std::shared_ptr<helpers::logger>, helpers::LogCallback);
}

#define OLOG_TRACE(...) SPDLOG_LOGGER_TRACE(OPTAR_LOGGER, __VA_ARGS__)
#define OLOG_DEBUG(...) SPDLOG_LOGGER_DEBUG(OPTAR_LOGGER, __VA_ARGS__)
#define OLOG_INFO(...)  SPDLOG_LOGGER_INFO(OPTAR_LOGGER, __VA_ARGS__)
#define OLOG_WARN(...)  SPDLOG_LOGGER_WARN(OPTAR_LOGGER, __VA_ARGS__)
#define OLOG_ERROR(...) SPDLOG_LOGGER_ERROR(OPTAR_LOGGER, __VA_ARGS__)

#else

#define OLOG_TRACE
#define OLOG_DEBUG
#define OLOG_INFO
#define OLOG_WARN
#define OLOG_ERROR

#include <cstdlib>
#include <string>

namespace optar {
    namespace helpers {
        typedef void logger;
        typedef void LogCallback;
    }

    void newLogger(std::string loggerName);
    std::shared_ptr<helpers::logger> getLogger(std::string loggerName);
    void flushLogger(std::string loggerName);
    void registerCallback(std::shared_ptr<helpers::logger>, helpers::LogCallback);
}

#endif

#endif
