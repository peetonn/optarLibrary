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

#define SPDLOG_ACTIVE_LEVEL SPDLOG_LEVEL_TRACE
#include <spdlog/spdlog.h>

#define OLOG_TRACE SPDLOG_TRACE
#define OLOG_DEBUG SPDLOG_DEBUG
#define OLOG_INFO SPDLOG_INFO
#define OLOG_WARN SPDLOG_WARNING
#define OLOG_ERROR SPDLOG_ERROR

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
    }

    void newLogger(std::string loggerName);
    std::shared_ptr<helpers::logger> getLogger(std::string loggerName);
    void flushLogger(std::string loggerName);
}

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
    }

    void newLogger(std::string loggerName);
    std::shared_ptr<helpers::logger> getLogger(std::string loggerName);
    void flushLogger(std::string loggerName);
}

#endif

#endif
