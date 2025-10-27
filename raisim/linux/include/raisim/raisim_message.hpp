//----------------------------//
// This file is part of RaiSim//
// Copyright 2020, RaiSim Tech//
//----------------------------//


#ifndef RAISIM_MESSAGE_HPP
#define RAISIM_MESSAGE_HPP

#include "raisim_message_logger.hpp"
#include <math.h>

#define RSMSG(msg, severity) { std::stringstream raimessagestream; \
                                raimessagestream<<msg; \
                                  raisim::RaiSimMsg().stream(__FILE__, __LINE__, raimessagestream, severity); }

#define RSINFO(msg) RSMSG(msg, raisim::RSEVERITY_INFO)
#define RSWARN(msg) RSMSG(msg, raisim::RSEVERITY_WARN)
#define RSFATAL(msg) RSMSG(msg, raisim::RSEVERITY_FATAL)
#define RSRETURN(con, msg) RSMSG(msg, raisim::RSEVERITY_INFO)return;

#define RSINFO_IF(con, msg) if(con) RSMSG(msg, raisim::RSEVERITY_INFO)
#define RSWARN_IF(con, msg) if(con) RSMSG(msg, raisim::RSEVERITY_WARN)
#define RSFATAL_IF(con, msg) if(con) RSMSG(msg, raisim::RSEVERITY_FATAL)
#define RSASSERT(con, msg) if(!(con)) RSMSG(msg, raisim::RSEVERITY_FATAL)
#define RSRETURN_IF(con, msg) if(con) {RSMSG(msg, raisim::RSEVERITY_INFO)return;}
#define RSISNAN(val) RSFATAL_IF(isnan(val), #val<<" is nan");
#define RSISNAN_MSG(val, msg) RSFATAL_IF(isnan(val), msg);

#define RSMSGF(fmt, severity, ...) { char rsbuffer[1024]; \
                                      std::snprintf(rsbuffer, sizeof(rsbuffer), fmt, __VA_ARGS__); \
                                      RSMSG(rsbuffer, severity); }

#define RSINFOF(fmt, ...) RSMSGF(fmt, raisim::RSEVERITY_INFO, __VA_ARGS__)
#define RSWARNF(fmt, ...) RSMSGF(fmt, raisim::RSEVERITY_WARN, __VA_ARGS__)
#define RSFATALF(fmt, ...) RSMSGF(fmt, raisim::RSEVERITY_FATAL, __VA_ARGS__)
#define RSRETURNF(con, fmt, ...) RSMSGF(fmt, raisim::RSEVERITY_INFO, __VA_ARGS__)return;

#define RSINFO_IFF(con, fmt, ...) if(con) RSMSGF(fmt, raisim::RSEVERITY_INFO, __VA_ARGS__)
#define RSWARN_IFF(con, fmt, ...) if(con) RSMSGF(fmt, raisim::RSEVERITY_WARN, __VA_ARGS__)
#define RSFATAL_IFF(con, fmt, ...) if(con) RSMSGF(fmt, raisim::RSEVERITY_FATAL, __VA_ARGS__)
#define RSASSERTF(con, fmt, ...) if(!(con)) RSMSGF(fmt, raisim::RSEVERITY_FATAL, __VA_ARGS__)
#define RSRETURN_IFF(con, fmt, ...) if(con) {RSMSGF(fmt, raisim::RSEVERITY_INFO, __VA_ARGS__) return;}
#define RSISNAN_MSGF(val, fmt, ...) RSFATAL_IFF(isnan(val), fmt, __VA_ARGS__);

#ifdef RSDEBUG
  #define DRSINFO(msg) RSINFO(msg)
  #define DRSWARN(msg) RSWARN(msg)
  #define DRSFATAL(msg) RSFATAL(msg)

  #define DRSINFO_IF(con, msg) RSINFO_IF(con, msg)
  #define DRSWARN_IF(con, msg) RSWARN_IF(con, msg)
  #define DRSFATAL_IF(con, msg) RSFATAL_IF(con, msg)

  #define DRSASSERT(con, msg) RSASSERT(con, msg)
  #define DRSRETURN_IF(con, msg) RSINFO_IF(con, msg) return;
  #define DRSISNAN(val) RSFATAL_IF(isnan(val), #val<<" is nan");
  #define DRSISNAN_MSG(val, msg) RSFATAL_IF(isnan(val), msg);

  #define DRSINFOF(fmt, ...) RSINFOF(fmt, __VA_ARGS__)
  #define DRSWARNF(fmt, ...) RSWARNF(fmt, __VA_ARGS__)
  #define DRSFATALF(fmt, ...) RSFATALF(fmt, __VA_ARGS__)

  #define DRSINFO_IFF(con, fmt, ...) RSINFO_IFF(con, fmt, __VA_ARGS__)
  #define DRSWARN_IFF(con, fmt, ...) RSWARN_IFF(con, fmt, __VA_ARGS__)
  #define DRSFATAL_IFF(con, fmt, ...) RSFATAL_IFF(con, fmt, __VA_ARGS__)

  #define DRSASSERTF(con, fmt, ...) RSASSERTF(con, fmt, __VA_ARGS__)
  #define DRSRETURN_IFF(con, fmt, ...) RSRETURN_IFF(con, fmt, __VA_ARGS__)
  #define DRSISNAN_MSGF(val, fmt, ...) RSISNAN_MSGF(val, fmt, __VA_ARGS__)
#else
  #define DRSINFO(msg)
  #define DRSWARN(msg)
  #define DRSFATAL(msg)

  #define DRSINFO_IF(con, msg)
  #define DRSWARN_IF(con, msg)
  #define DRSFATAL_IF(con, msg)

  #define DRSASSERT(con, msg)
  #define DRSRETURN_IF(con, msg)
  #define DRSISNAN_MSG(val, msg)
  #define DRSISNAN(val)

  #define DRSINFOF(fmt, ...)
  #define DRSWARNF(fmt, ...)
  #define DRSFATALF(fmt, ...)

  #define DRSINFO_IFF(con, fmt, ...)
  #define DRSWARN_IFF(con, fmt, ...)
  #define DRSFATAL_IFF(con, fmt, ...)

  #define DRSASSERTF(con, fmt, ...)
  #define DRSRETURN_IFF(con, fmt, ...)
  #define DRSISNAN_MSGF(val, fmt, ...)
#endif

#endif // RAISIM_MESSAGE_HPP