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
#endif

#endif // RAISIM_MESSAGE_HPP
