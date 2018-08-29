#pragma once

#include "ap_version.h"

#define THISFIRMWARE "APM:Copter V3.5.5"
#define FIRMWARE_VERSION 3,5,5,FIRMWARE_VERSION_TYPE_OFFICIAL

#ifndef GIT_VERSION
#define FIRMWARE_STRING THISFIRMWARE
#else
#define FIRMWARE_STRING THISFIRMWARE " (" GIT_VERSION ")"
#endif
#if AERIALTRONICS
// Modify version string
#undef FIRMWARE_STRING
#define FIRMWARE_STRING THISFIRMWARE "AT " __DATE__ " " __TIME__
#endif
