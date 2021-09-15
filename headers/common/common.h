/*
 * File:		common.h
 * Purpose:		File to be included by all project files
 *
 * Notes:
 */

#ifndef _COMMON_H_
#define _COMMON_H_

/********************************************************************/

/*
 * Debug prints ON (#define) or OFF (#undef)
 */
//#define DEBUG_PRINT 1
//#define DEBUG_PRINT_D0D1 1

/* 
 * Include the generic CPU header file 
 */
#include "mcf5xxx.h"
//#include "processor.h"			//FSL added this header which specifies the ColdFire processor and evb specific headers

/* 
 * Include any toolchain specfic header files 
 */
#if (defined(__CW__))
#include "build/cw/cw.h"
#define __CFM68K__ 0
#define __MC68K__  0
#elif (defined(__DCC__))
#include "build/wrs/diab.h"
#elif (defined(__ghs__))
#include "build/ghs/ghs.h"
#endif

/* 
 * Include common utilities
 */
//#include "io.h"
#include "mii.h"		//FSL added

/********************************************************************/

#endif /* _COMMON_H_ */
