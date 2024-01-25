#ifndef __CCD_PRECISION_H__
#define __CCD_PRECISION_H__

/* define either CCD_SINGLE or CCD_DOUBLE */

#if defined(CCD_IDESINGLE)
#define CCD_SINGLE
#elif defined(CCD_IDEDOUBLE)
#define CCD_DOUBLE
#else
#define CCD_DOUBLE
#endif

#endif
