// XGetopt.h  Version 1.2
//
// Author:  Hans Dietrich
//          hdietrich2@hotmail.com
//
// This software is released into the public domain.
// You are free to use it in any way you like.
//
// This software is provided "as is" with no expressed
// or implied warranty.  I accept no liability for any
// damage or loss of business that this software may cause.
//
///////////////////////////////////////////////////////////////////////////////

// Modified by Stephen Vidas, 2012

#if defined(_WIN32)

#ifndef XGETOPT_H
#define XGETOPT_H

#include <cstddef>
#include <windows.h>
#include <tchar.h>
#include <cstdio>
#include <string.h>

//#include <lpctstr.h>

typedef char TCHAR;

extern int optind, opterr;
extern TCHAR *optarg;

int getopt(int argc, TCHAR *argv[], TCHAR *optstring);

#endif //XGETOPT_H

#endif //defined(WIN32)
