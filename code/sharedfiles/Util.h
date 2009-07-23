#ifndef Header_Util
#define Header_Util

#include <math.h>
#include <time.h>

static double TRUNC(double value, int decimal_places)
{
    double integer = 0,
           fractional = 0,
           output = 0;

    int j = 0,
        places = 1;

    fractional = modf(value, &output);
    for( int i = 0; i < decimal_places + 1; i++ )
    {
        fractional = modf(fractional, &integer);
        for( j = 0; j < i; j++ )
        {
            places *= 10;
        }
        output += integer / places;
        places = 1;
        fractional *= 10;
    }
    return output;
}

static time_t GetTimeInSecond()
{
  time_t systime; /* time_t is a long */
  struct tm *systm;

  time(&systime); /* get the system time in seconds since EPOCH */
  systm=localtime(&systime); /* and return a pointer to the time structure */
  printf("time in seconds since EPOCH is %ld\n", systime);
  return systime;
}


#endif
