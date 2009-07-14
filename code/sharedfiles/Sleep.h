#ifndef Header_Sleep
#define Header_Sleep

#include <time.h>
#include <cstdio>


static void Sleep(unsigned int time)
{
        struct timespec t,r;
        t.tv_sec    = time / 1000;
        t.tv_nsec   = (time % 1000) * 1000000;

        while(nanosleep(&t,&r)==-1)
        t = r;

        printf("~~~~~~Finsished Sleep of %d s~~~~\n", time/1000);
}

static void Wait(int count){
    int c = 0;
    while(c<count){ //fake sleep to execute last motion command
        if(!(c%1000)){
            printf("Sleep:%d -- ",c);
            //playerc_client_read(client);
        }
        c++;
    }
    printf("Sleep finished\n");
    /* printf("current (x,y,theta): %f %f %f\n",
            position2d->px, position2d->py, position2d->pa); */
}

#endif

