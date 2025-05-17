#ifndef	_APP_H_
#define	_APP_H_

#include <sys/socket.h>
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/stat.h>
#include <unistd.h>
#include <pthread.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <wait.h>
#include <errno.h>
#include <netdb.h>
#include <sys/ipc.h>
#include <sys/msg.h>
#include <sys/time.h>
#include <stdint.h>
//#include <mxml.h>
#include <string.h>
#include <stdio.h>
#include <stdlib.h>
#include <math.h>         /*  math  */
// #define      NDEBUG      /*  禁用assert调用      */
#include <assert.h>
#include <sys/types.h> /* msggget */
#include <sys/ipc.h>
#include <sys/msg.h>

// control  print switch
#ifdef debug
#define __DEBUG
#endif

#ifdef __DEBUG

#define d_printf(f, a...)                                       \
    do {                                                          \
        { fprintf(stderr, "%s(%d): " f, __func__, __LINE__, ## a); } \
    } while (0)
#else
#define d_printf(f, a...)
#endif

#define ERR_EXIT(m)	\
	do	\
	{	\
		perror(m);	\
		exit(EXIT_FAILURE);	\
	} while(0)


#define ASSERT(x)     assert(x)

//Color
#define GREEN     1
#define YELLOW    2
#define RED       3

#endif
