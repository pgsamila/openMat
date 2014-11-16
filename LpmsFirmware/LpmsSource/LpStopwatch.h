#ifndef __LPSTOPWATCH_H
#define __LPSTOPWATCH_H


uint32_t m_nStart;               //DEBUG Stopwatch start cycle counter value
uint32_t m_nStop;                //DEBUG Stopwatch stop cycle counter value

#define DEMCR_TRCENA    0x01000000

/* Core Debug registers */
#define DEMCR           (*((volatile uint32_t *)0xE000EDFC))
#define DWT_CTRL        (*(volatile uint32_t *)0xe0001000)
#define CYCCNTENA       (1<<0)
#define DWT_CYCCNT      ((volatile uint32_t *)0xE0001004)
#define CPU_CYCLES      *DWT_CYCCNT

#define STOPWATCH_START { m_nStart = *((volatile unsigned int *)0xE0001004);}//DWT_CYCCNT;}
#define STOPWATCH_STOP  { m_nStop = *((volatile unsigned int *)0xE0001004);}


static inline void stopwatch_reset(void)
{
    /* Enable DWT */
    DEMCR |= DEMCR_TRCENA; 
    *DWT_CYCCNT = 0;             
    /* Enable CPU cycle counter */
    DWT_CTRL |= CYCCNTENA;
}

static inline uint32_t stopwatch_getticks()
{
    return CPU_CYCLES;
}

static inline void stopwatch_delay(uint32_t ticks)
{
    stopwatch_reset();
    while(1)
    {
            if (stopwatch_getticks() >= ticks)
                    break;
    }
}

static inline float CalcNanosecondsFromStopwatch(uint32_t nStart, uint32_t nStop)
{
    //uint32_t nTemp;
    //uint32_t n;
	float ticks =(nStop-nStart);//*1000.0f/1200000.0f;
	float dt;
	
	dt = ticks*1000.0f/SystemCoreClock; //ms
	/*
    nTemp *= 1000;                          // Scale cycles by 1000.
    n = SystemCoreClock / 1000000;          // Convert Hz to MHz. SystemCoreClock = 168000000
    nTemp = nTemp / n;                      // nanosec = (Cycles * 1000) / (Cycles/microsec)
	*/
	
    return dt;
} 


#endif