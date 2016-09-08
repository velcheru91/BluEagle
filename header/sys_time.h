/*
 * sys_time.h
 *
 *  Created on: Aug 25, 2016
 *      Author: Venugopal Velcheru
 */
//-----------------------------------------------------------------------------
// System specific clock settings
//-----------------------------------------------------------------------------
#ifndef PUBLIC_SYS_TIME_H_
#define PUBLIC_SYS_TIME_H_
//-----------------------------------------------------------------------------
// Device includes, defines, and assembler directives
//-----------------------------------------------------------------------------
#define DELAY_1uSEC		1
#define DELAY_1mSEC		1000
#define DELAY_1SEC		1000000
#define DELAY_10SEC		10000000
//-----------------------------------------------------------------------------
// Device prototype, function and method decleration
//-----------------------------------------------------------------------------
void sys_delay_usec(uint32_t us);


#endif /* PUBLIC_SYS_TIME_H_ */
