/**********************************************************************************
BSD 3-Clause License

Copyright (c) 2020, Orso Eric
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

1. Redistributions of source code must retain the above copyright notice, this
   list of conditions and the following disclaimer.

2. Redistributions in binary form must reproduce the above copyright notice,
   this list of conditions and the following disclaimer in the documentation
   and/or other materials provided with the distribution.

3. Neither the name of the copyright holder nor the names of its
   contributors may be used to endorse or promote products derived from
   this software without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
**********************************************************************************/

/**********************************************************************************
**  ENVIROMENT VARIABILE
**********************************************************************************/

#ifndef LONGAN_NANO_SCHEDULER_H_
    #define LONGAN_NANO_SCHEDULER_H_

/**********************************************************************************
**  GLOBAL INCLUDES
**********************************************************************************/

//Standard types with number of bits and signedness
#include <cstdint>

#ifndef LONGAN_NANO_CHRONO_H_
    //Longan Nano time functions
    #include "longan_nano_chrono.hpp"
#endif

/**********************************************************************************
**  DEFINES
**********************************************************************************/

//Enable the file trace debugger
//#define ENABLE_DEBUG
//file trace debugger
#ifdef ENABLE_DEBUG
    #include <cstdio.h> 
    #include "debug.h"
#endif
//If DEBUG is not needed, blank out the implementations
#ifndef DEBUG_H_
    #define DEBUG_VARS_PROTOTYPES()
    #define DEBUG_VARS()
    #define DSHOW( ... )
    #define DSTART( ... )
    #define DSTOP()
    #define DTAB( ... )
    #define DPRINT( ... )
    #define DPRINT_NOTAB( ... )
    #define DENTER( ... )
    #define DRETURN( ... )
    #define DENTER_ARG( ... )
    #define DRETURN_ARG( ... )
#endif

/**********************************************************************************
**  MACROS
**********************************************************************************/

/**********************************************************************************
**  NAMESPACE
**********************************************************************************/

//! @namespace Longan_nano class drivers for the longan nano GD32VF103 board 
namespace Longan_nano
{

/**********************************************************************************
**  TYPEDEFS
**********************************************************************************/

/**********************************************************************************
**  PROTOTYPE: STRUCTURES
**********************************************************************************/

/**********************************************************************************
**  PROTOTYPE: GLOBAL VARIABILES
**********************************************************************************/

/**********************************************************************************
**  PROTOTYPE: CLASS
**********************************************************************************/

/************************************************************************************/
//! @class      Scheduler
/************************************************************************************/
//!	@author     Orso Eric
//! @version    2020-MM-DD
//! @brief      Scheduler class handles the generation and profiling of execution task flags
//! @copyright  BSD 3-Clause License Copyright (c) 2020, Orso Eric
//! @details
//! \n  Scheduler class
//! \n  Built upon the Longan_nano::Chrono driver to achieve precise timings
//! \n  Generate task flags using Chrono class without interrupts not to interfere with the application
//! \n  Meant as simple hardwired scheduler that handles profiling and basic checks
//! \n  Feature: task execution flag | method .is_task_scheduled()
//! \n  Feature: Profiling | Profile execution time and CPU use of all tasks including scheduler .start() .stop() methods
//! \n  Feature: Overrun detection | Detect if a task is not finished method .is_task_overrun() .get_task_overrun()
//! \n  Feature: Fast/Prescaled tasks | One fast task. A number of prescaled tasks with integer multiple of execution time
//! \n  History versions
//! \n      2020-08-10
//! \n  class.hpp template add default Config and Error enums
//! \n      2020-08-11
//! \n  Formulation of class behaviour
//! \n      2020-08-15
//! \n  Working release
//! \n  Added profiling of uptime, CPU time and CPU load
//! \n  Scheduler now uses DeltaT error to compensate scheduling of the fast task, making sure the average is correct
/************************************************************************************/

class Scheduler
{
    //Visible to all
    public:
        /*********************************************************************************************************************************************************
        **********************************************************************************************************************************************************
        **  PUBLIC ENUMS
        **********************************************************************************************************************************************************
        *********************************************************************************************************************************************************/
    
        //! @brief Configurations of the class
        typedef enum _Config
        {
            SAFETY_CHECKS       = true,
            PEDANTIC_CHECKS     = true,
            NUM_TASKS           = 8,
            NUM_PRESCALED_TASKS = NUM_TASKS -1,
            MAX_PRESCALER       = UINT_FAST16_MAX,
            FAST_TASK_INDEX     = 0,
            //The scheduler itself when executed inside the main loop takes up a large fraction of resources
            //But running it fast only makes it more precise. It shouldn't count toward the CPU use
            PROFILE_SCHEDULER_CPU_LOAD  = false,
            AVG_FILTER          = 4,
        } Config;
        
        //! @brief Error codes of the class
        typedef enum _Error_code
        {
            OK,                 //OK
            RECOVERY_FAIL,      //error_recovery() failed to recover from an error
            UNINITIALIZED,      //Scheduler is not initialized
            BAD_ARGUMENTS,      //Function call with bad arguments
            CHRONO_FAIL,        //Chrono class failed to execute
            TASK_OVERRUN,       //A task was issued while its execution flag had yet to be cleared
            NUM_ERROR_CODES,    //Number of error codes of the class
        } Error_code;

        /*********************************************************************************************************************************************************
        **********************************************************************************************************************************************************
        **  PUBLIC TYPEDEFS
        **********************************************************************************************************************************************************
        *********************************************************************************************************************************************************/
        
        /*********************************************************************************************************************************************************
        **********************************************************************************************************************************************************
        **  CONSTRUCTORS
        **********************************************************************************************************************************************************
        *********************************************************************************************************************************************************/

        //Empty Constructor. Requires initialization and start later
        Scheduler( void );
        //Initialized constructor. Initialize with fast task settings
        Scheduler( int time_tmp, Longan_nano::Chrono::Unit unit_tmp );

        /*********************************************************************************************************************************************************
        **********************************************************************************************************************************************************
        **  DESTRUCTORS
        **********************************************************************************************************************************************************
        *********************************************************************************************************************************************************/

        //Empty destructor
        ~Scheduler( void );

        /*********************************************************************************************************************************************************
        **********************************************************************************************************************************************************
        **  PUBLIC OPERATORS
        **********************************************************************************************************************************************************
        *********************************************************************************************************************************************************/

        /*********************************************************************************************************************************************************
        **********************************************************************************************************************************************************
        **  PUBLIC SETTERS
        **********************************************************************************************************************************************************
        *********************************************************************************************************************************************************/

        //Set the speed of the fast tick. Index 0.
        bool set_fast_task( int time_tmp, Longan_nano::Chrono::Unit unit_tmp );
        //Set the prescaler of the prescaled tasks. Indexes 1 and above. return false = OK | true = fail
        bool set_prescaler( uint8_t index, int prescaler_tmp );

        /*********************************************************************************************************************************************************
        **********************************************************************************************************************************************************
        **  PUBLIC GETTERS
        **********************************************************************************************************************************************************
        *********************************************************************************************************************************************************/

        //Get current error state of the library
        Error_code get_error( void );
        //Get uptime of the application in seconds as floating point
        //float get_uptime( void );
        //Get uptime of the application in a given time unit
        int32_t get_uptime( Longan_nano::Chrono::Unit unit );
        //Get execution time of the application in a given time unit
        int32_t get_cpu_time( Longan_nano::Chrono::Unit unit );
        //Get CPU load with 100.0% equal to the given top
        int32_t get_cpu_load( int32_t top );
        //Get the average issue time for the fast task
        int32_t get_fast_task_avg_time( void );
        
        /*********************************************************************************************************************************************************
        **********************************************************************************************************************************************************
        **  PUBLIC TESTERS
        **********************************************************************************************************************************************************
        *********************************************************************************************************************************************************/

        //Return true if the scheduler flag of task {index is allowed to run}
        bool is_task_scheduled( uint8_t index );

        /*********************************************************************************************************************************************************
        **********************************************************************************************************************************************************
        **  PUBLIC METHODS
        **********************************************************************************************************************************************************
        *********************************************************************************************************************************************************/

        //Schedule tasks. Meant to be called inside the main loop
        int8_t update( void );
        //Signal the scheduler when a task is running and is done running. Scheduler takes care of profiling and housekeeping
        bool run( uint8_t index );
        bool done( uint8_t index );

        /*********************************************************************************************************************************************************
        **********************************************************************************************************************************************************
        **  PUBLIC STATIC METHODS
        **********************************************************************************************************************************************************
        *********************************************************************************************************************************************************/

        /*********************************************************************************************************************************************************
        **********************************************************************************************************************************************************
        **  PUBLIC VARS
        **********************************************************************************************************************************************************
        *********************************************************************************************************************************************************/

    //Visible only inside the class
    private:
        /*********************************************************************************************************************************************************
        **********************************************************************************************************************************************************
        **	PRIVATE ENUM
        **********************************************************************************************************************************************************
        *********************************************************************************************************************************************************/
        
        /*********************************************************************************************************************************************************
        **********************************************************************************************************************************************************
        **	PRIVATE TYPEDEFS
        **********************************************************************************************************************************************************
        *********************************************************************************************************************************************************/
    
        //! @brief Configuration of the fast task
        typedef struct _Fast_task
        {
            Longan_nano::Chrono::Unit unit;     //Time unit
            int32_t deltat;                     //Desired execution time of the fast task
            int32_t avg;                        //Measure DeltaT error and use it to correct the execution. Correction is clipped to the DeltaT, an error is issued if correction is too big
            int32_t error;                      //Profile the average execution time of the fast task
        } Fast_task;

        //! @brief Configuration of the prescaled tasks
        typedef struct _Prescaled_task
        {
            //Prescaler of the task. 0 means task unused
            uint16_t prescaler[ Config::NUM_PRESCALED_TASKS ];
            uint16_t prescaler_cnt[ Config::NUM_PRESCALED_TASKS ];
        } Prescaled_task;

        //! @brief Tasks execution and overrun flags
        typedef struct _Tasks
        {
            bool f_exe[ Config::NUM_TASKS ];
            bool f_overrun[ Config::NUM_TASKS ];
        } Tasks;

        /*********************************************************************************************************************************************************
        **********************************************************************************************************************************************************
        **	PRIVATE INIT
        **********************************************************************************************************************************************************
        *********************************************************************************************************************************************************/
        
        //Initialize class vars
        bool init_class_vars( void );

        /*********************************************************************************************************************************************************
        **********************************************************************************************************************************************************
        **	PRIVATE SETTER
        **********************************************************************************************************************************************************
        *********************************************************************************************************************************************************/

        //Set the task execution flag. Handle task overrun
        bool set_task_flag( uint8_t index );

        /*********************************************************************************************************************************************************
        **********************************************************************************************************************************************************
        **	PRIVATE METHODS
        **********************************************************************************************************************************************************
        *********************************************************************************************************************************************************/
        
        //Report an error. return false: OK | true: Unknown error code 
        bool report_error( Error_code error_code_tmp );
        //Tries to recover from an error. Automatically called by get_error. return false = OK | true = fail
        bool error_recovery( void );

        //Scheduler method to copy the code
        bool dummy( void );

        /*********************************************************************************************************************************************************
        **********************************************************************************************************************************************************
        **	PRIVATE VARS
        **********************************************************************************************************************************************************
        *********************************************************************************************************************************************************/

        //! @brief Number of tasks with valid DeltaT
        uint8_t g_num_tasks;
        //! @brief 
        Tasks g_task;

        //! @briefFast task settings
        Fast_task g_fast_task;
        //Prescaled tasks settings
        Prescaled_task g_prescaled_task;

        //! @brief Chrono timer for the fast task
        Longan_nano::Chrono g_fast_task_timer;
        //! @brief Chrono timer to profile the uptime. Time since scheduler started.
        Longan_nano::Chrono g_uptime_timer;
        //! @brief Chrono timer to profile CPU time and CPU use %
        Longan_nano::Chrono g_cpu_timer;
        //! @brief Error code of the class
        Error_code g_error;
};	//End Class: Scheduler

/*********************************************************************************************************************************************************
**********************************************************************************************************************************************************
**	CONSTRUCTORS
**********************************************************************************************************************************************************
*********************************************************************************************************************************************************/

/***************************************************************************/
//! @brief Constructor
//! \n Scheduler | void |
/***************************************************************************/
//! @return no return
//! @details
//! \n Empty constructor
/***************************************************************************/

Scheduler::Scheduler( void )
{
    DENTER();   //Trace Enter
    ///--------------------------------------------------------------------------
    ///	BODY
    ///--------------------------------------------------------------------------

    //Initialize class vars
    this -> init_class_vars();

    ///--------------------------------------------------------------------------
    ///	RETURN
    ///--------------------------------------------------------------------------
    DRETURN();  //Trace Return
    return;
}   //end constructor: Scheduler | void |

/***************************************************************************/
//! @brief Constructor
//! \n Scheduler | int | Longan_nano::Chrono::Unit |
/***************************************************************************/
//! @param time_tmp | int | fast task DeltaT
//! @param unit_tmp | Longan_nano::Chrono::Unit | unit of measure of the fast task time
//! @return no return
//! @details
//! \n Initialized constructor. Initialize with fast task settings
/***************************************************************************/

Scheduler::Scheduler( int time_tmp, Longan_nano::Chrono::Unit unit_tmp )
{
    DENTER();   //Trace Enter
    ///--------------------------------------------------------------------------
    ///	BODY
    ///--------------------------------------------------------------------------

    //Initialize class vars
    this -> init_class_vars();
    //Configure fast task
    this -> set_fast_task( time_tmp, unit_tmp );

    ///--------------------------------------------------------------------------
    ///	RETURN
    ///--------------------------------------------------------------------------
    DRETURN();  //Trace Return
    return;
}   //end constructor: Scheduler | void

/*********************************************************************************************************************************************************
**********************************************************************************************************************************************************
**	DESTRUCTORS
**********************************************************************************************************************************************************
*********************************************************************************************************************************************************/

/***************************************************************************/
//!	@brief Destructor
//! \n Scheduler | void
/***************************************************************************/
// @param
//! @return no return
//! @details
//! \n Empty destructor
/***************************************************************************/

Scheduler::~Scheduler( void )
{
    DENTER();		//Trace Enter
    ///--------------------------------------------------------------------------
    ///	VARS
    ///--------------------------------------------------------------------------

    ///--------------------------------------------------------------------------
    ///	INIT
    ///--------------------------------------------------------------------------

    ///--------------------------------------------------------------------------
    ///	BODY
    ///--------------------------------------------------------------------------

    ///--------------------------------------------------------------------------
    ///	RETURN
    ///--------------------------------------------------------------------------
    DRETURN();      //Trace Return
    return;         //OK
}   //end destructor: Scheduler | void

/*********************************************************************************************************************************************************
**********************************************************************************************************************************************************
**	PUBLIC GETTERS
**********************************************************************************************************************************************************
*********************************************************************************************************************************************************/

/***************************************************************************/
//! @brief Public getter
//! \n get_error | void |
/***************************************************************************/
//! @return Error_code
//! @details
//! \n Get current error state of the library
//! \n Try to recover from errors
/***************************************************************************/

inline Scheduler::Error_code Scheduler::get_error( void )
{
    DENTER(); //Trace Enter
    ///--------------------------------------------------------------------------
    ///	BODY
    ///--------------------------------------------------------------------------

    //Fetch error
    Error_code err_code = this -> g_error;
    //Try to recover from error
    bool f_ret = this -> error_recovery();
    //If: failed to recover
    if (f_ret == true)
    {
        //Do nothing
    }

    ///--------------------------------------------------------------------------
    ///	RETURN
    ///--------------------------------------------------------------------------
    DRETURN(); //Trace Return
    return err_code; //OK
}   //end Public getter: get_error | void |

/***************************************************************************/
//! @brief private method
//! \n get_uptime | Longan_nano::Chrono::Unit |
/***************************************************************************/
//! @param unit | Longan_nano::Chrono::Unit | time unit for the uptime
//! @return bool | <0: fail | >=0 uptime in time units
//! @details
//! \n Get uptime of the application in a given time unit
/***************************************************************************/

inline int32_t Scheduler::get_uptime( Longan_nano::Chrono::Unit unit )
{
    DENTER("time unit: %d\n", unit); //Trace Enter
    ///--------------------------------------------------------------------------
    ///	BODY
    ///--------------------------------------------------------------------------

    //Get uptime
    int32_t uptime = this -> g_uptime_timer.stop( unit );

    ///--------------------------------------------------------------------------
    ///	RETURN
    ///--------------------------------------------------------------------------
    DRETURN_ARG("uptime: %d\n", uptime); //Trace Return
    return uptime;	//OK
}   //end: get_uptime | Longan_nano::Chrono::Unit |

/***************************************************************************/
//! @brief private method
//! \n get_cpu_time | Longan_nano::Chrono::Unit |
/***************************************************************************/
//! @param unit | Longan_nano::Chrono::Unit | time unit for the uptime
//! @return bool | <0: fail | >=0 uptime in time units
//! @details
//! \n Get execution time of the application in a given time unit
//! \n How much time the CPU spent running tasks between .run and .done including the scheduler
//! \n This scheduler is not meant to be preemptive. interrupts will lengthen task execution 
//! \n .run and .done executed inside an interrupt will cause artefacts
//! \n if you want a proper fully featured scheduler go for free RTOS or such
/***************************************************************************/

inline int32_t Scheduler::get_cpu_time( Longan_nano::Chrono::Unit unit )
{
    DENTER("time unit: %d\n", unit); //Trace Enter
    ///--------------------------------------------------------------------------
    ///	BODY
    ///--------------------------------------------------------------------------

    //Get uptime
    int32_t cpu_load = this -> g_cpu_timer.get_accumulator( unit );

    ///--------------------------------------------------------------------------
    ///	RETURN
    ///--------------------------------------------------------------------------
    DRETURN_ARG("cpu_load: %d\n", cpu_load); //Trace Return
    return cpu_load;	//OK
}   //end: get_cpu_time | Longan_nano::Chrono::Unit |

/***************************************************************************/
//! @brief private method
//! \n get_cpu_load | int32_t |
/***************************************************************************/
//! @param unit | Longan_nano::Chrono::Unit | time unit for the uptime
//! @return bool | <0: fail | >=0 uptime in time units
//! @details
//! \n Get CPU load with 100.0% equal to the given top
//! \n E.g. load 50.0% top = 1024 -> return 512
//! \n E.g. load 12.1% top = 1000 -> return 121
/***************************************************************************/

inline int32_t Scheduler::get_cpu_load( int32_t top )
{
    DENTER("time unit: %d\n", unit); //Trace Enter
    ///--------------------------------------------------------------------------
    /// Uptime
    ///--------------------------------------------------------------------------
    //  Compute uptime and a valid time unit

    //Start with microsecond resolution
    Longan_nano::Chrono::Unit unit = Longan_nano::Chrono::Unit::microseconds;
    //Application uptime
    int32_t uptime;
    //While the uptime is invalid
    do
    {
        //Fetch the uptime in a given unit
        uptime = this -> g_uptime_timer.stop( unit );
        //If: invalid
        if (uptime < 0)
        {
            //Switch: try to get a bigger unit
            switch(unit)
            {
                case Longan_nano::Chrono::Unit::microseconds:
                {
                    unit = Longan_nano::Chrono::Unit::milliseconds;
                    break;
                }
                case Longan_nano::Chrono::Unit::milliseconds:
                {
                    unit = Longan_nano::Chrono::Unit::seconds;
                    break;
                }
                //If: fail anyway
                default:
                {
                    //Error
                    return -1;
                }
            }   //end Switch: try to get a bigger unit

        }   //end If: invalid
    }   //end While: the uptime is invalid
    while (uptime < 0);
    
    ///--------------------------------------------------------------------------
    /// CPU Load
    ///--------------------------------------------------------------------------
    //  Compute CPU Load

    int32_t exe_time = this -> g_cpu_timer.get_accumulator( unit );

    int64_t cpu_load = exe_time;
    cpu_load *= top;
    cpu_load /= uptime;
    //If: invalid
    if (cpu_load > INT32_MAX)
    {
        return -1;
    }

    ///--------------------------------------------------------------------------
    ///	RETURN
    ///--------------------------------------------------------------------------
    
    //Demote
    int32_t ret = cpu_load;
    DRETURN_ARG("CPU load: %d\n", ret); //Trace Return
    return ret;	//OK
}   //end: get_cpu_load | int32_t |

/***************************************************************************/
//! @brief public getter
//! \n get_fast_task_avg_time | void |
/***************************************************************************/
// @param
//! @return int32_t | <0: ERR | >=0: average time |
//! @details
//! \n Get the average issue time for the fast task
/***************************************************************************/

inline int32_t Scheduler::get_fast_task_avg_time( void )
{
    DENTER(); //Trace Enter
    ///--------------------------------------------------------------------------
    ///	RETURN
    ///--------------------------------------------------------------------------
    DRETURN_ARG("avg: %d\n", this -> g_fast_task_avg); //Trace Return
    return this -> g_fast_task.avg;
}   //end public getter: get_fast_task_avg_time | void |

/*********************************************************************************************************************************************************
**********************************************************************************************************************************************************
**	PUBLIC SETTERS
**********************************************************************************************************************************************************
*********************************************************************************************************************************************************/

/***************************************************************************/
//! @brief Public setter
//! \n set_fast_task | int | Longan_nano::Chrono::Unit |
/***************************************************************************/
//! @param time_tmp | int | fast task DeltaT
//! @param unit_tmp | Longan_nano::Chrono::Unit | unit of measure of the fast task time
//! @return bool | false = OK | true = fail
//! @details
//! \n Set the speed of the fast tick
//! \n Fast task has task index zero
/***************************************************************************/

inline bool Scheduler::set_fast_task( int time_tmp, Longan_nano::Chrono::Unit unit_tmp )
{
    DENTER_ARG( "DeltaT: %d | Unit: %d\n", time_tmp, (int)unit_tmp ); //Trace Enter
    ///--------------------------------------------------------------------------
    ///	CHECKS
    ///--------------------------------------------------------------------------

    //If: bad delay
    if ((Config::SAFETY_CHECKS == true) && (time_tmp < 0))
    {
        report_error( Error_code::BAD_ARGUMENTS );
        DRETURN_ARG("ERR: %d\n", (int)this -> get_error() );
        return true; //fail
    }

    ///--------------------------------------------------------------------------
    ///	BODY
    ///--------------------------------------------------------------------------

    //Configure the fast task
    this -> g_fast_task.deltat = time_tmp;
    this -> g_fast_task.unit = unit_tmp;
    //Start the clock
    this -> g_fast_task_timer.start();

    ///--------------------------------------------------------------------------
    ///	RETURN
    ///--------------------------------------------------------------------------
    DRETURN(); //Trace Return
    return false; //OK
}   //end Public setter: set_fast_task | int | Longan_nano::Chrono::Unit |

/***************************************************************************/
//! @brief Public setter
//! \n set_fast_task | int | Longan_nano::Chrono::Unit |
/***************************************************************************/
//! @param time_tmp | int | fast task DeltaT
//! @param unit_tmp | Longan_nano::Chrono::Unit | unit of measure of the fast task time
//! @return bool | false = OK | true = fail
//! @details
//! \n Set the prescaler of the prescaled tasks
/***************************************************************************/

bool Scheduler::set_prescaler( uint8_t index, int prescaler_tmp )
{
    DENTER_ARG( "index: %d | prescaler_tmp: %d\n", index, prescaler_tmp ); //Trace Enter
    ///--------------------------------------------------------------------------
    ///	CHECKS
    ///--------------------------------------------------------------------------

    //If: bad index
    if ((Config::SAFETY_CHECKS == true) && (index >= Config::NUM_TASKS))
    {
        report_error( Error_code::BAD_ARGUMENTS );
        DRETURN_ARG("ERR: Bad task index %d\n", index );
        return false;
    }

    ///--------------------------------------------------------------------------
    ///	BODY
    ///--------------------------------------------------------------------------

    //Set prescaler
    this -> g_prescaled_task.prescaler[ index -1 ] = prescaler_tmp;

    ///--------------------------------------------------------------------------
    ///	RETURN
    ///--------------------------------------------------------------------------
    DRETURN(); //Trace Return
    return false; //OK
}   //end Public setter: set_fast_task | int | Longan_nano::Chrono::Unit |

/*********************************************************************************************************************************************************
**********************************************************************************************************************************************************
**	PUBLIC TESTERS
**********************************************************************************************************************************************************
*********************************************************************************************************************************************************/

/***************************************************************************/
//! @brief public tester
//! \n is_task_scheduled | uint8_t |
/***************************************************************************/
//! @param index | uint8_t | index of the task
//! @return bool | false: not scheduled | true: scheduled |
//! @details
//! \n Return true if the scheduler flag of task {index} is allowed to run
/***************************************************************************/

inline bool Scheduler::is_task_scheduled( uint8_t index )
{
    DENTER(); //Trace Enter
    ///--------------------------------------------------------------------------
    ///	CHECKS
    ///--------------------------------------------------------------------------

    //If: bad index
    if ((Config::SAFETY_CHECKS == true) && (index >= Config::NUM_TASKS))
    {
        report_error( Error_code::BAD_ARGUMENTS );
        DRETURN_ARG("ERR: Bad task index %d\n", index );
        return false;
    }
    //If: task with no prescaler set (unused)
    //if ((Config::PEDANTIC_CHECKS == true) && (this -> g_))

    ///--------------------------------------------------------------------------
    ///	RETURN
    ///--------------------------------------------------------------------------
    DRETURN(); //Trace Return
    return this -> g_task.f_exe[index];	//OK
}   //end public tester: is_task_scheduled | uint8_t |

/*********************************************************************************************************************************************************
**********************************************************************************************************************************************************
**	PUBLIC METODS
**********************************************************************************************************************************************************
*********************************************************************************************************************************************************/

/***************************************************************************/
//! @brief public method
//! \n update | void |
/***************************************************************************/
//! @return int8_t | <0: ERROR | 0: Nothing happened | >0: Number of tasks issued |
//! @details
//! \n Schedule tasks. Meant to be called inside the main loop
/***************************************************************************/
        
int8_t Scheduler::update( void )
{
    DENTER(); //Trace Enter
    
    ///--------------------------------------------------------------------------
    ///	CHECKS
    ///--------------------------------------------------------------------------

    //Fetch the DeltaT
    int deltat = this -> g_fast_task_timer.stop( this -> g_fast_task.unit );
    //If: bad DeltaT
    if ((Config::SAFETY_CHECKS == true) && (deltat < 0))
    {
        //Report error
        this -> report_error( Error_code::CHRONO_FAIL );
        DRETURN_ARG("ERR: Chrono class failed\n");
        return true;    //fail
    }

    //If: fast task is not configured
    if (this -> g_fast_task.deltat == 0)
    {
        //Report error
        this -> report_error( Error_code::UNINITIALIZED );
        DRETURN_ARG("ERR: Scheduler not initialized\n");
        return -1;
    }

    ///--------------------------------------------------------------------------
    /// PROFILE
    ///--------------------------------------------------------------------------
    //  Accumulate time spent running the scheduler
    //If: scheduler resource use is being profiled
    if (Config::PROFILE_SCHEDULER_CPU_LOAD == true)
    {
        //Starting point
        this -> g_cpu_timer.start();
    }

    ///--------------------------------------------------------------------------
    /// FAST TASK
    ///--------------------------------------------------------------------------
    //  The fast task uses the Chrono timer to precisely schedule execution

    //Success flag
    bool f_ret;
    //Number of issued tasks
    uint8_t num_tasks = 0;

    //If: it's time to execute the fast task
    if (deltat >= this -> g_fast_task.deltat +this -> g_fast_task.error )
    //if (deltat >= this -> g_fast_task.deltat )
    {
        //Set the task flag
        f_ret = this -> set_task_flag( Config::FAST_TASK_INDEX );
        //if: fail
        if (f_ret == true)
        {
            //If: scheduler resource use is being profiled
            if (Config::PROFILE_SCHEDULER_CPU_LOAD == true)
            {
                //Profile execution time
                this -> g_cpu_timer.accumulate();
            }
            //Report error
            DRETURN_ARG("ERR: Fast Task Overrun\n");
            return -1;
        }
        //Profile average execution time of fast task
        this -> g_fast_task.avg = ((Config::AVG_FILTER -1) *this -> g_fast_task.avg +1 *deltat) /Config::AVG_FILTER;
        //Compute the error for the next round
        this -> g_fast_task.error = this -> g_fast_task.deltat -deltat;
        //Clip correction to half task DeltaT
        if (this -> g_fast_task.error > this -> g_fast_task.deltat /2)
        {
            this -> g_fast_task.error = this -> g_fast_task.deltat /2;   
        }
        else if (this -> g_fast_task.error < (-this -> g_fast_task.deltat) /2)
        {
            this -> g_fast_task.error = (-this -> g_fast_task.deltat) /2;   
        }
        //A task was issued
        num_tasks++;
    }   //End If: it's time to execute the fast task

    ///--------------------------------------------------------------------------
    /// PRESCALED TASKS
    ///--------------------------------------------------------------------------
    //  A number of prescaled tasks can be issued at integer multiples of the fast task execution time

    //if: fast task has been issued
    if (num_tasks > 0)
    {
        //for each prescaled task
        for (uint8_t t = 0; t < Config::NUM_PRESCALED_TASKS;t++)
        {
            //if: prescaled task is enabled
            if (this -> g_prescaled_task.prescaler[t] != 0)
            {
                //if: it's not time to release task
                if (this -> g_prescaled_task.prescaler_cnt[t] < this -> g_prescaled_task.prescaler[t])
                {
                    //Increase prescaler counter
                    this -> g_prescaled_task.prescaler_cnt[t]++;
                }
                //if: it's time to release task
                else
                {
                    //Reset task prescaler counter
                    this -> g_prescaled_task.prescaler_cnt[t] = 0;
                    //Set the task flag
                    f_ret = this -> set_task_flag( t +Config::FAST_TASK_INDEX +1 );
                    //if: fail
                    if (f_ret == true)
                    {
                        //If: scheduler resource use is being profiled
                        if (Config::PROFILE_SCHEDULER_CPU_LOAD == true)
                        {
                            //Profile execution time
                            this -> g_cpu_timer.accumulate();
                        }
                        //Report error
                        DRETURN_ARG("ERR: Fast Task Overrun\n");
                        return -1;
                    }
                    //A task was issued
                    num_tasks++;
                }   //end if: it's time to release task
            }   //end if: prescaled task is enabled
        }   //end for each prescaled task
    }   //end if: fast task has been issued

    //If: scheduler resource use is being profiled
    if (Config::PROFILE_SCHEDULER_CPU_LOAD == true)
    {
        //Profile execution time
        this -> g_cpu_timer.accumulate();
    }

    ///--------------------------------------------------------------------------
    ///	RETURN
    ///--------------------------------------------------------------------------
    DRETURN_ARG("Issued tasks: %d\n", num_tasks); //Trace Return
    return num_tasks;
}   //end public method: update | void |

/***************************************************************************/
//! @brief public method
//! \n run | uint8_t |
/***************************************************************************/
//! @param index | uint8_t | index of the task
//! @return  bool | false: OK | true: fail |
//! @details
//! \n Signal the scheduler when a task is running
//! \n Scheduler takes care of profiling and housekeeping
/***************************************************************************/

bool Scheduler::run( uint8_t index )
{
    DENTER(); //Trace Enter
    ///--------------------------------------------------------------------------
    ///	CHECKS
    ///--------------------------------------------------------------------------

    //If: bad index
    if ((Config::SAFETY_CHECKS == true) && (index >= Config::NUM_TASKS))
    {
        report_error( Error_code::BAD_ARGUMENTS );
        DRETURN_ARG("ERR: Bad task index %d\n", index );
        return true;    //Fail
    }

    ///--------------------------------------------------------------------------
    ///	BODY
    ///--------------------------------------------------------------------------

    //If: fast task
    if (index == Config::FAST_TASK_INDEX)
    {
        //Start the clock. Scheduling time starts from when the task is actually being run
        this -> g_fast_task_timer.start();
    }
    //Profile execution time
    this -> g_cpu_timer.start();

    ///--------------------------------------------------------------------------
    ///	RETURN
    ///--------------------------------------------------------------------------
    DRETURN(); //Trace Return
    return false;	//OK
}   //end public method: run | uint8_t |

/***************************************************************************/
//! @brief public method
//! \n done | uint8_t |
/***************************************************************************/
//! @param index | uint8_t | index of the task
//! @return  bool | false: OK | true: fail |
//! @details
//! \n Signal the scheduler when a task is running
//! \n Scheduler takes care of profiling and housekeeping
/***************************************************************************/

bool Scheduler::done( uint8_t index )
{
    DENTER(); //Trace Enter
    ///--------------------------------------------------------------------------
    ///	CHECKS
    ///--------------------------------------------------------------------------

    //If: bad index
    if ((Config::SAFETY_CHECKS == true) && (index >= Config::NUM_TASKS))
    {
        report_error( Error_code::BAD_ARGUMENTS );
        DRETURN_ARG("ERR: Bad task index %d\n", index );
        return true;    //Fail
    }
    //Accumulate execution time
    this -> g_cpu_timer.accumulate();

    ///--------------------------------------------------------------------------
    ///	BODY
    ///--------------------------------------------------------------------------

    //Clear task execution flag
    this -> g_task.f_exe[ index ] = false;

    ///--------------------------------------------------------------------------
    ///	RETURN
    ///--------------------------------------------------------------------------
    DRETURN(); //Trace Return
    return false;	//OK
}   //end public method: done | uint8_t |

/*********************************************************************************************************************************************************
**********************************************************************************************************************************************************
**	PRIVATE INIT
**********************************************************************************************************************************************************
*********************************************************************************************************************************************************/

/***************************************************************************/
//! @brief Private Method
//! \n init_class_vars | void
/***************************************************************************/
//! @return bool | false: OK | true: fail |
//! @details
//! \n Initialize class vars
/***************************************************************************/

bool Scheduler::init_class_vars( void )
{
    DENTER();		//Trace Enter
    ///--------------------------------------------------------------------------
    ///	VARS
    ///--------------------------------------------------------------------------

    ///--------------------------------------------------------------------------
    ///	INIT
    ///--------------------------------------------------------------------------

    ///--------------------------------------------------------------------------
    ///	BODY
    ///--------------------------------------------------------------------------

    //Class vars
    this -> g_num_tasks = 0;
    //Set fast task to unused
    this -> g_fast_task.deltat = 0;
    //Reset the DeltaT error of the fast task
    this -> g_fast_task.error = 0;
    //For: each task
    for (uint8_t t = 0; t < Config::NUM_TASKS;t++)
    {
        //Set flags to false
        this -> g_task.f_exe[t] = false;
        this -> g_task.f_overrun[t] = false;   
    }

    //For: each prescaled task
    for (uint8_t t = 0; t < Config::NUM_PRESCALED_TASKS;t++)
    {
        //Disable prescaled task
        this -> g_prescaled_task.prescaler[t] = 0;
        //Reset the counter
        this -> g_prescaled_task.prescaler_cnt[t] = 0;
    }

    //Initialize profiling timer
    this -> g_uptime_timer.restart();
    this -> g_cpu_timer.restart();

    ///--------------------------------------------------------------------------
    ///	RETURN
    ///--------------------------------------------------------------------------
    DRETURN();      //Trace Return
    return false;   //OK
}   //end method: init_class_vars | void

/*********************************************************************************************************************************************************
**********************************************************************************************************************************************************
**  PRIVATE SETTER
**********************************************************************************************************************************************************
*********************************************************************************************************************************************************/

/***************************************************************************/
//! @brief private setter
//! \n set_task_flag | uint8_t |
/***************************************************************************/
//! @param index | uint8_t | index of the task
//! @return bool | false: OK | true: fail |
//! @details
//! \n Set the task execution flag. Handle task overrun
/***************************************************************************/

inline bool Scheduler::set_task_flag( uint8_t index )
{
    DENTER_ARG( "index: %d\n", index ); //Trace Enter
    ///--------------------------------------------------------------------------
    ///	CHECKS
    ///--------------------------------------------------------------------------

    //If: bad index
    if ((Config::SAFETY_CHECKS == true) && (index >= Config::NUM_TASKS))
    {
        report_error( Error_code::BAD_ARGUMENTS );
        DRETURN_ARG("ERR: Bad task index %d\n", index );
        return true; //Fail
    }

    ///--------------------------------------------------------------------------
    ///	BODY
    ///--------------------------------------------------------------------------

    //If: execution flag is idle
    if (this -> g_task.f_exe[ index ] == false)
    {
        //Issue fast task execution flag
        this -> g_task.f_exe[ index ] = true;
    }
    //If: the execution flag is yet to be cleared
    else //if (this -> g_task.f_exe[ index ] == true)
    {
        //Clear flag. The CPU is overloaded
        this -> g_task.f_exe[ index ] = false;
        //Raise the overrun flag
        this -> g_task.f_overrun[ index ] = true;
        //Overrun
        this -> report_error( Error_code::TASK_OVERRUN );
        DRETURN_ARG("ERR: Task overrun\n");
        return true; //fail
    }

    ///--------------------------------------------------------------------------
    ///	RETURN
    ///--------------------------------------------------------------------------
    DRETURN(); //Trace Return
    return false;	//OK
}   //end private setter: set_task_flag | uint8_t |

/*********************************************************************************************************************************************************
**********************************************************************************************************************************************************
**  PRIVATE METHODS
**********************************************************************************************************************************************************
*********************************************************************************************************************************************************/

/***************************************************************************/
//! @brief Private method
//! \n error_recovery | void |
/***************************************************************************/
//! @return return false = OK | true = fail
//! @details
//! \n Tries to recover from an error
//! \n Automatically called by get_error. 
/***************************************************************************/

bool Scheduler::error_recovery( void )
{
    DENTER(); //Trace Enter
    ///--------------------------------------------------------------------------
    ///	VARS
    ///--------------------------------------------------------------------------

    ///--------------------------------------------------------------------------
    ///	INIT
    ///--------------------------------------------------------------------------

    ///--------------------------------------------------------------------------
    ///	BODY
    ///--------------------------------------------------------------------------

    ///--------------------------------------------------------------------------
    ///	RETURN
    ///--------------------------------------------------------------------------
    DRETURN();      //Trace Return
    return true;    //FAIL
}   //end private method: error_recovery | void |

/***************************************************************************/
//! @brief Private method
//! \n dummy | void
/***************************************************************************/
//! @param error_code_tmp | Error_code | error code
//! @return false: OK | true: Unknown error code 
//! @details
//! \n Report an error
/***************************************************************************/

bool Scheduler::report_error( Error_code error_code_tmp )
{
    DENTER(); //Trace Enter
    ///--------------------------------------------------------------------------
    ///	BODY
    ///--------------------------------------------------------------------------

    //A recovery fail error should not get recorded
    if (error_code_tmp == Error_code::RECOVERY_FAIL)
    {
        DRETURN_ARG("ERR: RECOVERY_FAIL\n" );    //Trace Return
        return false;   //OK
    }
    else if ((Config::PEDANTIC_CHECKS == true) && (error_code_tmp >= Error_code::NUM_ERROR_CODES))
    {
        DRETURN_ARG("ERR: Undefined error code: %d\n", error_code_tmp);    //Trace Return
        return true; //fail
    }

    this -> g_error = error_code_tmp;

    ///--------------------------------------------------------------------------
    ///	RETURN
    ///--------------------------------------------------------------------------
    DRETURN(); //Trace Return
    return false;	//OK
}   //end private method:

/***************************************************************************/
//! @brief private method
//! \n dummy | void
/***************************************************************************/
// @param
//! @return bool | false: OK | true: fail |
//! @details
//! \n Method
/***************************************************************************/

bool Scheduler::dummy( void )
{
    DENTER(); //Trace Enter
    ///--------------------------------------------------------------------------
    ///	VARS
    ///--------------------------------------------------------------------------

    ///--------------------------------------------------------------------------
    ///	INIT
    ///--------------------------------------------------------------------------

    ///--------------------------------------------------------------------------
    ///	BODY
    ///--------------------------------------------------------------------------

    ///--------------------------------------------------------------------------
    ///	RETURN
    ///--------------------------------------------------------------------------
    DRETURN(); //Trace Return
    return false;	//OK
}   //end:


/**********************************************************************************
**	NAMESPACE
**********************************************************************************/

} //End Namespace: User

#else
    #warning "Multiple inclusion of hader file LONGAN_NANO_SCHEDULER_H_"
#endif
