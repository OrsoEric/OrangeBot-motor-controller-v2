/****************************************************************************
**	OrangeBot Project
*****************************************************************************
**        /
**       /
**      /
** ______ \
**         \
**          \
*****************************************************************************
**	Quadrature Encoder
*****************************************************************************
**  Activate Timer 2 in quadrature mode
**  Count a quadrature encoder connected to PB4 and PB5
**  Display the encoder count on screen
**  Repository:
**  https://github.com/OrsoEric/OrangeBot-motor-controller-v2
**      2021-03-20
**  Remove uC temperature. Add ADC CH0, motor Vsense
****************************************************************************/

/****************************************************************************
**	INCLUDES
****************************************************************************/

//C++ std random number generators
#include <random>
//Longan Nano HAL
#include <gd32vf103.h>
//LED class
#include "longan_nano_led.hpp"
//Time class
#include "longan_nano_chrono.hpp"
//Scheduler class
#include "longan_nano_scheduler.hpp"
//Higher level abstraction layer to base Display Class. Provides character sprites and print methods with color
#include "longan_nano_screen.hpp"
//Driver for the VNH7040 H-Bridge
#include "longan_nano_vnh7040.hpp"

#include "pid_s32.hpp"
#include "pid_s16.h"

/****************************************************************************
**	NAMESPACES
****************************************************************************/

/****************************************************************************
**	DEFINES
****************************************************************************/

//forever
#define EVER (;;)

/****************************************************************************
**	MACROS
****************************************************************************/

/****************************************************************************
**	ENUM
****************************************************************************/

//Configurations
typedef enum _Config
{
    SCHEDULER_UNIT      = Longan_nano::Chrono::Unit::microseconds,
    //Microseconds between calls of the screen update method. User decides how much CPU to allocate to the screen by tuning this time.
    //Longer time will mean fewer sprites rendered per second, but no matter what, the screen class will not crash as only the most recent sprite is drawn
    SCREEN_US           = 100,
    //HMI UPDATE
    HMI_US              = 100000,
    //Microseconds between led toggles
    RED_LED_BLINK_US    = 250000,
    //Microseconds between paint of colored square
    PAINT_TASK_US       = 300000,

    VNH7040_TASK_US     = 1000,

    SCREEN_TASK_INDEX   = 0,
    HMI_TASK_INDEX      = 1,
    RED_LED_TASK_INDEX  = 2,
    PAINT_TASK_INDEX    = 3,
    VNH7040_TASK_INDEX  = 4,
    PAINT_START_HEIGHT  = 6,
} Config;

/****************************************************************************
**	STRUCT
****************************************************************************/

typedef struct _Scheduler
{
    bool f_screen       : 1;
    bool f_demo         : 1;
    bool f_overrun      : 1;
} Scheduler;

/****************************************************************************
**  PROTOTYPES
****************************************************************************/

//Initialize
extern bool init_enc( void );

extern bool init_adc_temp(void);

extern bool test_vnh7040( void );

/****************************************************************************
**  GLOBAL VARIABILES
****************************************************************************/

volatile bool g_rtc_flag = false;

//C++ Standard Random Number Generator
std::default_random_engine g_rng_engine;
std::uniform_int_distribution<uint8_t> g_rng_color( 0, Longan_nano::Screen::Config::PALETTE_SIZE -1 );
//Scheduler for the tasks
Scheduler g_scheduler = { 0 };
//VNH7040 Motor controller
Longan_nano::VNH7040 g_vnh7040;
//PID controller
Orangebot::Pid_s32 gcl_pid_controller;
int_fast32_t gs32_vnh7040_command;
int_fast32_t gs32_encoder_feedback;
bool gu1_pid_error = false;

Orangebot::Pid_s16 gcl_pid16_controller;

/****************************************************************************
**	FUNCTIONS
****************************************************************************/

/****************************************************************************
**	@brief main
**	main | void
****************************************************************************/
//! @return int |
//! @details Entry point of program
/***************************************************************************/

int main( void )
{
    //----------------------------------------------------------------
    //	VARS
    //----------------------------------------------------------------

    //Task scheduler. Initialize with fast tick for the screen driver
    Longan_nano::Scheduler g_scheduler = Longan_nano::Scheduler( Config::SCREEN_US, (Longan_nano::Chrono::Unit)Config::SCHEDULER_UNIT );
    //Display Driver
    Longan_nano::Screen g_screen;

    bool f_ret = false;

    //----------------------------------------------------------------
    //	INIT
    //----------------------------------------------------------------

    //Initialize LEDs
    Longan_nano::Leds::init();
    Longan_nano::Leds::set_color( Longan_nano::Leds::Color::BLACK );
    //Initialize scheduler
    f_ret |= g_scheduler.set_prescaler( Config::HMI_TASK_INDEX, Config::HMI_US /Config::SCREEN_US );
    f_ret |= g_scheduler.set_prescaler( Config::RED_LED_TASK_INDEX, Config::RED_LED_BLINK_US /Config::SCREEN_US );
    //f_ret |= g_scheduler.set_prescaler( Config::PAINT_TASK_INDEX, Config::PAINT_TASK_US /Config::SCREEN_US );
    f_ret |= g_scheduler.set_prescaler( Config::VNH7040_TASK_INDEX, Config::VNH7040_TASK_US /Config::SCREEN_US );
    //Initialize the Display
    g_screen.init();

    //Initialize Timer2 as Quadrature encoder decoder
    init_enc();
	//Initialize ADC
	init_adc_temp();
    //Initialize VNH7040
    f_ret = g_vnh7040.init();
    //PID Initialization
   	gcl_pid16_controller.gain_kp() = +12800;
	gcl_pid16_controller.gain_ki() = +100;
	gcl_pid16_controller.gain_kd() = +0;
	gcl_pid16_controller.set_limit_cmd( -10000, +10000 );


    /*
    gcl_pid_controller.set_fixed_point_position( 4 );
    gcl_pid_controller.set_pid_gain( 100.0, -10.0, 0.0 );
    gcl_pid_controller.set_command_limit( 0.1, 1000.0 );
    gcl_pid_controller.set_saturation_timeout(100);
    */
    //Error state
    gu1_pid_error = !gcl_pid_controller.is_ready();
    

    //----------------------------------------------------------------
    //	ERROR HANDLER
    //----------------------------------------------------------------

    if (f_ret == true)
    {
        g_screen.clear( Longan_nano::Screen::Color::RED );
        g_screen.print( 0, 0, "ERR: Init Failed", Longan_nano::Screen::Color::RED, Longan_nano::Screen::Color::BLACK );
        for EVER
        {
            //Update the screen
            g_screen.update();
        }
    }

    uint8_t task_index;

    //----------------------------------------------------------------
    //	BODY
    //----------------------------------------------------------------

    for EVER
    {
        //----------------------------------------------------------------
        //  Scheduler
        //----------------------------------------------------------------
        //  Hardwired scheduler to release tasks
        //  Thanks to the Longan Nano SysTick timer there is no need to use peripherals for timekeeping

        //Run the scheduler
        int8_t num_tasks = g_scheduler.update();
        //if: scheduling error
        if (num_tasks < 0)
        {
            g_screen.clear( Longan_nano::Screen::Color::LRED );
            g_screen.print( 0, 0, "ERR: Scheduler Error", Longan_nano::Screen::Color::LRED, Longan_nano::Screen::Color::BLACK );
        }

        //----------------------------------------------------------------
        //  Screen Driver
        //----------------------------------------------------------------

        //Set task index
        task_index = Config::SCREEN_TASK_INDEX;
        //if: task is scheduled for execution
        if (g_scheduler.is_task_scheduled( task_index ) == true)
        {
            //Task is running
            g_scheduler.run( task_index );
            
            //Update the screen
            g_screen.update();
            
            //Task is done running
            g_scheduler.done( task_index );
        }

        //----------------------------------------------------------------
        //  HMI Update
        //----------------------------------------------------------------

        //Set task index
        task_index = Config::HMI_TASK_INDEX;
        //if: task is scheduled for execution
        if (g_scheduler.is_task_scheduled( task_index )  == true)
        {
            //Task is running
            g_scheduler.run( task_index );
            
            //----------------------------------------------------------------
            //  Header
            //----------------------------------------------------------------
    
            g_screen.print( 0, 0, "Longan Nano Template" );

            //----------------------------------------------------------------
            //  CPU Load
            //----------------------------------------------------------------
    
            /*
            g_screen.print( 1, 0, "CPU Load" );
            //Use top of 100000 to get enough decimals. To print in the right format witn 100000->100.0% ENG exponent is -3 (TOP 100000 /10^ENG_EXP -3 = 100)
            g_screen.set_format( 8, Longan_nano::Screen::Format_align::ADJ_RIGHT, Longan_nano::Screen::Format_format::ENG, -3 );
            //Get CPU load with 100000 = 100.0%
            int cpu_load = g_scheduler.get_cpu_load( 100000 );
            g_screen.print( 1, 18, cpu_load );
            g_screen.print( 1, 19, '%' );
            */
            
            //----------------------------------------------------------------
            //  Uptime and CPU Time
            //----------------------------------------------------------------
            int tmp;
            //Since the time is measured in millisecond exponent is 10^-3
            g_screen.set_format( 8, Longan_nano::Screen::Format_align::ADJ_RIGHT, Longan_nano::Screen::Format_format::ENG, -3 );
            
            if (gu1_pid_error == true)
            {
                g_screen.print( 2, 0, "PID ERROR" );
            }
            else
            {
                g_screen.print( 2, 0, "PID OK" );
            }
            /*
            g_screen.print( 2, 0, "Uptime" );
            tmp = g_scheduler.get_uptime( Longan_nano::Chrono::Unit::milliseconds );
            g_screen.print( 2, 18, tmp );
            g_screen.print( 2, 19, 's' );
            */
            
            /*
            g_screen.print( 3, 0, "CPU Time" );
            tmp = g_scheduler.get_cpu_time( Longan_nano::Chrono::Unit::milliseconds );
            g_screen.print( 3, 18, tmp );
            g_screen.print( 3, 19, 's' );
            */

            g_screen.set_format( 8, Longan_nano::Screen::Format_align::ADJ_RIGHT, Longan_nano::Screen::Format_format::NUM );
            g_screen.print( 3, 0, "VNH7040" );
            g_screen.print( 3, 19, gs32_vnh7040_command );

            //----------------------------------------------------------------
            //  Execution time of fast task
            //---------------------------------------------------------------- 

            g_screen.print( 4, 0, "Fast Task" );
            tmp = g_scheduler.get_fast_task_avg_time();
            //The scheduler is using microseconds as time unit in the configuration
            g_screen.set_format( 8, Longan_nano::Screen::Format_align::ADJ_RIGHT, Longan_nano::Screen::Format_format::ENG, -6 );
            g_screen.print( 4, 18, tmp );
            g_screen.print( 4, 19, 's' );            

            //----------------------------------------------------------------
            //  ENC Profiling
            //---------------------------------------------------------------- 
            
            //Get the count of the timer
            tmp = gs32_encoder_feedback;
            g_screen.set_format( 8, Longan_nano::Screen::Format_align::ADJ_RIGHT, Longan_nano::Screen::Format_format::NUM );
            g_screen.print( 5, 0, "ENC CNT:" );
            g_screen.print( 5, 19, tmp );

            //----------------------------------------------------------------
            //  ADC Acquisition
            //---------------------------------------------------------------- 
	
            //Fetch and process temperature in celsius m[°c] 
            //int temperature = (1430000 - ADC_IDATA0(ADC0)*(3300000/4096)) / 4300 + 25000;
            int vnh7040_sense = ADC_IDATA0(ADC0) *3300 /4096;
            //Convert from VNH7040 sense voltage to motor current
            int vnh7040_current = vnh7040_sense *888/1000;
            //Fetch and process the reference in volts m[V]
            int vref_value = ADC_IDATA1(ADC0) *3300 /4096;
            adc_software_trigger_enable(ADC0, ADC_INSERTED_CHANNEL);
            //Compensate for the 'm' suffix, which is just used to have more digits
			g_screen.set_format( 8, Longan_nano::Screen::Format_align::ADJ_RIGHT, Longan_nano::Screen::Format_format::ENG, -3 );
            //Show temperature
			g_screen.print( 6, 0, "VNH7040 Vs:" );
            g_screen.print( 6, 18, vnh7040_sense );
			g_screen.print( 6, 19, 'V' );
            //Show voltage
            g_screen.print( 7, 0, "VNH7040 IM:" );
            g_screen.print( 7, 18, vnh7040_current );
            g_screen.print( 7, 19, 'A' );

            //Task is done running
            g_scheduler.done( task_index );
        }

        //----------------------------------------------------------------
        //  RED LED
        //----------------------------------------------------------------

        //Set task index
        task_index = Config::RED_LED_TASK_INDEX;
        //if: task is scheduled for execution
        if (g_scheduler.is_task_scheduled( task_index )  == true)
        {
            //Task is running
            g_scheduler.run( task_index );
        
            Longan_nano::Leds::toggle( Longan_nano::Leds::Color::RED );

            //Task is done running
            g_scheduler.done( task_index );
        }

        //----------------------------------------------------------------
        //  PAINT
        //----------------------------------------------------------------

        //Set task index
        task_index = Config::PAINT_TASK_INDEX;
        //if: task is scheduled for execution
        if (g_scheduler.is_task_scheduled( task_index )  == true)
        {
            //Task is running
            g_scheduler.run( task_index );
            
            //Roll a color
            Longan_nano::Screen::Color color_tmp = (Longan_nano::Screen::Color)g_rng_color(g_rng_engine);
            //Paint a colored square on screen
            g_screen.paint( Config::PAINT_START_HEIGHT, 0, Longan_nano::Screen::FRAME_BUFFER_HEIGHT -1, Longan_nano::Screen::FRAME_BUFFER_WIDTH -1, color_tmp );

            //Task is done running
            g_scheduler.done( task_index );
        }

        //----------------------------------------------------------------
        //  VNH7040
        //----------------------------------------------------------------

        //Set task index
        task_index = Config::VNH7040_TASK_INDEX;
        //if: task is scheduled for execution
        if (g_scheduler.is_task_scheduled( task_index ) == true)
        {
            //Task is running
            g_scheduler.run( task_index );
            
            //Feed a PWMramp
            //test_vnh7040();
            
            gs32_encoder_feedback = TIMER_CNT( TIMER2 );
            gs32_vnh7040_command = gcl_pid16_controller.exe( int32_t(32000), gs32_encoder_feedback  );
            gu1_pid_error = false;
            //gu1_pid_error = gcl_pid_controller.exe( 1000, gs32_encoder_feedback, gs32_vnh7040_command );
            //gs32_vnh7040_command = 100;
            
            if (gu1_pid_error == true)
            {
                gs32_vnh7040_command = 0;
                g_vnh7040.set_speed( 0 );
            }
            else
            {
                g_vnh7040.set_speed( gs32_vnh7040_command );
            }
            
            //Task is done running
            g_scheduler.done( task_index );
        }

        //----------------------------------------------------------------
        //  BUSY Wait
        //----------------------------------------------------------------
        //  Without wait, CPU load is 81% since the main is looping very fast when counting the scheduler
        //  Adding a busy wait lowers the CPU load to 12%
        //  The actual use of the task is about 1.1%, including the screen driver
        //  Scheduler execution should not be counted inside the CPU time as it only influences the precision of the flags
        //  The application can go into sleep for a time here instead of waiting

        //Longan_nano::Chrono::delay( Longan_nano::Chrono::Unit::microseconds, 1 );

    } //End forever

    //----------------------------------------------------------------
    //	RETURN
    //----------------------------------------------------------------

    return 0;
}	//end function: main


//Initialize timer2 as dual channel quadrature decoder
bool init_enc( void )
{
    //Clock the timer
    rcu_periph_clock_enable(RCU_TIMER2);
    //Clear initialization
    timer_deinit( TIMER2 );

    //Initialize timer GPIO
    rcu_periph_clock_enable( RCU_GPIOB );
    rcu_periph_clock_enable( RCU_AF );
    //TIMER2:PB4:CH0 TIMER2:PB5:CH1
    gpio_init( GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_4 );
    gpio_init( GPIOB, GPIO_MODE_IN_FLOATING, GPIO_OSPEED_50MHZ, GPIO_PIN_5 );
    gpio_pin_remap_config( GPIO_TIMER2_PARTIAL_REMAP, ENABLE );

    timer_auto_reload_shadow_enable(TIMER2);
    timer_parameter_struct timer_initpara;
    timer_struct_para_init(&timer_initpara);
    timer_initpara.prescaler         = 0;                   //Prescaler for the counter.
    timer_initpara.alignedmode       = TIMER_COUNTER_EDGE;  //Alignment specified by DIR bit
    timer_initpara.counterdirection  = TIMER_COUNTER_UP;    //Used in counter and encoder mode. Will be owerwritten by the quadrature mode
    timer_initpara.period            = UINT16_MAX;          //Counter Auto Reload. Value loaded for underflow or overflow
    timer_initpara.clockdivision     = TIMER_CKDIV_DIV1;    //Dead time and sampling clock DTS prescaler
    timer_init(TIMER2, &timer_initpara);
    
    //Quadrature encoder with double edge sensitivity
    timer_quadrature_decoder_mode_config( TIMER2, TIMER_ENCODER_MODE2, TIMER_IC_POLARITY_RISING, TIMER_IC_POLARITY_RISING );

    //timer_interrupt_flag_clear(TIMER2,TIMER_INT_FLAG_CH0);
    //timer_interrupt_enable(TIMER2,TIMER_INT_CH0);
    //Timer works in excess 0x8000
    TIMER_CNT( TIMER2 ) = uint16_t( 0x8000 );
    //timer_counter_value_config( TIMER2, 0x8000 );
    timer_enable(TIMER2);    
    
    return false;
}

bool init_adc_temp(void)
{

    //Clock GPIO
    rcu_periph_clock_enable(RCU_GPIOA);
    //Configure PA0 as analog input
    gpio_init( GPIOA, GPIO_MODE_AIN, GPIO_OSPEED_50MHZ, GPIO_PIN_0 );
    //Clock the ADC
    rcu_periph_clock_enable(RCU_ADC0);
    /* config ADC clock */
    rcu_adc_clock_config(RCU_CKADC_CKAPB2_DIV4);
    /* reset ADC */
    adc_deinit(ADC0);
    /* ADC mode config */
    adc_mode_config(ADC_MODE_FREE);
    /* ADC scan function enable */
    adc_special_function_config(ADC0, ADC_SCAN_MODE, ENABLE);
    /* ADC data alignment config */
    adc_data_alignment_config(ADC0, ADC_DATAALIGN_RIGHT);
    /* ADC temperature and Vrefint enable */
    adc_tempsensor_vrefint_enable();
    
    /* ADC channel length config */
    adc_channel_length_config(ADC0, ADC_INSERTED_CHANNEL, 2);
    /* ADC Channel 0, connected to VNH7040 VSense */
    adc_inserted_channel_config(ADC0, 0, ADC_CHANNEL_0, ADC_SAMPLETIME_239POINT5);
    /* ADC internal reference voltage channel config */
    adc_inserted_channel_config(ADC0, 1, ADC_CHANNEL_17, ADC_SAMPLETIME_239POINT5);

    /* ADC trigger config */
    adc_external_trigger_source_config(ADC0, ADC_INSERTED_CHANNEL, ADC0_1_EXTTRIG_INSERTED_NONE);
    adc_external_trigger_config(ADC0, ADC_INSERTED_CHANNEL, ENABLE);
    
    /* enable ADC interface */
    adc_enable(ADC0);
    Longan_nano::Chrono::delay( Longan_nano::Chrono::Unit::milliseconds, 1 );
    /* ADC calibration and reset calibration */
    adc_calibration_enable(ADC0);
	//Trigger f
	//adc_software_trigger_enable(ADC0, ADC_INSERTED_CHANNEL);
	
	return false;
}

/***************************************************************************/
//! @brief Private method
//! \n test_vnh7040 | void |
/***************************************************************************/
// @param
//! @return bool | false = OK | true = FAIL |
//! @details
//! \n Method
/***************************************************************************/

bool test_vnh7040( void )
{
    DENTER(); //Trace Enter
    ///--------------------------------------------------------------------------
    ///	VARS
    ///--------------------------------------------------------------------------

    static bool f_ramp = false;
    static int16_t speed = 0;
    const int16_t speed_delta = 16;
    const int16_t speed_max = 10000;

    ///--------------------------------------------------------------------------
    ///	BODY
    ///--------------------------------------------------------------------------

    //If increase speed
    if (f_ramp == false)
    {
        //increase speed
        speed += speed_delta;
        //if: speed cap
        if (speed >= speed_max)
        {
            //Clip to maximum
            speed = speed_max;
            //now decrease speed
            f_ramp = true;
        }
    }
    //If decrease speed
    else
    {
        //decrease speed
        speed -= speed_delta;
        //If: speed cap
        if (speed <= -speed_max)
        {
            //Clip to minimum
            speed = -speed_max;
            //Now increase speed
            f_ramp = false;
        }
    }
    //Set direction and speed setting of the VNH7040 controlled motor
    g_vnh7040.set_speed( speed );

    ///--------------------------------------------------------------------------
    ///	RETURN
    ///--------------------------------------------------------------------------
    DRETURN(); //Trace Return
    return false;	//OK
}   //end private method: test_vnh7040 | void |
