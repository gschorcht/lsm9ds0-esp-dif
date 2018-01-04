/**
 * Simple example with one sensor connected to I2C or SPI. It demonstrates the
 * different approaches to fetch the data. Either one of the interrupt signals
 * is used or new data are fetched periodically.
 *
 * Harware configuration:
 *
 *   I2C
 *
 *   +-----------------+   +----------+   +-----------------+   +----------+
 *   | ESP32           |   | LSM9DS0  |   | ESP8266         |   | LSM9DS0  |
 *   |                 |   |          |   |                 |   |          |
 *   |   GPIO 14 (SCL) ----> SCL      |   |   GPIO 14 (SCL) ----> SCL      |
 *   |   GPIO 13 (SDA) <---> SDA      |   |   GPIO 13 (SDA) <---> SDA      |
 *   |   GPIO 5        <---- INT1_XM  |   |   GPIO 5        <---- INT1_XM  |
 *   |   GPIO 4        <---- INT2_XM  |   |   GPIO 4        <---- INT2_XM  |
 *   |   GPIO 22       <---- INT_G    |   |   GPIO 2        <---- INT_G    |
 *   |   GPIO 23       <---- DRDY_G   |   |   GPIO ??       <---- DRDY_G   |
 *   +-----------------+   +----------+   +-----------------+   +----------+
 *
 *   SPI (two SPI interfaces needed -> only working on ESP32)  
 *
 *   +-----------------+   +----------+
 *   | ESP32           |   | LSM9DS0  |
 *   |                 |   |          |
 *   |   GPIO 16 (SCK) ----> SCK      |
 *   |   GPIO 17 (MOSI)----> SDI      |
 *   |   GPIO 18 (MISO)<---- SDO_XM   |
 *   |   GPIO 19 (CS)  ----> CS_XM    |
 *   |   GPIO 5        <---- INT1_XM  |
 *   |   GPIO 4        <---- INT2_XM  |
 *   |   GPIO 22       <---- INT_G    |
 *   |   GPIO 23       <---- DRDY_G   |
 *   +-----------------+    +---------+
 */

/* -- use following constants to define the example mode ----------- */

// #define SPI_USED     // SPI interface is used, otherwise I2C
// #define FIFO_MODE    // multiple sample read mode
// #define TEMP_USED    // temperature sensor used
// #define INT_DATA     // data interrupts used (data ready and FIFO status)
// #define INT_A_EVENT  // inertial event interrupts used (axis movement or 6D/4D orientation)
// #define INT_A_CLICK  // click detection interrupts used
// #define INT_M_THRESH // magnetic value exceeds threshold interrupt used
// #define INT_G_EVENT  // angular rate event interrupts used

#if defined(INT_DATA) || defined(INT_A_EVENT) || defined(INT_G_EVENT) || defined(INT_A_CLICK) || defined(INT_M_THRESH)
#define INT_USED
#endif

/* -- includes ----------------------------------------------------- */

#include "lsm9ds0.h"

/** -- platform dependent definitions ------------------------------ */

#ifdef ESP_PLATFORM  // ESP32 (ESP-IDF)

// user task stack depth for ESP32
#define TASK_STACK_DEPTH 2048

// SPI interface definitions for ESP32
#define SPI_BUS       HSPI_HOST
#define SPI_SCK_GPIO  16
#define SPI_MOSI_GPIO 17
#define SPI_MISO_GPIO 18
#define SPI_CS_GPIO   19

#else  // ESP8266 (esp-open-rtos)

// user task stack depth for ESP8266
#define TASK_STACK_DEPTH 256

// SPI interface definitions for ESP8266
#define SPI_BUS       1
#define SPI_SCK_GPIO  14
#define SPI_MOSI_GPIO 13
#define SPI_MISO_GPIO 12
#define SPI_CS_GPIO   2   // GPIO 15, the default CS of SPI bus 1, can't be used

#endif  // ESP_PLATFORM

// I2C interface defintions for ESP32 and ESP8266
#define I2C_BUS       0
#define I2C_SCL_PIN   14
#define I2C_SDA_PIN   13
#define I2C_FREQ      I2C_FREQ_100K

// interrupt GPIOs defintions for ESP8266 and ESP32
#define PIN_INT1_XM   5
#define PIN_INT2_XM   4
#define PIN_INT_G     6
#define PIN_DRDY_G    7

/* -- user tasks --------------------------------------------------- */

static lsm9ds0_am_sensor_t* sensor_am;
static lsm9ds0_g_sensor_t*  sensor_g;

/**
 * Common function used to get sensor data.
 */
void read_data ()
{
    #ifdef FIFO_MODE

    lsm9ds0_float_a_data_fifo_t a_fifo;

    // test for new accelerator data data
    if (lsm9ds0_new_a_data (sensor_am))
    {
        // fetch the accelerator data stored in FIFO
        uint8_t num = lsm9ds0_get_float_a_data_fifo (sensor_am, a_fifo);

        printf("%.3f LSM9DS0 a_num=%d\n", (double)sdk_system_get_time()*1e-3, num);

        for (int i=0; i < num; i++)
            // max. full scale is +-16 g and best resolution is 1 mg, i.e. 5 digits
            printf("%.3f LSM9DS0 (xyz)[g]  ax=%+7.3f ay=%+7.3f az=%+7.3f\n",
                   (double)sdk_system_get_time()*1e-3, 
                   a_fifo[i].ax, a_fifo[i].ay, a_fifo[i].az);
    }

    lsm9ds0_float_g_data_fifo_t g_fifo;

    // test for new accelerator data data
    if (lsm9ds0_new_g_data (sensor_g))
    {
        // fetch the accelerator data stored in FIFO
        uint8_t num = lsm9ds0_get_float_g_data_fifo (sensor_g, g_fifo);

        printf("%.3f LSM9DS0 g_num=%d\n", (double)sdk_system_get_time()*1e-3, num);

        for (int i=0; i < num; i++)
            // max. full scale is +-2000 dps and best sensitivity is 1 mdps, i.e. 7 digits
            printf("%.3f LSM9DS0 (xyz)[dps] dx=%+9.3f dx=%+9.3f dx=%+9.3f\n",
                   (double)sdk_system_get_time()*1e-3, 
                   g_fifo[i].x, g_fifo[i].y, g_fifo[i].z);
    }

    #else

    lsm9ds0_float_a_data_t  a_data;

    // test for new accelerator data and fetch them
    if (lsm9ds0_new_a_data (sensor_am) &&
        lsm9ds0_get_float_a_data (sensor_am, &a_data))
        // max. full scale is +-16 g and best resolution is 1 mg, i.e. 5 digits
        printf("%.3f LSM9DS0 (xyz)[g]   ax=%+9.3f ay=%+9.3f az=%+9.3f\n",
               (double)sdk_system_get_time()*1e-3, 
                a_data.ax, a_data.ay, a_data.az);
        
    lsm9ds0_float_g_data_t  g_data;

    if (lsm9ds0_new_g_data (sensor_g) &&
        lsm9ds0_get_float_g_data (sensor_g, &g_data))
        // max. full scale is +-2000 dps and best sensitivity is 1 mdps, i.e. 7 digits
        printf("%.3f LSM9DS0 (xyz)[dps] dx=%+9.3f dx=%+9.3f dx=%+9.3f\n",
               (double)sdk_system_get_time()*1e-3, g_data.x, g_data.y, g_data.z);

    #endif // FIFO_MODE

    lsm9ds0_float_m_data_t  m_data;

    // test for new magnetometer data and fetch them
    if (lsm9ds0_new_m_data (sensor_am) &&
        lsm9ds0_get_float_m_data (sensor_am, &m_data))
        // max. full scale is +-12 Gs and best resolution is 1 mGs, i.e. 5 digits
        printf("%.3f LSM9DS0 (xyz)[Gs]  mx=%+9.3f my=%+9.3f mz=%+9.3f\n",
               (double)sdk_system_get_time()*1e-3, 
                m_data.mx, m_data.my, m_data.mz);

    #ifdef TEMP_USED
    float temp = lsm9ds0_get_temperature (sensor_am);
    
    printf("%.3f LSM9DS0 (tmp)[°C] %+7.3f\n", (double)sdk_system_get_time()*1e-3, temp);
    #endif
}


#ifdef INT_USED
/**
 * In this case, any of the possible interrupts on interrupt signal *INT1* is
 * used to fetch the data.
 *
 * When interrupts are used, the user has to define interrupt handlers that
 * either fetches the data directly or triggers a task which is waiting to
 * fetch the data. In this example, the interrupt handler sends an event to
 * a waiting task to trigger the data gathering.
 */

static QueueHandle_t gpio_evt_queue = NULL;

// User task that fetches the sensor values.

void user_task_interrupt (void *pvParameters)
{
    uint8_t gpio;

    while (1)
    {
        if (xQueueReceive(gpio_evt_queue, &gpio, portMAX_DELAY))
        {
            lsm9ds0_int_am_data_source_t   am_data_src  = {};
            lsm9ds0_int_a_event_source_t   a_event_src  = {};
            lsm9ds0_int_a_click_source_t   a_click_src  = {};
            lsm9ds0_int_m_thresh_source_t  m_thresh_src = {};

            lsm9ds0_int_g_data_source_t    g_data_src   = {};
            lsm9ds0_int_g_event_source_t   g_event_src  = {};

            // get the source of the interrupt that reset *INTx* signals
            #ifdef INT_DATA
            lsm9ds0_get_int_am_data_source (sensor_am, &am_data_src);
            lsm9ds0_get_int_g_data_source  (sensor_g , &g_data_src);
            #elif INT_A_EVENT
            lsm9ds0_get_int_a_event_source (sensor_am, &a_event_src, lsm9ds0_int_a_event1_gen);
            #elif INT_A_CLICK
            lsm9ds0_get_int_a_click_source (sensor_am, &a_click_src);
            #elif INT_M_THRESH
            lsm9ds0_get_int_m_thresh_source(sensor_am, &m_thresh_src);
            #elif INT_G_EVENT
            lsm9ds0_get_int_g_event_source (sensor_g , &g_event_src);
            #endif

            // in case of DRDY interrupt
            if (am_data_src.a_data_ready || am_data_src.m_data_ready || g_data_src.data_ready)
                read_data ();
                
            // in case of FIFO interrupts read the whole FIFO
            else if (am_data_src.fifo_thresh || am_data_src.fifo_overrun ||
                     g_data_src.fifo_threshold  || g_data_src.fifo_overrun)
                read_data ();
    
            // in case of magnetic threshold interrupt
            else if (m_thresh_src.active)
            {
                printf("%.3f LSM9DS0 ", (double)sdk_system_get_time()*1e-3);
                if (m_thresh_src.x_pos) printf("mx exceeds threshold on positive side\n");
                if (m_thresh_src.y_pos) printf("my exceeds threshold on positive side\n");
                if (m_thresh_src.z_pos) printf("mz exceeds threshold on positive side\n");
                if (m_thresh_src.x_neg) printf("mx exceeds threshold on negative side\n");
                if (m_thresh_src.y_neg) printf("my exceeds threshold on negative side\n");
                if (m_thresh_src.z_neg) printf("mz exceeds threshold on negative side\n");
            }
            
            // in case of event interrupt
            else if (a_event_src.active)
            {
                printf("%.3f LSM9DS0 ", (double)sdk_system_get_time()*1e-3);
                if (a_event_src.x_low)  printf("ax is lower than threshold\n");
                if (a_event_src.y_low)  printf("ay is lower than threshold\n");
                if (a_event_src.z_low)  printf("az is lower than threshold\n");
                if (a_event_src.x_high) printf("ax is higher than threshold\n");
                if (a_event_src.y_high) printf("ay is higher than threshold\n");
                if (a_event_src.z_high) printf("az is higher than threshold\n");
            }

            // in case of click detection interrupt   
            else if (a_click_src.active)
               printf("%.3f LSM9DS0 %s\n", (double)sdk_system_get_time()*1e-3, 
                      a_click_src.s_click ? "single click" : "double click");
                      
            else if (g_event_src.active)
            {
                printf("%.3f LSM9DS0 ", (double)sdk_system_get_time()*1e-3);
                if (g_event_src.x_low)  printf("gx is lower than threshold\n");
                if (g_event_src.y_low)  printf("gy is lower than threshold\n");
                if (g_event_src.z_low)  printf("gz is lower than threshold\n");
                if (g_event_src.x_high) printf("gx is higher than threshold\n");
                if (g_event_src.y_high) printf("gy is higher than threshold\n");
                if (g_event_src.z_high) printf("gz is higher than threshold\n");
            }

        }
    }
}

// Interrupt handler which resumes user_task_interrupt on interrupt

void IRAM int_signal_handler (uint8_t gpio)
{
    // send an event with GPIO to the interrupt user task
    xQueueSendFromISR(gpio_evt_queue, &gpio, NULL);
}

#else // !INT_USED

/*
 * In this example, user task fetches the sensor values every seconds.
 */

void user_task_periodic(void *pvParameters)
{
    vTaskDelay (100/portTICK_PERIOD_MS);
    
    while (1)
    {
        // read sensor data
        read_data ();
        
        // passive waiting until 1 second is over
        vTaskDelay(200/portTICK_PERIOD_MS);
    }
}

#endif // INT_USED

/* -- main program ------------------------------------------------- */

void user_init(void)
{
    // Set UART Parameter.
    uart_set_baud(0, 115200);
    // Give the UART some time to settle
    vTaskDelay(1);

    /** -- MANDATORY PART -- */

    #ifdef SPI_USED

    // init the SPI interface at which LMS303D sensors are connected
    spi_bus_init (SPI_BUS, SPI_SCK_GPIO, SPI_MISO_GPIO, SPI_MOSI_GPIO);

    // init the sensor connected to SPI_BUS with SPI_CS_GPIO as chip select.
    sensor_am = lsm9ds0_init_am_sensor (SPI_BUS, 0, SPI_CS_GPIO);
    
    #else

    // init all I2C busses at which LSM9DS0 sensors are connected
    i2c_init (I2C_BUS, I2C_SCL_PIN, I2C_SDA_PIN, I2C_FREQ);
    
    // init the sensor connected to I2C_BUS.
    sensor_am = lsm9ds0_init_am_sensor (I2C_BUS, LSM9DS0_I2C_AM_ADDRESS_2, 0);
    sensor_g  = lsm9ds0_init_g_sensor  (I2C_BUS, LSM9DS0_I2C_G_ADDRESS_2 , 0);

    #endif
    
    if (sensor_am)
    {
        // --- SYSTEM CONFIGURATION PART ----
        
        #if !defined (INT_USED)

        // create a user task that fetches data from sensor periodically
        xTaskCreate(user_task_periodic, "user_task_periodic", TASK_STACK_DEPTH, NULL, 2, NULL);

        #else // INT_USED

        // create a task that is triggered only in case of interrupts to fetch the data
        xTaskCreate(user_task_interrupt, "user_task_interrupt", TASK_STACK_DEPTH, NULL, 2, NULL);

        // create an event queue to send interrupt events from interrupt
        // handler to the interrupt task
        gpio_evt_queue = xQueueCreate(10, sizeof(uint8_t));

        // configure interupt pins signals and set the interrupt handler
        gpio_enable(PIN_INT1_XM, GPIO_INPUT);
        gpio_enable(PIN_INT2_XM, GPIO_INPUT);
        gpio_enable(PIN_INT_G  , GPIO_INPUT);
        gpio_enable(PIN_DRDY_G , GPIO_INPUT);
        gpio_set_interrupt(PIN_INT1_XM, GPIO_INTTYPE_EDGE_POS, int_signal_handler);
        gpio_set_interrupt(PIN_INT2_XM, GPIO_INTTYPE_EDGE_POS, int_signal_handler);
        gpio_set_interrupt(PIN_INT_G  , GPIO_INTTYPE_EDGE_POS, int_signal_handler);
        gpio_set_interrupt(PIN_DRDY_G , GPIO_INTTYPE_EDGE_POS, int_signal_handler);

        #endif  // !defined(INT_USED)
        
        // -- SENSOR CONFIGURATION PART ---

        // Interrupt configuration has to be done before the sensor is set
        // into measurement mode
        
        // set the type of INTx signals if necessary
        lsm9ds0_config_int_am_signals (sensor_am, lsm9ds0_int_push_pull);
        lsm9ds0_config_int_g_signals  (sensor_g, lsm9ds0_int_g_high_active, lsm9ds0_int_push_pull);

        #ifdef INT_DATA
        // enable data interrupts on *INT2* (data ready or FIFO overrun and FIFO threshold)
        // data ready and FIFO status interrupts must not be enabled at the same time
        #ifdef FIFO_MODE
        lsm9ds0_enable_int_am (sensor_am, lsm9ds0_int_a_fifo_overrun, lsm9ds0_int_am_signal2, true);
        lsm9ds0_enable_int_am (sensor_am, lsm9ds0_int_a_fifo_thresh , lsm9ds0_int_am_signal2, true);
        lsm9ds0_enable_int_g  (sensor_g , lsm9ds0_int_g_fifo_overrun, true);
        lsm9ds0_enable_int_g  (sensor_g , lsm9ds0_int_g_fifo_thresh , true);
        #else
        lsm9ds0_enable_int_am (sensor_am, lsm9ds0_int_a_data_ready, lsm9ds0_int_am_signal2, true);
        lsm9ds0_enable_int_am (sensor_am, lsm9ds0_int_m_data_ready, lsm9ds0_int_am_signal2, true);
        lsm9ds0_enable_int_g  (sensor_g , lsm9ds0_int_g_data_ready, true);
        #endif // FIFO_MODE
        #endif // INT_DATA
        
        #ifdef INT_M_THRESH
        // enable magnetic threshold interrupts on signal *INT1* 
        lsm9ds0_int_m_thresh_config_t m_thresh_config;
    
        m_thresh_config.threshold    = 2000;
        m_thresh_config.x_enabled    = true;
        m_thresh_config.y_enabled    = true;
        m_thresh_config.z_enabled    = true;
        m_thresh_config.latch        = true;
        m_thresh_config.signal_level = lsm9ds0_int_m_high_active;
        
        lsm9ds0_set_int_m_thresh_config (sensor_am, &m_thresh_config);
        lsm9ds0_enable_int_am (sensor_am, lsm9ds0_int_m_thresh, lsm9ds0_int_am_signal1, true);
        #endif // INT_M_THRESH

        #ifdef INT_A_EVENT
        // enable inertial event interrupts on *INT1*
        lsm9ds0_int_a_event_config_t a_event_config;
    
        a_event_config.mode = lsm9ds0_or;       // axes movement wake-up
        // a_event_config.mode = lsm9ds0_and;   // free fall
        // a_event_config.mode = lsm9ds0_6d_movement;
        // a_event_config.mode = lsm9ds0_6d_position;
        // a_event_config.mode = lsm9ds0_4d_movement;
        // a_event_config.mode = lsm9ds0_4d_position;
        a_event_config.threshold = 50;
        a_event_config.x_low_enabled  = false;
        a_event_config.x_high_enabled = true;
        a_event_config.y_low_enabled  = false;
        a_event_config.y_high_enabled = true;
        a_event_config.z_low_enabled  = false;
        a_event_config.z_high_enabled = true;
        a_event_config.duration = 0;
        a_event_config.latch = true;
        
        lsm9ds0_set_int_a_event_config (sensor_am, &a_event_config, lsm9ds0_int_a_event1_gen);
        lsm9ds0_enable_int_am (sensor_am, lsm9ds0_int_a_event1, lsm9ds0_int_am_signal1, true);
        #endif // INT_A_EVENT

        #ifdef INT_A_CLICK
        // enable single click interrupt for z-axis on signal *INT1*
        lsm9ds0_int_a_click_config_t a_click_config;
        
        a_click_config.threshold = 10;
        a_click_config.x_single = false;
        a_click_config.x_double = false;        
        a_click_config.y_single = false;
        a_click_config.y_double = false;        
        a_click_config.z_single = true;
        a_click_config.z_double = false;
        a_click_config.latch = true;
        a_click_config.time_limit   = 1;
        a_click_config.time_latency = 1;
        a_click_config.time_window  = 3;
        
        lsm9ds0_set_int_a_click_config (sensor_am, &a_click_config);
        lsm9ds0_enable_int_am (sensor_am, lsm9ds0_int_a_click, lsm9ds0_int_am_signal1, true);
        #endif // INT_A_CLICK

        #ifdef INT_G_EVENT
        // enable angular rate event interrupts on *INT1*
        lsm9ds0_int_g_event_config_t g_event_config = {};
    
        g_event_config.x_high_enabled = true;
        g_event_config.y_high_enabled = true;
        g_event_config.z_high_enabled = true;
        g_event_config.x_low_enabled  = false;
        g_event_config.y_low_enabled  = false;
        g_event_config.z_low_enabled  = false;
        g_event_config.x_threshold = 5000;
        g_event_config.y_threshold = 5000;
        g_event_config.z_threshold = 5000;
    
        g_event_config.filter = lsm9ds0_g_hpf_only;
        g_event_config.and_or = false;
        g_event_config.duration = 0;
        g_event_config.latch = true;
    
        lsm9ds0_set_int_g_event_config (sensor_g, &g_event_config);
        lsm9ds0_enable_int_g (sensor_g, lsm9ds0_int_g_event, true);
        #endif // INT_G_EVENT
        
        #ifdef FIFO_MODE
        // clear the FIFO
        lsm9ds0_set_a_fifo_mode (sensor_am, lsm9ds0_bypass, 0);
        lsm9ds0_set_g_fifo_mode (sensor_g , lsm9ds0_bypass, 0);

        // activate the FIFO with a threshold of 10 samples (max. 31); if 
        // FIFO threshold interrupt is enabled, an interrupt is
        // generated when the FIFO content exceeds this threshold, i.e., 
        // when 11 samples are stored in FIFO
        lsm9ds0_set_a_fifo_mode (sensor_am, lsm9ds0_stream, 10);
        lsm9ds0_set_g_fifo_mode (sensor_g , lsm9ds0_stream, 10);
        #endif

        // configure HPF and implicitly reset the reference by a dummy read
        lsm9ds0_config_a_hpf (sensor_am, lsm9ds0_hpf_normal, true, true, true, true);
        
        #ifdef TEMP_USED
        // enable the temperature sensor_am
        lsm9ds0_enable_temperature (sensor_am, true);
        #endif
                
        // LAST STEP: Finally set scale and mode to start measurements
        lsm9ds0_set_a_scale(sensor_am, lsm9ds0_a_scale_2_g);
        lsm9ds0_set_m_scale(sensor_am, lsm9ds0_m_scale_4_Gs);
        lsm9ds0_set_g_scale(sensor_g , lsm9ds0_g_scale_245_dps);

        lsm9ds0_set_a_mode (sensor_am, lsm9ds0_a_odr_12_5, lsm9ds0_a_aaf_bw_773, true, true, true);
        lsm9ds0_set_m_mode (sensor_am, lsm9ds0_m_odr_12_5, lsm9ds0_m_low_res, lsm9ds0_m_continuous);
        lsm9ds0_set_g_mode (sensor_g , lsm9ds0_g_odr_95, 3, true, true, true);

        // -- SENSOR CONFIGURATION PART ---
    }
}

