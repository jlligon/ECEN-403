/**
 * @brief       BH1790GLC.h
 * @details     Optical Sensor for Heart Rate Monitor IC.
 *              Header file.
 *              Edited by James Ligon for 403
 *
 *
 * @return      N/A
 *
 * @author      Manuel Caballero
 * @date        07/December/2019
 * @version     07/December/2019    The ORIGIN
 * @pre         N/A.
 * @warning     N/A
 * @pre         This code belongs to AqueronteBlog ( http://unbarquero.blogspot.com ).
 */
#ifndef BH1790GLC_H
#define BH1790GLC_H

//#include "mbed.h"
#include <stdio.h>
#include <driver/i2c.h>
#include "driver/gpio.h"

/**
    Example:
@code
#include "mbed.h"
#include "BH1790GLC.h"

BH1790GLC myBH1790GLC   ( I2C_SDA, I2C_SCL, BH1790GLC::BH1790GLC_ADDRESS, 400000 );
Serial pc               ( USBTX, USBRX );

DigitalOut  myled ( LED1 );
Ticker      newReading;


//@brief Variables.
uint32_t    myState = 0;

//@brief   FUNCTION PROTOTYPES
void    changeDATA     ( void );



//@brief FUNCTION FOR APPLICATION MAIN ENTRY.
int main()
{
    BH1790GLC::BH1790GLC_data_t   BH1790GLC_data;
    BH1790GLC::BH1790GLC_status_t aux;

    pc.baud ( 115200 );

    myled   =   1;
    wait(3);
    myled   =   0;

    // Get Manufacturer IDs
    aux  =   myBH1790GLC.BH1790GLC_GetManufacturerID ( &BH1790GLC_data );

    // Get part IDs
    aux  =   myBH1790GLC.BH1790GLC_GetPartID ( &BH1790GLC_data );
    pc.printf( "Manufacturer ID: %x (0xE0) | Part ID: %x (0x0D)\r\n", BH1790GLC_data.manufacturer_id, BH1790GLC_data.part_id  );


    // Performs a software reset
    aux  =   myBH1790GLC.BH1790GLC_SoftReset ();

    // Configure the system control setting
    BH1790GLC_data.rdy                 =   BH1790GLC::MEAS_CONTROL1_RDY_OSC_BLOCK_ACTIVE;
    BH1790GLC_data.led_lighting_freq   =   BH1790GLC::MEAS_CONTROL1_LED_LIGHTING_FREQ_64HZ_MODE;
    BH1790GLC_data.rcycle              =   BH1790GLC::MEAS_CONTROL1_RCYCLE_32HZ_MODE;

    // Configure the measurement control setting
    BH1790GLC_data.led_en          =   BH1790GLC::MEAS_CONTROL2_LED_EN_0;
    BH1790GLC_data.led_on_time     =   BH1790GLC::MEAS_CONTROL2_LED_ON_TIME_0_6_MS_MODE;
    BH1790GLC_data.led_current     =   BH1790GLC::MEAS_CONTROL2_LED_CURRENT_1_MA_MODE;

    // Start measurement
    aux      =   myBH1790GLC.BH1790GLC_StartMeasurement ( BH1790GLC_data );

    newReading.attach( &changeDATA, 1 );                                        // the address of the function to be attached ( changeDATA ) and the interval ( 1s )

    // Let the callbacks take care of everything
    while(1) {
        sleep();

        myled = 1;

        if ( myState == 1 ) {
            // Get the raw DATAOUT values: DATAOUT_LEDOFF and DATAOUT_LEDON
            aux  =   myBH1790GLC.BH1790GLC_GetRawDataOut ( &BH1790GLC_data );

            // Transmit result over the UART
            pc.printf( "LED OFF: %x | LED ON: %x\r\n", BH1790GLC_data.dataOut_LED_OFF, BH1790GLC_data.dataOut_LED_ON  );

            myState  =   0;                                                     // Reset the variable
        }

        myled = 0;
    }
}


// @brief       changeDATA ( void  )
//
// @details     It changes myState variable
//
// @param[in]    N/A
//
// @param[out]   N/A.
//
//
// @return       N/A.
//
//
// @author      Manuel Caballero
// @date        07/December/2019
// @version     07/December/2019   The ORIGIN
// @pre         N/A
// @warning     N/A.
void changeDATA ( void )
{
    myState = 1;
}
@endcode
*/


/*!
 Library for the BH1790GLC Optical Sensor for Heart Rate Monitor IC.
*/
class BH1790GLC
{
public:
    /**
    * @brief   DEFAULT ADDRESS
    */
    typedef enum {
        BH1790GLC_ADDRESS  =   ( 0b1011011 << 1U )   /*!<  Device ADDR                   */
    } BH1790GLC_address_t;



    /**
      * @brief   REGISTER MAP
      */
    typedef enum {
        BH1790GLC_MANUFACTURER_ID     =   0x0F,   /*!<  Manufacturer ID                   */
        BH1790GLC_PART_ID             =   0x10,   /*!<  Part ID                           */
        BH1790GLC_RESET               =   0x40,   /*!<  SWRESET                           */
        BH1790GLC_MEAS_CONTROL1       =   0x41,   /*!<  Measurement setting Control       */
        BH1790GLC_MEAS_CONTROL2       =   0x42,   /*!<  Measurement setting Control       */
        BH1790GLC_MEAS_START          =   0x43,   /*!<  Start Measurement                 */
        BH1790GLC_DATAOUT_LEDOFF_LSB  =   0x54,   /*!<  Measurement Data LSB (LED OFF)    */
        BH1790GLC_DATAOUT_LEDOFF_MSB  =   0x55,   /*!<  Measurement Data MSB (LED OFF)    */
        BH1790GLC_DATAOUT_LEDON_LSB   =   0x56,   /*!<  Measurement Data LSB (LED ON)     */
        BH1790GLC_DATAOUT_LEDON_MSB   =   0x57    /*!<  Measurement Data MSB (LED ON)     */
    } BH1790GLC_register_map_t;



    /**
      * @brief   MANUFACTOR ID REGISTER
      */
    typedef enum {
        MANUFACTOR_ID_MANUFACTURER_ID               =   0xE0            /*!<  Manufacturer ID                           */
    } BH1790GLC_manufactor_id_manufacturer_id_t;


    /**
      * @brief   PART ID REGISTER
      */
    typedef enum {
        PART_ID_PART_ID                             =   0x0D            /*!<  Part ID                                   */
    } BH1790GLC_part_id_part_id_t;


    /**
      * @brief   RESET REGISTER
     */
    /* SWRESET <7>
     *    NOTE: Software reset.
     */
    typedef enum {
        RESET_SWRESET_MASK                          =   ( 1U << 7U ),   /*!<  SWRESET mask                              */
        RESET_SWRESET_ENABLED                       =   ( 1U << 7U ),   /*!<  Software reset is performed               */
        RESET_SWRESET_DISABLED                      =   ( 0U << 7U )    /*!<  Software reset is completed               */
    } BH1790GLC_reset_swreset_t;


    /**
      * @brief   MEAS_CONTROL1 REGISTER
      *
     */
    /* RDY <7>
     *    NOTE: OSC block.
     */
    typedef enum {
        MEAS_CONTROL1_RDY_MASK                      =   ( 1U << 7U ),   /*!<  RDY mask                                  */
        MEAS_CONTROL1_RDY_PROHIBITED                =   ( 0U << 7U ),   /*!<  Prohibited                                */
        MEAS_CONTROL1_RDY_OSC_BLOCK_ACTIVE          =   ( 1U << 7U )    /*!<  OSC block clock to internal block         */
    } BH1790GLC_meas_control1_rdy_t;


    /* LED_LIGHTING_FREQ <2>
     *    NOTE: Select LED emitting frequency.
     */
    typedef enum {
        MEAS_CONTROL1_LED_LIGHTING_FREQ_MASK        =   ( 1U << 2U ),       /*!<  LED_LIGHTLING_FREQ mask                   */
        MEAS_CONTROL1_LED_LIGHTING_FREQ_128HZ_MODE  =   ( 0U << 2U ),       /*!<  128Hz Mode                                */
        MEAS_CONTROL1_LED_LIGHTING_FREQ_64HZ_MODE   =   ( 1U << 2U )        /*!<  64Hz Mode                                 */
    } BH1790GLC_meas_control1_led_lighting_freq_t;


    /* RCYCLE <1:0>
     *    NOTE: Select Data reading frequency.
     */
    typedef enum {
        MEAS_CONTROL1_RCYCLE_MASK                   =   ( 0b11 << 0U ),     /*!<  RCYCLE mask                               */
        MEAS_CONTROL1_RCYCLE_64HZ_MODE              =   ( 0b01 << 0U ),     /*!<  128Hz Mode                                */
        MEAS_CONTROL1_RCYCLE_32HZ_MODE              =   ( 0b10 << 0U )      /*!<  64Hz Mode                                 */
    } BH1790GLC_meas_control1_rcycle_t;


    /**
      * @brief   MEAS_CONTROL2 REGISTER
      *
     */
    /* LED_EN <7:6>
     *    NOTE: Select LED driver mode.
     */
    typedef enum {
        MEAS_CONTROL2_LED_EN_MASK                   =   ( 0b11 << 6U ),     /*!<  LED_EN mask                               */
        MEAS_CONTROL2_LED_EN_0                      =   ( 0b00 << 6U ),     /*!<  LED1 and LED2 pulsed light emit           */
        MEAS_CONTROL2_LED_EN_1                      =   ( 0b01 << 6U ),     /*!<  LED1 constant light LED2 pulsed light emit*/
        MEAS_CONTROL2_LED_EN_2                      =   ( 0b10 << 6U ),     /*!<  LED1 pulsed light emit LED2 constant light*/
        MEAS_CONTROL2_LED_EN_3                      =   ( 0b11 << 6U )      /*!<  LED1 and LED2 constant light              */
    } BH1790GLC_meas_control2_led_en_t;


    /* LED_ON_TIME <5>
     *    NOTE: Select LED emitting time.
     */
    typedef enum {
        MEAS_CONTROL2_LED_ON_TIME_MASK              =   ( 1U << 5U ),       /*!<  LED_ON_TIME mask                          */
        MEAS_CONTROL2_LED_ON_TIME_0_3_MS_MODE       =   ( 0U << 5U ),       /*!<  0.3ms Mode                                */
        MEAS_CONTROL2_LED_ON_TIME_0_6_MS_MODE       =   ( 1U << 5U )        /*!<  0.6ms Mode                                */
    } BH1790GLC_meas_control2_led_on_time_t;


    /* LED_CURRENT <3:0>
     *    NOTE: Select LED lighting current.
     */
    typedef enum {
        MEAS_CONTROL2_LED_CURRENT_MASK              =   ( 0b1111 << 0U ),   /*!<  LED_CURRENT mask                          */
        MEAS_CONTROL2_LED_CURRENT_0_MA_MODE         =   ( 0x0 << 0U ),      /*!<  0mA Mode                                  */
        MEAS_CONTROL2_LED_CURRENT_1_MA_MODE         =   ( 0x8 << 0U ),      /*!<  1mA Mode                                  */
        MEAS_CONTROL2_LED_CURRENT_2_MA_MODE         =   ( 0x9 << 0U ),      /*!<  2mA Mode                                  */
        MEAS_CONTROL2_LED_CURRENT_3_MA_MODE         =   ( 0xA << 0U ),      /*!<  3mA Mode                                  */
        MEAS_CONTROL2_LED_CURRENT_6_MA_MODE         =   ( 0xB << 0U ),      /*!<  6mA Mode                                  */
        MEAS_CONTROL2_LED_CURRENT_10_MA_MODE        =   ( 0xC << 0U ),      /*!<  10mA Mode                                 */
        MEAS_CONTROL2_LED_CURRENT_20_MA_MODE        =   ( 0xD << 0U ),      /*!<  20mA Mode                                 */
        MEAS_CONTROL2_LED_CURRENT_30_MA_MODE        =   ( 0xE << 0U ),      /*!<  30mA Mode                                 */
        MEAS_CONTROL2_LED_CURRENT_60_MA_MODE        =   ( 0xF << 0U )       /*!<  60mA Mode                                 */
    } BH1790GLC_meas_control2_led_current_t;


    /**
      * @brief   MEAS_START REGISTER
      *
     */
    /* LED_EN <0>
     *    NOTE: Flag of start measurement. Start measurement by writing 'MEAS_ST=1' after writing 'RDY=1'. Measurement doesn�t restart if writing
     *          'MEAS_ST=1' after start measurement. When stop measurement, write 'SWRESET=1' without writing 'MEAS_ST=0'.
     */
    typedef enum {
        MEAS_START_MEAS_ST_MASK                     =   ( 1U << 0U ),       /*!<  MEAS_ST mask                              */
        MEAS_START_MEAS_ST_MEASUREMENT_START        =   ( 1U << 0U )        /*!<  Measurement start                         */
    } BH1790GLC_meas_start_meas_st_t;



#ifndef BH1790GLC_VECTOR_STRUCT_H
#define BH1790GLC_VECTOR_STRUCT_H
    typedef struct {
        /* Raw dataout   */
        uint16_t dataOut_LED_OFF;                                       /*!<  Green Data Count Value when LED no emitting   */
        uint16_t dataOut_LED_ON;                                        /*!<  Green Data Count Value when LED emitting      */

        /* System control setting    */
        BH1790GLC_meas_control1_rdy_t               rdy;                /*!<  OSC block is supply clock to internal block   */
        BH1790GLC_meas_control1_led_lighting_freq_t led_lighting_freq;  /*!<  Select LED emitting frequency                 */
        BH1790GLC_meas_control1_rcycle_t            rcycle;             /*!<  Select Measurement time                       */

        /* Measurement control setting   */
        BH1790GLC_meas_control2_led_en_t            led_en;             /*!<  Select LED driver mode                        */
        BH1790GLC_meas_control2_led_on_time_t       led_on_time;        /*!<  Select LED emitting time                      */
        BH1790GLC_meas_control2_led_current_t       led_current;        /*!<  Select LED driver current                     */

        /* Device identifications   */
        uint8_t manufacturer_id;                                        /*!<  Manufacturer ID                               */
        uint8_t part_id;                                                /*!<  Part ID                                       */
    } BH1790GLC_data_t;
#endif


    /**
      * @brief   INTERNAL CONSTANTS
      */
    typedef enum {
        BH1790GLC_SUCCESS  =   0U,                     /*!<  I2C communication success     */
        BH1790GLC_FAILURE  =   1U,                     /*!<  I2C communication failure     */
        I2C_SUCCESS        =   0U                      /*!<  I2C communication was fine    */
    } BH1790GLC_status_t;




    /** Create an BH1790GLC object connected to the specified I2C pins.
      *
      * @param sda     I2C data pin
      * @param scl     I2C clock pin
      * @param addr    I2C slave address
      * @param freq    I2C frequency
      */
    BH1790GLC ( gpio_num_t sda, gpio_num_t scl, uint32_t addr, uint32_t freq );

    /** Delete BH1790GLC object.
     */
    ~BH1790GLC();

    /** It gets the manufacturer ID.
    */
    BH1790GLC_status_t BH1790GLC_GetManufacturerID      ( BH1790GLC_data_t* myManufacturerID  );

    /** It gets the part ID.
      */
    BH1790GLC_status_t BH1790GLC_GetPartID              ( BH1790GLC_data_t* myPartID          );

    /** It performs a soft reset.
      */
    BH1790GLC_status_t BH1790GLC_SoftReset              ( void                                );

    /** It triggers a new measurement sample.
      */
    BH1790GLC_status_t BH1790GLC_StartMeasurement       ( BH1790GLC_data_t myConfData         );

    /** It gets the DATAOUT ( DATAOUT_LEDOFF and DATAOUT_LEDON data ). Raw data value.
      */
    BH1790GLC_status_t BH1790GLC_GetRawDataOut          ( BH1790GLC_data_t* myRawDataOut      );


private:
    I2C         _i2c;
    uint32_t    _BH1790GLC_Addr;
};

#endif
