/**
 * @brief       BH1790GLC.cpp
 * @details     Optical Sensor for Heart Rate Monitor IC.
 *              Function file.
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

#include "BH1790GLC.h"


BH1790GLC::BH1790GLC ( int sda, int scl, uint32_t addr, uint32_t freq )
    : _i2c             ( sda, scl )
    , _BH1790GLC_Addr  ( addr )
{
    _i2c.frequency( freq );
}


BH1790GLC::~BH1790GLC()
{
}


/**
 * @brief       BH1790GLC_GetManufacturerID ( BH1790GLC_data_t* )
 *
 * @details     It gets the manufacturer ID.
 *
 * @param[in]    N/A.
 *
 * @param[out]   myManufacturerID:  Manufacturer ID.
 *
 *
 * @return       Status of BH1790GLC_GetManufacturerID.
 *
 *
 * @author      Manuel Caballero
 * @date        07/December/2019
 * @version     07/December/2019   The ORIGIN
 * @pre         N/A
 * @warning     N/A.
 */
BH1790GLC::BH1790GLC_status_t BH1790GLC::BH1790GLC_GetManufacturerID ( BH1790GLC_data_t* myManufacturerID )
{
    char     cmd = 0;
    uint32_t aux;

    /* Read the register */
    cmd  =   BH1790GLC_MANUFACTURER_ID;
    aux  =   _i2c.write ( _BH1790GLC_Addr, &cmd, 1U, true );
    aux  =   _i2c.read  ( _BH1790GLC_Addr, &cmd, 1U );


    /* Parse data   */
    myManufacturerID->manufacturer_id   =    cmd;



    if ( aux == I2C_SUCCESS ) {
        return   BH1790GLC_SUCCESS;
    } else {
        return   BH1790GLC_FAILURE;
    }
}



/**
 * @brief       BH1790GLC_GetPartID ( BH1790GLC_data_t* )
 *
 * @details     It gets the part ID.
 *
 * @param[in]    N/A.
 *
 * @param[out]   myPartID:  Part ID.
 *
 *
 * @return       Status of BH1790GLC_GetPartID.
 *
 *
 * @author      Manuel Caballero
 * @date        07/December/2019
 * @version     07/December/2019   The ORIGIN
 * @pre         N/A
 * @warning     N/A.
 */
BH1790GLC::BH1790GLC_status_t BH1790GLC::BH1790GLC_GetPartID ( BH1790GLC_data_t* myPartID )
{
    char     cmd = 0;
    uint32_t aux;

    /* Read the register */
    cmd  =   BH1790GLC_PART_ID;
    aux  =   _i2c.write ( _BH1790GLC_Addr, &cmd, 1U, true );
    aux  =   _i2c.read  ( _BH1790GLC_Addr, &cmd, 1U );


    /* Parse data   */
    myPartID->part_id   =    cmd;



    if ( aux == I2C_SUCCESS ) {
        return   BH1790GLC_SUCCESS;
    } else {
        return   BH1790GLC_FAILURE;
    }
}



/**
 * @brief       BH1790GLC_SoftReset ( void )
 *
 * @details     It performs a software reset.
 *
 * @param[in]    N/A.
 *
 * @param[out]   N/A.
 *
 *
 * @return       Status of BH1790GLC_SoftReset.
 *
 *
 * @author      Manuel Caballero
 * @date        07/December/2019
 * @version     07/December/2019   The ORIGIN
 * @pre         N/A
 * @warning     N/A.
 */
BH1790GLC::BH1790GLC_status_t BH1790GLC::BH1790GLC_SoftReset ( void )
{
    char     cmd[2] = { 0 };
    uint32_t aux;

    /* Update the register */
    cmd[0]   =   BH1790GLC_RESET;
    cmd[1]   =   RESET_SWRESET_ENABLED;
    aux      =   _i2c.write ( _BH1790GLC_Addr, &cmd[0], sizeof( cmd )/sizeof( cmd[0] ), false );



    if ( aux == I2C_SUCCESS ) {
        return   BH1790GLC_SUCCESS;
    } else {
        return   BH1790GLC_FAILURE;
    }
}


/**
 * @brief       BH1790GLC_StartMeasurement ( BH1790GLC_data_t )
 *
 * @details     It triggers a new measurement sample.
 *
 * @param[in]    myConfData:        System control setting and Measurement control setting.
 *
 * @param[out]   N/A.
 *
 *
 * @return       Status of BH1790GLC_StartMeasurement.
 *
 *
 * @author      Manuel Caballero
 * @date        07/December/2019
 * @version     07/December/2019   The ORIGIN
 * @pre         Start measurement by writing MEAS_ST=1 after writing RDY=1. Measurement doesn`t
 *              restart if writing MEAS_ST=1 after start measurement. When stop measurement, write SWRESET=1
 *              without writing MEAS_ST=0 (for changing some parameters as well).
 * @pre         The user MUST RESPECT the measurement time, T_INT = 28ms ( maximum ).
 * @pre         The function uses auto-increment.
 * @warning     System control setting and measurement control setting must be initialized in order to guarantee
 *              the correct behavior of the device.
 */
BH1790GLC::BH1790GLC_status_t BH1790GLC::BH1790GLC_StartMeasurement ( BH1790GLC_data_t myConfData )
{
    char     cmd[4] = { 0 };
    uint32_t aux;

    /* Write the register */
    cmd[0]   =   BH1790GLC_MEAS_CONTROL1;
    cmd[1]   =   ( myConfData.rdy | myConfData.led_lighting_freq | myConfData.rcycle );
    cmd[2]   =   ( myConfData.led_en | myConfData.led_on_time | myConfData.led_current );
    cmd[3]   =   MEAS_START_MEAS_ST_MEASUREMENT_START;
    aux      =   _i2c.write ( _BH1790GLC_Addr, &cmd[0], sizeof( cmd )/sizeof( cmd[0] ), false );



    if ( aux == I2C_SUCCESS ) {
        return   BH1790GLC_SUCCESS;
    } else {
        return   BH1790GLC_FAILURE;
    }
}



/**
 * @brief       BH1790GLC_GetRawDataOut ( void )
 *
 * @details     It gets the DATAOUT ( DATAOUT_LEDOFF and DATAOUT_LEDON data ). Raw data value.
 *
 * @param[in]    N/A.
 *
 * @param[out]   myRawDataOut:      Raw data for DATAOUT_LEDOFF and DATAOUT_LEDON.
 *
 *
 * @return       Status of BH1790GLC_GetRawDataOut.
 *
 *
 * @author      Manuel Caballero
 * @date        07/December/2019
 * @version     07/December/2019   The ORIGIN
 * @pre         This function uses auto-increment.
 * @warning     N/A.
 */
BH1790GLC::BH1790GLC_status_t BH1790GLC::BH1790GLC_GetRawDataOut ( BH1790GLC_data_t* myRawDataOut )
{
    char     cmd[4] = { 0 };
    uint32_t aux;

    /* Read the register */
    cmd[0]   =   BH1790GLC_DATAOUT_LEDOFF_LSB;
    aux      =   _i2c.write ( _BH1790GLC_Addr, &cmd[0], 1U, true );
    aux      =   _i2c.read  ( _BH1790GLC_Addr, &cmd[0], sizeof( cmd )/sizeof( cmd[0] ) );

    /* Parse the data    */
    myRawDataOut->dataOut_LED_OFF    =   cmd[1];
    myRawDataOut->dataOut_LED_OFF  <<=   8U;
    myRawDataOut->dataOut_LED_OFF   |=   cmd[0];

    myRawDataOut->dataOut_LED_ON     =   cmd[3];
    myRawDataOut->dataOut_LED_ON  <<=    8U;
    myRawDataOut->dataOut_LED_ON    |=   cmd[2];



    if ( aux == I2C_SUCCESS ) {
        return   BH1790GLC_SUCCESS;
    } else {
        return   BH1790GLC_FAILURE;
    }
}
