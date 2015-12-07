#include "mma7660.h"

typedef union __attribute__((__packed__)) 
{
    struct __attribute__((__packed__)) 
    {
        uint8_t orientation_data : 5;
        uint8_t sign  : 1;
        uint8_t alert : 1;
        uint8_t       : 1;
    } fields;
    uint8_t value;
} mma7660_raw_data_t;    

uint32_t mma7660_register_write(nrf_drv_twi_t const * const p_twi_instance, uint8_t reg, uint8_t val)
{
    uint8_t tx_buf[2];
    tx_buf[0] = reg;
    tx_buf[1] = val;
    return (nrf_drv_twi_tx(p_twi_instance, MMA7660_DEFAULT_ADDRESS, tx_buf, sizeof(tx_buf), false) != NRF_SUCCESS);
}

uint32_t mma7660_register_read(nrf_drv_twi_t const * const p_twi_instance, uint8_t reg, uint8_t *buf, uint8_t num_of_bytes)
{
    buf[0] = reg;
    uint32_t err_code = nrf_drv_twi_tx(p_twi_instance, MMA7660_DEFAULT_ADDRESS, buf, sizeof(uint8_t), false);
    if (err_code != NRF_SUCCESS)
        return err_code;
    
    err_code = nrf_drv_twi_rx(p_twi_instance, MMA7660_DEFAULT_ADDRESS, buf, num_of_bytes, false);            
    return err_code;
}

uint32_t mma7660_init(nrf_drv_twi_t const * const p_twi_instance, mma7660_mode_t samples_per_second)
{
    uint32_t err_code = NRF_ERROR_INVALID_STATE;
    if (p_twi_instance != NULL)
    {
        // Clear the MODE register, maybe not needed.
        err_code = mma7660_register_write(p_twi_instance, MMA7660_MODE, 0);
        if (err_code != NRF_SUCCESS)
            return err_code;

        err_code = mma7660_register_write(p_twi_instance, MMA7660_SR, (uint8_t)samples_per_second);
        if (err_code != NRF_SUCCESS)
            return err_code;
        
        err_code = mma7660_register_write(p_twi_instance, MMA7660_MODE, 1);
    }
    return err_code;           
}

uint32_t mma7660_read_xyz(nrf_drv_twi_t const * const p_twi_instance, mma7660_accelerometer_data_t * p_data)
{
    uint32_t err_code = NRF_ERROR_INVALID_STATE;
    mma7660_raw_data_t buffer[3];
    
    if (p_twi_instance != NULL)
    {
        err_code = mma7660_register_read(p_twi_instance, MMA7660_X, (uint8_t*)buffer, sizeof(buffer));
        if (err_code != NRF_SUCCESS)
            return err_code;
        
        /*mma7660_raw_data_t *ptr = (mma7660_raw_data_t *) buffer;
        // Verify received data.
        for (int i = 0; i < sizeof(buffer); i++)
        {
            // Check ALERT bit, re-read register and start over.
            if (ptr->fields.alert)
            {                
                do
                {
                    err_code = mma7660_register_read(p_twi_instance, i, (uint8_t*)&buffer[i], 1);
                    if (err_code != NRF_SUCCESS)
                        return err_code;
                } while (
                if (i) i--;
                continue;
            }
            // Increment ptr
            ptr++;
        }*/
        // Pack the raw data into a more representable format and cast to int8_t (or with 0xE0, see MMA7660 data output values)
        p_data->x.orientation_data = buffer[0].fields.sign ? (buffer[0].fields.orientation_data | 0xE0) : buffer[0].fields.orientation_data;        
        p_data->y.orientation_data = buffer[1].fields.sign ? (buffer[1].fields.orientation_data | 0xE0) : buffer[1].fields.orientation_data;       
        p_data->z.orientation_data = buffer[2].fields.sign ? (buffer[2].fields.orientation_data | 0xE0) : buffer[2].fields.orientation_data;
    }
    return err_code;
}
