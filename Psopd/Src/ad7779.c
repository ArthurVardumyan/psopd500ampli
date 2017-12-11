/***************************************************************************//**
 *   @file   ad7779.c
 *   @brief  Implementation of AD7779 Driver.
 *   @author DBogdan (dragos.bogdan@analog.com)
********************************************************************************
 * Copyright 2016(c) Analog Devices, Inc.
 *
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions are met:
 *  - Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 *  - Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 *  - Neither the name of Analog Devices, Inc. nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *  - The use of this software may or may not infringe the patent rights
 *    of one or more patent holders.  This license does not release you
 *    from the requirement that you obtain separate licenses from these
 *    patent holders to use this software.
 *  - Use of the software either in source or binary form, must be run
 *    on or directly connected to an Analog Devices Inc. component.
 *
 * THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES "AS IS" AND ANY EXPRESS OR
 * IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
 * MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED.
 * IN NO EVENT SHALL ANALOG DEVICES BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 * LIMITED TO, INTELLECTUAL PROPERTY RIGHTS, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*******************************************************************************/

/******************************************************************************/
/***************************** Include Files **********************************/
/******************************************************************************/
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include "stm32f4xx_hal.h"
#include "defines.h"
#include "ad7779.h"
#include "communication.h"
#include "crc16.h"

uint8_t readout_buffer[32];
extern uint32_t greenled_counter;
extern ad7779_dev   ad7779device;
extern readout_packet_t packet;
extern uint32_t odr_time_us;

/******************************************************************************/
/*************************** Constants Definitions ****************************/
/******************************************************************************/
const uint8_t pin_mode_options[16][4] = {
/*	GAIN_1	GAIN_2	GAIN_4	GAIN_8 */
	{0x03,	0xFF,	0x07,	0xFF},	// DEC_RATE_128, HIGH_RES, EXT_REF
	{0x0A,	0xFF,	0xFF,	0xFF},	// DEC_RATE_128, HIGH_RES, INT_REF
	{0x0D,	0xFF,	0xFF,	0xFF},	// DEC_RATE_128, LOW_PWR, EXT_REF
	{0x0E,	0xFF,	0xFF,	0xFF},	// DEC_RATE_128, LOW_PWR, INT_REF
	{0x02,	0x04,	0x06,	0xFF},	// DEC_RATE_256, HIGH_RES, EXT_REF
	{0x09,	0xFF,	0xFF,	0xFF},	// DEC_RATE_256, HIGH_RES, INT_REF
	{0x0C,	0xFF,	0xFF,	0xFF},	// DEC_RATE_256, LOW_PWR, EXT_REF
	{0x0F,	0xFF,	0xFF,	0xFF},	// DEC_RATE_256, LOW_PWR, INT_REF
	{0x01,	0xFF,	0x05,	0xFF},	// DEC_RATE_512, HIGH_RES, EXT_REF
	{0x08,	0xFF,	0xFF,	0xFF},	// DEC_RATE_512, HIGH_RES, INT_REF
	{0x08,	0xFF,	0xFF,	0xFF},	// DEC_RATE_512, LOW_PWR, EXT_REF
	{0xFF,	0xFF,	0xFF,	0xFF},	// DEC_RATE_512, LOW_PWR, INT_REF
	{0x00,	0xFF,	0xFF,	0xFF},	// DEC_RATE_1024, HIGH_RES, EXT_REF
	{0xFF,	0xFF,	0xFF,	0xFF},	// DEC_RATE_1024, HIGH_RES, INT_REF
	{0xFF,	0xFF,	0xFF,	0xFF},	// DEC_RATE_1024, LOW_PWR, EXT_REF
	{0xFF,	0xFF,	0xFF,	0xFF}	// DEC_RATE_1024, LOW_PWR, INT_REF	
};

/******************************************************************************/
/************************** Functions Implementation **************************/
/******************************************************************************/
/**
 * Compute CRC8 checksum.
 * @param data - The data buffer.
 * @param data_size - The size of the data buffer.
 * @return CRC8 checksum.
 */
uint8_t ad7779_compute_crc8(uint8_t *data,
							uint8_t data_size)
{
	uint8_t i;
	uint8_t crc = 0;

	while (data_size) {
		for (i = 0x80; i != 0; i >>= 1) {
			if (((crc & 0x80) != 0) != ((*data & i) != 0)) {
				crc <<= 1;
				crc ^= AD7779_CRC8_POLY;
			} else
				crc <<= 1;
		}
		data++;
		data_size--;
	}

	return crc;
}

/**
 * SPI internal register read from device.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param reg_data - The register data.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad7779_spi_int_reg_read(ad7779_dev *dev,
								uint8_t reg_addr,
								uint8_t *reg_data)
{
	uint8_t txbuf[3];
        uint8_t rxbuf[3];
	uint8_t buf_size = 2;
	uint8_t crc;
	int32_t ret;

	txbuf[0] = 0x80 | (reg_addr & 0x7F);
	txbuf[1] = 0x00;
	txbuf[2] = 0x00;
	if (dev->spi_crc_en == AD7779_ENABLE) buf_size = 3;
	ret = HAL_SPI_TransmitReceive(&dev->spi_dev, txbuf, rxbuf, buf_size, SPI_TIMEOUT_1);

	*reg_data = rxbuf[1];
	if (dev->spi_crc_en == AD7779_ENABLE) {
		rxbuf[0] = 0x80 | (reg_addr & 0x7F);
		crc = ad7779_compute_crc8(&rxbuf[0], 2);
		if (crc != rxbuf[2]) {
			printf("%s: CRC Error.\n", __func__);
			ret = HAL_ERROR;
		}
	}

	return ret;
}

/**
 * SPI internal register write to device.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param reg_data - The register data.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad7779_spi_int_reg_write(ad7779_dev *dev,
								 uint8_t reg_addr,
								 uint8_t reg_data)
{
	uint8_t txbuf[3];
        uint8_t rxbuf[3];
	uint8_t buf_size = 2;
	int32_t ret;

	txbuf[0] = 0x00 | (reg_addr & 0x7F);
	txbuf[1] = reg_data;
	if (dev->spi_crc_en == AD7779_ENABLE) {
		txbuf[2] = ad7779_compute_crc8(&txbuf[0], 2);
		buf_size = 3;
	}
	ret = HAL_SPI_TransmitReceive(&dev->spi_dev, txbuf, rxbuf, buf_size, SPI_TIMEOUT_1);
	dev->cached_reg_val[reg_addr] = reg_data;

	return ret;
}

/**
 * SPI internal register read from device using a mask.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param mask - The mask.
 * @param data - The register data.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad7779_spi_int_reg_read_mask(ad7779_dev *dev,
									 uint8_t reg_addr,
									 uint8_t mask,
									 uint8_t *data)
{
	uint8_t reg_data;
	int32_t ret;

	ret = ad7779_spi_int_reg_read(dev, reg_addr, &reg_data);
	*data = (reg_data & mask);

	return ret;
}

/**
 * SPI internal register write to device using a mask.
 * @param dev - The device structure.
 * @param reg_addr - The register address.
 * @param mask - The mask.
 * @param data - The register data.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad7779_spi_int_reg_write_mask(ad7779_dev *dev,
									  uint8_t reg_addr,
									  uint8_t mask,
									  uint8_t data)
{
	uint8_t reg_data;
	int32_t ret;

	reg_data = dev->cached_reg_val[reg_addr];
	reg_data &= ~mask;
	reg_data |= data;
	ret = ad7779_spi_int_reg_write(dev, reg_addr, reg_data);

	return ret;
}


/**
 * Set SPI operation mode.
 * @param dev - The device structure.
 * @param mode - The SPI operation mode.
 *				 Accepted values: AD7779_INT_REG
 *								  AD7779_SD_CONV
 *								  AD7779_SAR_CONV
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad7779_set_spi_op_mode(ad7779_dev *dev,
							   ad7779_spi_op_mode mode)
{
	int32_t ret;
	uint8_t cfg_2;
	uint8_t cfg_3;

	switch (mode) {
	case AD7779_SD_CONV:
		cfg_2 = 0;
		cfg_3 = AD7779_SPI_SLAVE_MODE_EN;
		break;
	case AD7779_SAR_CONV:
		cfg_2 = AD7779_SAR_DIAG_MODE_EN;
		cfg_3 = 0;
		break;
	default:	// AD7779_INT_REG
		cfg_2 = 0;
		cfg_3 = 0;
	}
	ret = ad7779_spi_int_reg_write_mask(dev,
										AD7779_REG_GENERAL_USER_CONFIG_2,
										AD7779_SAR_DIAG_MODE_EN,
										cfg_2);
	ret |= ad7779_spi_int_reg_write_mask(dev,
										 AD7779_REG_GENERAL_USER_CONFIG_3,
										 AD7779_SPI_SLAVE_MODE_EN,
										 cfg_3);
	dev->spi_op_mode = mode;

	return ret;
}

/**
 * Get SPI operation mode.
 * @param dev - The device structure.
 * @param mode - The SPI operation mode.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad7779_get_spi_op_mode(ad7779_dev *dev,
							   ad7779_spi_op_mode *mode)
{
	*mode = dev->spi_op_mode;

	return HAL_OK;
}



/**
 * Set the state (enable, disable) of the channel.
 * @param dev - The device structure.
 * @param ch - The channel number.
 * 			   Accepted values: AD7779_CH0
 * 			   					AD7779_CH1
 * 			   					AD7779_CH2
 * 			   					AD7779_CH3
 * 			   					AD7779_CH4
 * 			   					AD7779_CH5
 * 			   					AD7779_CH6
 * 			   					AD7779_CH7
 * @param state - The channel state.
 * 				  Accepted values: AD7779_ENABLE
 * 								   AD7779_DISABLE
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad7779_set_state(ad7779_dev *dev,
						 ad7779_ch ch,
						 ad7779_state state)
{
	int32_t ret;

	ret = ad7779_spi_int_reg_write_mask(dev,
										AD7779_REG_CH_DISABLE,
										AD7779_CH_DISABLE(0x1),
										AD7779_CH_DISABLE(state));

	return ret;
}

/**
 * Get the state (enable, disable) of the selected channel.
 * @param dev - The device structure.
 * @param ch - The channel number.
 * 			   Accepted values: AD7779_CH0
 * 			   					AD7779_CH1
 * 			   					AD7779_CH2
 * 			   					AD7779_CH3
 * 			   					AD7779_CH4
 * 			   					AD7779_CH5
 * 			   					AD7779_CH6
 * 			   					AD7779_CH7
 * @param state - The channel state.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad7779_get_state(ad7779_dev *dev,
						 ad7779_ch ch,
						 ad7779_state *state)
{
	*state = dev->state[ch];

	return HAL_OK;
}

/**
 * Set the gain of the selected channel.
 * @param dev - The device structure.
 * @param ch - The channel number.
 * 			   Accepted values: AD7779_CH0
 * 			   					AD7779_CH1
 * 			   					AD7779_CH2
 * 			   					AD7779_CH3
 * 			   					AD7779_CH4
 * 			   					AD7779_CH5
 * 			   					AD7779_CH6
 * 			   					AD7779_CH7
 * @param gain - The gain value.
 * 				 Accepted values: AD7779_GAIN_1
 * 								  AD7779_GAIN_2
 * 								  AD7779_GAIN_4
 * 								  AD7779_GAIN_8
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad7779_set_gain(ad7779_dev *dev,
						ad7779_ch ch,
						ad7779_gain gain)
{
	int32_t ret;

            if (dev->ctrl_mode == AD7779_SPI_CTRL)
            {
		dev->gain[ch] = gain;
		ret = ad7779_spi_int_reg_write_mask(dev,
				AD7779_REG_CH_CONFIG(ch),
				AD7779_CH_GAIN(0x3),
                                AD7779_CH_GAIN(gain));
	}

	return ret;
}

/**
 * Get the gain of the selected channel.
 * @param dev - The device structure.
 * @param ch - The channel number.
 * 			   Accepted values: AD7779_CH0
 * 			   					AD7779_CH1
 * 			   					AD7779_CH2
 * 			   					AD7779_CH3
 * 			   					AD7779_CH4
 * 			   					AD7779_CH5
 * 			   					AD7779_CH6
 * 			   					AD7779_CH7
 * @param gain - The gain value.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad7779_get_gain(ad7779_dev *dev,
						ad7779_ch ch,
						ad7779_gain *gain)
{
	*gain = dev->gain[ch];

	return HAL_OK;
}

/**
 * Set the decimation rate.
 * @param dev - The device structure.
 * @param integer_val - The integer value.
 * @param decimal_val - The decimal value.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad7779_set_dec_rate(ad7779_dev *dev,
							uint16_t int_val,
							uint16_t dec_val)
{
	int32_t ret;
	uint8_t msb;
	uint8_t lsb;

            if (dev->ctrl_mode == AD7779_SPI_CTRL)
            {
		msb = (int_val & 0x0F00) >> 8;
		lsb = (int_val & 0x00FF) >> 0;
		ret = ad7779_spi_int_reg_write(dev,
				AD7779_REG_SRC_N_MSB,
						msb);
		ret |= ad7779_spi_int_reg_write(dev,
				AD7779_REG_SRC_N_LSB,
						lsb);
		dec_val = (dec_val * 65536) / 1000;
		msb = (dec_val & 0xFF00) >> 8;
		lsb = (dec_val & 0x00FF) >> 0;
		ret |= ad7779_spi_int_reg_write(dev,
                              AD7779_REG_SRC_IF_MSB,
						msb);
		ret |= ad7779_spi_int_reg_write(dev,
                              AD7779_REG_SRC_IF_LSB,
                                               lsb);
		dev->dec_rate_int = int_val;
		dev->dec_rate_int = dec_val;
	}

	return ret;
}

/**
 * Get the decimation rate.
 * @param dev - The device structure.
 * @param integer_val - The integer value.
 * @param decimal_val - The decimal value.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad7779_get_dec_rate(ad7779_dev *dev,
							uint16_t *int_val,
							uint16_t *dec_val)
{
	*int_val = dev->dec_rate_int;
	*dec_val = dev->dec_rate_int;

	return HAL_OK;
}

/**
 * Set the power mode.
 * @param dev - The device structure.
 * @param pwr_mode - The power mode.
 * 					 Accepted values: AD7779_HIGH_RES
 *									  AD7779_LOW_PWR
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad7779_set_power_mode(ad7779_dev *dev,
							  ad7779_pwr_mode pwr_mode)
{
	int32_t ret;

	ret = ad7779_spi_int_reg_write_mask(dev,
										AD7779_REG_GENERAL_USER_CONFIG_1,
										AD7779_MOD_POWERMODE,
										pwr_mode ? AD7779_MOD_POWERMODE : 0);
	dev->pwr_mode = pwr_mode;

	return ret;
}

/**
 * Get the power mode.
 * @param dev - The device structure.
 * @param pwr_mode - The power mode.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad7779_get_power_mode(ad7779_dev *dev,
							  ad7779_pwr_mode *pwr_mode)
{
	*pwr_mode = dev->pwr_mode;

	return HAL_OK;
}

/**
 * Set the reference type.
 * @param dev - The device structure.
 * @param pwr_mode - The reference type.
 * 					 Accepted values: AD7779_EXT_REF
 *									  AD7779_INT_REF
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad7779_set_reference_type(ad7779_dev *dev,
								  ad7779_ref_type ref_type)
{
	int32_t ret;

	ret = ad7779_spi_int_reg_write_mask(dev,
										AD7779_REG_GENERAL_USER_CONFIG_1,
										AD7779_PDB_REFOUT_BUF,
										ref_type ? AD7779_PDB_REFOUT_BUF : 0);
	dev->ref_type = ref_type;

	return ret;
}

/**
 * Get the reference type.
 * @param dev - The device structure.
 * @param pwr_mode - The reference type.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad7779_get_reference_type(ad7779_dev *dev,
								  ad7779_ref_type *ref_type)
{
	*ref_type = dev->ref_type;

	return HAL_OK;
}

/**
 * Set the DCLK divider.
 * @param dev - The device structure.
 * @param div - The DCLK divider.
 *				Accepted values: AD7779_DCLK_DIV_1
 *								 AD7779_DCLK_DIV_2
 *								 AD7779_DCLK_DIV_4
 *								 AD7779_DCLK_DIV_8
 *								 AD7779_DCLK_DIV_16
 *								 AD7779_DCLK_DIV_32
 *								 AD7779_DCLK_DIV_64
 *								 AD7779_DCLK_DIV_128
 * @return SUCCESS in case of success, negative error code otherwise.
 */

int32_t ad7779_set_dclk_div(ad7779_dev *dev,
							ad7768_dclk_div div)
{
  
	int32_t ret;

		 if (dev->ctrl_mode == AD7779_SPI_CTRL)
                 {
                   ret = ad7779_spi_int_reg_write_mask(dev,
		   AD7779_REG_CH_DISABLE,
		   AD7779_DCLK_CLK_DIV(0x3),
		   AD7779_DCLK_CLK_DIV(div));
                 }
	dev->dclk_div = div;

	return ret;
}

/**
 * Get the DCLK divider.
 * @param dev - The device structure.
 * @param div - The DCLK divider.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad7779_get_dclk_div(ad7779_dev *dev,
							ad7768_dclk_div *div)
{
	*div = dev->dclk_div;

	return HAL_OK;
}

/**
 * Set the synchronization offset of the selected channel.
 * @param dev - The device structure.
 * @param ch - The channel number.
 * 			   Accepted values: AD7779_CH0
 * 			   					AD7779_CH1
 * 			   					AD7779_CH2
 * 			   					AD7779_CH3
 * 			   					AD7779_CH4
 * 			   					AD7779_CH5
 * 			   					AD7779_CH6
 * 			   					AD7779_CH7
 * @param sync_offset - The synchronization offset value.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad7779_set_sync_offset(ad7779_dev *dev,
							   ad7779_ch ch,
							   uint8_t sync_offset)
{
	int32_t ret;

	if (dev->ctrl_mode == AD7779_PIN_CTRL) {
		printf("%s: This feature is not available in PIN control mode.\n",
			   __func__);
		return HAL_ERROR;
	}

	ret = ad7779_spi_int_reg_write(dev,
								   AD7779_REG_CH_SYNC_OFFSET(ch),
								   sync_offset);
	dev->sync_offset[ch] = sync_offset;

	return ret;
}

/**
 * Get the synchronization offset of the selected channel.
 * @param dev - The device structure.
 * @param ch - The channel number.
 * 			   Accepted values: AD7779_CH0
 * 			   					AD7779_CH1
 * 			   					AD7779_CH2
 * 			   					AD7779_CH3
 * 			   					AD7779_CH4
 * 			   					AD7779_CH5
 * 			   					AD7779_CH6
 * 			   					AD7779_CH7
 * @param sync_offset - The synchronization offset value.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad7779_get_sync_offset(ad7779_dev *dev,
							   ad7779_ch ch,
							   uint8_t *sync_offset)
{
	if (dev->ctrl_mode == AD7779_PIN_CTRL) {
		printf("%s: This feature is not available in PIN control mode.\n",
			   __func__);
		return HAL_ERROR;
	}

	*sync_offset = dev->sync_offset[ch];

	return HAL_OK;
}

/**
 * Set the offset correction of the selected channel.
 * @param dev - The device structure.
 * @param ch - The channel number.
 * 			   Accepted values: AD7779_CH0
 * 			   					AD7779_CH1
 * 			   					AD7779_CH2
 * 			   					AD7779_CH3
 * 			   					AD7779_CH4
 * 			   					AD7779_CH5
 * 			   					AD7779_CH6
 * 			   					AD7779_CH7
 * @param offset - The offset value.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad7779_set_offset_corr(ad7779_dev *dev,
							   ad7779_ch ch,
							   uint32_t offset)
{
	int32_t ret;
	uint8_t upper_byte;
	uint8_t mid_byte;
	uint8_t lower_byte;

	if (dev->ctrl_mode == AD7779_PIN_CTRL) {
		printf("%s: This feature is not available in PIN control mode.\n",
			   __func__);
		return HAL_ERROR;
	}

	upper_byte = (offset & 0xFF0000) >> 16;
	mid_byte = (offset & 0x00FF00) >> 8;
	lower_byte = (offset & 0x0000FF) >> 0;
	ret = ad7779_spi_int_reg_write(dev,
								   AD7779_REG_CH_OFFSET_UPPER_BYTE(ch),
								   upper_byte);
	ret |= ad7779_spi_int_reg_write(dev,
									AD7779_REG_CH_OFFSET_MID_BYTE(ch),
									mid_byte);
	ret |= ad7779_spi_int_reg_write(dev,
									AD7779_REG_CH_OFFSET_LOWER_BYTE(ch),
									lower_byte);
	dev->offset_corr[ch] = offset;

	return ret;
}

/**
 * Get the offset correction of the selected channel.
 * @param dev - The device structure.
 * @param ch - The channel number.
 * 			   Accepted values: AD7779_CH0
 * 			   					AD7779_CH1
 * 			   					AD7779_CH2
 * 			   					AD7779_CH3
 * 			   					AD7779_CH4
 * 			   					AD7779_CH5
 * 			   					AD7779_CH6
 * 			   					AD7779_CH7
 * @param offset - The offset value.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad7779_get_offset_corr(ad7779_dev *dev,
							   ad7779_ch ch,
							   uint32_t *offset)
{
	if (dev->ctrl_mode == AD7779_PIN_CTRL) {
		printf("%s: This feature is not available in PIN control mode.\n",
			   __func__);
		return HAL_ERROR;
	}

	*offset = dev->offset_corr[ch];

	return HAL_OK;
}

/**
 * Set the gain correction of the selected channel.
 * @param dev - The device structure.
 * @param ch - The channel number.
 * 			   Accepted values: AD7779_CH0
 * 			   					AD7779_CH1
 * 			   					AD7779_CH2
 * 			   					AD7779_CH3
 * 			   					AD7779_CH4
 * 			   					AD7779_CH5
 * 			   					AD7779_CH6
 * 			   					AD7779_CH7
 * @param gain - The gain value.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad7779_set_gain_corr(ad7779_dev *dev,
							 ad7779_ch ch,
							 uint32_t gain)
{
	int32_t ret;
	uint8_t upper_byte;
	uint8_t mid_byte;
	uint8_t lower_byte;

	if (dev->ctrl_mode == AD7779_PIN_CTRL) {
		printf("%s: This feature is not available in PIN control mode.\n",
			   __func__);
		return HAL_ERROR;
	}

	gain &= 0xFFFFFF;
	upper_byte = (gain & 0xff0000) >> 16;
	mid_byte = (gain & 0x00ff00) >> 8;
	lower_byte = (gain & 0x0000ff) >> 0;
	ret = ad7779_spi_int_reg_write(dev,
								   AD7779_REG_CH_GAIN_UPPER_BYTE(ch),
								   upper_byte);
	ret |= ad7779_spi_int_reg_write(dev,
									AD7779_REG_CH_GAIN_MID_BYTE(ch),
									mid_byte);
	ret |= ad7779_spi_int_reg_write(dev,
									AD7779_REG_CH_GAIN_LOWER_BYTE(ch),
									lower_byte);
	dev->gain_corr[ch] = gain;

	return ret;
}

/**
 * Get the gain correction of the selected channel.
 * @param dev - The device structure.
 * @param ch - The channel number.
 * 			   Accepted values: AD7779_CH0
 * 			   					AD7779_CH1
 * 			   					AD7779_CH2
 * 			   					AD7779_CH3
 * 			   					AD7779_CH4
 * 			   					AD7779_CH5
 * 			   					AD7779_CH6
 * 			   					AD7779_CH7
 * @param gain - The gain value.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad7779_get_gain_corr(ad7779_dev *dev,
							 ad7779_ch ch,
							 uint32_t *gain)
{
	if (dev->ctrl_mode == AD7779_PIN_CTRL) {
		printf("%s: This feature is not available in PIN control mode.\n",
			   __func__);
		return HAL_ERROR;
	}

	*gain = dev->gain_corr[ch];

	return HAL_OK;
}

/**
 * Set the reference buffer operation mode of the selected pin.
 * @param dev - The device structure.
 * @param refx_pin - The selected pin.
 * 					 Accepted values: AD7779_REFX_P
 * 									  AD7779_REFX_N
 * @param mode - The reference buffer operation mode.
 * 				 Accepted values: AD7779_REF_BUF_ENABLED
 * 								  AD7779_REF_BUF_PRECHARGED
 * 								  AD7779_REF_BUF_DISABLED
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad7779_set_ref_buf_op_mode(ad7779_dev *dev,
								   ad7779_refx_pin refx_pin,
								   ad7779_ref_buf_op_mode mode)
{
	int32_t ret;
	uint8_t config_1;
	uint8_t config_2;

	if (dev->ctrl_mode == AD7779_PIN_CTRL) {
		printf("%s: This feature is not available in PIN control mode.\n",
			   __func__);
		return HAL_ERROR;
	}

	if (refx_pin == AD7779_REFX_P) {
		switch (mode) {
		case AD7779_REF_BUF_ENABLED:
			config_1 = AD7779_REF_BUF_POS_EN;
			config_2 = 0;
			break;
		case AD7779_REF_BUF_PRECHARGED:
			config_1 = AD7779_REF_BUF_POS_EN;
			config_2 = AD7779_REFBUFP_PREQ;
			break;
		default:
			config_1 = 0;
			config_2 = 0;
		}
		ret = ad7779_spi_int_reg_write_mask(dev,
											AD7779_REG_BUFFER_CONFIG_1,
											AD7779_REF_BUF_POS_EN,
											config_1);
		ret |= ad7779_spi_int_reg_write_mask(dev,
											 AD7779_REG_BUFFER_CONFIG_2,
											 AD7779_REFBUFP_PREQ,
											 config_2);
	} else {
		switch (mode) {
		case AD7779_REF_BUF_ENABLED:
			config_1 = AD7779_REF_BUF_NEG_EN;
			config_2 = 0;
			break;
		case AD7779_REF_BUF_PRECHARGED:
			config_1 = AD7779_REF_BUF_NEG_EN;
			config_2 = AD7779_REFBUFN_PREQ;
			break;
		default:
			config_1 = 0;
			config_2 = 0;
		}
		ret = ad7779_spi_int_reg_write_mask(dev,
											AD7779_REG_BUFFER_CONFIG_1,
											AD7779_REF_BUF_NEG_EN,
											config_1);
		ret |= ad7779_spi_int_reg_write_mask(dev,
											 AD7779_REG_BUFFER_CONFIG_2,
											 AD7779_REFBUFN_PREQ,
											 config_2);
	}
	dev->ref_buf_op_mode[refx_pin] = mode;

	return ret;
}

/**
 * Get the reference buffer operation mode of the selected pin.
 * @param dev - The device structure.
 * @param refx_pin - The selected pin.
 * 					 Accepted values: AD7779_REFX_P
 * 									  AD7779_REFX_N
 * @param mode - The reference buffer operation mode.
 * 				 Accepted values: AD7779_REF_BUF_ENABLED
 * 								  AD7779_REF_BUF_PRECHARGED
 * 								  AD7779_REF_BUF_DISABLED
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad7779_get_ref_buf_op_mode(ad7779_dev *dev,
								   ad7779_refx_pin refx_pin,
								   ad7779_ref_buf_op_mode *mode)
{
	if (dev->ctrl_mode == AD7779_PIN_CTRL) {
		printf("%s: This feature is not available in PIN control mode.\n",
			   __func__);
		return HAL_ERROR;
	}

	*mode = dev->ref_buf_op_mode[refx_pin];

	return HAL_OK;
}

/**
 * Set the SAR ADC configuration.
 * @param dev - The device structure.
 * @param state - The SAR ADC state.
 * 				  Accepted values: AD7779_ENABLE
 * 								   AD7779_DISABLE
 * @param mux - The SAR mux input configuration.
 * 				Accepted values: AD7779_AUXAINP_AUXAINN
 *								 AD7779_DVBE_AVSSX
 *								 AD7779_REF1P_REF1N
 *								 AD7779_REF2P_REF2N
 *								 AD7779_REF_OUT_AVSSX
 *								 AD7779_VCM_AVSSX
 *								 AD7779_AREG1CAP_AVSSX_ATT
 *								 AD7779_AREG2CAP_AVSSX_ATT
 *								 AD7779_DREGCAP_DGND_ATT
 *								 AD7779_AVDD1A_AVSSX_ATT
 *								 AD7779_AVDD1B_AVSSX_ATT
 *								 AD7779_AVDD2A_AVSSX_ATT
 *								 AD7779_AVDD2B_AVSSX_ATT
 *								 AD7779_IOVDD_DGND_ATT
 *								 AD7779_AVDD4_AVSSX
 *								 AD7779_DGND_AVSS1A_ATT
 *								 AD7779_DGND_AVSS1B_ATT
 *								 AD7779_DGND_AVSSX_ATT
 *								 AD7779_AVDD4_AVSSX_ATT
 *								 AD7779_REF1P_AVSSX
 *								 AD7779_REF2P_AVSSX
 *								 AD7779_AVSSX_AVDD4_ATT
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad7779_set_sar_cfg(ad7779_dev *dev,
						   ad7779_state state,
						   ad7779_sar_mux mux)
{
	int32_t ret;

	ret = ad7779_spi_int_reg_write_mask(dev,
										AD7779_REG_GENERAL_USER_CONFIG_1,
										AD7779_PDB_SAR,
										(state == AD7779_ENABLE) ?
												AD7779_PDB_SAR : 0);
	ret |= ad7779_spi_int_reg_write(dev,
									AD7779_REG_GLOBAL_MUX_CONFIG,
									AD7779_GLOBAL_MUX_CTRL(mux));
	dev->sar_state = state;
	dev->sar_mux = mux;

	return ret;
}

/**
 * Get the SAR ADC configuration.
 * @param dev - The device structure.
 * @param state - The SAR ADC state.
 * @param mux - The SAR mux input configuration.
 * @return SUCCESS in case of success, negative error code otherwise.
 */
int32_t ad7779_get_sar_cfg(ad7779_dev *dev,
						   ad7779_state *state,
						   ad7779_sar_mux *mux)
{
	*state = dev->sar_state;
	*mux = dev->sar_mux;

	return HAL_OK;
}


int32_t ad7779_do_spi_soft_reset(ad7779_dev *dev)
{
	uint8_t buf[8] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
	int32_t ret;

	/* Keeping the SDI pin high during 64 consecutives clocks generates a
	   software reset */
        ret = HAL_SPI_Transmit(&dev->spi_dev, buf, 8, SPI_TIMEOUT_1);
        

	return ret;
}

 void adc_read_sd(ad7779_dev *dev)
 {
   memset( readout_buffer , 0x00 , 8 * 4  );
   HAL_SPI_TransmitReceive(&dev->spi_dev, &readout_buffer[ 0 ], &readout_buffer[ 0 ] , 8 * 4 , SPI_TIMEOUT_1);
 }


/** ----------------------------------------------------
 * Масштабирование выходных значений
 * @param sdout - выходной вектор
 */

int32_t i24toi32( uint32_t x )
{
	int32_t res = ( int32_t )x;
	res &= 0x00FFFFFF;
	if( res & 0x00800000 )
	{
		res ^= 0x00FFFFFF;
		res += 1; res = -res;
	}
	return res;
}

void scale_redout(ad7779_dev *dev, float* sdout )
{
	const double u_quant = 2.5 / 8388607; //2.5 / 16777216.0;
	const uint8_t gain_table[] = { 1 , 2 , 4 , 8 };
	const uint8_t channel_ords[] = { 3 , 2 , 1 , 0 , 4 , 5 , 6, 7 };

	if( sdout == NULL ) return;
	for( int i = 0 ; i < ELECTRODESNUMBER ; i++ )
	{
		int idx = channel_ords[ i ];
		int32_t x = i24toi32(( readout_buffer[ idx * 4 ] << 24 ) +
                                     ( readout_buffer[ idx * 4 + 1  ] << 16 ) +
				    ( readout_buffer[ idx * 4 + 2 ] << 8 ) +
                                    ( readout_buffer[ idx * 4 + 3 ] ) );
		sdout[ i ] = x * u_quant / gain_table[dev->gain[i]];

	}
}


/**
 * Initialize the device.
 * @param device - The device structure.
 * @param init_param - The structure that contains the device initial
 * 					   parameters.
 * @return SUCCESS in case of success, negative error code otherwise.
 */

/* Функция загрузки настроек в АЦП */

int32_t ad7779_setup(ad7779_dev *dev)
 {

  uint8_t i;
  int32_t ret = 0;
  
  for (i = AD7779_REG_CH_CONFIG(0); i <= AD7779_REG_SRC_UPDATE; i++)
  {
    ret |= ad7779_spi_int_reg_read(dev, i, &dev->cached_reg_val[i]);
  }
  
  /* Set the gain of the selected channel. */
 
  if (dev->ctrl_mode == AD7779_SPI_CTRL)
   {
     for (i = AD7779_CH0; i <= AD7779_CH7; i++)
      {
        ret |= ad7779_set_gain(dev, (ad7779_ch)i, dev->gain[i]);
      }
   }
  
  ret |= ad7779_spi_int_reg_write(dev, AD7779_REG_GENERAL_USER_CONFIG_1, 0x74);
  ret |= ad7779_spi_int_reg_write(dev, AD7779_REG_GENERAL_USER_CONFIG_2, 0x09);
  ret |= ad7779_spi_int_reg_write(dev, AD7779_REG_GENERAL_USER_CONFIG_3, 0x00);
  
  ret |= ad7779_spi_int_reg_write(dev, AD7779_REG_BUFFER_CONFIG_1, 0x38);
  ret |= ad7779_spi_int_reg_write(dev, AD7779_REG_BUFFER_CONFIG_2, 0xC0);
   
  ret |= ad7779_spi_int_reg_write(dev, AD7779_REG_DOUT_FORMAT, 0x00);
  
  ret |= ad7779_spi_int_reg_write(dev, AD7779_REG_ADC_MUX_CONFIG, 0x40);
    
   /* Set the state (enable, disable) of the channel. */
  ret |= ad7779_spi_int_reg_write(dev, AD7779_REG_CH_DISABLE, 0x00);
  
  /* Set the sync_offset, offset_corr, gain_corr */
  for (i = AD7779_CH0; i <= AD7779_CH7; i++)
   {
     dev->sync_offset[i] = 0;
     dev->offset_corr[i] = 0;
     //dev->gain_corr[i] = 0;
     if (dev->ctrl_mode == AD7779_SPI_CTRL)
      {
        ret |= ad7779_set_sync_offset(dev, (ad7779_ch)i, dev->sync_offset[i]);
	ret |= ad7779_set_offset_corr(dev, (ad7779_ch)i, dev->offset_corr[i]);
	//ret |= ad7779_set_gain_corr(dev, (ad7779_ch)i, dev->gain_corr[i]);
      }
   }
  
  ret |= ad7779_spi_int_reg_write(dev, AD7779_REG_CHX_ERR_REG_EN, 0xFE);
  ret |= ad7779_spi_int_reg_write(dev, AD7779_REG_GEN_ERR_REG_1_EN, 0x0E);
  ret |= ad7779_spi_int_reg_write(dev, AD7779_REG_GEN_ERR_REG_2_EN, 0x3C);
  
/* Set the decimation rate. */
  if (dev->ctrl_mode == AD7779_SPI_CTRL)
   {
     ret |= ad7779_set_dec_rate(dev, dev->dec_rate_int, dev->dec_rate_dec);
     HAL_Delay(1);
     ret |= ad7779_spi_int_reg_write(dev, AD7779_REG_SRC_UPDATE, 0x00U);
     HAL_Delay(2);
     ret |= ad7779_spi_int_reg_write(dev, AD7779_REG_SRC_UPDATE, AD7779_SCR_UPDATE);
     HAL_Delay(2);
     ret |= ad7779_spi_int_reg_write(dev, AD7779_REG_SRC_UPDATE, 0x00U);
   }
  
  for (i = AD7779_REG_CH_CONFIG(0); i <= AD7779_REG_SRC_UPDATE; i++)
  {
    ret |= ad7779_spi_int_reg_read(dev, i, &dev->cached_reg_val[i]);
  }
  /*
  if (!ret)
		ret = ret;//printf("AD7779 successfully initialized\n");
	else    
		printf("AD7779 initialization error (%d)\n", ret);
*/
	return ret;
  
 }
       
       
  // ************************************************************************************************************************************************* //
       
       