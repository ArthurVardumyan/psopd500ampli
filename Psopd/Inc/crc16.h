/**
 * @file crc16.h
 * @brief МОДУЛЬ ВЫЧИСЛЕНИЯ СКС16
 */
   
#ifndef CRC16_H_
#define CRC16_H_

/// Начальное значние CRC
#define CRC16_INIT  	( ( uint16_t )0xFFFF )

#ifdef __cplusplus
extern "C" {
#endif
/**
 * Вычисляет CRC по полиному 0xA001
 * @param crc - предыдущее значнеи CRC
 * @param a - входной байт
 * @return обновленное значение CRC
 */
    uint16_t crc16_update_poly( uint16_t crc , uint8_t a );

/**
 * Вычисляет CRC блока
 * @param crc - предыдущее значнеи CRC
 * @param a - входной байт
 * @param len длина блока
 * @return обновленное значение CRC
 */
    uint16_t crc16_update_blk( uint16_t crc , uint8_t* a , uint32_t len );
#ifdef __cplusplus
}
#endif
#endif /* CRC16_H_ */


uint16_t buildCRC(readout_packet_t *pack);