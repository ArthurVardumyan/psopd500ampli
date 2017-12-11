
#include "stdint.h"
#include "communication.h"
#include "crc16.h"


/**
 * Вычисляет CRC по полиному 0xA001
 * @param crc - предыдущее значнеи CRC
 * @param a - входной байт
 * @return обновленное значение CRC
 */
uint16_t crc16_update_poly( uint16_t crc , uint8_t a )
{
    crc ^= a;
    uint_fast8_t i;
    for ( i = 0 ; i < 8 ; ++i )
    {
        if ( crc & 1 )
            crc = ( crc >> 1 ) ^ 0xA001;
        else
            crc = ( crc >> 1 );
    }
    return crc;
}

/**
 * Вычисляет CRC блока
 */
uint16_t crc16_update_blk( uint16_t crc , uint8_t* a , uint32_t len )
{
    uint8_t* ptr = a;
    while( len-- )
        crc = crc16_update_poly( crc , *ptr++ );
    return crc;
}


uint16_t buildCRC(readout_packet_t *pack)
	{
		pack->crc = crc16_update_blk( 0xFFFF , ( uint8_t* )&pack->length , 2 );
		pack->crc = crc16_update_blk( pack->crc, ( uint8_t* )&pack->payload.cmd , sizeof( pack->payload ) );
                return pack->crc;
	}