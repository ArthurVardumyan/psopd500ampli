/**
 * @file crc16.h
 * @brief ������ ���������� ���16
 */
   
#ifndef CRC16_H_
#define CRC16_H_

/// ��������� ������� CRC
#define CRC16_INIT  	( ( uint16_t )0xFFFF )

#ifdef __cplusplus
extern "C" {
#endif
/**
 * ��������� CRC �� �������� 0xA001
 * @param crc - ���������� ������� CRC
 * @param a - ������� ����
 * @return ����������� �������� CRC
 */
    uint16_t crc16_update_poly( uint16_t crc , uint8_t a );

/**
 * ��������� CRC �����
 * @param crc - ���������� ������� CRC
 * @param a - ������� ����
 * @param len ����� �����
 * @return ����������� �������� CRC
 */
    uint16_t crc16_update_blk( uint16_t crc , uint8_t* a , uint32_t len );
#ifdef __cplusplus
}
#endif
#endif /* CRC16_H_ */


uint16_t buildCRC(readout_packet_t *pack);