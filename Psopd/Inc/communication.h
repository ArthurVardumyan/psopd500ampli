

 //--------------------------------------------------------------------
/** -----------------------------------------------------
 * Структура полезной нагрузки пакета
 */
#pragma pack(push, 1)
typedef struct PAYLOAD_S
{
	uint8_t		cmd;
        uint32_t	counter;
        float 		readout[ 8 ];
	
}  payload_t;
/** -----------------------------------------------------
 * Структура всего пакета
 */
typedef struct READOUTPACKET_S
{
  
  uint8_t 	prefix[ 2 ];
  uint16_t	length;
  payload_t	payload;
  uint16_t	crc;
  
} readout_packet_t;

#pragma pack(pop)

//READOUTPACKET_S():length(sizeof( payload_t )),crc(0){ prefix[ 0 ] = 'B'; prefix[ 1 ] = 'L'; }