

typedef unsigned short UINT16;
typedef unsigned int  UINT32;
typedef unsigned char UINT8;
typedef unsigned long UINT64;

typedef struct __attribute__((__packed__)) pkt {
	UINT16 w_msgSign;              // B space
	UINT16 w_msglen;               //message complete length (incl. msglen)
	UINT32 dw_id;                  //message ID
	UINT8  b_format;               //message frame format information
	UINT64 qw_timestamp;           //message timestamp
	UINT16 w_reserved;             //reserved data for future use
	UINT16 w_datalen;              //num of data bytes (length of ab_data)
	UINT8  ab_data[];              //data bytes
} IXXAT_BIN_PKT;
