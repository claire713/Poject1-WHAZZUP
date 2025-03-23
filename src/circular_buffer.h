#include <stdint.h>
#define CIRC_BUF_SIZE 128   //circular buffar can hold 128 bytes/characters
typedef struct {
	char data[CIRC_BUF_SIZE];    //storage array for buffer
	uint32_t head;				//track where next character will be written
	uint32_t tail;			   //track where next character will be read from
	uint32_t count;           //amount of bytes stored
} circular_buffer;


void init_circ_buf(circular_buffer *buf);
int put_circ_buf(circular_buffer *buf,char c);
int get_circ_buf(circular_buffer *buf,char *c);
