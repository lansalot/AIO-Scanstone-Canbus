
//All this is from torriem - AgOpen fourm

#include "canframe.h"

void j1939_decode(long ID, unsigned long* PGN, byte* priority, byte* src_addr, byte* dest_addr)
{
	/* decode j1939 fields from 29-bit CAN id */
	*src_addr = 255;
	*dest_addr = 255;

	*priority = (int)((ID & 0x1C000000) >> 26);	//Bits 27,28,29

	*PGN = ID & 0x01FFFF00;	//Tony Note: Changed this from 0x00FFFF00 to decode PGN 129029, it now gets the 17th bit of the 18 bit PGN (Bits 9-25, Bit 26 is not used)
	*PGN = *PGN >> 8;

	ID = ID & 0x000000FF;	//Bits 1-8
	*src_addr = (int)ID;

	/* decode dest_addr if a peer to peer message */
	if ((*PGN > 0 && *PGN <= 0xEFFF) || (*PGN > 0x10000 && *PGN <= 0x1EFFF)) 
	{
		*dest_addr = (int)(*PGN & 0xFF);
		*PGN = *PGN & 0x01FF00;
	}
}

long j1939_encode(unsigned long pgn, byte priority, byte src_addr, byte dest_addr)
{

	long id;
	id = (long)(priority & 0x07) << 26; //three bits only	- Tony Note: Added long cast on priority others Arduino drops bits when shifting bits 26 left 
	/* if a peer to peer message, encode dest_addr */
	if ((pgn > 0 && pgn <= 0xEFFF) || (pgn > 0x10000 && pgn <= 0x1EFFF))
	{
		pgn = pgn & 0x01FF00;
		pgn = pgn | dest_addr;
	}
	id = id | (pgn << 8);
	id = id | src_addr;

	return id;
}
