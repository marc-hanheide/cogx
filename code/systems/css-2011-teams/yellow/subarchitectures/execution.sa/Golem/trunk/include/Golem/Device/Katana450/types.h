#ifndef _CANOPEN_TYPES_H_
#define _CANOPEN_TYPES_H_

#ifndef _URANUS_H_
#ifndef _ERIS_H_
	typedef short          int8;
	typedef int            int16;
	typedef long           int32;
	typedef unsigned char  Uint8;
	typedef unsigned short Uint16;
	typedef unsigned long  Uint32;
#endif
#endif

typedef unsigned char  CANopen_ui8;
typedef signed   char  CANopen_i8;

typedef unsigned short CANopen_ui16;
typedef signed   short CANopen_i16;

typedef unsigned int   CANopen_ui32;
typedef signed   int   CANopen_i32;

enum ECanOpenType {
	CANopen_uint8,
	CANopen_int8,
	CANopen_uint16,
	CANopen_int16,
	CANopen_uint32,
	CANopen_int32
};

#endif // _CANOPEN_TYPES_H_
