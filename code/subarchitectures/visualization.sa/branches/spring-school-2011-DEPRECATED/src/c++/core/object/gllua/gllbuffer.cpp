/*
** Auxiliary functions to support OpenGL-Lua binding.
** It implements functions to manipulate a C buffer.
** Waldemar Celes
** TeCGraf/PUC-Rio
** Ago 1998
*/

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <windows.h>
#endif
#include <GL/gl.h>
#include <stdlib.h>
extern "C" {
#include "lua.h"
}
#include "gllbuffer.h"

//extern lua_State *luaS; // link to doris - NT

/* additional buffer information */
typedef union
{
    struct
    {
        GLenum type;
        int size;
    } d;
    double dummy;  /* the union is to ensure alignment */
} GLLinfo;

/* macros to access info */
#define info(b) ((GLLinfo*)((char*)b-sizeof(GLLinfo)))
#define type(b) ((info(b))->d.type)
#define size(b) ((info(b))->d.size)


static int get_nbytes (GLenum type)
{
   switch (type)
   {
   case GL_UNSIGNED_BYTE:
   case GL_BYTE:
       return sizeof(char);
   case GL_UNSIGNED_SHORT:
   case GL_SHORT:
       return sizeof(short);
   case GL_UNSIGNED_INT:
   case GL_INT:
       return sizeof(int);
   case GL_FLOAT:
       return sizeof(float);
#ifdef GL_VERSION_1_1
   case GL_DOUBLE:
       return sizeof(double);
#endif
   default:
       //lua_pushliteral(luaS,"unknown 'type' in function 'gllCreateBuffer'.");
       //lua_error(luaS);
       return 0;
   }
}

GLLbuffer* gllCreateBuffer1 (GLsizei n, GLenum type, lua_Number* values)
{
   int nb = get_nbytes(type);
    
   GLLbuffer* b = (unsigned char*)malloc(sizeof(GLLinfo)+n*nb) + sizeof(GLLinfo);
   size(b) = n;
   type(b) = type;
   if (values)
   {
       int i;
       for (i=0; i<n; i++)
           gllSetBuffer(b,i,values[i]);
   }
   return b;
}

GLLbuffer* gllCreateBuffer2 (GLsizei n,GLenum type)
{
   return gllCreateBuffer1(n,type,NULL);
}

void gllDeleteBuffer (GLLbuffer* b)
{
    free(info(b));
}

void gllSetBuffer (GLLbuffer* b, int i, lua_Number v)
{
   if (size(b)<=i)
   {
       //lua_pushliteral(luaS,"buffer overflow in function 'gllSetBuffer'.");
       //lua_error(luaS);
       return;
   }
    
   switch (type(b))
   {
   case GL_UNSIGNED_BYTE:
       {
           unsigned char* a = (unsigned char*)b;
           a[i] = (unsigned char)v; 
       }
       break;
   case GL_BYTE:
       {
           char* a = (char*)b;
           a[i] = (char)v; 
       }
       break;
   case GL_UNSIGNED_SHORT:
       {
           unsigned short* a = (unsigned short*)b;
           a[i] = (unsigned short)v; 
       }
       break;
   case GL_SHORT:
       {
           short* a = (short*)b;
           a[i] = (short)v; 
       }
       break;
   case GL_UNSIGNED_INT:
       {
           unsigned int* a = (unsigned int*)b;
           a[i] = (unsigned int)v; 
       }
       break;
   case GL_INT:
       {
           int* a = (int*)b;
           a[i] = (int)v; 
       }
       break;
   case GL_FLOAT:
       {
           float* a = (float*)b;
           a[i] = (float)v; 
       }
       break;
#if GL_VERSION_1_1
   case GL_DOUBLE:
       {
           double* a = (double*)b;
           a[i] = (double)v; 
       }
       break;
#endif
   default:
       break;
   }
}

lua_Number gllGetBuffer (GLLbuffer* b, int i)
{
   if (size(b)<=i)
   {
       //lua_pushliteral(luaS,"buffer overflow in function 'gllGetBuffer'.");
       //lua_error(luaS);
       return 0.0;
   }
    
   switch (type(b))
   {
   case GL_UNSIGNED_BYTE:
       {
           unsigned char* a = (unsigned char*)b;
           return a[i]; 
       }
       break;
   case GL_BYTE:
       {
           char* a = (char*)b;
           return a[i]; 
       }
       break;
   case GL_UNSIGNED_SHORT:
       {
           unsigned short* a = (unsigned short*)b;
           return a[i]; 
       }
       break;
   case GL_SHORT:
       {
           short* a = (short*)b;
           return a[i]; 
       }
       break;
   case GL_UNSIGNED_INT:
       {
           unsigned int* a = (unsigned int*)b;
           return (lua_Number) a[i]; 
       }
       break;
   case GL_INT:
       {
           int* a = (int*)b;
           return (lua_Number) a[i]; 
       }
       break;
   case GL_FLOAT:
       {
           float* a = (float*)b;
           return (lua_Number) a[i]; 
       }
       break;
#if GL_VERSION_1_1
   case GL_DOUBLE:
       {
           double* a = (double*)b;
           return (lua_Number) a[i]; 
       }
       break;
#endif
   default:
       break;
   }
   return 0.0;
}

