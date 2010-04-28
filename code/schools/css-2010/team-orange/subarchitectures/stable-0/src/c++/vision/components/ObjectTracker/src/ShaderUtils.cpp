#include "ShaderUtils.h"
#include <stdio.h>

void *malloc_align(size_t size, int align)
{
    char *tmp = (char*)malloc(size+align-1+sizeof(void*));
    char *addr = tmp+align-1+sizeof(void*);
    addr -= (long)addr & (align-1);
    ((void**)addr)[-1] = tmp;
    return addr;
}

void free_align(void *addr)
{
    void *tmp = ((void**)addr)[-1];
    free(tmp);
}

char *read_text_file(const char* file)
{
    if(!file) return NULL;

    FILE *f = fopen(file ,"r");
    if(!f) return NULL;

    fseek(f, 0, SEEK_END);
    int sz = ftell(f);
    fseek(f, 0, SEEK_SET);

    char *retv = (char*)malloc(sz+1);
    fread(retv, sz, 1, f);
    retv[sz] = 0;
    fclose(f);

    return retv;
}


