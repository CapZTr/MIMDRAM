#define _POSIX_C_SOURCE 200112L

#include <stdlib.h>

#define ROW_SIZE   8192
#define ALIGNMENT  (ROW_SIZE * 16 * 2)

extern void rowop_ap(void *dst);
extern void rowop_aap(void *dst, void *src);

void *pud_get_row(void) {
    void *ptr = NULL;
    if (posix_memalign(&ptr, ALIGNMENT, ALIGNMENT) != 0) {
        exit(-1);
    }
    return ptr;
}

void pud_ap(void *dst)             { rowop_ap(dst); }
void pud_aap(void *dst, void *src) { rowop_aap(dst, src); }
