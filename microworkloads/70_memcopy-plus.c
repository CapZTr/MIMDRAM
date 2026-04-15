#include <stdio.h>
#include <stdlib.h>
#include <malloc.h>
#include <immintrin.h>
#include <m5op.h>
#include "mimdram.h"

int main(int argc, char **argv) {
    init_ambit();
    srand(121324314);

    int col_length = atoi(argv[1]);
    int num_vals = 1024 << (atoi(argv[2]));

    int total_bits = col_length * num_vals;
    int total_bytes = total_bits / 8;
    int total_ints = total_bits / 32;
    int total_vecs = total_bits / 128;

    int per_col_bits = num_vals;
    int per_col_bytes = num_vals / 8;
    int per_col_ints = num_vals / 32;
    int per_col_vecs = num_vals / 128;
    int per_col_rows = (per_col_bytes + ROW_SIZE - 1) / ROW_SIZE;

    // Allocate src and dst in different banks.
    //
    // In RoRaBaChCo:  bank = (addr / ROW_SIZE) % (BANK_COUNT * RANK_COUNT)
    // posix_memalign(..., ALIGNMENT, ...) always yields addr % ALIGNMENT == 0,
    // i.e. (addr / ROW_SIZE) % ROWS_PER_VECTOR == 0  →  bank 0, rank 0.
    //
    // To put dst in bank DST_BANK instead, allocate an ALIGNMENT-aligned base and
    // add DST_BANK * ROW_SIZE.  The extra ROW_SIZE padding in the allocation size
    // ensures we never read/write past the end of the buffer.
    // DST_BANK must be in [1, ROWS_PER_VECTOR-1] so src and dst are in different banks.
#define DST_BANK 1
    unsigned **src  = malloc(col_length * sizeof(unsigned *));
    unsigned **dst  = malloc(col_length * sizeof(unsigned *));
    void     **src_bufs = malloc(col_length * sizeof(void *));
    void     **dst_bufs = malloc(col_length * sizeof(void *));

    for (int j = 0; j < col_length; j++) {
        if (posix_memalign(&src_bufs[j], ALIGNMENT, per_col_bytes) ||
            posix_memalign(&dst_bufs[j], ALIGNMENT, per_col_bytes + ROW_SIZE)) {
            printf("Failed to allocate column buffer.\n");
            exit(-1);
        }
        src[j] = (unsigned *)src_bufs[j];                                   /* bank 0       */
        dst[j] = (unsigned *)((char *)dst_bufs[j] + DST_BANK * ROW_SIZE);   /* bank DST_BANK */
        for (int i = 0; i < per_col_bytes / 4; i++)
            src[j][i] = rand();
    }

    // run some iterations of the algorithm
    for (int iter = 0; iter < 2; iter++) {
        m5_reset_stats(0, 0);

        FOR_ALL_VECTORS {
            for (int j = 0; j < col_length; j++) {
                unsigned *s = VECTOR(src[j]);
                unsigned *d = VECTOR(dst[j]);

                COPY_VECTORS(d, s);
            }
        }
    }
    m5_dump_stats(0, 0);

    return 0;
}
