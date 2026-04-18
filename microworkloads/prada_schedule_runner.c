#define _POSIX_C_SOURCE 200112L
 
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <stdint.h>
#include <m5op.h>
#include "mimdram.h"
 
/* -----------------------------------------------------------------------
 * Architecture constants
 * --------------------------------------------------------------------- */
#define BANKS 32
 
#define BANK_ROW(ptr, bank)  ((void *)((char *)(ptr) + (bank) * ROW_SIZE))
 
#define AAP_ONE_BANK(dst, src, bank) \
    rowop_aap(BANK_ROW((dst), (bank)), BANK_ROW((src), (bank)))
#define ANAP_ONE_BANK(dst, src, bank) \
    rowop_anap(BANK_ROW((dst), (bank)), BANK_ROW((src), (bank)))
#define AAAP_ONE_BANK(dst, src1, src2, bank) \
    rowop_aaap(BANK_ROW((dst), (bank)), BANK_ROW((src1), (bank)), BANK_ROW((src2), (bank)));
#define AAAAAP_ONE_BANK(dst, src1, src2, src3, src4, bank) \
    rowop_aaaaap(BANK_ROW((dst), (bank)), BANK_ROW((src1), (bank)), BANK_ROW((src2), (bank)), BANK_ROW((src3), (bank)), BANK_ROW((src4), (bank)));
#define COPY_BANK_TO_BANK(dst, dst_bank, src, src_bank) \
    rowop_copy(BANK_ROW((dst), (dst_bank)), BANK_ROW((src), (src_bank)))
 
/* -----------------------------------------------------------------------
 * Verbosity level (set via -v / -vv command-line flag):
 *   0 = silent  (default, suitable for gem5 simulation)
 *   1 = ops      show each time-step and operation group
 *   2 = insns    additionally show every bit-serial instruction
 * --------------------------------------------------------------------- */
static int g_verbose = 0;
 
static unsigned *alloc_vec_all_banks(void) {
    unsigned *p = NULL;
    if (posix_memalign((void **)&p, ALIGNMENT, ALIGNMENT)) {
        fprintf(stderr, "alloc_vec_all_banks: out of memory\n");
        exit(1);
    }
    return p;
}
 
/* -----------------------------------------------------------------------
 * Bit-serial primitives used by execute_mul
 * --------------------------------------------------------------------- */
static inline void execute_row_add(
    unsigned *lhs_bit, unsigned *rhs_bit, unsigned *out_bit,
    unsigned *cin_bit, unsigned *cout_bit, unsigned **tmp,
    const int *bank_ids, int bank_count)
{
    for (int i = 0; i < bank_count; i++) AAAP_ONE_BANK(tmp[1], lhs_bit, tmp[0], bank_ids[i]);
    for (int i = 0; i < bank_count; i++) AAAP_ONE_BANK(tmp[3], rhs_bit, tmp[2], bank_ids[i]);
    for (int i = 0; i < bank_count; i++) AAAP_ONE_BANK(out_bit, cin_bit, cout_bit, bank_ids[i]);
    for (int i = 0; i < bank_count; i++) AAAP_ONE_BANK(cout_bit, tmp[0], tmp[2], bank_ids[i]);
    for (int i = 0; i < bank_count; i++) ANAP_ONE_BANK(tmp[4], tmp[2], bank_ids[i]);
    for (int i = 0; i < bank_count; i++) AAAAAP_ONE_BANK(out_bit, tmp[2], tmp[4], tmp[1], tmp[3], bank_ids[i]);
}
 
static inline void execute_row_and(
    unsigned *lhs_bit, unsigned *rhs_bit, unsigned *out_bit, unsigned **tmp,
    const int *bank_ids, int bank_count)
{
    for (int i = 0; i < bank_count; i++) AAP_ONE_BANK(tmp[0],    lhs_bit,    bank_ids[i]);
    for (int i = 0; i < bank_count; i++) AAP_ONE_BANK(tmp[1],    rhs_bit,    bank_ids[i]);
    for (int i = 0; i < bank_count; i++) AAP_ONE_BANK(out_bit,    C_0,        bank_ids[i]);
    for (int i = 0; i < bank_count; i++) AAAP_ONE_BANK(out_bit, tmp[0], tmp[1], bank_ids[i]);
}
 
/* -----------------------------------------------------------------------
 * Core execute_add
 * --------------------------------------------------------------------- */
void execute_add(
    int lhs_bw, int rhs_bw,
    const int *bank_ids, int bank_count,
    unsigned **lhs, unsigned **rhs, unsigned **out, unsigned **tmp)
{
    int bw = (lhs_bw < rhs_bw) ? lhs_bw : rhs_bw;
 
    /* initialise carry to 0 */
    // for (int i = 0; i < bank_count; i++) AAAP_ONE_BANK(out[0], C_0, , bank_ids[i]);

    unsigned *carry = C_0;
    for (int j = 0; j < bw; j++) {
        if (g_verbose >= 2)
            printf("      [bit=%d] full-adder: out[%d] = lhs[%d] + rhs[%d] + carry  ->  carry\n",
                   j, j, j, j);
        execute_row_add(lhs[j], rhs[j], out[j], carry, carry, tmp, bank_ids, bank_count);
    }
}
