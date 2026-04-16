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
#define AP_ONE_BANK(dst, bank) \
    rowop_ap(BANK_ROW((dst), (bank)))
#define COPY_BANK_TO_BANK(dst, dst_bank, src, src_bank) \
    rowop_copy(BANK_ROW((dst), (dst_bank)), BANK_ROW((src), (src_bank)))
 
/* -----------------------------------------------------------------------
 * Verbosity level (set via -v / -vv command-line flag):
 *   0 = silent  (default, suitable for gem5 simulation)
 *   1 = ops      show each time-step and operation group
 *   2 = insns    additionally show every bit-serial instruction
 * --------------------------------------------------------------------- */
static int g_verbose = 0;
 
/* -----------------------------------------------------------------------
 * RowCopyTask
 * --------------------------------------------------------------------- */
typedef struct {
    unsigned **src;
    unsigned **dst;
    int src_bank;
    int dst_bank;
    int bitwidth;
} RowCopyTask;
 
/* -----------------------------------------------------------------------
 * Row allocator.
 *
 * All rows used in DRAM operations must be in the same subarray or the
 * gem5 assertion fires. In gem5 SE mode, consecutive
 * posix_memalign(ALIGNMENT, ALIGNMENT) calls produce physically-contiguous
 * ALIGNMENT-sized blocks (physical ≈ virtual for sequential allocations),
 * so they all land in the same subarray as long as the total count stays
 * below ROWS_PER_SUBARRAY (512).
 *
 * A single large mmap (SUBARRAY_SIZE = 128 MB) does NOT work: gem5 SE
 * cannot guarantee the physical start address is 128 MB-aligned, so the
 * block can span two physical subarray regions.
 *
 * Budget: 18 ambit (init_ambit) + 8*SCHED_MAX_BW + 4 data rows.
 * SCHED_MAX_BW=16 → 18 + 132 = 150 rows  ≪  512.
 * --------------------------------------------------------------------- */
static unsigned *alloc_vec_all_banks(void) {
    unsigned *p = NULL;
    if (posix_memalign((void **)&p, ALIGNMENT, ALIGNMENT)) {
        fprintf(stderr, "alloc_vec_all_banks: out of memory\n");
        exit(1);
    }
    return p;
}
 
/* -----------------------------------------------------------------------
 * Core execute_add
 * --------------------------------------------------------------------- */
void execute_add(
    int lhs_bw, int rhs_bw,
    const int *bank_ids, int bank_count,
    unsigned **lhs, unsigned **rhs, unsigned **out)
{
    int bw = (lhs_bw < rhs_bw) ? lhs_bw : rhs_bw;
 
    /* initialise carry to 0 */
    for (int i = 0; i < bank_count; i++) AAP_ONE_BANK(B_DCC1, C_0, bank_ids[i]);
 
    for (int j = 0; j < bw; j++) {
        if (g_verbose >= 2)
            printf("      [bit=%d] full-adder: out[%d] = lhs[%d] + rhs[%d] + carry  ->  carry\n",
                   j, j, j, j);
        for (int i = 0; i < bank_count; i++) AAP_ONE_BANK(B_T0_T1_T2, B_DCC1,     bank_ids[i]);
        for (int i = 0; i < bank_count; i++) AAP_ONE_BANK(B_T2_T3,    lhs[j],     bank_ids[i]);
        for (int i = 0; i < bank_count; i++) AAP_ONE_BANK(B_DCC1,     rhs[j],     bank_ids[i]);
        for (int i = 0; i < bank_count; i++) AP_ONE_BANK (B_DCC1_T0_T3,            bank_ids[i]);
        for (int i = 0; i < bank_count; i++) AAP_ONE_BANK(B_T0_T3,    B_DCC1N,    bank_ids[i]);
        for (int i = 0; i < bank_count; i++) AP_ONE_BANK (B_T0_T1_T2,              bank_ids[i]);
        for (int i = 0; i < bank_count; i++) AAP_ONE_BANK(B_T1,       rhs[j],     bank_ids[i]);
        for (int i = 0; i < bank_count; i++) AAP_ONE_BANK(out[j],     B_T1_T2_T3, bank_ids[i]);
    }
}
 
/* -----------------------------------------------------------------------
 * Bit-serial primitives used by execute_mul
 * --------------------------------------------------------------------- */
static inline void execute_row_add(
    unsigned *lhs_bit, unsigned *rhs_bit, unsigned *out_bit,
    unsigned *cin_bit, unsigned *cout_bit,
    const int *bank_ids, int bank_count)
{
    for (int i = 0; i < bank_count; i++) AAP_ONE_BANK(B_DCC1,     cin_bit,      bank_ids[i]);
    for (int i = 0; i < bank_count; i++) AAP_ONE_BANK(B_T0_T1_T2, B_DCC1,       bank_ids[i]);
    for (int i = 0; i < bank_count; i++) AAP_ONE_BANK(B_T2_T3,    lhs_bit,      bank_ids[i]);
    for (int i = 0; i < bank_count; i++) AAP_ONE_BANK(B_DCC1,     rhs_bit,      bank_ids[i]);
    for (int i = 0; i < bank_count; i++) AAP_ONE_BANK(cout_bit,   B_DCC1_T0_T3, bank_ids[i]);
    for (int i = 0; i < bank_count; i++) AAP_ONE_BANK(B_T0_T3,    B_DCC1N,      bank_ids[i]);
    for (int i = 0; i < bank_count; i++) AP_ONE_BANK (B_T0_T1_T2,                bank_ids[i]);
    for (int i = 0; i < bank_count; i++) AAP_ONE_BANK(B_T1,       rhs_bit,      bank_ids[i]);
    for (int i = 0; i < bank_count; i++) AAP_ONE_BANK(out_bit,    B_T1_T2_T3,   bank_ids[i]);
}
 
static inline void execute_row_and(
    unsigned *lhs_bit, unsigned *rhs_bit, unsigned *out_bit,
    const int *bank_ids, int bank_count)
{
    for (int i = 0; i < bank_count; i++) AAP_ONE_BANK(B_T0,    lhs_bit,    bank_ids[i]);
    for (int i = 0; i < bank_count; i++) AAP_ONE_BANK(B_T1,    rhs_bit,    bank_ids[i]);
    for (int i = 0; i < bank_count; i++) AAP_ONE_BANK(B_T2,    C_0,        bank_ids[i]);
    for (int i = 0; i < bank_count; i++) AAP_ONE_BANK(out_bit, B_T0_T1_T2, bank_ids[i]);
}
 
/* -----------------------------------------------------------------------
 * Core execute_mul
 * --------------------------------------------------------------------- */
void execute_mul(
    int lhs_bw, int rhs_bw,
    const int *bank_ids, int bank_count,
    unsigned **lhs, unsigned **rhs, unsigned **out,
    unsigned **partial, unsigned **tmp, unsigned **carry)
{
    /* --- Phase 1: initialise out[0] and partial[] using rhs[0] --------- */
    if (g_verbose >= 2)
        printf("      [init]    AND  out[0] = lhs[0] & rhs[0]\n");
    execute_row_and(lhs[0], rhs[0], out[0], bank_ids, bank_count);
 
    for (int i = 0; i < lhs_bw - 1; i++) {
        if (g_verbose >= 2)
            printf("      [init]    AND  partial[%d] = lhs[%d] & rhs[0]\n", i, i + 1);
        execute_row_and(lhs[i + 1], rhs[0], partial[i], bank_ids, bank_count);
    }
 
    for (int i = 0; i < bank_count; i++) AAP_ONE_BANK(carry[1], C_0, bank_ids[i]);
 
    /* --- Phase 2: accumulate rhs[1..rhs_bw-2] -------------------------- */
    for (int i = 0; i < rhs_bw - 1; i++) {
        if (g_verbose >= 2)
            printf("      [rhs=%d]   AND  tmp = lhs[0] & rhs[%d]\n", i, i);
        execute_row_and(lhs[0], rhs[i], tmp[0], bank_ids, bank_count);
 
        if (g_verbose >= 2)
            printf("      [rhs=%d]   ADD  out[%d] = tmp + partial[0] + 0  ->carry[0]\n", i, i);
        execute_row_add(tmp[0], partial[0], out[i], C_0, carry[0], bank_ids, bank_count);
 
        for (int j = 1; j < lhs_bw - 1; j++) {
            if (g_verbose >= 2)
                printf("      [rhs=%d]   AND  tmp = lhs[%d] & rhs[%d]\n", i, j, i);
            execute_row_and(lhs[j], rhs[i], tmp[0], bank_ids, bank_count);
 
            if (g_verbose >= 2)
                printf("      [rhs=%d]   ADD  partial[%d] = tmp + partial[%d] + carry[0]  ->carry[0]\n",
                       i, j - 1, j);
            execute_row_add(tmp[0], partial[j], partial[j - 1], carry[0], carry[0], bank_ids, bank_count);
        }
 
        if (g_verbose >= 2)
            printf("      [rhs=%d]   AND  tmp = lhs[%d] & rhs[%d]\n", i, lhs_bw - 1, i);
        execute_row_and(lhs[lhs_bw - 1], rhs[i], tmp[0], bank_ids, bank_count);
 
        if (g_verbose >= 2)
            printf("      [rhs=%d]   ADD  partial[%d] = tmp + carry[0] + carry[1]  ->carry[1]\n",
                   i, lhs_bw - 2);
        execute_row_add(tmp[0], carry[0], partial[lhs_bw - 2], carry[1], carry[1], bank_ids, bank_count);
    }
 
    /* --- Phase 3: final row rhs[rhs_bw-1] ------------------------------ */
    if (g_verbose >= 2)
        printf("      [final]   AND  tmp = lhs[0] & rhs[%d]\n", rhs_bw - 1);
    execute_row_and(lhs[0], rhs[rhs_bw - 1], tmp[0], bank_ids, bank_count);
 
    for (int i = 1; i < lhs_bw - 1; i++) {
        if (g_verbose >= 2)
            printf("      [final]   AND  tmp = lhs[%d] & rhs[%d]\n", i, rhs_bw - 1);
        execute_row_and(lhs[i], rhs[rhs_bw - 1], tmp[0], bank_ids, bank_count);
 
        if (g_verbose >= 2)
            printf("      [final]   ADD  out[%d] = tmp + partial[%d] + carry[0]  ->carry[0]\n",
                   rhs_bw - 1 + i, i - 1);
        execute_row_add(tmp[0], partial[i - 1], out[rhs_bw - 1 + i], carry[0], carry[0], bank_ids, bank_count);
    }
 
    if (g_verbose >= 2)
        printf("      [final]   AND  tmp = lhs[%d] & rhs[%d]\n", lhs_bw - 1, rhs_bw - 1);
    execute_row_and(lhs[lhs_bw - 1], rhs[rhs_bw - 1], tmp[0], bank_ids, bank_count);
 
    if (g_verbose >= 2)
        printf("      [final]   ADD  out[%d] = tmp + carry[1] + carry[0]  ->out[%d]\n",
               lhs_bw + rhs_bw - 2, lhs_bw + rhs_bw - 1);
    execute_row_add(tmp[0], carry[1], out[lhs_bw + rhs_bw - 2], carry[0], out[lhs_bw + rhs_bw - 1], bank_ids, bank_count);
}
 
/* -----------------------------------------------------------------------
 * Core execute_row_copy_batch
 * --------------------------------------------------------------------- */
void execute_row_copy_batch(const RowCopyTask *tasks, int task_count) {
    if (!tasks || task_count <= 0) return;
 
    int max_bw = 0;
    for (int t = 0; t < task_count; t++) {
        if (tasks[t].bitwidth > max_bw) max_bw = tasks[t].bitwidth;
    }
 
    for (int j = 0; j < max_bw; j++) {
        if (g_verbose >= 2) {
            printf("      [bit=%d]", j);
            for (int t = 0; t < task_count; t++) {
                if (j < tasks[t].bitwidth)
                    printf("  COPY bank %d->%d", tasks[t].src_bank, tasks[t].dst_bank);
            }
            printf("\n");
        }
        for (int t = 0; t < task_count; t++) {
            if (j >= tasks[t].bitwidth) continue;
            COPY_BANK_TO_BANK(
                tasks[t].dst[j], tasks[t].dst_bank,
                tasks[t].src[j], tasks[t].src_bank);
        }
    }
}
 
/* =======================================================================
 * Schedule runner
 * ===================================================================== */
 
#define MAX_SCHEDULE_STEPS  512
#define MAX_OPS_PER_STEP     64
#define SCHED_MAX_BANKS      32
 
/* Max bitwidth: keeps total row count below ROWS_PER_SUBARRAY (512).
 * Budget: 18 (ambit) + 8*16 + 4 = 150 rows.  Handles up to 16-bit ops. */
#define SCHED_MAX_BW         16
 
typedef enum { OP_MULI, OP_ADDI, OP_ROW_COPY } OpType;
 
typedef struct {
    OpType type;
    /* addi / muli */
    int lhs_bw, rhs_bw, bank;
    /* row_copy */
    int src_bank, dst_bank, bitwidth;
    /* timing (informational) */
    int start, end;
} SchedOp;
 
typedef struct {
    int     start_time;
    SchedOp ops[MAX_OPS_PER_STEP];
    int     count;
} TimeStep;
 
/* -----------------------------------------------------------------------
 * Pre-allocated row pools — one set per operation class.
 * All allocated from the subarray pool (see above).
 * --------------------------------------------------------------------- */
static unsigned *g_mul_lhs    [SCHED_MAX_BW];
static unsigned *g_mul_rhs    [SCHED_MAX_BW];
static unsigned *g_mul_out    [2 * SCHED_MAX_BW];
static unsigned *g_mul_partial[SCHED_MAX_BW];
static unsigned *g_mul_tmp    [1];
static unsigned *g_mul_carry  [2];
 
static unsigned *g_add_lhs[SCHED_MAX_BW];
static unsigned *g_add_rhs[SCHED_MAX_BW];
static unsigned *g_add_out[SCHED_MAX_BW + 1];
 
static void alloc_row_pools(void) {
    for (int j = 0; j < SCHED_MAX_BW; j++) {
        g_mul_lhs[j]                = alloc_vec_all_banks();
        g_mul_rhs[j]                = alloc_vec_all_banks();
        g_mul_out[j]                = alloc_vec_all_banks();
        g_mul_out[SCHED_MAX_BW + j] = alloc_vec_all_banks();
        g_mul_partial[j]            = alloc_vec_all_banks();
        g_add_lhs[j]                = alloc_vec_all_banks();
        g_add_rhs[j]                = alloc_vec_all_banks();
        g_add_out[j]                = alloc_vec_all_banks();
    }
    g_add_out[SCHED_MAX_BW] = alloc_vec_all_banks();
    g_mul_tmp[0]   = alloc_vec_all_banks();
    g_mul_carry[0] = alloc_vec_all_banks();
    g_mul_carry[1] = alloc_vec_all_banks();
}
 
/* -----------------------------------------------------------------------
 * Track the "current output" rows per bank so row_copy can find its src.
 * --------------------------------------------------------------------- */
typedef enum { BANK_LAST_NONE, BANK_LAST_MUL, BANK_LAST_ADD } BankLastOp;
static BankLastOp g_bank_last[SCHED_MAX_BANKS];
 
static unsigned **src_rows_for_bank(int bank) {
    return (g_bank_last[bank] == BANK_LAST_MUL) ? g_mul_out : g_add_out;
}
 
/* -----------------------------------------------------------------------
 * Parser — only C standard library, no regex / strtok tricks.
 * --------------------------------------------------------------------- */
static int parse_schedule(const char *path, TimeStep *steps, int *nsteps) {
    FILE *f = fopen(path, "r");
    if (!f) { perror(path); return -1; }
 
    char line[512];
    *nsteps = 0;
    int cur = -1;
 
    while (fgets(line, sizeof(line), f)) {
        int t;
        if (sscanf(line, "start_time=%d:", &t) == 1) {
            if (*nsteps >= MAX_SCHEDULE_STEPS) {
                fprintf(stderr, "schedule_runner: too many time steps (max %d)\n",
                        MAX_SCHEDULE_STEPS);
                fclose(f);
                return -1;
            }
            cur = (*nsteps)++;
            steps[cur].start_time = t;
            steps[cur].count      = 0;
            continue;
        }
 
        if (cur < 0 || steps[cur].count >= MAX_OPS_PER_STEP) continue;
 
        SchedOp *op = &steps[cur].ops[steps[cur].count];
        int a, b, c, d, e;
 
        if (sscanf(line,
                   " muli(lhs_bw=%d, rhs_bw=%d, start=%d, end=%d, bank=%d)",
                   &a, &b, &c, &d, &e) == 5) {
            op->type   = OP_MULI;
            op->lhs_bw = a; op->rhs_bw = b;
            op->start  = c; op->end    = d; op->bank = e;
            steps[cur].count++;
 
        } else if (sscanf(line,
                          " addi(lhs_bw=%d, rhs_bw=%d, start=%d, end=%d, bank=%d)",
                          &a, &b, &c, &d, &e) == 5) {
            op->type   = OP_ADDI;
            op->lhs_bw = a; op->rhs_bw = b;
            op->start  = c; op->end    = d; op->bank = e;
            steps[cur].count++;
 
        } else if (sscanf(line,
                          " row_copy(src_bank=%d, dst_bank=%d, bitwidth=%d, start=%d, end=%d)",
                          &a, &b, &c, &d, &e) == 5) {
            op->type     = OP_ROW_COPY;
            op->src_bank = a; op->dst_bank = b; op->bitwidth = c;
            op->start    = d; op->end      = e;
            steps[cur].count++;
        }
        /* Unrecognised lines (blank, comments, etc.) are silently skipped */
    }
 
    fclose(f);
    return 0;
}
 
/* -----------------------------------------------------------------------
 * Group same-(lhs_bw, rhs_bw) ops so they run in a single batched call.
 * --------------------------------------------------------------------- */
typedef struct {
    int lhs_bw, rhs_bw;
    int bank_ids[SCHED_MAX_BANKS];
    int bank_count;
} OpGroup;
 
static int find_or_add_group(OpGroup *groups, int *n, int lhs_bw, int rhs_bw) {
    for (int i = 0; i < *n; i++)
        if (groups[i].lhs_bw == lhs_bw && groups[i].rhs_bw == rhs_bw)
            return i;
    int i = (*n)++;
    groups[i].lhs_bw    = lhs_bw;
    groups[i].rhs_bw    = rhs_bw;
    groups[i].bank_count = 0;
    return i;
}
 
/* -----------------------------------------------------------------------
 * Execute one time-step: group → dispatch.
 * --------------------------------------------------------------------- */
static void execute_time_step(const TimeStep *step) {
    OpGroup   mul_groups[MAX_OPS_PER_STEP];
    OpGroup   add_groups[MAX_OPS_PER_STEP];
    int       n_mul = 0, n_add = 0;
 
    RowCopyTask copy_tasks[MAX_OPS_PER_STEP];
    int         n_copy = 0;
 
    /* --- classify and group ------------------------------------------ */
    for (int i = 0; i < step->count; i++) {
        const SchedOp *op = &step->ops[i];
 
        if (op->type == OP_MULI) {
            int g = find_or_add_group(mul_groups, &n_mul, op->lhs_bw, op->rhs_bw);
            OpGroup *grp = &mul_groups[g];
            if (grp->bank_count < SCHED_MAX_BANKS)
                grp->bank_ids[grp->bank_count++] = op->bank;
 
        } else if (op->type == OP_ADDI) {
            int g = find_or_add_group(add_groups, &n_add, op->lhs_bw, op->rhs_bw);
            OpGroup *grp = &add_groups[g];
            if (grp->bank_count < SCHED_MAX_BANKS)
                grp->bank_ids[grp->bank_count++] = op->bank;
 
        } else { /* OP_ROW_COPY */
            RowCopyTask *t  = &copy_tasks[n_copy++];
            t->src      = src_rows_for_bank(op->src_bank);
            t->dst      = g_add_lhs;
            t->src_bank = op->src_bank;
            t->dst_bank = op->dst_bank;
            t->bitwidth = op->bitwidth;
        }
    }
 
    if (g_verbose >= 1)
        printf("[t=%-4d] %d op(s)\n", step->start_time, step->count);
 
    /* --- dispatch mul groups ----------------------------------------- */
    for (int g = 0; g < n_mul; g++) {
        OpGroup *grp = &mul_groups[g];
        if (g_verbose >= 1) {
            printf("  MULI lhs=%-2d rhs=%-2d  banks:", grp->lhs_bw, grp->rhs_bw);
            for (int k = 0; k < grp->bank_count; k++) printf(" %d", grp->bank_ids[k]);
            printf("\n");
        }
        execute_mul(grp->lhs_bw, grp->rhs_bw,
                    grp->bank_ids, grp->bank_count,
                    g_mul_lhs, g_mul_rhs, g_mul_out,
                    g_mul_partial, g_mul_tmp, g_mul_carry);
        for (int k = 0; k < grp->bank_count; k++)
            g_bank_last[grp->bank_ids[k]] = BANK_LAST_MUL;
    }
 
    /* --- dispatch add groups ----------------------------------------- */
    for (int g = 0; g < n_add; g++) {
        OpGroup *grp = &add_groups[g];
        if (g_verbose >= 1) {
            printf("  ADDI lhs=%-2d rhs=%-2d  banks:", grp->lhs_bw, grp->rhs_bw);
            for (int k = 0; k < grp->bank_count; k++) printf(" %d", grp->bank_ids[k]);
            printf("\n");
        }
        execute_add(grp->lhs_bw, grp->rhs_bw,
                    grp->bank_ids, grp->bank_count,
                    g_add_lhs, g_add_rhs, g_add_out);
        for (int k = 0; k < grp->bank_count; k++)
            g_bank_last[grp->bank_ids[k]] = BANK_LAST_ADD;
    }
 
    /* --- dispatch row copies ----------------------------------------- */
    if (n_copy > 0) {
        if (g_verbose >= 1) {
            printf("  COPY");
            for (int t = 0; t < n_copy; t++)
                printf("  bank %d->%d (bw=%d)",
                       copy_tasks[t].src_bank, copy_tasks[t].dst_bank,
                       copy_tasks[t].bitwidth);
            printf("\n");
        }
        execute_row_copy_batch(copy_tasks, n_copy);
    }
}
 
/* -----------------------------------------------------------------------
 * main
 * --------------------------------------------------------------------- */
int main(int argc, char *argv[]) {
    if (argc < 2) {
        fprintf(stderr, "usage: %s <schedule.txt> [-v|-vv]\n", argv[0]);
        return 1;
    }
 
    /* Parse optional verbosity flag (-v = ops, -vv = ops + insns) */
    for (int i = 2; i < argc; i++) {
        if (strcmp(argv[i], "-vv") == 0)      g_verbose = 2;
        else if (strcmp(argv[i], "-v") == 0)  g_verbose = 1;
    }
 
    /* init_ambit() first (18 rows), then data rows — all via sequential
     * posix_memalign(ALIGNMENT, ALIGNMENT) so they are physically contiguous
     * and within a single DRAM subarray. */
    init_ambit();
    alloc_row_pools();
 
    static TimeStep steps[MAX_SCHEDULE_STEPS];
    int nsteps = 0;
 
    if (parse_schedule(argv[1], steps, &nsteps) != 0) {
        fprintf(stderr, "schedule_runner: failed to parse '%s'\n", argv[1]);
        return 1;
    }
 
    if (g_verbose >= 1)
        printf("Parsed %d time step(s) from '%s'\n\n", nsteps, argv[1]);
 
    /* Two iterations: first is warmup, second is the measured run.
     * m5_reset_stats at the top of each iteration ensures only the
     * last iteration's stats are captured by m5_dump_stats. */
    for (int iter = 0; iter < 2; iter++) {
        m5_reset_stats(0, 0);
 
        if (g_verbose >= 1)
            printf("=== %s (iter %d) ===\n",
                   iter == 0 ? "Warmup     " : "Measurement", iter);
 
        memset(g_bank_last, 0, sizeof(g_bank_last));
        for (int s = 0; s < nsteps; s++)
            execute_time_step(&steps[s]);
 
        if (g_verbose >= 1) printf("\n");
    }
 
    m5_dump_stats(0, 0);
 
    if (g_verbose >= 1) printf("Done.\n");
 
    return 0;
}