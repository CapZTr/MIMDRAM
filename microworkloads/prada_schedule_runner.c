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
    rowop_aaap(BANK_ROW((dst), (bank)), BANK_ROW((src1), (bank)), BANK_ROW((src2), (bank)))
#define AAAAAP_ONE_BANK(dst, src1, src2, src3, src4, bank) \
    rowop_aaaaap(BANK_ROW((dst), (bank)), BANK_ROW((src1), (bank)), BANK_ROW((src2), (bank)), BANK_ROW((src3), (bank)), BANK_ROW((src4), (bank)))
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
    for (int i = 0; i < bank_count; i++) AAP_ONE_BANK(tmp[0],  lhs_bit, bank_ids[i]);
    for (int i = 0; i < bank_count; i++) AAP_ONE_BANK(tmp[1],  rhs_bit, bank_ids[i]);
    for (int i = 0; i < bank_count; i++) AAP_ONE_BANK(out_bit, C_0,     bank_ids[i]);
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

    unsigned *carry = C_0;
    for (int j = 0; j < bw; j++) {
        if (g_verbose >= 2)
            printf("      [bit=%d] full-adder: out[%d] = lhs[%d] + rhs[%d] + carry  ->  carry\n",
                   j, j, j, j);
        execute_row_add(lhs[j], rhs[j], out[j], carry, carry, tmp, bank_ids, bank_count);
    }
}

/* -----------------------------------------------------------------------
 * Core execute_mul
 * --------------------------------------------------------------------- */
void execute_mul(
    int lhs_bw, int rhs_bw,
    const int *bank_ids, int bank_count,
    unsigned **lhs, unsigned **rhs, unsigned **out,
    unsigned **partial, unsigned **tmp)
{
    unsigned *carry0 = tmp[5];
    unsigned *carry1 = tmp[6];

    /* --- Phase 1: initialise out[0] and partial[] using rhs[0] --------- */
    if (g_verbose >= 2)
        printf("      [init]    AND  out[0] = lhs[0] & rhs[0]\n");
    execute_row_and(lhs[0], rhs[0], out[0], tmp, bank_ids, bank_count);

    for (int i = 0; i < lhs_bw - 1; i++) {
        if (g_verbose >= 2)
            printf("      [init]    AND  partial[%d] = lhs[%d] & rhs[0]\n", i, i + 1);
        execute_row_and(lhs[i + 1], rhs[0], partial[i], tmp, bank_ids, bank_count);
    }

    for (int i = 0; i < bank_count; i++) AAP_ONE_BANK(carry1, C_0, bank_ids[i]);

    /* --- Phase 2: accumulate rhs[1..rhs_bw-2] -------------------------- */
    for (int i = 0; i < rhs_bw - 1; i++) {
        if (g_verbose >= 2)
            printf("      [rhs=%d]   AND  tmp = lhs[0] & rhs[%d]\n", i, i);
        execute_row_and(lhs[0], rhs[i], tmp[0], tmp, bank_ids, bank_count);

        if (g_verbose >= 2)
            printf("      [rhs=%d]   ADD  out[%d] = tmp + partial[0] + 0  ->carry[0]\n", i, i);
        execute_row_add(tmp[0], partial[0], out[i], C_0, carry0, tmp, bank_ids, bank_count);

        for (int j = 1; j < lhs_bw - 1; j++) {
            if (g_verbose >= 2)
                printf("      [rhs=%d]   AND  tmp = lhs[%d] & rhs[%d]\n", i, j, i);
            execute_row_and(lhs[j], rhs[i], tmp[0], tmp, bank_ids, bank_count);

            if (g_verbose >= 2)
                printf("      [rhs=%d]   ADD  partial[%d] = tmp + partial[%d] + carry[0]  ->carry[0]\n",
                       i, j - 1, j);
            execute_row_add(tmp[0], partial[j], partial[j - 1], carry0, carry0, tmp, bank_ids, bank_count);
        }

        if (g_verbose >= 2)
            printf("      [rhs=%d]   AND  tmp = lhs[%d] & rhs[%d]\n", i, lhs_bw - 1, i);
        execute_row_and(lhs[lhs_bw - 1], rhs[i], tmp[0], tmp, bank_ids, bank_count);

        if (g_verbose >= 2)
            printf("      [rhs=%d]   ADD  partial[%d] = tmp + carry[0] + carry[1]  ->carry[1]\n",
                   i, lhs_bw - 2);
        execute_row_add(tmp[0], carry0, partial[lhs_bw - 2], carry1, carry1, tmp, bank_ids, bank_count);
    }

    /* --- Phase 3: final row rhs[rhs_bw-1] ------------------------------ */
    if (g_verbose >= 2)
        printf("      [final]   AND  tmp = lhs[0] & rhs[%d]\n", rhs_bw - 1);
    execute_row_and(lhs[0], rhs[rhs_bw - 1], tmp[0], tmp, bank_ids, bank_count);

    for (int i = 1; i < lhs_bw - 1; i++) {
        if (g_verbose >= 2)
            printf("      [final]   AND  tmp = lhs[%d] & rhs[%d]\n", i, rhs_bw - 1);
        execute_row_and(lhs[i], rhs[rhs_bw - 1], tmp[0], tmp, bank_ids, bank_count);

        if (g_verbose >= 2)
            printf("      [final]   ADD  out[%d] = tmp + partial[%d] + carry[0]  ->carry[0]\n",
                   rhs_bw - 1 + i, i - 1);
        execute_row_add(tmp[0], partial[i - 1], out[rhs_bw - 1 + i], carry0, carry0, tmp, bank_ids, bank_count);
    }

    if (g_verbose >= 2)
        printf("      [final]   AND  tmp = lhs[%d] & rhs[%d]\n", lhs_bw - 1, rhs_bw - 1);
    execute_row_and(lhs[lhs_bw - 1], rhs[rhs_bw - 1], tmp[0], tmp, bank_ids, bank_count);

    if (g_verbose >= 2)
        printf("      [final]   ADD  out[%d] = tmp + carry[1] + carry[0]  ->out[%d]\n",
               lhs_bw + rhs_bw - 2, lhs_bw + rhs_bw - 1);
    execute_row_add(tmp[0], carry1, out[lhs_bw + rhs_bw - 2], carry0, out[lhs_bw + rhs_bw - 1], tmp, bank_ids, bank_count);
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
#define SCHED_MAX_BW         64
#define PRADA_TMP_ROWS       10

typedef enum { OP_MULI, OP_ADDI, OP_ROW_COPY } OpType;

typedef struct {
    OpType type;
    int lhs_bw, rhs_bw, bank;
    int src_bank, dst_bank, bitwidth;
    int start, end;
} SchedOp;

typedef struct {
    int     start_time;
    SchedOp ops[MAX_OPS_PER_STEP];
    int     count;
} TimeStep;

static unsigned *g_lhs    [SCHED_MAX_BW];
static unsigned *g_rhs    [SCHED_MAX_BW];
static unsigned *g_out    [2 * SCHED_MAX_BW];
static unsigned *g_partial[SCHED_MAX_BW];
static unsigned *g_tmp    [PRADA_TMP_ROWS];

typedef struct {
    int max_lhs_bw;
    int max_rhs_bw;
    int max_out_bw;
    int max_partial_bw;
} PoolWidths;

static void init_pool_widths(PoolWidths *pw) {
    memset(pw, 0, sizeof(*pw));
}

static int derive_pool_widths(const TimeStep *steps, int nsteps, PoolWidths *pw) {
    init_pool_widths(pw);
    for (int s = 0; s < nsteps; s++) {
        const TimeStep *step = &steps[s];
        for (int i = 0; i < step->count; i++) {
            const SchedOp *op = &step->ops[i];
            if (op->type == OP_MULI) {
                if (op->lhs_bw > pw->max_lhs_bw) pw->max_lhs_bw = op->lhs_bw;
                if (op->rhs_bw > pw->max_rhs_bw) pw->max_rhs_bw = op->rhs_bw;
                if (op->lhs_bw + op->rhs_bw > pw->max_out_bw)
                    pw->max_out_bw = op->lhs_bw + op->rhs_bw;
                if (op->lhs_bw > pw->max_partial_bw)
                    pw->max_partial_bw = op->lhs_bw;
            } else if (op->type == OP_ADDI) {
                int add_bw = (op->lhs_bw < op->rhs_bw) ? op->lhs_bw : op->rhs_bw;
                if (op->lhs_bw > pw->max_lhs_bw) pw->max_lhs_bw = op->lhs_bw;
                if (op->rhs_bw > pw->max_rhs_bw) pw->max_rhs_bw = op->rhs_bw;
                if (add_bw > pw->max_out_bw) pw->max_out_bw = add_bw;
            } else if (op->type == OP_ROW_COPY) {
                if (op->bitwidth > pw->max_lhs_bw) pw->max_lhs_bw = op->bitwidth;
            }
        }
    }

    if (pw->max_lhs_bw > SCHED_MAX_BW || pw->max_rhs_bw > SCHED_MAX_BW ||
        pw->max_partial_bw > SCHED_MAX_BW || pw->max_out_bw > 2 * SCHED_MAX_BW) {
        fprintf(stderr, "schedule_runner: schedule bitwidth exceeds SCHED_MAX_BW=%d\n",
                SCHED_MAX_BW);
        return -1;
    }
    return 0;
}

static int check_row_budget(const PoolWidths *pw) {
    int pool_rows =
        pw->max_lhs_bw + pw->max_rhs_bw + pw->max_out_bw + pw->max_partial_bw +
        PRADA_TMP_ROWS + 1; /* tmp rows + C_0 */
    if (pool_rows > ROWS_PER_SUBARRAY) {
        fprintf(stderr,
                "schedule_runner: row budget overflow: need %d rows, but ROWS_PER_SUBARRAY=%d\n",
                pool_rows, ROWS_PER_SUBARRAY);
        return -1;
    }
    return 0;
}

static void alloc_row_pools(const PoolWidths *pw) {
    C_0 = alloc_vec_all_banks();

    for (int j = 0; j < pw->max_lhs_bw; j++) g_lhs[j] = alloc_vec_all_banks();
    for (int j = 0; j < pw->max_rhs_bw; j++) g_rhs[j] = alloc_vec_all_banks();
    for (int j = 0; j < pw->max_out_bw; j++) g_out[j] = alloc_vec_all_banks();
    for (int j = 0; j < pw->max_partial_bw; j++) g_partial[j] = alloc_vec_all_banks();
    for (int j = 0; j < PRADA_TMP_ROWS; j++) g_tmp[j] = alloc_vec_all_banks();
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
    }

    fclose(f);
    return 0;
}

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
    groups[i].lhs_bw     = lhs_bw;
    groups[i].rhs_bw     = rhs_bw;
    groups[i].bank_count = 0;
    return i;
}

static void execute_time_step(const TimeStep *step) {
    OpGroup   mul_groups[MAX_OPS_PER_STEP];
    OpGroup   add_groups[MAX_OPS_PER_STEP];
    int       n_mul = 0, n_add = 0;

    RowCopyTask copy_tasks[MAX_OPS_PER_STEP];
    int         n_copy = 0;

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

        } else {
            RowCopyTask *t  = &copy_tasks[n_copy++];
            t->src      = g_out;
            t->dst      = g_lhs;
            t->src_bank = op->src_bank;
            t->dst_bank = op->dst_bank;
            t->bitwidth = op->bitwidth;
        }
    }

    if (g_verbose >= 1)
        printf("[t=%-4d] %d op(s)\n", step->start_time, step->count);

    for (int g = 0; g < n_mul; g++) {
        OpGroup *grp = &mul_groups[g];
        if (g_verbose >= 1) {
            printf("  MULI lhs=%-2d rhs=%-2d  banks:", grp->lhs_bw, grp->rhs_bw);
            for (int k = 0; k < grp->bank_count; k++) printf(" %d", grp->bank_ids[k]);
            printf("\n");
        }
        execute_mul(grp->lhs_bw, grp->rhs_bw,
                    grp->bank_ids, grp->bank_count,
                    g_lhs, g_rhs, g_out,
                    g_partial, g_tmp);
    }

    for (int g = 0; g < n_add; g++) {
        OpGroup *grp = &add_groups[g];
        if (g_verbose >= 1) {
            printf("  ADDI lhs=%-2d rhs=%-2d  banks:", grp->lhs_bw, grp->rhs_bw);
            for (int k = 0; k < grp->bank_count; k++) printf(" %d", grp->bank_ids[k]);
            printf("\n");
        }
        execute_add(grp->lhs_bw, grp->rhs_bw,
                    grp->bank_ids, grp->bank_count,
                    g_lhs, g_rhs, g_out, g_tmp);
    }

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

int main(int argc, char *argv[]) {
    if (argc < 2) {
        fprintf(stderr, "usage: %s <schedule.txt> [-v|-vv]\n", argv[0]);
        return 1;
    }

    for (int i = 2; i < argc; i++) {
        if (strcmp(argv[i], "-vv") == 0)      g_verbose = 2;
        else if (strcmp(argv[i], "-v") == 0)  g_verbose = 1;
    }

    static TimeStep steps[MAX_SCHEDULE_STEPS];
    int nsteps = 0;
    PoolWidths widths;

    if (parse_schedule(argv[1], steps, &nsteps) != 0) {
        fprintf(stderr, "schedule_runner: failed to parse '%s'\n", argv[1]);
        return 1;
    }

    if (derive_pool_widths(steps, nsteps, &widths) != 0) return 1;
    if (check_row_budget(&widths) != 0) return 1;

    alloc_row_pools(&widths);

    if (g_verbose >= 1)
        printf("Parsed %d time step(s) from '%s'\n\n", nsteps, argv[1]);

    for (int iter = 0; iter < 2; iter++) {
        m5_reset_stats(0, 0);

        if (g_verbose >= 1)
            printf("=== %s (iter %d) ===\n",
                   iter == 0 ? "Warmup     " : "Measurement", iter);

        for (int s = 0; s < nsteps; s++)
            execute_time_step(&steps[s]);

        if (g_verbose >= 1) printf("\n");
    }

    m5_dump_stats(0, 0);

    if (g_verbose >= 1) printf("Done.\n");

    return 0;
}
