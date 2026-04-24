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

#define SCHED_MAX_BANKS      32
#define SCHED_MAX_BW         64
#define PRADA_TMP_ROWS       10

typedef enum { OP_MULI = 0, OP_ADDI = 1, OP_ROW_COPY = 2 } OpType;

typedef struct {
    char     magic[8];
    uint32_t version;
    uint32_t record_size;
    uint64_t num_records;
    int64_t  last_end_time;
} TraceHeader;

typedef struct {
    int64_t  start;
    int64_t  end;
    uint32_t banks;
    uint16_t lhs_bw;
    uint16_t rhs_bw;
    uint8_t  kind;
    uint8_t  src;
    uint8_t  dst;
    uint8_t  pad[5];
} TraceRecord;
_Static_assert(sizeof(TraceHeader) == 32, "TraceHeader must be 32 bytes");
_Static_assert(sizeof(TraceRecord) == 32, "TraceRecord must be 32 bytes");

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
 * Parser
 * --------------------------------------------------------------------- */
static int validate_record(const TraceRecord *rec) {
    if (rec->kind > OP_ROW_COPY) return -1;
    if (rec->end < rec->start) return -1;
    if (rec->lhs_bw > SCHED_MAX_BW || rec->rhs_bw > SCHED_MAX_BW) return -1;
    if (rec->kind == OP_ROW_COPY && (rec->src >= SCHED_MAX_BANKS || rec->dst >= SCHED_MAX_BANKS))
        return -1;
    return 0;
}

static int read_trace_header(FILE *f, TraceHeader *hdr) {
    if (fread(hdr, sizeof(*hdr), 1, f) != 1) return -1;
    if (memcmp(hdr->magic, "CIMTRACE", 8) != 0) return -1;
    if (hdr->version != 1 || hdr->record_size != sizeof(TraceRecord)) return -1;
    return 0;
}

static int bank_mask_to_ids(uint32_t mask, int *bank_ids) {
    int n = 0;
    for (int b = 0; b < SCHED_MAX_BANKS; b++) {
        if (mask & (1u << b)) bank_ids[n++] = b;
    }
    return n;
}

static void execute_trace_record(const TraceRecord *rec) {
    if (rec->kind == OP_MULI || rec->kind == OP_ADDI) {
        int bank_ids[SCHED_MAX_BANKS];
        int bank_count = bank_mask_to_ids(rec->banks, bank_ids);

        if (g_verbose >= 1) {
            printf("  %s lhs=%-2u rhs=%-2u  banks:",
                   rec->kind == OP_MULI ? "MULI" : "ADDI",
                   rec->lhs_bw, rec->rhs_bw);
            for (int k = 0; k < bank_count; k++) printf(" %d", bank_ids[k]);
            printf("\n");
        }

        if (rec->kind == OP_MULI) {
            execute_mul((int)rec->lhs_bw, (int)rec->rhs_bw,
                        bank_ids, bank_count,
                        g_lhs, g_rhs, g_out,
                        g_partial, g_tmp);
        } else {
            execute_add((int)rec->lhs_bw, (int)rec->rhs_bw,
                        bank_ids, bank_count,
                        g_lhs, g_rhs, g_out, g_tmp);
        }
    } else {
        RowCopyTask task;
        task.src      = g_out;
        task.dst      = g_lhs;
        task.src_bank = rec->src;
        task.dst_bank = rec->dst;
        task.bitwidth = rec->lhs_bw;

        if (g_verbose >= 1) {
            printf("  COPY  bank %d->%d (bw=%d)\n",
                   task.src_bank, task.dst_bank, task.bitwidth);
        }
        execute_row_copy_batch(&task, 1);
    }
}

static int derive_pool_widths_from_trace(const char *path, PoolWidths *pw, TraceHeader *out_hdr) {
    init_pool_widths(pw);

    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); return -1; }
    setvbuf(f, NULL, _IOFBF, 1 << 20);

    TraceHeader hdr;
    if (read_trace_header(f, &hdr) != 0) {
        fprintf(stderr, "schedule_runner: invalid trace header in '%s'\n", path);
        fclose(f);
        return -1;
    }

    for (uint64_t i = 0; i < hdr.num_records; i++) {
        TraceRecord rec;
        if (fread(&rec, sizeof(rec), 1, f) != 1) {
            fprintf(stderr, "schedule_runner: truncated trace at record %llu\n",
                    (unsigned long long)i);
            fclose(f);
            return -1;
        }
        if (validate_record(&rec) != 0) {
            fprintf(stderr, "schedule_runner: invalid record at index %llu\n",
                    (unsigned long long)i);
            fclose(f);
            return -1;
        }

        if (rec.kind == OP_MULI) {
            if (rec.lhs_bw > pw->max_lhs_bw) pw->max_lhs_bw = rec.lhs_bw;
            if (rec.rhs_bw > pw->max_rhs_bw) pw->max_rhs_bw = rec.rhs_bw;
            if ((int)rec.lhs_bw + (int)rec.rhs_bw > pw->max_out_bw)
                pw->max_out_bw = rec.lhs_bw + rec.rhs_bw;
            if (rec.lhs_bw > pw->max_partial_bw) pw->max_partial_bw = rec.lhs_bw;
        } else if (rec.kind == OP_ADDI) {
            int add_bw = (rec.lhs_bw < rec.rhs_bw) ? rec.lhs_bw : rec.rhs_bw;
            if (rec.lhs_bw > pw->max_lhs_bw) pw->max_lhs_bw = rec.lhs_bw;
            if (rec.rhs_bw > pw->max_rhs_bw) pw->max_rhs_bw = rec.rhs_bw;
            if (add_bw > pw->max_out_bw) pw->max_out_bw = add_bw;
        } else {
            if (rec.lhs_bw > pw->max_lhs_bw) pw->max_lhs_bw = rec.lhs_bw;
        }
    }

    if (pw->max_lhs_bw > SCHED_MAX_BW || pw->max_rhs_bw > SCHED_MAX_BW ||
        pw->max_partial_bw > SCHED_MAX_BW || pw->max_out_bw > 2 * SCHED_MAX_BW) {
        fprintf(stderr, "schedule_runner: schedule bitwidth exceeds SCHED_MAX_BW=%d\n",
                SCHED_MAX_BW);
        fclose(f);
        return -1;
    }

    if (out_hdr) *out_hdr = hdr;
    fclose(f);
    return 0;
}

static int execute_trace_once(const char *path) {
    FILE *f = fopen(path, "rb");
    if (!f) { perror(path); return -1; }
    setvbuf(f, NULL, _IOFBF, 1 << 20);

    TraceHeader hdr;
    if (read_trace_header(f, &hdr) != 0) {
        fprintf(stderr, "schedule_runner: invalid trace header in '%s'\n", path);
        fclose(f);
        return -1;
    }

    int64_t current_start = 0;
    int has_start = 0;

    for (uint64_t i = 0; i < hdr.num_records; i++) {
        TraceRecord rec;
        if (fread(&rec, sizeof(rec), 1, f) != 1) {
            fprintf(stderr, "schedule_runner: truncated trace at record %llu\n",
                    (unsigned long long)i);
            fclose(f);
            return -1;
        }
        if (validate_record(&rec) != 0) {
            fprintf(stderr, "schedule_runner: invalid record at index %llu\n",
                    (unsigned long long)i);
            fclose(f);
            return -1;
        }

        if (!has_start || rec.start != current_start) {
            current_start = rec.start;
            has_start = 1;
            if (g_verbose >= 1) printf("[t=%-4lld]\n", (long long)current_start);
        }
        execute_trace_record(&rec);
    }
    fclose(f);
    return 0;
}

int main(int argc, char *argv[]) {
    if (argc < 2) {
        fprintf(stderr, "usage: %s <schedule.bin> [-v|-vv]\n", argv[0]);
        return 1;
    }

    for (int i = 2; i < argc; i++) {
        if (strcmp(argv[i], "-vv") == 0)      g_verbose = 2;
        else if (strcmp(argv[i], "-v") == 0)  g_verbose = 1;
    }

    PoolWidths widths;
    TraceHeader hdr;
    if (derive_pool_widths_from_trace(argv[1], &widths, &hdr) != 0) return 1;
    if (check_row_budget(&widths) != 0) return 1;

    alloc_row_pools(&widths);

    if (g_verbose >= 1) {
        printf("Loaded trace '%s': records=%llu, last_end_time=%lld\n\n",
               argv[1], (unsigned long long)hdr.num_records, (long long)hdr.last_end_time);
    }

    // for (int iter = 0; iter < 2; iter++) {
    //     m5_reset_stats(0, 0);

    //     if (g_verbose >= 1)
    //         printf("=== %s (iter %d) ===\n",
    //                iter == 0 ? "Warmup     " : "Measurement", iter);

    //     if (execute_trace_once(argv[1]) != 0) return 1;

    //     if (g_verbose >= 1) printf("\n");
    // }

    m5_reset_stats(0, 0);

    if (execute_trace_once(argv[1]) != 0) return 1;

    m5_dump_stats(0, 0);

    if (g_verbose >= 1) printf("Done.\n");

    return 0;
}
