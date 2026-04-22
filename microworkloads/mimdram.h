#ifndef AMBIT_PLUS_H
#define AMBIT_PLUS_H

#include <stdlib.h>
#include <stdint.h>

extern void rowop_ap(void *dst);
extern void rowop_aap(void *dst, void *src);
extern void rowop_copy(void *dst, void *src);
extern void rowop_aaap(void *dst, void *src1, void *src2);
extern void rowop_anap(void *dst, void *src);
extern void rowop_aaaaap(void *dst, void *src1, void *src2, void *src3, void *src4);
/* PUD COTS DDR4 primitives */
extern void rowop_rowclone(void *dst, void *src);
extern void rowop_mrc(void *dst_last, void *src_first);
extern void rowop_maj(void *dst_last, void *src_first);
extern void rowop_bulk_write(void *dst_last, void *src_first, uint64_t data);
extern void rowop_not_xsub(void *dst, void *src);
extern void rowop_and_xsub(void *com, void *ref);
extern void rowop_or_xsub(void *com, void *ref);
extern void rowop_frac(void *dst);

#define ROW_SIZE 8192
#define BANK_COUNT 16
#define RANK_COUNT 2
#define ROWS_PER_VECTOR (BANK_COUNT * RANK_COUNT)
#define ALIGNMENT (ROW_SIZE * ROWS_PER_VECTOR)

/* Must match gem5's rows_per_subarray parameter */
#define ROWS_PER_SUBARRAY 512
#define SUBARRAY_SIZE ((size_t)ROWS_PER_SUBARRAY * ALIGNMENT)

#define FOR_ALL_VECTORS for(int base_row = 0; base_row < per_col_rows; base_row += ROWS_PER_VECTOR)
#define VECTOR(ptr) ((void *)(ptr) + base_row*ROW_SIZE)
#define FOR_ALL_ROWS_IN_VECTOR for(int row = 0; row < per_col_rows - base_row && row < ROWS_PER_VECTOR; row ++)
#define ROW(ptr) ((void *)(ptr) + row*ROW_SIZE)
#define AAP_VECTORS(dst, src)              FOR_ALL_ROWS_IN_VECTOR { rowop_aap  (ROW(dst), ROW(src)); }
#define COPY_VECTORS(dst, src)             FOR_ALL_ROWS_IN_VECTOR { rowop_copy (ROW(dst), ROW(src)); }
#define AP_VECTOR(dst)                     FOR_ALL_ROWS_IN_VECTOR { rowop_ap   (ROW(dst)); }
#define AAAP_VECTORS(dst, src1, src2)        FOR_ALL_ROWS_IN_VECTOR { rowop_aaap      (ROW(dst), ROW(src1), ROW(src2)); }
#define ANAP_VECTORS(dst, src)               FOR_ALL_ROWS_IN_VECTOR { rowop_anap      (ROW(dst), ROW(src)); }
#define AAAAAP_VECTORS(dst, s1, s2, s3, s4)  FOR_ALL_ROWS_IN_VECTOR { rowop_aaaaap   (ROW(dst), ROW(s1), ROW(s2), ROW(s3), ROW(s4)); }
/* PUD COTS DDR4 primitives */
#define ROWCLONE_VECTORS(dst, src)           FOR_ALL_ROWS_IN_VECTOR { rowop_rowclone  (ROW(dst), ROW(src)); }
#define MRC_VECTORS(last, first)             FOR_ALL_ROWS_IN_VECTOR { rowop_mrc       (ROW(last), ROW(first)); }
#define MAJ_VECTORS(last, first)             FOR_ALL_ROWS_IN_VECTOR { rowop_maj       (ROW(last), ROW(first)); }
#define BULK_WRITE_VECTORS(last, first, d)   FOR_ALL_ROWS_IN_VECTOR { rowop_bulk_write(ROW(last), ROW(first), (d)); }
#define NOT_XSUB_VECTORS(dst, src)           FOR_ALL_ROWS_IN_VECTOR { rowop_not_xsub  (ROW(dst), ROW(src)); }
#define AND_XSUB_VECTORS(com, ref)           FOR_ALL_ROWS_IN_VECTOR { rowop_and_xsub  (ROW(com), ROW(ref)); }
#define OR_XSUB_VECTORS(com, ref)            FOR_ALL_ROWS_IN_VECTOR { rowop_or_xsub   (ROW(com), ROW(ref)); }
#define FRAC_VECTORS(dst)                    FOR_ALL_ROWS_IN_VECTOR { rowop_frac       (ROW(dst)); }

static void *B_T0         = NULL;
static void *B_T1         = NULL;
static void *B_T2         = NULL;
static void *B_T3         = NULL;
static void *B_DCC0       = NULL;
static void *B_DCC0N      = NULL;
static void *B_DCC1       = NULL;
static void *B_DCC1N      = NULL;
static void *B_DCC0N_T0   = NULL;
static void *B_DCC1N_T1   = NULL;
static void *B_T2_T3      = NULL;
static void *B_T0_T3      = NULL;
static void *B_T0_T1_T2   = NULL;
static void *B_T1_T2_T3   = NULL;
static void *B_DCC0_T1_T2 = NULL;
static void *B_DCC1_T0_T3 = NULL;
static void *C_0          = NULL;
static void *C_1          = NULL;

static void init_ambit(void) {
	int dummy = 0;
	dummy += posix_memalign(&B_T0        , ALIGNMENT, ALIGNMENT);
	dummy += posix_memalign(&B_T1        , ALIGNMENT, ALIGNMENT);
	dummy += posix_memalign(&B_T2        , ALIGNMENT, ALIGNMENT);
	dummy += posix_memalign(&B_T3        , ALIGNMENT, ALIGNMENT);
	dummy += posix_memalign(&B_DCC0      , ALIGNMENT, ALIGNMENT);
	dummy += posix_memalign(&B_DCC0N     , ALIGNMENT, ALIGNMENT);
	dummy += posix_memalign(&B_DCC1      , ALIGNMENT, ALIGNMENT);
	dummy += posix_memalign(&B_DCC1N     , ALIGNMENT, ALIGNMENT);
	dummy += posix_memalign(&B_DCC0N_T0  , ALIGNMENT, ALIGNMENT);
	dummy += posix_memalign(&B_DCC1N_T1  , ALIGNMENT, ALIGNMENT);
	dummy += posix_memalign(&B_T2_T3     , ALIGNMENT, ALIGNMENT);
	dummy += posix_memalign(&B_T0_T3     , ALIGNMENT, ALIGNMENT);
	dummy += posix_memalign(&B_T0_T1_T2  , ALIGNMENT, ALIGNMENT);
	dummy += posix_memalign(&B_T1_T2_T3  , ALIGNMENT, ALIGNMENT);
	dummy += posix_memalign(&B_DCC0_T1_T2, ALIGNMENT, ALIGNMENT);
	dummy += posix_memalign(&B_DCC1_T0_T3, ALIGNMENT, ALIGNMENT);
	dummy += posix_memalign(&C_0         , ALIGNMENT, ALIGNMENT);
	dummy += posix_memalign(&C_1         , ALIGNMENT, ALIGNMENT);

	if(dummy) {
		printf("Ambit initialization failed.");
		exit(-1);
	}
}

static unsigned *allocate_vector(size_t width) {
	unsigned *vector;
	if(posix_memalign((void *) &vector, ALIGNMENT, width)) {
		printf("Failed to allocate vector.\n");
		exit(-1);
	}
	return vector;
}

static unsigned **allocate_vector_array(size_t width, size_t height) {
	unsigned **vector_array;
	if(NULL == (vector_array = malloc(height * sizeof(unsigned *)))) {
		printf("Failed to allocate array of vectors.\n");
		exit(-1);
	}
	for(int i = 0; i < height; i ++) {
		vector_array[i] = allocate_vector(width);
	}
	return vector_array;
}

static unsigned *random_vector(size_t width) {
	unsigned *vector = allocate_vector(width);
	for (int i = 0; i < width/4; i ++) {
		vector[i] = rand();
	}
	return vector;
}

static unsigned **random_vector_array(size_t width, size_t height) {
	unsigned **vector_array = allocate_vector_array(width, height);
	for (int i = 0; i < height; i ++) {
		for (int j = 0; j < width/4; j ++) {
			vector_array[i][j] = rand();
		}
	}
	return vector_array;
}

#endif
