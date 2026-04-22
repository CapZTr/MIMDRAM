
#define TWO_BYTE_OP(name, number)         \
        .globl name;                      \
        .func name;                       \
name:                                     \
        .byte 0x0F, 0x04;                 \
        .word number;                     \
        ret;                              \
        .endfunc;

TWO_BYTE_OP(rowop_and,    0x5c)
TWO_BYTE_OP(rowop_or,     0x5d)
TWO_BYTE_OP(rowop_not,    0x5e)
TWO_BYTE_OP(rowop_ap,     0x60)
TWO_BYTE_OP(rowop_aap,    0x61)
TWO_BYTE_OP(rowop_copy,   0x62)
TWO_BYTE_OP(rowop_anap,       0x63)
TWO_BYTE_OP(rowop_aaap,       0x64)
TWO_BYTE_OP(rowop_aaaaap,     0x65)
/* PUD COTS DDR4 primitives (PULSAR / FCDRAM) */
TWO_BYTE_OP(rowop_rowclone,   0x66)
TWO_BYTE_OP(rowop_mrc,        0x67)
TWO_BYTE_OP(rowop_maj,        0x68)
TWO_BYTE_OP(rowop_bulk_write, 0x69)
TWO_BYTE_OP(rowop_not_xsub,   0x6a)
TWO_BYTE_OP(rowop_and_xsub,   0x6b)
TWO_BYTE_OP(rowop_or_xsub,    0x6c)
TWO_BYTE_OP(rowop_frac,       0x6d)
