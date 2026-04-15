#include <m5op.h>

extern void gemv(void);

int main(int argc, char **argv) {
    m5_reset_stats(0, 0);   // 通知 gem5 开始统计
    gemv();
    m5_dump_stats(0, 0);    // 通知 gem5 结束统计并输出
    return 0;
}
