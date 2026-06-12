#include <stdio.h>
#include <stdint.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include "soem/soem.h"


static int run = 1;
static ecx_contextt ctx;
static uint8 IOmap[4096];

void signal_handler(int sig)
{
    run = 0;
}

// SDO写入辅助函数（带错误检查）
int sdo_write(ecx_contextt *ctx, uint16_t slave, uint16_t index, uint8_t subindex, void *data, int size) {
    int wkc = ecx_SDOwrite(ctx, slave, index, subindex, FALSE, size, data, EC_TIMEOUTRXM);
    if (wkc <= 0) {
        printf("ERROR: SDO write failed! Index: 0x%04X:%02X, WKC: %d\n", index, subindex, wkc);
        return -1;
    }
    usleep(10000); // 给从站一点时间处理
    return 0;
}

int main(int argc, char *argv[])
{
    char *ifname = "enp1s0";
    int slave = 1;

    signal(SIGINT, signal_handler);

    if (ecx_init(&ctx, ifname))
    {
        printf("ecx_init on %s succeeded.\n", ifname);

        int wkc = ecx_config_init(&ctx);
        if (wkc > 0)
        {
            printf("%d slaves found and configured.\n", ctx.slavecount);

            // --- 配置最小PDO映射 ---
            // RxPDO 0x1600 -> 607A TargetPosition
            uint8_t num_pdo = 1;
            uint32_t mapping = 0x607A0020; // 32bit
            sdo_write(&ctx, slave, 0x1C12, 0x00, &num_pdo, sizeof(uint8_t));
            sdo_write(&ctx, slave, 0x1600, 0x01, &mapping, sizeof(uint32_t));
            sdo_write(&ctx, slave, 0x1C12, 0x01, (void*)&((uint16_t){0x1600}), sizeof(uint16_t));

            // TxPDO 0x1A00 -> 6064 ActualPosition
            mapping = 0x60640020; // 32bit
            sdo_write(&ctx, slave, 0x1C13, 0x00, &num_pdo, sizeof(uint8_t));
            sdo_write(&ctx, slave, 0x1A00, 0x01, &mapping, sizeof(uint32_t));
            sdo_write(&ctx, slave, 0x1C13, 0x01, (void*)&((uint16_t){0x1A00}), sizeof(uint16_t));

            // --- 映射PDO到IOmap ---
            memset(IOmap, 0, sizeof(IOmap));
            ecx_config_map_group(&ctx, IOmap, 0);
            ecx_configdc(&ctx);

            // 进入OP
            ecx_statecheck(&ctx, slave, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);

            printf("PDO map for TargetPosition and ActualPosition set.\n");

            int32_t *target_pos = (int32_t*)ctx.slavelist[slave-1].outputs;
            int32_t *actual_pos = (int32_t*)ctx.slavelist[slave-1].inputs;

            while(run)
            {
                ecx_send_processdata(&ctx);
                ecx_receive_processdata(&ctx, EC_TIMEOUTRXM);

                // 写目标位置
                //*target_pos = 1000;

                // 读实际位置
                printf("ActualPosition=%d\n", *actual_pos);

                usleep(1000); // 1ms周期
            }
        }
        else
        {
            printf("No slaves found!\n");
        }

        ecx_close(&ctx);
    }
    else
    {
        printf("No socket connection on %s\n", ifname);
    }

    return 0;
}