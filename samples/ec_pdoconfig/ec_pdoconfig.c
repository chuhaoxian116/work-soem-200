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

// SDO读取辅助函数
int sdo_read(ecx_contextt *ctx, uint16_t slave, uint16_t index, uint8_t subindex, void *data, int *size) {
    int wkc = ecx_SDOread(ctx, slave, index, subindex, FALSE, size, data, EC_TIMEOUTRXM);
    if (wkc <= 0) {
        printf("ERROR: SDO read failed! Index: 0x%04X:%02X, WKC: %d\n", 
               index, subindex, wkc);
        return -1;
    }
    return 0;
}

int main(int argc, char *argv[])
{
    char *ifname = "enp1s0";
    int slave = 1;
    uint8_t zero = 0;
    uint8_t one = 1;
    uint32_t mapping;
    uint16_t pdo_index;
    int size;

    signal(SIGINT, signal_handler);

    if (ecx_init(&ctx, ifname))
    {
        printf("ecx_init on %s succeeded.\n", ifname);

        int wkc = ecx_config_init(&ctx);
        if (wkc > 0)
        {
            printf("%d slaves found and configured.\n", ctx.slavecount);
            printf("Slave %d: %s\n", slave, ctx.slavelist[slave-1].name);
            printf("Vendor: 0x%08X, Product: 0x%08X, Revision: 0x%08X\n",
                   ctx.slavelist[slave-1].eep_man,
                   ctx.slavelist[slave-1].eep_id,
                   ctx.slavelist[slave-1].eep_rev);

            // 关键：确保从站处于PRE-OP状态才能配置PDO
            ecx_statecheck(&ctx, slave, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
            printf("Slave %d entered PRE-OP state\n", slave);

            // --- 配置最小PDO映射 ---
            // RxPDO 0x1600 -> 607A TargetPosition
            printf("\nConfiguring RxPDO 0x1600...\n");
           // 1. 先清空RxPDO映射表
            sdo_write(&ctx, slave, 0x1600, 0x00, &zero, sizeof(uint8_t));
            
            // 2. 设置映射条目
            mapping = 0x607A0020; // Index:0x607A, Subindex:0x00, BitLength:32
            sdo_write(&ctx, slave, 0x1600, 0x01, &mapping, sizeof(uint32_t));
            
            // 3. 启用RxPDO映射表（设置条目数）
            sdo_write(&ctx, slave, 0x1600, 0x00, &one, sizeof(uint8_t));
            
            // 4. 清空RxPDO分配表（0x1C12）
            sdo_write(&ctx, slave, 0x1C12, 0x00, &zero, sizeof(uint8_t));
            
            // 5. 将RxPDO 0x1600分配给同步管理器
            pdo_index = 0x1600;
            sdo_write(&ctx, slave, 0x1C12, 0x01, &pdo_index, sizeof(uint16_t));
            
            // 6. 启用RxPDO分配表
            sdo_write(&ctx, slave, 0x1C12, 0x00, &one, sizeof(uint8_t));

            // --- 配置最小PDO映射 ---
            // TxPDO 0x1A00 -> 6064 ActualPosition
            printf("Configuring TxPDO 0x1A00...\n");
            // 1. 先清空TxPDO映射表
            sdo_write(&ctx, slave, 0x1A00, 0x00, &zero, sizeof(uint8_t));
            
            // 2. 设置映射条目
            mapping = 0x60640020; // Index:0x6064, Subindex:0x00, BitLength:32
            sdo_write(&ctx, slave, 0x1A00, 0x01, &mapping, sizeof(uint32_t));
            
            // 3. 启用TxPDO映射表
            sdo_write(&ctx, slave, 0x1A00, 0x00, &one, sizeof(uint8_t));
            
            // 4. 清空TxPDO分配表（0x1C13）
            sdo_write(&ctx, slave, 0x1C13, 0x00, &zero, sizeof(uint8_t));
            
            // 5. 将TxPDO 0x1A00分配给同步管理器
            pdo_index = 0x1A00;
            sdo_write(&ctx, slave, 0x1C13, 0x01, &pdo_index, sizeof(uint16_t));
            
            // 6. 启用TxPDO分配表
            sdo_write(&ctx, slave, 0x1C13, 0x00, &one, sizeof(uint8_t));

            // --- 映射PDO到IOmap ---
            memset(IOmap, 0, sizeof(IOmap));
            ecx_config_map_group(&ctx, IOmap, 0);

            printf("\nPDO Mapping Information:\n");
            printf("Slave %d outputs offset: %d bytes, outputs length: %d bytes\n",
                   slave, ctx.slavelist[slave-1].Ooffset,
                   ctx.slavelist[slave-1].Obytes);
            printf("Slave %d inputs offset: %d bytes, inputs length: %d bytes\n",
                   slave, ctx.slavelist[slave-1].Ioffset,
                   ctx.slavelist[slave-1].Ibytes);

            // 配置分布式时钟（如果从站支持）
            if (ctx.slavelist[slave-1].hasdc) {
                ecx_configdc(&ctx);
                printf("Distributed clock configured\n");
            }

            // 进入OP状态
            printf("\nSwitching to OPERATIONAL state...\n");
            ctx.slavelist[slave].state = EC_STATE_OPERATIONAL;
            ecx_writestate(&ctx, slave);
            wkc = ecx_statecheck(&ctx, slave, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE * 4);
            if (wkc != EC_STATE_OPERATIONAL) {
                printf("WARNING: Failed to enter OP state, current state: 0x%02X\n", 
                       ctx.slavelist[slave].state);
            } else {
                printf("Successfully entered OPERATIONAL state\n");
            }

            printf("\nPDO map for TargetPosition and ActualPosition set.\n");

            int32_t *target_pos = (int32_t*)ctx.slavelist[slave-1].outputs;
            int32_t *actual_pos = (int32_t*)ctx.slavelist[slave-1].inputs;

            while(run)
            {
                ecx_send_processdata(&ctx);
                wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRXM);
                
                if (wkc > 0) {
                    // 读实际位置
                    printf("ActualPosition=%d\n wkc: %d", *actual_pos, wkc);
                } else {
                    printf("Processdata timeout, WKC: %d\n", wkc);
                }

                usleep(1000); // 1ms周期
            }
        }
        else
        {
            printf("No slaves found!\n");
        }

        // 安全关闭：回到SAFE-OP状态
        ctx.slavelist[slave].state = EC_STATE_SAFE_OP;
        ecx_writestate(&ctx, slave);
        ecx_statecheck(&ctx, slave, EC_STATE_SAFE_OP, EC_TIMEOUTSTATE);
        ecx_close(&ctx);
    }
    else
    {
        printf("No socket connection on %s\n", ifname);
    }

    return 0;
}