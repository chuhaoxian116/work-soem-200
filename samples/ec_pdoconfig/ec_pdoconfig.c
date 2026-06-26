#include <stdio.h>
#include <stdint.h>
#include <signal.h>
#include <unistd.h>
#include <string.h>
#include "soem/soem.h"


static int run = 1;
static ecx_contextt ctx;
static uint8 IOmap[4096];
static uint64_t count = 0;

// RxPDO结构体（主站→从站）：必须与映射顺序完全一致
typedef struct {
    uint16_t control_word;        // 0x6040:00h 控制字 UNSIGNED16
    int8_t operation_mode;        // 0x6060:00h 操作模式 INTEGER8
    int32_t target_position;      // 0x607A:00h 目标位置 INTEGER32
} __attribute__((packed)) RxPDO_t;

// TxPDO结构体（从站→主站）：必须与映射顺序完全一致
typedef struct {
    uint16_t error_code;          // 0x603F:00h 错误码 UNSIGNED16
    uint16_t status_word;         // 0x6041:00h 状态字 UNSIGNED16
    int8_t operation_mode_display;// 0x6061:00h 操作模式显示 INTEGER8
    int32_t actual_position;      // 0x6064:00h 位置反馈 INTEGER32
} __attribute__((packed)) TxPDO_t;

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
    uint8_t rx_entries = 3; // RxPDO条目数
    uint8_t tx_entries = 4; // TxPDO条目数
    int size;

    signal(SIGINT, signal_handler);

    if (ecx_init(&ctx, ifname))
    {
        printf("ecx_init on %s succeeded.\n", ifname);

        int wkc = ecx_config_init(&ctx);
        if (wkc > 0)
        {
            printf("%d slaves found and configured.\n", ctx.slavecount);
            printf("Slave %d: %s\n", slave, ctx.slavelist[slave].name);
            printf("Vendor: 0x%08X, Product: 0x%08X, Revision: 0x%08X\n",
                   ctx.slavelist[slave].eep_man,
                   ctx.slavelist[slave].eep_id,
                   ctx.slavelist[slave].eep_rev);

            // 关键：确保从站处于PRE-OP状态才能配置PDO
            ecx_statecheck(&ctx, slave, EC_STATE_PRE_OP, EC_TIMEOUTSTATE);
            printf("Slave %d entered PRE-OP state\n", slave);

            // --- 配置最小PDO映射 ---
            // RxPDO 0x1600 -> 607A TargetPosition
            printf("\nConfiguring RxPDO 0x1600...\n");
           // 1. 先清空RxPDO映射表
            sdo_write(&ctx, slave, 0x1600, 0x00, &zero, sizeof(uint8_t));
            
            // 2. 设置映射条目（顺序必须与RxPDO_t结构体一致）
            mapping = 0x60400010; // 控制字: Index=0x6040, Subindex=0x00, BitLength=16
            sdo_write(&ctx, slave, 0x1600, 0x01, &mapping, sizeof(uint32_t));
            
            mapping = 0x60600008; // 操作模式: Index=0x6060, Subindex=0x00, BitLength=8
            sdo_write(&ctx, slave, 0x1600, 0x02, &mapping, sizeof(uint32_t));
            
            mapping = 0x607A0020; // 目标位置: Index=0x607A, Subindex=0x00, BitLength=32
            sdo_write(&ctx, slave, 0x1600, 0x03, &mapping, sizeof(uint32_t));
            
            // 3. 启用RxPDO映射表（设置条目数）
            sdo_write(&ctx, slave, 0x1600, 0x00, &rx_entries, sizeof(uint8_t));
            
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
            
            // 2. 设置映射条目（顺序必须与TxPDO_t结构体一致）
            mapping = 0x603F0010; // 错误码: Index=0x603F, Subindex=0x00, BitLength=16
            sdo_write(&ctx, slave, 0x1A00, 0x01, &mapping, sizeof(uint32_t));
            
            mapping = 0x60410010; // 状态字: Index=0x6041, Subindex=0x00, BitLength=16
            sdo_write(&ctx, slave, 0x1A00, 0x02, &mapping, sizeof(uint32_t));
            
            mapping = 0x60610008; // 操作模式显示: Index=0x6061, Subindex=0x00, BitLength=8
            sdo_write(&ctx, slave, 0x1A00, 0x03, &mapping, sizeof(uint32_t));
            
            mapping = 0x60640020; // 位置反馈: Index=0x6064, Subindex=0x00, BitLength=32
            sdo_write(&ctx, slave, 0x1A00, 0x04, &mapping, sizeof(uint32_t));
            
            // 3. 启用TxPDO映射表
            sdo_write(&ctx, slave, 0x1A00, 0x00, &tx_entries, sizeof(uint8_t));
            
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
                   slave, ctx.slavelist[slave].Ooffset,
                   ctx.slavelist[slave].Obytes);
            printf("Slave %d inputs offset: %d bytes, inputs length: %d bytes\n",
                   slave, ctx.slavelist[slave].Ioffset,
                   ctx.slavelist[slave].Ibytes);

            // 验证PDO大小是否匹配
            if (ctx.slavelist[slave].Obytes != sizeof(RxPDO_t)) {
                printf("WARNING: RxPDO size mismatch! Expected %zu bytes, got %d bytes\n",
                       sizeof(RxPDO_t), ctx.slavelist[slave].Obytes);
            }
            if (ctx.slavelist[slave].Ibytes != sizeof(TxPDO_t)) {
                printf("WARNING: TxPDO size mismatch! Expected %zu bytes, got %d bytes\n",
                       sizeof(TxPDO_t), ctx.slavelist[slave].Ibytes);
            }

            // 配置分布式时钟（如果从站支持）
            if (ctx.slavelist[slave].hasdc) {
                ecx_configdc(&ctx);
                printf("Distributed clock configured\n");
            }

            // 进入OP状态
            printf("\nSwitching to OPERATIONAL state...\n");
            ctx.slavelist[slave].state = EC_STATE_OPERATIONAL;
            ecx_writestate(&ctx, slave);
            wkc = ecx_statecheck(&ctx, slave, EC_STATE_OPERATIONAL, EC_TIMEOUTSTATE);
            if (wkc != EC_STATE_OPERATIONAL) {
                printf("WARNING: Failed to enter OP state, wkc:%d current state: 0x%02X StatusCode=0x%4.4x : %s\n", 
                       wkc, ctx.slavelist[slave].state, ctx.slavelist[slave].ALstatuscode, ec_ALstatuscode2string(ctx.slavelist[slave].ALstatuscode));
            } else {
                printf("Successfully entered OPERATIONAL state\n");
            }

            printf("\nPDO map for TargetPosition and ActualPosition set.\n");

            RxPDO_t *rx_pdo = (RxPDO_t*)ctx.slavelist[slave].outputs;
            TxPDO_t *tx_pdo = (TxPDO_t*)ctx.slavelist[slave].inputs;

            // 初始化控制字为0（安全状态）
            rx_pdo->control_word = 0x0000;
            // 设置操作模式为位置模式（0x08，与你当前值一致）
            rx_pdo->operation_mode = 0x08;

            while(run)
            {
                ecx_send_processdata(&ctx);
                wkc = ecx_receive_processdata(&ctx, EC_TIMEOUTRXM);
                
                if (wkc > 0) {
                    // 每5000个周期打印一次PDO数据
                    if(count % 5000 == 0) { 
                        // 打印所有PDO数据
                        printf("Error Code: 0x%04X | Status Word: 0x%04X | Op Mode Display: %d | Actual Position: %d | WKC: %d | ctx State: 0x%02X\n",
                                tx_pdo->error_code,
                                tx_pdo->status_word,
                                tx_pdo->operation_mode_display,
                                tx_pdo->actual_position,
                                wkc,
                                ctx.slavelist[slave].state);
                        if(ctx.slavelist[slave].state != EC_STATE_OPERATIONAL)
                        {
                            printf("Slave %d is not in OPERATIONAL state\n", slave);
                            ctx.slavelist[slave].state = EC_STATE_OPERATIONAL;
                            ecx_writestate(&ctx, slave);
                        }
                    }
                    
                } else {
                    printf("Processdata timeout, WKC: %d\n", wkc);
                }
                count++;
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