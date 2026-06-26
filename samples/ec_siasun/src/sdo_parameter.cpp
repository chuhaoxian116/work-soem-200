#include "sdo_parameter.h"

#include <cmath>
#include <cstdio>

namespace siasun {
namespace {

/* SDO 写入三步协议中 mailbox word 的类型，仅用于日志区分。*/
enum class MailboxWordKind {
    Header,
    Value,
    Id,
};

/*
 * 将 mailbox word 类型转换为日志文本。
 *
 * kind：当前写入的 mailbox word 类型。
 */
const char *mailbox_word_kind_name(MailboxWordKind kind) {
    switch (kind) {
    case MailboxWordKind::Header:
        return "header";
    case MailboxWordKind::Value:
        return "value";
    case MailboxWordKind::Id:
        return "id";
    }
    return "unknown";
}

/*
 * 将 Axis*.xml 的 value/qFmt 编码为示例协议使用的 32-bit 参数字。
 *
 * value：Axis*.xml 中的参数值。
 * qfmt：Axis*.xml 中的定点缩放位数。
 */
uint32_t encode_parameter_value(double value, int qfmt) {
    /* sign：负数时置 bit16，和原 SINSUN 参数示例保持一致。*/
    const uint32_t sign = value < 0.0 ? 0x10000U : 0U;
    /* scaled：abs(value) * 2^qfmt 后的 16-bit 幅值。*/
    const auto scaled = static_cast<uint16_t>(
        std::fabs(value) * std::pow(2.0, static_cast<double>(qfmt)));
    return sign + scaled;
}

/*
 * 将 32-bit mailbox word 写到伺服 0x2020:00。
 *
 * context：SOEM 主站上下文。
 * slave：SOEM 1-based 从站编号。
 * word：要写入的 32-bit 参数字。
 * kind：当前写入 header/value/id 中的哪一步。
 * parameter：当前参数，仅用于日志。
 */
int download_mailbox_word(ecx_contextt *context,
                          uint16_t slave,
                          uint32_t word,
                          MailboxWordKind kind,
                          const ServoParameter &parameter) {
    if (kLogSdoDownloadDetails) {
        std::printf("[SDO] servo=%u param id=%d name=%s step=%s "
                    "download 0x2020:00 word=0x%08X\n",
                    slave,
                    parameter.id,
                    parameter.name.c_str(),
                    mailbox_word_kind_name(kind),
                    word);
    }

    int last_wkc = 0;
    for (int retry = 0; retry < 3; ++retry) {
        last_wkc = ecx_SDOwrite(context,
                                slave,
                                0x2020,
                                0x00,
                                FALSE,
                                sizeof(word),
                                &word,
                                EC_TIMEOUTRXM * 4);
        if (last_wkc > 0) {
            osal_usleep(10000);
            return 0;
        }
        osal_usleep(20000);
    }

    ecx_readstate(context);
    std::fprintf(stderr,
                 "SDO download failed: slave=%u state=0x%02X AL=0x%04X "
                 "word=0x%08X wkc=%d\n",
                 slave,
                 context->slavelist[slave].state,
                 context->slavelist[slave].ALstatuscode,
                 word,
                 last_wkc);
    return -1;
}

}  // namespace

int write_servo_parameter(ecx_contextt *context,
                          uint16_t slave,
                          const ServoParameter &parameter) {
    /* header_word：写参数 mailbox 的头字，来自 SINSUN 参数下发示例 step1。*/
    const uint32_t header_word =
        (0x02U << 24) | (0x01U << 16) | (0x63U << 8) | 0x01U;
    /* value_word：参数值数据字，按 value/qFmt 编码。*/
    const uint32_t value_word =
        encode_parameter_value(parameter.value, parameter.qfmt);
    /* id_word：参数 ID 数据字，来自 SINSUN 参数下发示例 step3。*/
    const uint32_t id_word = 0x10000U + static_cast<uint32_t>(parameter.id);

    if (download_mailbox_word(context, slave, header_word,
                              MailboxWordKind::Header, parameter) ||
        download_mailbox_word(context, slave, value_word,
                              MailboxWordKind::Value, parameter) ||
        download_mailbox_word(context, slave, id_word,
                              MailboxWordKind::Id, parameter)) {
        return -1;
    }

    return 0;
}

int write_axis_parameters(ecx_contextt *context,
                          const AxisParameterSet &parameters) {
    for (std::size_t axis = 0; axis < kServoCount; ++axis) {
        /* slave：SOEM 1-based 从站编号，axis 0-5 对应 slave 1-6。*/
        const uint16_t slave = static_cast<uint16_t>(axis + 1);
        std::printf("[SDO] servo %zu writing %zu XML parameters\n",
                    axis + 1,
                    parameters[axis].size());

        for (const auto &parameter : parameters[axis]) {
            if (write_servo_parameter(context, slave, parameter)) {
                std::fprintf(stderr,
                             "failed to write servo %zu parameter id=%d "
                             "name=%s value=%f qfmt=%d\n",
                             axis + 1,
                             parameter.id,
                             parameter.name.c_str(),
                             parameter.value,
                             parameter.qfmt);
                return -1;
            }
        }

        /* apply_parameter：id=2,value=1,qFmt=0，用于使能已下载参数。*/
        const ServoParameter apply_parameter{2, "tRequestParaFlag", 1.0, 0};
        std::printf("[SDO] servo %zu applying downloaded parameters\n",
                    axis + 1);
        if (write_servo_parameter(context, slave, apply_parameter)) {
            return -1;
        }
    }

    std::printf("[SDO] all servo SDO parameters written successfully\n");
    return 0;
}

}  // namespace siasun
