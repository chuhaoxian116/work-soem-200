#include "sdo_parameter.h"

#include <cmath>
#include <cstdio>

namespace siasun {
namespace {

enum class MailboxWordKind {
    Header,
    Value,
    Id,
};

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

uint32_t encode_parameter_value(double value, int qfmt) {
    const uint32_t sign = value < 0.0 ? 0x10000U : 0U;
    const auto scaled = static_cast<uint16_t>(
        std::fabs(value) * std::pow(2.0, static_cast<double>(qfmt)));
    return sign + scaled;
}

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

    const int wkc = ecx_SDOwrite(context,
                                 slave,
                                 0x2020,
                                 0x00,
                                 FALSE,
                                 sizeof(word),
                                 &word,
                                 EC_TIMEOUTRXM);
    if (wkc <= 0) {
        std::fprintf(stderr,
                     "SDO download failed: slave=%u word=0x%08X wkc=%d\n",
                     slave,
                     word,
                     wkc);
        return -1;
    }

    osal_usleep(10000);
    return 0;
}

}  // namespace

int write_servo_parameter(ecx_contextt *context,
                          uint16_t slave,
                          const ServoParameter &parameter) {
    const uint32_t header_word =
        (0x02U << 24) | (0x01U << 16) | (0x63U << 8) | 0x01U;
    const uint32_t value_word =
        encode_parameter_value(parameter.value, parameter.qfmt);
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
