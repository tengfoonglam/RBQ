#pragma once

#include <dds/dds.hpp>

namespace rbq_sdk {

inline dds::sub::qos::DataReaderQos default_reader_qos() {
    dds::sub::qos::DataReaderQos qos;
    qos << dds::core::policy::Reliability::BestEffort()
        << dds::core::policy::History::KeepLast(1)
        << dds::core::policy::ResourceLimits(32, 32, 8);
    return qos;
}

inline dds::pub::qos::DataWriterQos default_writer_qos() {
    dds::pub::qos::DataWriterQos qos;
    qos << dds::core::policy::Reliability::BestEffort()
        << dds::core::policy::History::KeepLast(1)
        << dds::core::policy::ResourceLimits(32, 32, 8);
    return qos;
}

} // namespace rbq_sdk
