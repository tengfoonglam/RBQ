#pragma once

#include <memory>
#include <string>
#include <functional>
#include <fstream>

#include <dds/dds.hpp>

namespace rbq_sdk {

inline void exportConfig(const std::string& interface = "lo") {
    return;
    std::string path = "cyclonedds.xml";
    std::ofstream xml(path);
    xml << R"(
<?xml version="1.0" encoding="utf-8" ?>
<CycloneDDS xmlns="https://cdds.io/config" xmlns:xsi="http://www.w3.org/2001/XMLSchema-instance" xsi:schemaLocation="https://cdds.io/config https://raw.githubusercontent.com/eclipse-cyclonedds/cyclonedds/master/etc/cyclonedds.xsd">
    <Domain Id="any">
        <General>
            <Interfaces>
                <NetworkInterface name=")" << interface << R"(" priority="default" multicast="default" />
            </Interfaces>
            <AllowMulticast>spdp</AllowMulticast>
        </General>
    </Domain>
</CycloneDDS>
)";
    char* abs = realpath(path.c_str(), nullptr);
    std::string uri = std::string("file://") + (abs ? abs : path);
    if (abs) free(abs);
    setenv("CYCLONEDDS_URI", uri.c_str(), 1);
    std::cout << "[DDS] Config set: " << uri << " interface=" << interface << std::endl;
}

inline static dds::sub::qos::DataReaderQos default_reader_qos() {
    dds::sub::qos::DataReaderQos qos;
    qos << dds::core::policy::Reliability::BestEffort()
        << dds::core::policy::History::KeepLast(1)
        << dds::core::policy::ResourceLimits(32, 32, 8);
    return qos;
}

inline static dds::pub::qos::DataWriterQos default_writer_qos() {
    dds::pub::qos::DataWriterQos qos;
    qos << dds::core::policy::Reliability::BestEffort()
        << dds::core::policy::History::KeepLast(1)
        << dds::core::policy::ResourceLimits(32,32,8);
    return qos;
}

} // namespace rbq_sdk
