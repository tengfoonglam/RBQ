#pragma once

#include <memory>
#include <string>
#include <functional>
#include <fstream>

#include <dds/dds.hpp>

#include "Settings.hpp"

namespace rbq_sdk {

template <typename T>
class Publisher {
public:
    Publisher() {}
    Publisher(const std::string& topic_name,
              const std::string& interface,
              int dp_domain_id = 0,
              dds::pub::qos::DataWriterQos writer_qos = default_writer_qos())
    {
        exportConfig(interface);

        dp_    = std::make_shared<dds::domain::DomainParticipant>(dp_domain_id);
        pub_   = std::make_shared<dds::pub::Publisher>(*dp_);
        topic_ = std::make_shared<dds::topic::Topic<T>>(*dp_, topic_name);
        writer_= std::make_shared<dds::pub::DataWriter<T>>(*pub_, *topic_, writer_qos);
    }

    ~Publisher() = default;

    void write(const T& msg) {
        if(writer_)
            writer_->write(msg);
    }

    dds::domain::DomainParticipant& participant() { return *dp_; }
    dds::pub::Publisher&            publisher()   { return *pub_; }
    dds::topic::Topic<T>&           topic()       { return *topic_; }
    dds::pub::DataWriter<T>&        writer()      { return *writer_; }

private:
    std::shared_ptr<dds::domain::DomainParticipant> dp_;
    std::shared_ptr<dds::pub::Publisher>            pub_;
    std::shared_ptr<dds::topic::Topic<T>>           topic_;
    std::shared_ptr<dds::pub::DataWriter<T>>        writer_;
};

} // namespace rbq_sdk
