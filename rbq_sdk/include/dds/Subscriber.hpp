#pragma once

#include <memory>
#include <string>
#include <functional>
#include <fstream>

#include <dds/dds.hpp>

#include "Settings.hpp"

namespace rbq_sdk {

template <typename T>
class Subscriber {
    struct Listener : dds::sub::DataReaderListener<T> {
        T* target_{nullptr};
        std::function<void(const T&)> on_sample_;

        Listener(T* t, std::function<void(const T&)> cb)
            : target_(t), on_sample_(std::move(cb)) {}

        void on_data_available(dds::sub::DataReader<T>& r) override {
            auto samples = r.take();
            for (const auto& s : samples) {
                if (!s.info().valid()) continue;
                const auto& m = s.data();
                if (target_) *target_ = m;
                if (on_sample_) on_sample_(m);
            }
        }

        void on_requested_deadline_missed(dds::sub::DataReader<T>&,
                                          const dds::core::status::RequestedDeadlineMissedStatus&) override {}
        void on_requested_incompatible_qos(dds::sub::DataReader<T>&,
                                           const dds::core::status::RequestedIncompatibleQosStatus&) override {}
        void on_sample_rejected(dds::sub::DataReader<T>&,
                                const dds::core::status::SampleRejectedStatus&) override {}
        void on_liveliness_changed(dds::sub::DataReader<T>&,
                                   const dds::core::status::LivelinessChangedStatus&) override {}
        void on_subscription_matched(dds::sub::DataReader<T>&,
                                     const dds::core::status::SubscriptionMatchedStatus&) override {}
        void on_sample_lost(dds::sub::DataReader<T>&,
                            const dds::core::status::SampleLostStatus&) override {}
    };

public:
    Subscriber() {}
    Subscriber(
        T* target_ptr,
        const std::string& topic_name,
        const std::string& interface,
        int dp_domain_id = 0,
        dds::sub::qos::DataReaderQos reader_qos = default_reader_qos(),
        std::function<void(const T&)> on_sample = {})
    {
        exportConfig(interface);

        dp_ = std::make_shared<dds::domain::DomainParticipant>(dp_domain_id);
        sub_ = std::make_shared<dds::sub::Subscriber>(*dp_);
        topic_ = std::make_shared<dds::topic::Topic<T>>(*dp_, topic_name);
        listener_ = std::make_shared<Listener>(target_ptr, std::move(on_sample));
        reader_ = std::make_shared<dds::sub::DataReader<T>>(
            *sub_, *topic_, reader_qos, listener_.get(),
            dds::core::status::StatusMask::data_available());
    }
    Subscriber(
        std::function<void(const T&)> on_sample,
        const std::string& topic_name,
        const std::string& interface,
        int dp_domain_id = 0,
        dds::sub::qos::DataReaderQos reader_qos = default_reader_qos())
    {
        exportConfig(interface);

        dp_ = std::make_shared<dds::domain::DomainParticipant>(dp_domain_id);
        sub_ = std::make_shared<dds::sub::Subscriber>(*dp_);
        topic_ = std::make_shared<dds::topic::Topic<T>>(*dp_, topic_name);
        listener_ = std::make_shared<Listener>(nullptr, std::move(on_sample));
        reader_ = std::make_shared<dds::sub::DataReader<T>>(
            *sub_, *topic_, reader_qos, listener_.get(),
            dds::core::status::StatusMask::data_available());
    }

    ~Subscriber() = default;

    void set_target(T* target_ptr) { listener_->target_ = target_ptr; }

    dds::domain::DomainParticipant& participant() { return *dp_; }
    dds::sub::Subscriber&           subscriber()  { return *sub_; }
    dds::topic::Topic<T>&           topic()       { return *topic_; }
    dds::sub::DataReader<T>&        reader()      { return *reader_; }

private:
    std::shared_ptr<dds::domain::DomainParticipant> dp_;
    std::shared_ptr<dds::sub::Subscriber>           sub_;
    std::shared_ptr<dds::topic::Topic<T>>           topic_;
    std::shared_ptr<Listener>                       listener_;
    std::shared_ptr<dds::sub::DataReader<T>>        reader_;
};

} // namespace rbq_sdk
