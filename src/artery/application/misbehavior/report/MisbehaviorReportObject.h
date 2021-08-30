//
// Created by bastian on 05.07.21.
//

#ifndef ARTERY_MISBEHAVIORREPORTOBJECT_H
#define ARTERY_MISBEHAVIORREPORTOBJECT_H


#include <omnetpp/cobject.h>
#include <vanetza/asn1/misbehavior_report.hpp>
#include <memory>

namespace artery
{

    class MisbehaviorReportObject : public omnetpp::cObject
    {
    public:
        MisbehaviorReportObject(const MisbehaviorReportObject&) = default;
        MisbehaviorReportObject& operator=(const MisbehaviorReportObject&) = default;

        MisbehaviorReportObject(vanetza::asn1::MisbehaviorReport&&);
        MisbehaviorReportObject& operator=(vanetza::asn1::MisbehaviorReport&&);

        MisbehaviorReportObject(const vanetza::asn1::MisbehaviorReport&);
        MisbehaviorReportObject& operator=(const vanetza::asn1::MisbehaviorReport&);

        MisbehaviorReportObject(const std::shared_ptr<const vanetza::asn1::MisbehaviorReport>&);
        MisbehaviorReportObject& operator=(const std::shared_ptr<const vanetza::asn1::MisbehaviorReport>&);

        const vanetza::asn1::MisbehaviorReport& asn1() const;

        std::shared_ptr<const vanetza::asn1::MisbehaviorReport> shared_ptr() const;

        omnetpp::cObject* dup() const override;

    private:
        std::shared_ptr<const vanetza::asn1::MisbehaviorReport> m_misbehavior_report_wrapper;
    };

} // namespace artery

#endif //ARTERY_MISBEHAVIORREPORTOBJECT_H
