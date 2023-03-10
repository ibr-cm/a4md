//
// Created by bastian on 05.07.21.
//

#include "MisbehaviorReportObject.h"
#include <omnetpp.h>
#include <cassert>

namespace artery
{

    using namespace vanetza::asn1;

    Register_Abstract_Class(MisbehaviorReportObject)

    MisbehaviorReportObject::MisbehaviorReportObject(vanetza::asn1::MisbehaviorReport&& misbehavior_report) :
            m_misbehavior_report_wrapper(std::make_shared<vanetza::asn1::MisbehaviorReport>(std::move(misbehavior_report)))
    {
    }

    MisbehaviorReportObject& MisbehaviorReportObject::operator=(vanetza::asn1::MisbehaviorReport&& misbehavior_report)
    {
        m_misbehavior_report_wrapper = std::make_shared<vanetza::asn1::MisbehaviorReport>(std::move(misbehavior_report));
        return *this;
    }

    MisbehaviorReportObject::MisbehaviorReportObject(const vanetza::asn1::MisbehaviorReport& misbehavior_report) :
            m_misbehavior_report_wrapper(std::make_shared<vanetza::asn1::MisbehaviorReport>(misbehavior_report))
    {
    }

    MisbehaviorReportObject& MisbehaviorReportObject::operator=(const vanetza::asn1::MisbehaviorReport& misbehavior_report)
    {
        m_misbehavior_report_wrapper = std::make_shared<vanetza::asn1::MisbehaviorReport>(misbehavior_report);
        return *this;
    }

    MisbehaviorReportObject::MisbehaviorReportObject(const std::shared_ptr<const vanetza::asn1::MisbehaviorReport>& ptr) :
            m_misbehavior_report_wrapper(ptr)
    {
        assert(m_misbehavior_report_wrapper);
    }

    MisbehaviorReportObject& MisbehaviorReportObject::operator=(const std::shared_ptr<const vanetza::asn1::MisbehaviorReport>& ptr)
    {
        m_misbehavior_report_wrapper = ptr;
        assert(m_misbehavior_report_wrapper);
        return *this;
    }

    std::shared_ptr<const vanetza::asn1::MisbehaviorReport> MisbehaviorReportObject::shared_ptr() const
    {
        assert(m_misbehavior_report_wrapper);
        return m_misbehavior_report_wrapper;
    }

    const vanetza::asn1::MisbehaviorReport& MisbehaviorReportObject::asn1() const
    {
        return *m_misbehavior_report_wrapper;
    }

    omnetpp::cObject* MisbehaviorReportObject::dup() const
    {
        return new MisbehaviorReportObject { *this };
    }

} // namespace artery
