#include "F2MDParameters.h"

using namespace omnetpp;
namespace artery {
    F2MDParameters::F2MDParameters() = default;

    AttackParameters F2MDParameters::attackParameters = AttackParameters{};
    DetectionParameters F2MDParameters::detectionParameters = DetectionParameters{};
    ReportParameters F2MDParameters::reportParameters = ReportParameters{};
    MisbehaviorAuthorityParameters F2MDParameters::misbehaviorAuthorityParameters = MisbehaviorAuthorityParameters{};
    MiscParameters F2MDParameters::miscParameters = MiscParameters{};
}