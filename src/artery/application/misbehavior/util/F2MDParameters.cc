#include "F2MDParameters.h"

using namespace omnetpp;
namespace artery {
    F2MDParameters::F2MDParameters() = default;

    AttackParameters F2MDParameters::attackParameters = AttackParameters{};
    DetectionParameters F2MDParameters::detectionParameters = DetectionParameters{};
    MiscParameters F2MDParameters::miscParameters = MiscParameters{};
    ReportParameters F2MDParameters::reportParameters = ReportParameters{};
}