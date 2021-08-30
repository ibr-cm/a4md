#include <utility>

//
// Created by bastian on 20.08.21.
//

#ifndef ARTERY_REPORTSUMMARY_H
#define ARTERY_REPORTSUMMARY_H

namespace artery {

    class ReportedPseudonym;
    namespace ma {
        struct ReportSummary {
            std::string id;
            double score;
            std::shared_ptr <ReportedPseudonym> reportedPseudonym;

            ReportSummary(std::string id, double score,
                          std::shared_ptr <ReportedPseudonym> reportedPseudonym)
                    : id(std::move(id)),
                      score(score),
                      reportedPseudonym(std::move(reportedPseudonym)) {};
        };

    } // namespace ma
} // namespace artery

#endif //ARTERY_REPORTSUMMARY_H
