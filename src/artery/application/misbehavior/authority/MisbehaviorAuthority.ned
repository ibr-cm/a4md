package artery.application.misbehavior.authority;

simple MisbehaviorAuthority
{
    parameters:
        // ##### Statistics
        // ### Static MA stats
        @signal[totalReportCount](type=int);
        @statistic[totalReportCount](record=last,vector?);

        @signal[parsedReportCount](type=int);
        @statistic[parsedReportCount](record=last,vector?);

        @signal[validReportCount](type=int);
        @statistic[validReportCount](record=last,vector?);

        @signal[truePositiveCount](type=int);
        @statistic[truePositiveCount](record=last,vector?);

        @signal[falsePositiveCount](type=int);
        @statistic[falsePositiveCount](record=last,vector?);

        @signal[reportSize](type=int);
        @statistic[reportSize](record=min,max,mean,sum,vector?);

        // ### Stats per ReportedPseudonym per Report

        @signal[reportedPseudonym_*_reportingStationId](type=int);
        @statisticTemplate[reportedPseudonym_reportingPseudonym](record=count,vector;checkSignals=false);

        @signal[reportedPseudonym_*_reportValidity](type=int);
        @statisticTemplate[reportedPseudonym_reportValidity](record=count,sum,mean,vector;checkSignals=false);

        @signal[reportedPseudonym_*_reportScore](type=double);
        @statisticTemplate[reportedPseudonym_reportScore](record=count,sum,mean,vector;checkSignals=false);

        @signal[reportedPseudonym_*_reportDetectionType](type=int);
        @statisticTemplate[reportedPseudonym_reportDetectionType](record=count,vector,histogram;checkSignals=false);

        @signal[reportedPseudonym_*_reportDetectionLevel](type=int);
        @statisticTemplate[reportedPseudonym_reportDetectionLevel](record=count,vector,histogram;checkSignals=false);

        @signal[reportedPseudonym_*_reportErrorCode](type=int);
        @statisticTemplate[reportedPseudonym_reportErrorCode](record=count,vector,histogram;checkSignals=false);

        // ### Stats per ReportingPseudonym per Report

        @signal[reportingPseudonym_*_reportedPseudonym](type=int);
        @statisticTemplate[reportingPseudonym_reportedPseudonym](record=count,vector;checkSignals=false);

        @signal[reportingPseudonym_*_reportValidity](type=int);
        @statisticTemplate[reportingPseudonym_reportValidity](record=count,sum,mean,vector;checkSignals=false);

        @signal[reportingPseudonym_*_reportScore](type=double);
        @statisticTemplate[reportingPseudonym_reportScore](record=count,sum,mean,vector;checkSignals=false);

        @signal[reportingPseudonym_*_reportDetectionType](type=int);
        @statisticTemplate[reportingPseudonym_reportDetectionType](record=count,vector,histogram;checkSignals=false);

        @signal[reportingPseudonym_*_reportDetectionLevel](type=int);
        @statisticTemplate[reportingPseudonym_reportDetectionLevel](record=count,vector,histogram;checkSignals=false);

        @signal[reportingPseudonym_*_reportErrorCode](type=int);
        @statisticTemplate[reportingPseudonym_reportErrorCode](record=count,vector,histogram;checkSignals=false);

        @signal[reportingPseudonym_*_averageReportScore](type=double);
        @statisticTemplate[reportingPseudonym_averageReportScore](record=count,vector;checkSignals=false);

        // ###### Parameters
        bool test = default(true);
		string datetime;
        double maxReportAge @unit(s) = default(5.0s);
        int reportCountThreshold = default(5);
        int checkType = default(0);
        double misbehaviorThreshold = default(0.5);

        // Report storage parameters
        double reportCleanupInterval @unit(s) = default(10s);
        double reportCleanupAge @unit(s) = default(1s);
        double reportCamRetentionTime @unit(s) = default(10s);
        double reportCamRetentionCleanupInterval @unit(s) = default(10s);

        // Report scoring parameters
        bool considerReportAge = default(true);
        bool considerReportValidity = default(true);
        bool considerReporterScore = default(true);
        bool considerEvidenceScore = default(true);
        int evidenceScoreMaxCamCount = default(5);

        // Web Ui parameters
        bool enableWebGui = default(false);
        string webGuiDataUrl = default("http://localhost:9981/updateData");
        double guiJsonDataUpdateInterval @unit(s) = default(10s);
        double updateTimeStep @unit(s) = default(1s);
        int displaySteps = default(40);
        int recentReportedCount = default(10);
}