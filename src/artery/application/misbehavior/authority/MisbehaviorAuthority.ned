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

        @signal[stationId_*_reportingStationId](type=int);
        @statisticTemplate[reportReportingStationIdByStationId](record=count,sum,mean,vector;checkSignals=false);

        @signal[stationId_*_reportValidity](type=int);
        @statisticTemplate[reportValidityByStationId](record=count,sum,mean,vector;checkSignals=false);

        @signal[stationId_*_reportScore](type=int);
        @statisticTemplate[reportScoreByStationId](record=count,sum,mean,vector;checkSignals=false);

        @signal[stationId_*_reportDetectionType](type=int);
        @statisticTemplate[reportDetectionTypeByStationId](record=count,vector,histogram;checkSignals=false);

        @signal[stationId_*_reportDetectionLevel](type=int);
        @statisticTemplate[reportDetectionLevelByStationId](record=count,vector,histogram;checkSignals=false);

        @signal[stationId_*_reportErrorCode](type=int);
        @statisticTemplate[reportErrorCodeByStationId](record=count,vector,histogram;checkSignals=false);

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