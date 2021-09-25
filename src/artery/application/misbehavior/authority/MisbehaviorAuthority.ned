package artery.application.misbehavior.authority;

simple MisbehaviorAuthority
{
    parameters:
		string datetime;
        double maxReportAge @unit(s) = default(5.0s);
        int reportCountThreshold = default(5);
        int checkType = default(0);
        double misbehaviorThreshold = default(0.5);

        // Report storage parameters
        double reportCleanupInterval @unit(s) = default(1s);
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

        bool enableVectorRecording = default(false);
}