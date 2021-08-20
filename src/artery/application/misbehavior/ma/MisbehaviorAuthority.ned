package artery.application.misbehavior.ma;

simple MisbehaviorAuthority
{
    parameters:
        bool test = default(true);
		string datetime;
        double maxReportAge @unit(s) = default(5.0s);
        int reportCountThreshold = default(5);
        int checkType = default(0);
        double misbehaviorThreshold = default(0.5);
        double reportCleanupInterval @unit(s) = default(1s);
        double reportCleanupAge @unit(s) = default(1s);
        double reportCamRetentionTime @unit(s) = default(10s);
        double reportCamRetentionCleanupInterval @unit(s) = default(10s);

        bool enableWebGui = default(false);
        string webGuiDataUrl = default("http://localhost:9981/updateData");
        double guiJsonDataUpdateInterval @unit(s) = default(10s);
        double updateTimeStep @unit(s) = default(1s);
        int displaySteps = default(40);
        int recentReportedCount = default(10);
}