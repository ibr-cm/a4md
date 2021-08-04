package artery.application.misbehavior.ma;

simple MisbehaviorAuthority
{
    parameters:
        bool test = default(true);
		string datetime;
        double maxReportAge @unit(s) = default(5.0s);
        int reportCountThreshold = default(5);

        bool enableWebGui = default(false);
        string webGuiDataUrl = default("http://localhost:9981/updateData");
        double guiJsonDataUpdateInterval @unit(s) = default(10s);
        double updateTimeStep @unit(s) = default(1s);
        int displaySteps = default(40);
        int recentReportedCount = default(10);
}