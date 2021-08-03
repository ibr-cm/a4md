package artery.application.misbehavior.ma;

simple MisbehaviorAuthority
{
    parameters:
        bool test = default(true);
		string datetime;
        double maxReportAge @unit(s) = default(5.0s);
        int reportCountThreshold = default(5);

        double updateTimeStep @unit(s) = default(1s);
        int displaySteps = default(40);
        int recentReportedCount = default(10);
}