package artery.application.misbehavior.ma;

simple MisbehaviorAuthority
{
    parameters:
        bool test = default(true);
		string datetime;
        double maxReportAge @unit(s) = default(5.0s);
}