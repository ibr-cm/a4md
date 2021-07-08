//
// Artery V2X Simulation Framework
// Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
//

package artery.application;

// Middleware for ITS G5 services
simple VehicleMiddleware extends Middleware
{
	parameters:
		@class(VehicleMiddleware);
		@signal[IdentityChanged](type=long);

		string localEnvironmentModule = default("");
		string globalEnvironmentModule = default("");
		string mobilityModule;


        // 0..4093 cm, outOfRange: 4094, unavailable: 4095
        volatile double semiMajorConfidence = default(chi_square(2) * 50);

        // 0..4093 cm, outOfRange: 4094, unavailable: 4095
        volatile double semiMinorConfidence = default(chi_square(2) * 50);

        // 0..13, outOfRange: 14, unavailable: 15
        volatile double altitudeConfidence = default(15);

        // 0..125 deciDegree, outOfRange: 126, unavailable: 127
        volatile double headingConfidence = default(chi_square(2) * 13);

        // 0..125 cm/s, outOfRange: 126, unavailable: 127
        volatile double speedConfidence = default(chi_square(2) * 13);

        // 0..100 dm/s^2, outOfRange: 101, unavailable: 102
        volatile double longitudinalAccelerationConfidence = default(chi_square(2) * 6);
}
