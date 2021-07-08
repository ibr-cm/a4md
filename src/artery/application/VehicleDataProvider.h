//
// Copyright (C) 2014 Raphael Riebl <raphael.riebl@thi.de>
//
// This program is free software; you can redistribute it and/or modify
// it under the terms of the GNU General Public License as published by
// the Free Software Foundation; either version 2 of the License, or
// (at your option) any later version.
//
// This program is distributed in the hope that it will be useful,
// but WITHOUT ANY WARRANTY; without even the implied warranty of
// MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
// GNU General Public License for more details.
//
// You should have received a copy of the GNU General Public License
// along with this program; if not, write to the Free Software
// Foundation, Inc., 59 Temple Place, Suite 330, Boston, MA  02111-1307  USA
//

#ifndef ARTERY_VEHICLEDATAPROVIDER_H_
#define ARTERY_VEHICLEDATAPROVIDER_H_

#include "artery/application/VehicleKinematics.h"
#include "artery/utility/Geometry.h"
#include <omnetpp/simtime.h>
#include <omnetpp/crng.h>
#include <boost/circular_buffer.hpp>
#include <boost/units/systems/si/angular_acceleration.hpp>
#include <vanetza/geonet/station_type.hpp>
#include <vanetza/units/acceleration.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/velocity.hpp>
#include <vanetza/units/angular_velocity.hpp>
#include <vanetza/units/curvature.hpp>
#include <vanetza/asn1/its/ReferencePosition.h>
#include <vanetza/asn1/its/Heading.h>
#include <vanetza/asn1/its/Speed.h>
#include <vanetza/asn1/its/LongitudinalAcceleration.h>
#include <traci/API.h>
#include <traci/Boundary.h>
#include <cstdint>
#include <map>

namespace artery
{

class VehicleDataProvider
{
	public:
		using StationType = vanetza::geonet::StationType;

		VehicleDataProvider(uint32_t id);

		// prevent inadvertent VDP copies
		VehicleDataProvider(const VehicleDataProvider&) = delete;
		VehicleDataProvider& operator=(const VehicleDataProvider&) = delete;

		void update(const VehicleKinematics&);
		void updateApproximate(const VehicleKinematics &dynamics, omnetpp::cRNG *rng, const std::shared_ptr<const traci::API>& traciAPI, const traci::Boundary &simulationBoundary);
		omnetpp::SimTime updated() const { return mLastUpdate; }

		const ReferencePosition_t& approximateReferencePosition() const { return mReferencePosition; };
		const Heading_t& approximateHeading() const { return mHeading; };
		const Speed_t& approximateSpeed() const { return mSpeed; };
		const LongitudinalAcceleration_t& approximateAcceleration() const { return mLongitudinalAcceleration; };
		const Position& position() const { return mVehicleKinematics.position; }
		vanetza::units::GeoAngle longitude() const { return mVehicleKinematics.geo_position.longitude; } // positive for east
		vanetza::units::GeoAngle latitude() const { return mVehicleKinematics.geo_position.latitude; } // positive for north
		vanetza::units::Velocity speed() const { return mVehicleKinematics.speed; }
		vanetza::units::Acceleration acceleration() const { return mVehicleKinematics.acceleration; }
		vanetza::units::Angle heading() const { return mVehicleKinematics.heading; } // degree from north, clockwise
		vanetza::units::AngularVelocity yaw_rate() const { return mVehicleKinematics.yaw_rate; } // left turn positive
		vanetza::units::Curvature curvature() const { return mCurvature; } // 1/m radius, left turn positive
		double curvature_confidence() const { return mConfidence; } // percentage value

		void setStationType(StationType);
		StationType getStationType() const;

		void setStationId(uint32_t id);
		uint32_t getStationId() const { return mStationId; }
		uint32_t station_id() const { return mStationId; } /*< deprecated, use getStationId */

		void setSemiMajorConfidence(SemiAxisLength_t semiMajorConfidence);
        void setSemiMinorConfidence(SemiAxisLength_t semiMinorConfidence);
		void setAltitudeConfidence(AltitudeConfidence_t altitudeConfidence);
		void setHeadingConfidence(HeadingConfidence_t headingConfidence);
		void setSpeedConfidence(SpeedConfidence_t speedConfidence);
		void setLongitudinalAccelerationConfidence(AccelerationConfidence_t longitudinalAccelerationConfidence);

	private:
		typedef boost::units::quantity<boost::units::si::angular_acceleration> AngularAcceleration;
		void calculateCurvature();
		void calculateCurvatureConfidence();
		double mapOntoConfidence(AngularAcceleration) const;

		uint32_t mStationId;
		StationType mStationType;
		VehicleKinematics mVehicleKinematics;
		vanetza::units::Curvature mCurvature;
		double mConfidence;
		omnetpp::SimTime mLastUpdate;
		boost::circular_buffer<vanetza::units::Curvature> mCurvatureOutput;
		boost::circular_buffer<AngularAcceleration> mCurvatureConfidenceOutput;
		vanetza::units::AngularVelocity mCurvatureConfidenceInput;
		static const std::map<AngularAcceleration, double> mConfidenceTable;
		ReferencePosition_t mReferencePosition;
		Heading_t mHeading;
		Speed_t mSpeed;
        LongitudinalAcceleration_t mLongitudinalAcceleration;
        long semiMajorOrientationOffset;
};

} // namespace artery

#endif /* ARTERY_VEHICLEDATAPROVIDER_H_ */
