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

#include "artery/application/VehicleDataProvider.h"
#include <boost/math/constants/constants.hpp>
#include <boost/units/cmath.hpp>
#include <boost/units/systems/si/prefixes.hpp>
#include <omnetpp/csimulation.h>
#include <omnetpp/crng.h>
#include <vanetza/units/frequency.hpp>
#include <vanetza/units/time.hpp>
#include <vanetza/units/angle.hpp>
#include <vanetza/units/angular_velocity.hpp>
#include <artery/traci/Cast.h>
#include <cassert>
#include <cmath>
#include <limits>
#include <stdexcept>
#include <algorithm>
#include <artery/application/misbehavior/util/HelperFunctions.h>

namespace artery {

    const double pi = boost::math::constants::pi<double>();
    const auto degree_per_second_squared =
            vanetza::units::degree / (vanetza::units::si::second * vanetza::units::si::second);
    const std::map<VehicleDataProvider::AngularAcceleration, double> VehicleDataProvider::mConfidenceTable{
            {AngularAcceleration(0.0 * degree_per_second_squared),                                     1.0},
            {AngularAcceleration(0.5 * degree_per_second_squared),                                     0.9},
            {AngularAcceleration(1.0 * degree_per_second_squared),                                     0.8},
            {AngularAcceleration(1.5 * degree_per_second_squared),                                     0.7},
            {AngularAcceleration(2.0 * degree_per_second_squared),                                     0.6},
            {AngularAcceleration(2.5 * degree_per_second_squared),                                     0.5},
            {AngularAcceleration(5.0 * degree_per_second_squared),                                     0.4},
            {AngularAcceleration(10.0 * degree_per_second_squared),                                    0.3},
            {AngularAcceleration(15.0 * degree_per_second_squared),                                    0.2},
            {AngularAcceleration(20.0 * degree_per_second_squared),                                    0.1},
            {AngularAcceleration(25.0 * degree_per_second_squared),                                    0.0},
            {AngularAcceleration(std::numeric_limits<double>::infinity() * degree_per_second_squared), 0.0}
    };


    VehicleDataProvider::VehicleDataProvider(uint32_t id) :
            mStationId(id), mStationType(StationType::Unknown),
            mConfidence(0.0), mLastUpdate(omnetpp::SimTime::getMaxTime()),
            mCurvatureOutput(2), mCurvatureConfidenceOutput(2) {
        while (!mCurvatureConfidenceOutput.full()) {
            using namespace vanetza::units::si;
            mCurvatureConfidenceOutput.push_front(0.0 * radians_per_second / second);
        }
        mReferencePosition = {};
        mReferencePosition.positionConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;
        mReferencePosition.positionConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
        mReferencePosition.positionConfidenceEllipse.semiMajorOrientation = HeadingValue_unavailable;
        mReferencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
        mReferencePosition.altitude.altitudeValue = AltitudeValue_unavailable;
        mHeading = {};
        mHeading.headingConfidence = HeadingConfidence_unavailable;
        mSpeed = {};
        mSpeed.speedConfidence = SpeedConfidence_unavailable;
        mLongitudinalAcceleration = {};
        mLongitudinalAcceleration.longitudinalAccelerationConfidence = AccelerationConfidence_unavailable;
        semiMajorOrientationOffset = 0;
    }

    void VehicleDataProvider::calculateCurvature() {
        using namespace vanetza::units::si;
        static const vanetza::units::Frequency f_cut = 0.33 * hertz;
        static const vanetza::units::Duration t_sample = 0.1 * seconds;
        static const vanetza::units::Curvature lower_threshold = 1.0 / 2500.0 * vanetza::units::reciprocal_metre;
        static const vanetza::units::Curvature upper_threshold = 1.0 * vanetza::units::reciprocal_metre;
        static const double damping = 1.0;

        if (fabs(mVehicleKinematics.speed) < 1.0 * meter_per_second) {
            // assume straight road below minimum speed
            mCurvature = 0.0 * vanetza::units::reciprocal_metre;
        } else {
            // curvature calculation algorithm
            mCurvature = (mVehicleKinematics.yaw_rate / radians) / mVehicleKinematics.speed;

            if (!mCurvatureOutput.full()) {
                // save first two values for initialization
                mCurvatureOutput.push_front(mCurvature);
                mCurvature = 0.0 * vanetza::units::reciprocal_metre;
            } else {
                static const auto omega = 2.0 * pi * f_cut;
                mCurvature = -mCurvatureOutput[1] +
                             (2.0 + 2.0 * omega * damping * t_sample) * mCurvatureOutput[0] +
                             omega * omega * t_sample * t_sample * mCurvature;
                mCurvature /= 1.0 + 2.0 * omega * damping * t_sample + omega * omega * t_sample * t_sample;
                mCurvatureOutput.push_front(mCurvature);

                // assume straight road below threshold
                if (fabs(mCurvature) < lower_threshold) {
                    mCurvature = 0.0 * vanetza::units::reciprocal_metre;
                } else if (fabs(mCurvature) > upper_threshold) {
                    // clamp minimum radius to 1 meter
                    mCurvature = upper_threshold;
                }
            }
        }
    }

    void VehicleDataProvider::calculateCurvatureConfidence() {
        assert(mCurvatureConfidenceOutput.full());
        using namespace vanetza::units::si;
        static const vanetza::units::Frequency f_cut = 1.0 * hertz;
        static const vanetza::units::Duration t_sample = 100.0 * seconds;
        static const double damping = 1.0;
        static const auto omega = 2.0 * pi * f_cut;

        AngularAcceleration filter = -mCurvatureConfidenceOutput[1] +
                                     (2.0 + 2.0 * omega * damping * t_sample) * mCurvatureConfidenceOutput[0] +
                                     omega * omega * t_sample * mVehicleKinematics.yaw_rate -
                                     omega * omega * t_sample * mCurvatureConfidenceInput;
        filter /= 1.0 + 2.0 * omega * damping * t_sample + omega * omega * t_sample * t_sample;
        mCurvatureConfidenceOutput.push_front(filter);
        mCurvatureConfidenceInput = mVehicleKinematics.yaw_rate;
        mConfidence = mapOntoConfidence(abs(filter));
    }

    void VehicleDataProvider::update(const VehicleKinematics &dynamics) {
        using namespace omnetpp;
        using namespace vanetza::units::si;
        using boost::units::isnan;

        const vanetza::units::Duration delta{
                (simTime() - mLastUpdate).inUnit(SIMTIME_MS) * boost::units::si::milli * seconds
        };

        if (delta > 0.0 * seconds) {
            const auto old_heading = mVehicleKinematics.heading;
            const auto old_speed = mVehicleKinematics.speed;
            mVehicleKinematics = dynamics;

            if (isnan(mVehicleKinematics.acceleration)) {
                mVehicleKinematics.acceleration = (mVehicleKinematics.speed - old_speed) / delta;
            }

            if (isnan(mVehicleKinematics.yaw_rate)) {
                auto diff_heading = old_heading - mVehicleKinematics.heading; // left turn positive
                if (diff_heading > pi * radian) {
                    diff_heading -= 2.0 * pi * radians;
                } else if (diff_heading < -pi * radians) {
                    diff_heading += 2.0 * pi * radians;
                }
                mVehicleKinematics.yaw_rate = diff_heading / delta;
            }
        } else if (delta < 0.0 * seconds) {
            // initialization
            mVehicleKinematics = dynamics;

            if (isnan(mVehicleKinematics.acceleration)) {
                mVehicleKinematics.acceleration = vanetza::units::Acceleration::from_value(0.0);
            }
            if (isnan(mVehicleKinematics.yaw_rate)) {
                mVehicleKinematics.yaw_rate = vanetza::units::AngularVelocity::from_value(0.0);
            }
        } else {
            // update has been called for this time step already before
            return;
        }

        mLastUpdate = simTime();
        calculateCurvature();
        calculateCurvatureConfidence();
    }

    auto decidegree = vanetza::units::degree * boost::units::si::deci;

    template<typename T, typename U>
    long round(const boost::units::quantity<T> &q, const U &u) {
        boost::units::quantity<U> v{q};
        return std::round(v.value());
    }

    void VehicleDataProvider::updateApproximate(const VehicleKinematics &dynamics, omnetpp::cRNG *rng,
                                                const std::shared_ptr<const traci::API> &traciAPI,
                                                const traci::Boundary &simulationBoundary) {
        using namespace omnetpp;
        using namespace vanetza::units::si;
        using boost::units::isnan;

        long actualSpeed = (long) (dynamics.speed.value() * 100);
        if (mSpeed.speedConfidence != SpeedConfidence_unavailable) {
            long speedConfidence = mSpeed.speedConfidence;
            mSpeed.speedValue = intuniform(rng, std::max(0, (int) (actualSpeed - speedConfidence)),
                                           std::min(16382, (int) (actualSpeed + speedConfidence)));
        } else {
            mSpeed.speedValue = actualSpeed;
        }

        long actualAcceleration = (long) (dynamics.acceleration.value() * 10);
        if (mLongitudinalAcceleration.longitudinalAccelerationConfidence != AccelerationConfidence_unavailable) {
            long accelerationConfidence = mLongitudinalAcceleration.longitudinalAccelerationConfidence;
            mLongitudinalAcceleration.longitudinalAccelerationValue = intuniform(rng, std::max(-160,
                                                                                               (int) (actualAcceleration -
                                                                                                      accelerationConfidence)),
                                                                                 std::min(160,
                                                                                          (int) (actualAcceleration +
                                                                                                 accelerationConfidence)));
        } else {
            mLongitudinalAcceleration.longitudinalAccelerationValue = actualAcceleration;
        }

        long actualHeading = round(dynamics.heading, decidegree);
        if (mHeading.headingConfidence != HeadingConfidence_unavailable) {
            long headingConfidence = mHeading.headingConfidence;
            long newHeading = intuniform(rng, (int) (actualHeading - headingConfidence),
                                         (int) (actualHeading + headingConfidence));
            mHeading.headingValue = (newHeading + 3600) % 3600;
        } else {
            mHeading.headingValue = actualHeading;
        }

        setPositionWithJitter(mReferencePosition, mVehicleKinematics.position, mHeading.headingValue,
                              semiMajorOrientationOffset, simulationBoundary, traciAPI, rng);

    }

    double VehicleDataProvider::mapOntoConfidence(AngularAcceleration x) const {
        auto it = mConfidenceTable.lower_bound(x);
        if (it == mConfidenceTable.end()) {
            throw std::domain_error("input value is less than smallest entry in confidence table");
        }
        return it->second;
    }

    void VehicleDataProvider::setStationType(StationType type) {
        mStationType = type;
    }

    auto VehicleDataProvider::getStationType() const -> StationType {
        return mStationType;
    }

    void VehicleDataProvider::setStationId(uint32_t id) {
        mStationId = id;
    }

    void VehicleDataProvider::setSemiMajorConfidence(SemiAxisLength_t semiMajorConfidence) {
        if (semiMajorConfidence == SemiAxisLength_unavailable) {
            mReferencePosition.positionConfidenceEllipse.semiMajorConfidence = SemiAxisLength_unavailable;
        } else {
            mReferencePosition.positionConfidenceEllipse.semiMajorConfidence = std::max((long) 1, std::min((long) 4093,
                                                                                                           (long) semiMajorConfidence));
            if (mReferencePosition.positionConfidenceEllipse.semiMinorConfidence != SemiAxisLength_unavailable &&
                mReferencePosition.positionConfidenceEllipse.semiMinorConfidence >
                mReferencePosition.positionConfidenceEllipse.semiMajorConfidence) {
                semiMajorOrientationOffset = 900;
                std::swap(mReferencePosition.positionConfidenceEllipse.semiMajorConfidence,
                          mReferencePosition.positionConfidenceEllipse.semiMinorConfidence);
            } else {
                semiMajorOrientationOffset = 0;
            }
        }
    }

    void VehicleDataProvider::setSemiMinorConfidence(SemiAxisLength_t semiMinorConfidence) {
        if (semiMinorConfidence == SemiAxisLength_unavailable) {
            mReferencePosition.positionConfidenceEllipse.semiMinorConfidence = SemiAxisLength_unavailable;
        } else {
            mReferencePosition.positionConfidenceEllipse.semiMinorConfidence = std::max((long) 1, std::min((long) 4093,
                                                                                                           (long) semiMinorConfidence));
            if (mReferencePosition.positionConfidenceEllipse.semiMajorConfidence != SemiAxisLength_unavailable &&
                mReferencePosition.positionConfidenceEllipse.semiMinorConfidence >
                mReferencePosition.positionConfidenceEllipse.semiMajorConfidence) {
                semiMajorOrientationOffset = 900;
                std::swap(mReferencePosition.positionConfidenceEllipse.semiMajorConfidence,
                          mReferencePosition.positionConfidenceEllipse.semiMinorConfidence);

            } else {
                semiMajorOrientationOffset = 0;
            }
        }
    }

    void VehicleDataProvider::setAltitudeConfidence(AltitudeConfidence_t altitudeConfidence) {
        if (altitudeConfidence == AltitudeConfidence_unavailable) {
            mReferencePosition.altitude.altitudeConfidence = AltitudeConfidence_unavailable;
        } else {
            mReferencePosition.altitude.altitudeConfidence = std::max((long) 0,
                                                                      std::min((long) 13, (long) altitudeConfidence));
        }
    }

    void VehicleDataProvider::setSpeedConfidence(SpeedConfidence_t speedConfidence) {
        if (speedConfidence == SpeedConfidence_unavailable) {
            mSpeed.speedConfidence = SpeedConfidence_unavailable;
        } else {
            mSpeed.speedConfidence = std::max((long) 1, std::min((long) 125, (long) speedConfidence));
        }
    }

    void VehicleDataProvider::setHeadingConfidence(HeadingConfidence_t headingConfidence) {
        if (headingConfidence == HeadingConfidence_unavailable) {
            mHeading.headingConfidence = HeadingConfidence_unavailable;
        } else {
            mHeading.headingConfidence = std::max((long) 1, std::min((long) 125, (long) headingConfidence));
        }
    }

    void VehicleDataProvider::setLongitudinalAccelerationConfidence(
            AccelerationConfidence_t longitudinalAccelerationConfidence) {
        if (longitudinalAccelerationConfidence == AccelerationConfidence_unavailable) {
            mLongitudinalAcceleration.longitudinalAccelerationConfidence = AccelerationConfidence_unavailable;
        } else {
            mLongitudinalAcceleration.longitudinalAccelerationConfidence = std::max((long) 1, std::min((long) 0,
                                                                                                       (long) longitudinalAccelerationConfidence));
        }
    }

} // namespace artery
