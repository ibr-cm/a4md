/*
 * Artery V2X Simulation Framework
 * Copyright 2014-2017 Raphael Riebl
 * Licensed under GPLv2, see COPYING file for detailed license and warranty terms.
 */

#include "artery/application/StationType.h"
#include "artery/application/VehicleMiddleware.h"
#include "artery/traci/ControllableVehicle.h"
#include "artery/traci/MobilityBase.h"
#include "artery/utility/InitStages.h"
#include "inet/common/ModuleAccess.h"

using namespace omnetpp;

namespace artery {

    Define_Module(VehicleMiddleware)


    bool VehicleMiddleware::staticInitializationComplete = false;
    std::shared_ptr<const traci::API> VehicleMiddleware::mTraciAPI;
    traci::Boundary VehicleMiddleware::mSimulationBoundary;

    VehicleMiddleware::VehicleMiddleware() :
            mVehicleDataProvider(0) // OMNeT++ assigns RNG after construction: set final station ID later
    {
    }

    void VehicleMiddleware::initialize(int stage) {
        if (stage == InitStages::Self) {
            findHost()->subscribe(MobilityBase::stateChangedSignal, this);
            initializeVehicleController(par("mobilityModule"));
            initializeStationType(mVehicleController->getVehicleClass());
            getFacilities().register_const(&mVehicleDataProvider);


            if (!staticInitializationComplete) {
                staticInitializationComplete = true;
                mTraciAPI = getFacilities().get_const<traci::VehicleController>().getTraCI();
                mSimulationBoundary = traci::Boundary{mTraciAPI->simulation.getNetBoundary()};
            }

            double semiMajorConfidence = par("semiMajorConfidence");
            double semiMinorConfidence = par("semiMinorConfidence");
            double altitudeConfidence = par("altitudeConfidence");
            double headingConfidence = par("headingConfidence");
            double speedConfidence = par("speedConfidence");
            double longitudinalAccelerationConfidence = par("longitudinalAccelerationConfidence");
            mVehicleDataProvider.setSemiMajorConfidence((long) semiMajorConfidence);
            mVehicleDataProvider.setSemiMinorConfidence((long) semiMinorConfidence);
            mVehicleDataProvider.setAltitudeConfidence((long) altitudeConfidence);
            mVehicleDataProvider.setHeadingConfidence((long) headingConfidence);
            mVehicleDataProvider.setSpeedConfidence((long) speedConfidence);
            mVehicleDataProvider.setLongitudinalAccelerationConfidence((long) longitudinalAccelerationConfidence);

            mVehicleDataProvider.update(getKinematics(*mVehicleController));
            mVehicleDataProvider.updateApproximate(getKinematics(*mVehicleController), getRNG(0), mTraciAPI,
                                                   mSimulationBoundary);

            Identity identity;
            identity.traci = mVehicleController->getVehicleId();
            identity.application = Identity::randomStationId(getRNG(0));
            mVehicleDataProvider.setStationId(identity.application);
            emit(Identity::changeSignal, Identity::ChangeTraCI | Identity::ChangeStationId, &identity);
        }

        Middleware::initialize(stage);
    }

    void VehicleMiddleware::finish() {
        Middleware::finish();
        findHost()->unsubscribe(MobilityBase::stateChangedSignal, this);
    }

    void VehicleMiddleware::initializeStationType(const std::string &vclass) {
        auto gnStationType = deriveStationTypeFromVehicleClass(vclass);
        setStationType(gnStationType);
        mVehicleDataProvider.setStationType(gnStationType);
    }

    void VehicleMiddleware::initializeVehicleController(cPar &mobilityPar) {
        auto mobility = inet::getModuleFromPar<ControllableVehicle>(mobilityPar, findHost());
        mVehicleController = mobility->getVehicleController();
        ASSERT(mVehicleController);
        getFacilities().register_mutable(mVehicleController);
    }

    void VehicleMiddleware::receiveSignal(cComponent *component, simsignal_t signal, cObject *obj, cObject *details) {
        if (signal == MobilityBase::stateChangedSignal && mVehicleController) {
            mVehicleDataProvider.update(getKinematics(*mVehicleController));
            mVehicleDataProvider.updateApproximate(getKinematics(*mVehicleController), getRNG(0), mTraciAPI,
                                                   mSimulationBoundary);
        }
    }

} // namespace artery

