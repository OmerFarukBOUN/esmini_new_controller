// ControllerZenohHostVehicleData.hpp
#pragma once

#include "Controller.hpp"
#include "osi_hostvehicledata.pb.h"
#include "vehicle.hpp"
#include "zenoh.hxx"

#define CONTROLLER_ZENOH_HOSTVEHICLEDATA_TYPE_NAME "ZenohHostVehicleDataController"
#define DEFAULT_ZENOH_EXPRESSION                   "hostvehicledata/"
#define ZENOH_SYNCHRONOUS_MODE_TIMEOUT_MS          500

namespace scenarioengine
{

    class ControllerZenohHostVehicleData : public Controller
    {
    public:
        typedef struct
        {
            osi3::HostVehicleData data;
            bool                  new_data_received;
        } ZenohPacket;

        enum class ExecMode
        {
            EXEC_MODE_ASYNCHRONOUS = 0,
            EXEC_MODE_SYNCHRONOUS  = 1,
        };

        ControllerZenohHostVehicleData(InitArgs* args);
        ~ControllerZenohHostVehicleData();

        void Init();
        void Step(double timeStep);
        int  Activate(const ControlActivationMode (&mode)[static_cast<unsigned int>(ControlDomains::COUNT)]);
        void ReportKeyEvent(int key, bool down);

        virtual const char* GetTypeName()
        {
            return CONTROLLER_ZENOH_HOSTVEHICLEDATA_TYPE_NAME;
        }

        virtual int GetType()
        {
            return Controller::Type::CONTROLLER_ZENOH_HOSTVEHICLEDATA;
        }


    private:
        vehicle::Vehicle                                        vehicle_;
        std::optional<zenoh::Session>                           zenoh_session_;
        std::string                                             zenoh_key_expression_;
        static std::string                                      baseExpression_;
        std::unique_ptr<zenoh::Receiver<osi3::HostVehicleData>> zenoh_receiver_;
        ExecMode                                                execMode_;
        bool                                              deadReckon_;
        osi3::HostVehicleData                                   lastMsg;
        void                                                    updateVehicleState();
    };

    Controller* InstantiateControllerZenohHostVehicleData(void* args);
}  // namespace scenarioengine