// ControllerZenohHostVehicleData.cpp
#include "ControllerZenohHostVehicleData.hpp"
#include "CommonMini.hpp"
#include "Entities.hpp"
#include "ScenarioGateway.hpp"
#include "logger.hpp"

using namespace scenarioengine;

std::string ControllerZenohHostVehicleData::baseExpression_ = DEFAULT_ZENOH_EXPRESSION;

Controller* scenarioengine::InstantiateControllerZenohHostVehicleData(void* args)
{
    Controller::InitArgs* initArgs = static_cast<Controller::InitArgs*>(args);

    return new ControllerZenohHostVehicleData(initArgs);
}

ControllerZenohHostVehicleData::ControllerZenohHostVehicleData(InitArgs* args) : Controller(args), execMode_(ExecMode::EXEC_MODE_SYNCHRONOUS)
{
    LOG_INFO("Initializing ControllerZenohHostVehicleData");
    if (args && args->properties && args->properties->ValueExists("zenohKey"))
    {
        zenoh_key_expression_ = args->properties->GetValueStr("zenohKey");
    }

    if (args && args->properties && args->properties->ValueExists("zenohBaseExpression"))
    {
        baseExpression_ = args->properties->GetValueStr("zenohBaseExpression");
    }

    if (args && args->properties && args->properties->ValueExists("execMode"))
    {
        if (args->properties->GetValueStr("execMode") == "asynchronous")
        {
            execMode_ = ExecMode::EXEC_MODE_ASYNCHRONOUS;
        }
        else if (args->properties->GetValueStr("execMode") == "synchronous")
        {
            execMode_ = ExecMode::EXEC_MODE_SYNCHRONOUS;
        }
        else
        {
            LOG_ERROR_AND_QUIT("ControllerExternalDriverModel unexpected arg execMode {}", args->properties->GetValueStr("execMode"));
        }
    }

    if (args && args->properties && args->properties->ValueExists("deadReckon"))
    {
        if (args->properties->GetValueStr("deadReckon") == "false")
        {
            deadReckon_ = false;
        }
        else if (args->properties->GetValueStr("deadReckon") == "true")
        {
            deadReckon_ = true;
        }
        else
        {
            LOG_ERROR_AND_QUIT("ControllerExternalDriverModel unexpected arg execMode {}", args->properties->GetValueStr("deadReckon"));
        }
    }

    mode_ = ControlOperationMode::MODE_OVERRIDE;
    LOG_INFO("Initialized ControllerZenohHostVehicleData");
}

ControllerZenohHostVehicleData::~ControllerZenohHostVehicleData()
{
}

void ControllerZenohHostVehicleData::Init()
{
    if (baseExpression_ == "")
    {
        baseExpression_ = DEFAULT_ZENOH_EXPRESSION;
        LOG_WARN("ControllerZenohHostVehicleData: using default Zenoh key expression {}", baseExpression_);
    }
    Controller::Init();
}
int i = 0;
void ControllerZenohHostVehicleData::Step(double timeStep)
{
    std::optional<osi3::HostVehicleData> receivedSample;
    osi3::HostVehicleData                msg;
    if (execMode_ == ExecMode::EXEC_MODE_ASYNCHRONOUS)
    {
        receivedSample = zenoh_receiver_->receive(0);
    }
    else
    {
        receivedSample = zenoh_receiver_->receive(ZENOH_SYNCHRONOUS_MODE_TIMEOUT_MS);
    }
    if (receivedSample.has_value())
    {
        msg                        = *receivedSample;
        lastMsg                    = msg;
        roadmanager::Position* pos = &gateway_->getObjectStatePtrById(static_cast<int>(msg.host_vehicle_id().value()))->state_.pos;
        pos->SetMode(roadmanager::Position::PosModeType::SET,

                     roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_ABS | roadmanager::Position::PosMode::P_REL |
                         roadmanager::Position::PosMode::R_REL);
    }
    if (receivedSample.has_value() || deadReckon_)
    {
        // LOG_INFO("Sample Received: {}", i++);
        if (timeStep > SMALL_NUMBER)
        {
            // In driver input mode the vehicle is updated continuously wrt latest input
            auto total_acc = lastMsg.vehicle_powertrain().pedal_position_acceleration() - lastMsg.vehicle_brake_system().pedal_position_brake();
            auto total_steer = lastMsg.vehicle_steering().vehicle_steering_wheel().angle();
            LOG_INFO("Total Acc: {}, Total Steer: {}", total_acc, total_steer);
            vehicle_.DrivingControlAnalog(
                timeStep,
                lastMsg.vehicle_powertrain().pedal_position_acceleration() - lastMsg.vehicle_brake_system().pedal_position_brake(),
                lastMsg.vehicle_steering().vehicle_steering_wheel().angle());

            // Register updated vehicle position
            gateway_->updateObjectWorldPosXYH(object_->id_, 0.0, vehicle_.posX_, vehicle_.posY_, vehicle_.heading_);
            gateway_->updateObjectSpeed(object_->id_, 0.0, vehicle_.speed_);
            gateway_->updateObjectWheelAngle(object_->id_, 0.0, msg.vehicle_steering().vehicle_steering_wheel().angle());

            // Fetch Z and Pitch from OpenDRIVE position
            roadmanager::Position* pos = &gateway_->getObjectStatePtrById(static_cast<int>(msg.host_vehicle_id().value()))->state_.pos;
            vehicle_.SetZ(pos->GetZ());
            vehicle_.SetPitch(pos->GetP());
        }
        Controller::Step(timeStep);
    }

    // auto packet             = host_vehicle_datas_[zenoh_key_expression_];
    // auto host_vehicle_data_ = packet.data;

    // if (packet.new_data_received)
    // {
    //     LOG_INFO("Received new host vehicle data");
    //     roadmanager::Position* pos = &gateway_->getObjectStatePtrById(static_cast<int>(host_vehicle_data_.host_vehicle_id().value()))->state_.pos;
    //     pos->SetMode(roadmanager::Position::PosModeType::SET,

    //                  roadmanager::Position::PosMode::Z_REL | roadmanager::Position::PosMode::H_ABS | roadmanager::Position::PosMode::P_REL |
    //                      roadmanager::Position::PosMode::R_REL);

    //     host_vehicle_datas_[zenoh_key_expression_].new_data_received = false;
    //     if (timeStep > SMALL_NUMBER)
    //     {
    //         vehicle_.DrivingControlAnalog(timeStep,
    //                                       host_vehicle_data_.vehicle_powertrain().pedal_position_acceleration() -
    //                                           host_vehicle_data_.vehicle_brake_system().pedal_position_brake(),
    //                                       host_vehicle_data_.vehicle_steering().vehicle_steering_wheel().angle());

    //         // Register updated vehicle position
    //         gateway_->updateObjectWorldPosXYH(object_->id_, 0.0, vehicle_.posX_, vehicle_.posY_, vehicle_.heading_);
    //         gateway_->updateObjectSpeed(object_->id_, 0.0, vehicle_.speed_);
    //         gateway_->updateObjectWheelAngle(object_->id_, 0.0, host_vehicle_data_.vehicle_steering().vehicle_steering_wheel().angle());

    //         // Fetch Z and Pitch from OpenDRIVE position
    //         roadmanager::Position* pos =
    //         &gateway_->getObjectStatePtrById(static_cast<int>(host_vehicle_data_.host_vehicle_id().value()))->state_.pos;
    //         vehicle_.SetZ(pos->GetZ());
    //         vehicle_.SetPitch(pos->GetP());
    //     }
    //     Controller::Step(timeStep);
    // }
}

int ControllerZenohHostVehicleData::Activate(const ControlActivationMode (&mode)[static_cast<unsigned int>(ControlDomains::COUNT)])
{
    if (object_)
    {
        if (zenoh_key_expression_.empty() || zenoh_key_expression_ == "")
        {
            zenoh_key_expression_ = baseExpression_ + std::to_string(object_->GetId());
        }

        if (!zenoh_receiver_ || (zenoh_receiver_ && zenoh_receiver_->getKeyExpression().as_string_view() != zenoh_key_expression_))
        {
            try
            {
                LOG_INFO("Initializing Zenoh subscriber for key: {}", zenoh_key_expression_.c_str());
                // std::cout << "Initializing Zenoh subscriber for key: " << zenoh_key_expression_ << std::endl;
                zenoh::Config config = zenoh::Config::create_default();
                // config.insert_json5("connect/endpoints", R"(["tcp/127.0.0.1:7447"])"); // Connect to zenoh_bridge
                // config.insert_json5("listen/endpoints", R"(["tcp/0.0.0.0:7448"])");       // Listen on its own port
                zenoh_session_ = zenoh::Session::open(std::move(config));
                if (execMode_ == ExecMode::EXEC_MODE_ASYNCHRONOUS)
                {
                    zenoh_receiver_ = std::make_unique<zenoh::Receiver<osi3::HostVehicleData>>(
                        *zenoh_session_,
                        zenoh_key_expression_,
                        [](const zenoh::Sample& sample)
                        {
                            // LOG_INFO("Coming data");
                            osi3::HostVehicleData data;
                            const auto&           vec = sample.get_payload().as_vector();
                            data.ParseFromArray(vec.data(), static_cast<int>(vec.size()));
                            return data;
                        },
                        1);
                }
                else
                {
                    zenoh_receiver_ =
                        std::make_unique<zenoh::Receiver<osi3::HostVehicleData>>(*zenoh_session_,
                                                                                 zenoh_key_expression_,
                                                                                 [](const zenoh::Sample& sample)
                                                                                 {
                                                                                    // LOG_INFO("Coming data");
                                                                                     osi3::HostVehicleData data;
                                                                                     const auto&           vec = sample.get_payload().as_vector();
                                                                                     data.ParseFromArray(vec.data(), static_cast<int>(vec.size()));
                                                                                     return data;
                                                                                 });
                }
                LOG_INFO("Zenoh subscriber created for key: {}", zenoh_key_expression_.c_str());
            }
            catch (const std::exception& e)
            {
                LOG_ERROR_AND_QUIT("Zenoh initialization failed: {}", e.what());
            }
        }

        vehicle_.Reset();
        vehicle_.SetPos(object_->pos_.GetX(), object_->pos_.GetY(), object_->pos_.GetZ(), object_->pos_.GetH());
        vehicle_.SetLength(object_->boundingbox_.dimensions_.length_);
        vehicle_.speed_ = object_->GetSpeed();
        vehicle_.SetMaxAcc(20.0);
        vehicle_.SetMaxSpeed(30.0);
    }

    return Controller::Activate(mode);
}

void ControllerZenohHostVehicleData::ReportKeyEvent(int key, bool down)
{
    (void)key;
    (void)down;
}