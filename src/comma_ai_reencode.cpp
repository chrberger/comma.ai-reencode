/*
 * Copyright (C) 2018  Christian Berger
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "comma.ai.hpp"

#include <cmath>
#include <cstdint>
#include <fstream>
#include <iostream>
#include <string>
#include <thread>

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{0};
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ( (0 == commandlineArguments.count("in")) || (0 == commandlineArguments.count("out")) ) {
        std::cerr << argv[0] << " reencodes an existing comma.ai dataset into the OpenDLV Standard Message Set." << std::endl;
        std::cerr << "Usage:   " << argv[0] << " --in=<existing recording> --out=<output> [--verbose]" << std::endl;
        std::cerr << "Example: " << argv[0] << " --in=myRec.rec --out=myNewRec.rec" << std::endl;
        retCode = 1;
    } else {
        cluon::OD4Session od4(254, [](auto){});

        const bool VERBOSE{commandlineArguments.count("verbose") != 0};

        (void)VERBOSE;
        std::string recFile{commandlineArguments["in"]};
        std::string outFile{commandlineArguments["out"]};

        std::fstream fin(recFile, std::ios::in|std::ios::binary);
        if (fin.good()) {
            std::fstream fout(outFile, std::ios::out|std::ios::binary);
            while (fin.good()) {
                auto retVal{cluon::extractEnvelope(fin)};
                if (retVal.first) {
                    if (static_cast<int32_t>(commaai::Vehicle::ID()) == retVal.second.dataType()) {
                        cluon::data::Envelope reencoded{std::move(retVal.second)};
                        commaai::Vehicle src = cluon::extractMessage<commaai::Vehicle>(cluon::data::Envelope(reencoded));

                        //car_accel
                        //opendlv.proxy.AccelerationReading
                        {
                            cluon::data::Envelope tmp{reencoded};
                            tmp.dataType(opendlv::proxy::AccelerationReading::ID());
                            tmp.senderStamp(0);
                            opendlv::proxy::AccelerationReading car_accel;
                            car_accel.accelerationX(static_cast<float>(src.car_accel()))
                                     .accelerationY(0)
                                     .accelerationZ(0);

                            cluon::ToProtoVisitor encoder;
                            car_accel.accept(encoder);
                            tmp.serializedData(encoder.encodedData());

                            const std::string data{od4.serializeAsOD4Container(std::move(tmp))};
                            fout.write(data.c_str(), data.size());
                            fout.flush();
                        }

                        //fiber_accel_[x,y,z]
                        //opendlv.proxy.AccelerationReading
                        {
                            cluon::data::Envelope tmp{reencoded};
                            tmp.dataType(opendlv::proxy::AccelerationReading::ID());
                            tmp.senderStamp(1);
                            opendlv::proxy::AccelerationReading fiber_accel;
                            fiber_accel.accelerationX(static_cast<float>(src.fiber_accel_x()))
                                       .accelerationY(static_cast<float>(src.fiber_accel_y()))
                                       .accelerationZ(static_cast<float>(src.fiber_accel_z()));

                            cluon::ToProtoVisitor encoder;
                            fiber_accel.accept(encoder);
                            tmp.serializedData(encoder.encodedData());

                            const std::string data{od4.serializeAsOD4Container(std::move(tmp))};
                            fout.write(data.c_str(), data.size());
                            fout.flush();
                        }

                        //fiber_compass_[x,y,z]
                        //opendlv.proxy.MagneticFieldReading
                        {
                            cluon::data::Envelope tmp{reencoded};
                            tmp.dataType(opendlv::proxy::MagneticFieldReading::ID());
                            tmp.senderStamp(0);
                            opendlv::proxy::MagneticFieldReading fiber_compass;
                            fiber_compass.magneticFieldX(static_cast<float>(src.fiber_compass_x()))
                                         .magneticFieldY(static_cast<float>(src.fiber_compass_y()))
                                         .magneticFieldZ(static_cast<float>(src.fiber_compass_z()));

                            cluon::ToProtoVisitor encoder;
                            fiber_compass.accept(encoder);
                            tmp.serializedData(encoder.encodedData());

                            const std::string data{od4.serializeAsOD4Container(std::move(tmp))};
                            fout.write(data.c_str(), data.size());
                            fout.flush();
                        }

                        //fiber_gyro_[x,y,z]
                        //opendlv.proxy.AngularVelocityReading
                        {
                            cluon::data::Envelope tmp{reencoded};
                            tmp.dataType(opendlv::proxy::AngularVelocityReading::ID());
                            tmp.senderStamp(0);
                            opendlv::proxy::AngularVelocityReading fiber_gyro;
                            fiber_gyro.angularVelocityX(static_cast<float>(src.fiber_gyro_x()))
                                         .angularVelocityY(static_cast<float>(src.fiber_gyro_y()))
                                         .angularVelocityZ(static_cast<float>(src.fiber_gyro_z()));

                            cluon::ToProtoVisitor encoder;
                            fiber_gyro.accept(encoder);
                            tmp.serializedData(encoder.encodedData());

                            const std::string data{od4.serializeAsOD4Container(std::move(tmp))};
                            fout.write(data.c_str(), data.size());
                            fout.flush();
                        }

                        //gas
                        //opendlv.proxy.PedalPositionRequest
                        {
                            cluon::data::Envelope tmp{reencoded};
                            tmp.dataType(opendlv::proxy::PedalPositionRequest::ID());
                            tmp.senderStamp(0);
                            opendlv::proxy::PedalPositionRequest gas;
                            gas.position(static_cast<float>(src.gas()));

                            cluon::ToProtoVisitor encoder;
                            gas.accept(encoder);
                            tmp.serializedData(encoder.encodedData());

                            const std::string data{od4.serializeAsOD4Container(std::move(tmp))};
                            fout.write(data.c_str(), data.size());
                            fout.flush();
                        }


                        //speed
                        //opendlv.proxy.GroundSpeedReading
                        {
                            cluon::data::Envelope tmp{reencoded};
                            tmp.dataType(opendlv::proxy::GroundSpeedReading::ID());
                            tmp.senderStamp(0);
                            opendlv::proxy::GroundSpeedReading speed;
                            speed.groundSpeed(static_cast<float>(src.speed()));

                            cluon::ToProtoVisitor encoder;
                            speed.accept(encoder);
                            tmp.serializedData(encoder.encodedData());

                            const std::string data{od4.serializeAsOD4Container(std::move(tmp))};
                            fout.write(data.c_str(), data.size());
                            fout.flush();
                        }

                        //speed_abs
                        //opendlv.proxy.GroundSpeedReading
                        {
                            cluon::data::Envelope tmp{reencoded};
                            tmp.dataType(opendlv::proxy::GroundSpeedReading::ID());
                            tmp.senderStamp(1);
                            opendlv::proxy::GroundSpeedReading speed_abs;
                            speed_abs.groundSpeed(static_cast<float>(fabs(src.speed())));

                            cluon::ToProtoVisitor encoder;
                            speed_abs.accept(encoder);
                            tmp.serializedData(encoder.encodedData());

                            const std::string data{od4.serializeAsOD4Container(std::move(tmp))};
                            fout.write(data.c_str(), data.size());
                            fout.flush();
                        }

                        //steering_angle
                        //opendlv.proxy.GroundSteeringRequest  (if steering angle at road wheel)
                        {
                            cluon::data::Envelope tmp{reencoded};
                            tmp.dataType(opendlv::proxy::GroundSteeringRequest::ID());
                            tmp.senderStamp(0);
                            opendlv::proxy::GroundSteeringRequest steering_angle;
                            steering_angle.groundSteering(static_cast<float>(src.steering_angle()));

                            cluon::ToProtoVisitor encoder;
                            steering_angle.accept(encoder);
                            tmp.serializedData(encoder.encodedData());

                            const std::string data{od4.serializeAsOD4Container(std::move(tmp))};
                            fout.write(data.c_str(), data.size());
                            fout.flush();
                        }

                        //steering_torque
                        //opendlv.proxy.TorqueReading
                        {
                            cluon::data::Envelope tmp{reencoded};
                            tmp.dataType(opendlv::proxy::TorqueReading::ID());
                            tmp.senderStamp(0);
                            opendlv::proxy::TorqueReading steering_torque;
                            steering_torque.torque(static_cast<float>(src.steering_torque()));

                            cluon::ToProtoVisitor encoder;
                            steering_torque.accept(encoder);
                            tmp.serializedData(encoder.encodedData());

                            const std::string data{od4.serializeAsOD4Container(std::move(tmp))};
                            fout.write(data.c_str(), data.size());
                            fout.flush();
                        }

                        //velodyne_[latitude,longitudinal,heading]
                        //opendlv.logic.sensation.Geolocation
                        {
                            cluon::data::Envelope tmp{reencoded};
                            tmp.dataType(opendlv::logic::sensation::Geolocation::ID());
                            tmp.senderStamp(0);
                            opendlv::logic::sensation::Geolocation velodyne;
                            velodyne.latitude(static_cast<float>(src.velodyne_gps_latitude()))
                                    .longitude(static_cast<float>(src.velodyne_gps_longitude()))
                                    .heading(static_cast<float>(src.velodyne_heading()));

                            cluon::ToProtoVisitor encoder;
                            velodyne.accept(encoder);
                            tmp.serializedData(encoder.encodedData());

                            const std::string data{od4.serializeAsOD4Container(std::move(tmp))};
                            fout.write(data.c_str(), data.size());
                            fout.flush();
                        }

//blinker, 0 or 1?
//opendlv.proxy.SwitchStateReading

//brake
//opendlv.proxy.PressureRequest   (or perhaps add: opendlv.proxy.BrakePressureRequest)

//brake_computer
//opendlv.proxy.PressureRequest

//brake_user
//opendlv.proxy.PressureRequest

//gear_choice
//opendlv.proxy.SwitchStateReading

//rpm
//opendlv.proxy.ShaftAngularVelocityReading   (needs to be added)

//rpm_post_torque
//opendlv.proxy.TorqueReading

//slefdrive
//opendlv.system.SystemOperationState

//speed_fl
//opendlv.proxy.ShaftAngularVelocityReading

//speed_fr
//opendlv.proxy.ShaftAngularVelocityReading

//speed_rl
//opendlv.proxy.ShaftAngularVelocityReading

//speed_rr
//opendlv.proxy.ShaftAngularVelocityReading

//standstill
//opendlv.system.SystemOperationState


                    }
                }
            }
        }
        else {
            std::cerr << "[" << argv[0] << "] '" << recFile << "' could not be opened." << std::endl;
        }
    }
    return retCode;
}
