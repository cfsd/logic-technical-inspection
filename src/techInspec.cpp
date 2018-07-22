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

#include "logic-techInspec.hpp"

#include <cstdint>
#include <iostream>
#include <sstream>
#include <string>
#include <thread>
#include <cmath>
#include <ctime>
#include <chrono>

#define PI 3.14159265359f

int32_t main(int32_t argc, char **argv) {
    int32_t retCode{0};
    int32_t senderStamp = 316;
    auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
    if ((0 == commandlineArguments.count("cid")) || (0 == commandlineArguments.count("verbose"))) {
        std::cout << argv[0] << " not enought input arguments. Assigning default values." << std::endl;
        std::cout << "Default: " << argv[0] << " --cid=111 --cidBB=150" << std::endl;
        std::cout << "Usage:   " << argv[0] << " --cid=<OpenDaVINCI session>" << std::endl;
    }

    const uint16_t cid{(commandlineArguments["cid"].size() != 0) ? static_cast<uint16_t>(std::stoi(commandlineArguments["cid"])) : (uint16_t) 111};
    const uint16_t cidBB{(commandlineArguments["cidBB"].size() != 0) ? static_cast<uint16_t>(std::stoi(commandlineArguments["cidBB"])) : (uint16_t) 150};

    cluon::OD4Session od4{cid};
    cluon::OD4Session od4BB{cidBB};

  //  TechInspec techInspec(od4);

    double initTime;
    bool readyState = false;

    auto catchState{[&initTime,&readyState](cluon::data::Envelope &&envelope)
      {
        auto message = cluon::extractMessage<opendlv::proxy::SwitchStateReading>(std::move(envelope));

        if((envelope.senderStamp()==1401) && message.state()==2 && readyState==false){
          cluon::data::TimeStamp tp = cluon::time::now();
          initTime = (double)(tp.seconds() + tp.microseconds()*1e-6);
          readyState = true;
        }
        if((envelope.senderStamp()==1401) && (message.state()==0 || message.state()==3 || message.state()==4)){
          readyState = false;
        }
      }};

      od4.dataTrigger(opendlv::proxy::SwitchStateReading::ID(), catchState);

      double t = 0;
      float freq = 0.25;
      float t1 = 26;
        // Just sleep as this microservice is data driven.
      using namespace std::literals::chrono_literals;
      while (od4.isRunning()) {
        std::this_thread::sleep_for(0.033s);
        opendlv::system::SignalStatusMessage heartBeat;
        heartBeat.code(1);

        cluon::data::TimeStamp sampleTime = cluon::time::now();

        od4BB.send(heartBeat,sampleTime,senderStamp);

        double currentTime = (double)(sampleTime.seconds() + sampleTime.microseconds()*1e-6);
        opendlv::proxy::GroundSteeringRequest groundSteeringRequest;
        opendlv::proxy::TorqueRequest torqueRequest;

        if(readyState==true){
          t = currentTime - initTime;
        }

        if(readyState==true && t < 25.5){
          groundSteeringRequest.groundSteering((float)(46.76*20*PI/180*sin(2*PI*freq*t)));

          torqueRequest.torque(25);

          od4BB.send(groundSteeringRequest,sampleTime,senderStamp);
          od4BB.send(torqueRequest,sampleTime,1502);
          od4BB.send(torqueRequest,sampleTime,1503);


        } else if(readyState==true && (t > t1 || t>28) ) {
          groundSteeringRequest.groundSteering(0.0f);
          od4BB.send(groundSteeringRequest,sampleTime,senderStamp);

          torqueRequest.torque(0);

          od4BB.send(torqueRequest,sampleTime,1502);
          od4BB.send(torqueRequest,sampleTime,1503);

        } else if(readyState==true && t > 29) {
          opendlv::proxy::SwitchStateReading message;
          message.state(1);
        }
     }

    return retCode;
}
