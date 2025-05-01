/*
 * Copyright (C) 2019 Ola Benderius
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

#include <iostream>
 #include <cstdint>
 #include <memory>
 #include <mutex>
 #include <thread>
 #include <array>
 #include <string>
 #include <sstream>
 #include <vector>
 #include <iterator>
 #include <cmath>
 #include <random>
 #include <chrono>

#include "cluon-complete.hpp"
#include "opendlv-standard-message-set.hpp"
#include "tinyso.hpp"

int32_t main(int32_t argc, char **argv) {
  int32_t retCode{0};
  auto commandlineArguments = cluon::getCommandlineArguments(argc, argv);
  if (0 == commandlineArguments.count("j")) {
    std::cerr << argv[0] 
      << " is a trainer for a crazyflie control algorithm." << std::endl;
    std::cerr << "Usage:   " << argv[0] 
      << " --j=<Number of parallel threads (simulations)>" 
      << " [--cid-start=<CID interval start (end: cid-start+j). Default: 111>]" 
      << " [--verbose]" << std::endl;
    std::cerr << "Example: " << argv[0] << " --j=2 --verbose" << std::endl;
    retCode = 1;
  } 
  else if ( 0 == commandlineArguments.count("maxtime") ){
    std::cerr << "Please include the max time" << std::endl;
  }
  else {
    bool const verbose = (commandlineArguments.count("verbose") != 0);
    uint16_t const jobs = std::stoi(commandlineArguments["j"]);
    uint16_t const maxTime = std::stoi(commandlineArguments["maxtime"]);
    uint32_t const cidStart = (commandlineArguments.count("cid-start") != 0) 
      ? std::stoi(commandlineArguments["cid-start"]) : 111;

    uint32_t generationCount = 10000;
    tinyso::CrossoverMethod crossoverMethod = tinyso::CrossoverMethod::Split;
    uint32_t individualLength = 12;
    uint32_t eliteSize = 1;
    uint32_t populationSize = 30;
    uint32_t tournamentSize = 2;
    float probCrossover = 0.3f;
    float probMutation = 0.1f;
    float probSelectTournament = 0.8f;
    
    uint32_t simMaxTime = 4000;
    bool randomSeed = true;

    std::random_device rd;
    std::mt19937 rg(rd());

    bool terminate{false};

    auto StartTime = std::chrono::high_resolution_clock::now();
    auto EndTime = std::chrono::high_resolution_clock::now();

    std::map<uint32_t, std::mutex> cidLocks;
    for (uint32_t i{0}; i < jobs; i++) {
      uint32_t cid = cidStart + i;
      cidLocks.emplace(std::piecewise_construct, std::make_tuple(cid),
          std::make_tuple());
    }

    auto evaluate{[&cidStart, &jobs, &cidLocks, &terminate, &StartTime, &EndTime, &maxTime](
        tinyso::Individual const &ind, uint32_t const) -> double
      {
        uint32_t cid = cidStart;
        while (!cidLocks[cid].try_lock()) {
          std::this_thread::sleep_for(std::chrono::milliseconds(10));
          cid++;
          if (cid > cidStart + jobs) {
            cid = cidStart;
          }
        }

        double eta{1.0};

        struct flagStruct{
          int16_t task_completed{0};
          float task_elapsed{0};
          float avg_obsstatic_elapsed{0};
          int16_t obsstatic_timer{0};
          float avg_obsdynamic_elapsed{0};
          int16_t obsdynamic_timer{0};
          float avg_targetfinding_elapsed{0};
          int16_t targetfinding_timer{0};
          float avg_frontreaching_elapsed{0};
          int16_t frontreaching_timer{0};
          float avg_lookaround_elapsed{0};
          int16_t lookaround_timer{0};
          float avg_closeball_elapsed{0};
          int16_t closeball_timer{0};
          float avg_closestaticobs_elapsed{0};
          int16_t closestaticobs_timer{0};
          int16_t stuck_timer{0};
        };
        flagStruct cur_flagStruct;

        {
          cluon::OD4Session od4(cid);
          bool isRunning{true};

          auto onFlagRead{[&od4, &cur_flagStruct](
              cluon::data::Envelope &&envelope)
            {
              auto msg = cluon::extractMessage<opendlv::logic::sensation::CompleteFlag>(
                  std::move(envelope));

              cur_flagStruct.task_completed = msg.task_completed();
              if ( cur_flagStruct.task_completed == 1 ){
                cur_flagStruct.task_elapsed = msg.task_elapsed();
                cur_flagStruct.avg_obsstatic_elapsed = msg.avg_obsstatic_elapsed();
                cur_flagStruct.obsstatic_timer = msg.obsstatic_timer();
                cur_flagStruct.avg_obsdynamic_elapsed = msg.avg_obsdynamic_elapsed();
                cur_flagStruct.obsdynamic_timer = msg.obsdynamic_timer();
                cur_flagStruct.avg_targetfinding_elapsed = msg.avg_targetfinding_elapsed();
                cur_flagStruct.targetfinding_timer = msg.targetfinding_timer();
                cur_flagStruct.avg_frontreaching_elapsed = msg.avg_frontreaching_elapsed();
                cur_flagStruct.frontreaching_timer = msg.frontreaching_timer();
                cur_flagStruct.avg_lookaround_elapsed = msg.avg_lookaround_elapsed();
                cur_flagStruct.lookaround_timer = msg.lookaround_timer();
                cur_flagStruct.avg_closeball_elapsed = msg.avg_closeball_elapsed();
                cur_flagStruct.closeball_timer = msg.closeball_timer();
                cur_flagStruct.avg_closestaticobs_elapsed = msg.avg_closestaticobs_elapsed();
                cur_flagStruct.closestaticobs_timer = msg.closestaticobs_timer();
                cur_flagStruct.stuck_timer = msg.stuck_timer();
              }
              else{
                // Do nothing
              }
            }};

          od4.dataTrigger(opendlv::logic::sensation::CompleteFlag::ID(), onFlagRead);

          struct cfPos {
              float x;
              float y;
          };
          std::mutex frameMutex;
          cfPos cur_pos{-4.0f, -4.0f};
          auto onFrame{[&cur_pos, &frameMutex](cluon::data::Envelope &&envelope)
          {
              uint32_t const senderStamp = envelope.senderStamp();
              auto frame = cluon::extractMessage<opendlv::sim::Frame>(std::move(envelope));
              std::lock_guard<std::mutex> lck(frameMutex);
              switch (senderStamp) {
                  case 0: 
                      cur_pos.x = frame.x();
                      cur_pos.y = frame.y();
                      break;
              }
          }};
          od4.dataTrigger(opendlv::sim::Frame::ID(), onFrame);

          std::mutex stateMutex;
          std::atomic<float> cur_state_yaw{0.0f};
          auto onStateRead = [&cur_state_yaw](cluon::data::Envelope &&env){
              auto senderStamp = env.senderStamp();
              // Now, we unpack the cluon::data::Envelope to get the desired DistanceReading.
              opendlv::logic::sensation::CrazyFlieState cfState = cluon::extractMessage<opendlv::logic::sensation::CrazyFlieState>(std::move(env));
              // Store distance readings.
              //  std::lock_guard<std::mutex> lck(stateMutex);
              cur_state_yaw = cfState.cur_yaw(); 
          };
          // Finally, we register our lambda for the message identifier for opendlv::proxy::DistanceReading.
          od4.dataTrigger(opendlv::logic::sensation::CrazyFlieState::ID(), onStateRead);

          // Give the param individuals
          opendlv::logic::sensation::GAParam gaParam;
          gaParam.safeDist_ratio(ind[0]);
          gaParam.dodge_dist_totune(ind[1]);
          gaParam.cur_distToMove_ratio(ind[2]);
          gaParam.time_ToMove_ratio(ind[3]);
          gaParam.cur_distToMove_goto_ratio(ind[4]);
          gaParam.time_ToMove_goto_ratio(ind[5]);
          gaParam.cur_distToMove_target_ratio(ind[6]);
          gaParam.time_ToMove_target_ratio(ind[7]);
          gaParam.angTurn_targetFinding(ind[8]);
          gaParam.time_ToTurn_ratio(ind[9]);
          gaParam.angTurn_lookAround(ind[10]);
          gaParam.ReadyToStart(1);
          od4.send(gaParam);

          // Setup start time
          while(cur_flagStruct.task_completed == 1){
            std::this_thread::sleep_for(std::chrono::milliseconds(10));
          } // Wait until task set
          StartTime = std::chrono::high_resolution_clock::now();
          std::cout << "Start the " << jobs << " thread training..." << std::endl;          
          std::cout << ", with params: " << std::endl;
          for (uint32_t i{0}; i < 12; i++) {
            std::cout << "  param " << i << ": " << ind[i] << std::endl;
          }

          bool hasOverTimeLimit = false;
          bool hasOverRange = false;
          bool hasStuck = false;
          cfPos pre_pos = {-4.0f, -4.0f};
          int nCount = 0;
          float dist_stuck = -1.0f;
          while (cur_flagStruct.task_completed == 0 && od4.isRunning()) {
            std::this_thread::sleep_for(std::chrono::milliseconds(10));

            // Check end time
            EndTime = std::chrono::high_resolution_clock::now();
            const std::chrono::duration<double> elapsed = EndTime - StartTime;
            if ( elapsed.count() >= maxTime ){
              hasOverTimeLimit = true;
              std::cout << "The training over time limit..." << std::endl;
              break;
            }

            if ( cur_pos.x != -4.0f && cur_pos.y != -4.0f ){
              pre_pos = cur_pos;
              if ( cur_pos.x < -1.0f || cur_pos.x > 1.5f || cur_pos.y < -1.5f || cur_pos.y > 0.5f ){
                hasOverRange = true;
                std::cout << "The training over the room range..." << std::endl;
                break;
              }
            }

            if ( cur_flagStruct.stuck_timer >= 1 ){
              hasStuck = true;
              std::cout << "The training stucks at some points..." << std::endl;
              break;
            }

            if ( cur_pos.x != -4.0f && cur_pos.y != -4.0f ){
              if ( nCount < 2000 ){
                nCount += 1;
              }
              else{
                float dist = std::sqrt(std::pow(cur_pos.x-pre_pos.x,2) + std::pow(cur_pos.y-pre_pos.y,2));
                if ( dist <= 0.01f ){
                  hasStuck = true;
                  std::cout << "The training stucks at some points..." << std::endl;
                  break;
                }
                pre_pos = cur_pos;
                nCount = 0;
              }
            }            
          }

          if ( !od4.isRunning() ){
            terminate = true;
          }

          if (cur_flagStruct.task_completed == 1 || hasOverTimeLimit || hasOverRange || hasStuck ) {
            std::cout << "The " << jobs << " thread training should stop now" << std::endl;

            // Reset param
            opendlv::logic::sensation::GAParam gaParam;
            gaParam.ReadyToStart(0);
            od4.send(gaParam);

            while(cur_flagStruct.task_completed == 0){
              std::this_thread::sleep_for(std::chrono::milliseconds(10));
            } // Wait until task completed

            // Reset position
            opendlv::logic::action::CrazyFlieCommand cfcommand;
            cluon::data::TimeStamp sampleTime;
            od4.send(cfcommand, sampleTime, 4);

            float dist = std::sqrt(std::pow(cur_pos.x,2) + std::pow(cur_pos.y,2));
            while( dist > 0.01f || std::abs( cur_state_yaw ) >= 1.0f / 180.0f * M_PI ){
              std::this_thread::sleep_for(std::chrono::milliseconds(10));
              dist = std::sqrt(std::pow(cur_pos.x,2) + std::pow(cur_pos.y,2));
              // std::cout << "The distance is: " << dist << ",x: " << cur_pos.x << ",y: " << cur_pos.y << std::endl;
            } // Wait until the current position back to (0,0)
            std::cout << "The " << jobs << " thread training complete reset..." << std::endl;
          }

          // Elapsed
          if ( hasOverTimeLimit || hasOverRange || hasStuck ){
            eta = 300;
          }
          else{
            eta += std::isnan(cur_flagStruct.task_elapsed) ? 0.0 : cur_flagStruct.task_elapsed;
            eta += std::isnan(cur_flagStruct.avg_obsstatic_elapsed) ? 0.0 : cur_flagStruct.avg_obsstatic_elapsed;
            eta += std::isnan(cur_flagStruct.avg_obsdynamic_elapsed) ? 0.0 : cur_flagStruct.avg_obsdynamic_elapsed;
            eta += std::isnan(cur_flagStruct.avg_lookaround_elapsed) ? 0.0 : cur_flagStruct.avg_lookaround_elapsed;
            eta += std::isnan(cur_flagStruct.avg_targetfinding_elapsed) ? 0.0 : 0.5 * cur_flagStruct.avg_targetfinding_elapsed;
            eta += std::isnan(cur_flagStruct.avg_frontreaching_elapsed) ? 0.0 : 0.5 * cur_flagStruct.avg_frontreaching_elapsed;
          }

          // Count
          eta += cur_flagStruct.obsstatic_timer >= 0.0f ? cur_flagStruct.obsstatic_timer : 10;
          eta += cur_flagStruct.obsdynamic_timer >= 0.0f ? cur_flagStruct.obsdynamic_timer : 10;
          eta += cur_flagStruct.lookaround_timer >= 0.0f ? cur_flagStruct.lookaround_timer : 30;
          eta += cur_flagStruct.targetfinding_timer >= 0.0f ? 0.5*cur_flagStruct.targetfinding_timer : 15;
          eta += cur_flagStruct.frontreaching_timer >= 0.0f ? 0.5*cur_flagStruct.frontreaching_timer : 15;
          eta += cur_flagStruct.closeball_timer;
          eta += cur_flagStruct.closestaticobs_timer;

          // Percentage
          float percentage = ( std::isnan(cur_flagStruct.avg_obsstatic_elapsed) ? 0.0 : cur_flagStruct.avg_obsstatic_elapsed * cur_flagStruct.obsstatic_timer 
          + std::isnan(cur_flagStruct.avg_obsdynamic_elapsed) ? 0.0 : cur_flagStruct.avg_obsdynamic_elapsed * cur_flagStruct.obsdynamic_timer
          + std::isnan(cur_flagStruct.avg_lookaround_elapsed) ? 0.0 : cur_flagStruct.avg_lookaround_elapsed * cur_flagStruct.lookaround_timer ) 
          / cur_flagStruct.task_elapsed;
          eta += percentage > 0.0f ? percentage : 1.0f;   
          
          std::cout << "The " << jobs << " thread training complete fitness computing: " << 1.0 / eta << std::endl;
          std::this_thread::sleep_for(std::chrono::milliseconds(10));

          cidLocks[cid].unlock();
        }
        return 1.0 / eta;
      }};


    if (verbose) {
      std::cout << "Starting the training using " << jobs << " threads." 
        << std::endl;
    }

    tinyso::GeneticAlgorithm ga(evaluate, crossoverMethod, individualLength, 
        eliteSize, populationSize, tournamentSize, probCrossover, probMutation, 
        probSelectTournament);

    for (uint32_t i{0}; i < generationCount && !terminate; i++) {
      ga.NextGeneration(jobs);

      if (verbose) {
        auto bestInd = ga.GetBestIndividual();
        std::cout << " .. generation " << i << ", best fitness " 
          << ga.GetBestFitness() << " (" << bestInd[0];
        for (uint32_t j{1}; j < individualLength; j++) {
          std::cout << ", " << bestInd[j];
        }
        std::cout << ")" << std::endl;
      }
    }
      
    if (verbose) {
      auto bestInd = ga.GetBestIndividual();
      std::cout << "Training done, best fitness " 
        << ga.GetBestFitness() << std::endl;
      for (uint32_t i{0}; i < individualLength; i++) {
        std::cout << "  param " << i << ": " << bestInd[i] << std::endl;
      }
    }
    retCode = 0;
  }
  return retCode;
}
