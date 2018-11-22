//
// Copyright (C) 2016 David Eckhoff <david.eckhoff@fau.de>
//
// Documentation for these modules is at http://veins.car2x.org/
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

#ifndef TraCIDemoRSU11p_H
#define TraCIDemoRSU11p_H

#include <set>
#include <array>

#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"

static const constexpr int NumIntersections = 2;
static const constexpr int Redundancy = 2;

enum Direction {
  North = 0,
  East = 1,
  South = 2,
  West = 3,
};

inline int charToInt(const char c) {
  if (c < '0' or c > '9')
    abort();
  return c - '0';
}

inline Direction getDirection(const char c) {
  switch (c) {
    case 'N':
      return North;
    case 'E':
      return East;
    case 'S':
      return South;
    case 'W':
      return West;
    default:
      abort();
  }
}

using VehicleSet = std::set<std::string>;
using RedundantVehicleSet = std::array<VehicleSet, Redundancy>;
using RedundantInputFlow = std::array<int, Redundancy>;

struct RedundantCrossingData {

  RedundantCrossingData() = default;
  RedundantCrossingData(const RedundantCrossingData &) = default;
  RedundantCrossingData(RedundantCrossingData &&) = default;
  RedundantCrossingData &operator=(const RedundantCrossingData &) = default;
  RedundantCrossingData &operator=(RedundantCrossingData &&) = default;

  std::array<int, 4> RealFlow;
  std::array<RedundantInputFlow, 4> DetectedFlow;
  std::array<VehicleSet, 4> RealVehicles;
  std::array<RedundantVehicleSet, 4> DetectedVehicles;

};

struct SafeCOPScenario {
  SafeCOPScenario() = default;
  SafeCOPScenario(const SafeCOPScenario &) = default;
  SafeCOPScenario(SafeCOPScenario &&) = default;
  SafeCOPScenario &operator=(const SafeCOPScenario &) = default;
  SafeCOPScenario &operator=(SafeCOPScenario &&) = default;

  std::array<RedundantCrossingData, NumIntersections> TrafficLights;
};

/**
 * Small RSU Demo using 11p
 */
class TraCIDemoRSU11p : public BaseWaveApplLayer {
private:
  cMessage *sendTimerEvt = nullptr;
  cMessage *checkTrafficEvt = nullptr;
  cMessage *initialEvt = nullptr;
  Veins::TraCIScenarioManager *manager = nullptr;

public:
  virtual void initialize(int stage) override;
  virtual void finish() override;

protected:
  virtual void handleSelfMsg(cMessage *msg);
  void setOptimalProgram(const std::string &TLName);
  void computeTrafficFlows(TraCICommandInterface *traci);

  SafeCOPScenario Scenario;

};

#endif
