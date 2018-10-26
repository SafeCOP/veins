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

#include <fstream>
#include <iostream>
#include <limits>
#include <memory>

#include "veins/modules/application/traci/TraCIDemoRSU11p.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCIScenarioManager.h"

#define MYDEBUG EV

#define RECURRING_TIMER_EVT 2
#define CHECK_TRAFFIC_EVT 3
#define INITIAL_EVT 4

#define STATS_FILENAME "perfMetrics.txt"

Define_Module(TraCIDemoRSU11p);

void TraCIDemoRSU11p::initialize(int stage) {
  BaseWaveApplLayer::initialize(stage);

  std::ofstream f_perf;
  f_perf.open(STATS_FILENAME);
  if (f_perf.is_open()) {
    f_perf << "Performance metric for traffic light optimal control\n\n"
              "simulation time\tnot optimized\toptimized\ttl program"
           << std::endl;
  }
  f_perf.close();

  if (stage + 1 < numInitStages())
    return;

  // Initialize the pointer to the scenario manager
  manager = Veins::TraCIScenarioManagerAccess().get();
  assert(manager != nullptr);

  checkTrafficEvt = new cMessage("OptimalTraffic Event", CHECK_TRAFFIC_EVT);
  scheduleAt(simTime() + 90, checkTrafficEvt);

  initialEvt = new cMessage("RSU Startup Event", INITIAL_EVT);
  scheduleAt(simTime(), initialEvt);
}

static inline void dumpCapacityMetric(const std::string &TLName,
                                      simtime_t SimTime,
                                      double balancedCapacity,
                                      double maxCapacity,
                                      const std::string &CurrProgram) {
  std::ofstream f_perf;
  std::string filename = TLName + STATS_FILENAME;
  f_perf.open(filename, std::ios::app);
  if (f_perf.is_open()) {
    f_perf << SimTime << "\t\t"
           << balancedCapacity << "\t\t"
           << maxCapacity << "\t\t"
           << CurrProgram << std::endl;
  }
  f_perf.close();
}

// Traffic light cycle length
static const constexpr int t_C = 90;

// Fixed time for yellow in each direction
static const constexpr int YellowTime = 4;

// Total fixed time for yellow
static const constexpr int TotalYellowTime = 2 * YellowTime;

static double computeCapacity(const double VerGreenTime,
                              const int NorthInputFlow,
                              const int SouthInputFlow,
                              const int WestInputFlow,
                              const int EastInputFlow) {
  // Saturation flow of access i. In this case all the accesses are equal.
  // The exact reason why the value 120 was choosed it's not clear to me,
  // but I guess that given that it's a constant it does not really matter
  // for this specific example.
  static const constexpr double s_i = 120.0;

  const double f_N = static_cast<double>(NorthInputFlow);
  const double f_S = static_cast<double>(SouthInputFlow);
  const double f_W = static_cast<double>(WestInputFlow);
  const double f_E = static_cast<double>(EastInputFlow);
  const double g_E_Ver = static_cast<double>(VerGreenTime);
  const double g_E_Hor = t_C - VerGreenTime - TotalYellowTime;

  // compute capacities for vertical and horizontal directions
  double WCapacity = (WestInputFlow != 0) ? ((s_i * g_E_Hor) / (f_W * t_C)) :
                                            std::numeric_limits<double>::max();
  double ECapacity = (EastInputFlow != 0) ? ((s_i * g_E_Hor) / (f_E * t_C)) :
                                          std::numeric_limits<double>::max();
  double NCapacity = (NorthInputFlow != 0) ? ((s_i * g_E_Ver) / (f_N * t_C)) :
                                             std::numeric_limits<double>::max();
  double SCapacity = (SouthInputFlow != 0) ? ((s_i * g_E_Ver) / (f_S * t_C)) :
                                             std::numeric_limits<double>::max();
  // compute minimal capacity for this program
  return std::min(std::min(ECapacity, WCapacity), std::min(NCapacity, SCapacity));
}

static bool endsWith(const std::string &haystack, const std::string &needle) {
  const auto needle_len = needle.length();
  const auto haystack_len = haystack.length();
  if (needle_len > haystack_len)
    return false;
  const auto offset = haystack_len - needle_len;
  return std::memcmp((void *)(needle.data()),
                     (void *)(haystack.data() + offset),
                     needle_len) == 0;
}

void TraCIDemoRSU11p::setOptimalProgram(const std::string &TLName) {
  TraCICommandInterface::Trafficlight TL = traci->trafficlight(TLName);
  /**
   * This function tries to set the traffic light program to maximize the
   * intersection capacity, as defined in the paper "Global Sensitivity
   * Analysis of the Optimal Capacity of Intersections" - A. Di Febbraro,
   * N. Sacco Bauda` - IFAC Proceedings Volumes, Vol.47, Issue 3, 2014 -
   * https://doi.org/10.3182/20140824-6-ZA-1003.01975
   */
  int TLId = endsWith(TLName, "_new") ? 1 : 0;
  auto &CurrNorthInputFlow = NorthInputFlow[TLId];
  auto &CurrSouthInputFlow = SouthInputFlow[TLId];
  auto &CurrWestInputFlow = WestInputFlow[TLId];
  auto &CurrEastInputFlow = EastInputFlow[TLId];
  const double BalancedCapacity = computeCapacity(41,
                                                  CurrNorthInputFlow,
                                                  CurrSouthInputFlow,
                                                  CurrWestInputFlow,
                                                  CurrEastInputFlow);
  double MaxCapacity = 0.0;
  int MaxCapacityVerGreenSecs = 0;

  // Maximization loop on p_k, where p_k is the duration of the next green
  // phase of the intersection (for vertical direction)
  for (int VerGreenSecs = 4; VerGreenSecs < 79; ++VerGreenSecs) {

    // In this basic crossing we only have 4 accesses. The index i used in
    // the paper to represent accesses to the intersection should go from
    // 0 to 4, but we can simplify and assume only two direction:
    // horizontal and vertical.

    const double VerGreenTime = static_cast<double>(VerGreenSecs);
    const double ProgramCapacity = computeCapacity(VerGreenTime,
                                                  CurrNorthInputFlow,
                                                  CurrSouthInputFlow,
                                                  CurrWestInputFlow,
                                                  CurrEastInputFlow);

    // Store the id of the program with max capacity, along with the value of
    // the max capacity, used later for statistics
    if (ProgramCapacity > MaxCapacity) {
      MaxCapacity = ProgramCapacity;
      MaxCapacityVerGreenSecs = VerGreenSecs;
    }
  }

  std::string NewProgram = "VerticalGreen"
                           + std::to_string(MaxCapacityVerGreenSecs) + "Secs";

  dumpCapacityMetric(TLName, simTime(), BalancedCapacity, MaxCapacity, NewProgram);

  const std::string &CurrProgram = TL.getCurrentProgram();
  if (CurrProgram == NewProgram)
    return;

  int NewVerGreenSecs = MaxCapacityVerGreenSecs;
  int NewHorGreenSecs = t_C - TotalYellowTime - NewVerGreenSecs;
  using Phase = TraCICommandInterface::Trafficlight::Phase;
  std::vector<Phase> Phases = {};

  Phases.push_back({ NewVerGreenSecs, "ggrrggrr" });
  Phases.push_back({ YellowTime, "yyrryyrr" });
  Phases.push_back({ NewHorGreenSecs, "rrggrrgg" });
  Phases.push_back({ YellowTime, "rryyrryy" });

  assert(Phases.size() == 4);

  int PhaseID = TL.getPhaseIndex();
  assert(PhaseID < 4 and PhaseID > -1);

  int CurrPhaseDuration = TL.getCurrPhaseDuration();
  int NextSwitchTime = TL.getNextSwitchSecs();
  int SumoSimTime = traci->getCurrentTimeSecs();
  int CurrPhaseLeftDuration = NextSwitchTime - SumoSimTime;
  int CurrPhaseConsumedDuration = CurrPhaseDuration - CurrPhaseLeftDuration;
  int NewPhaseDuration = Phases.at(PhaseID).Duration;
  int NewPhaseLeftDuration = NewPhaseDuration - CurrPhaseConsumedDuration;

  // Set the tl program according to the maximization of the capacity
  TL.addCompleteProgram(NewProgram, Phases, 0);
  TL.setProgram(NewProgram);
  TL.setPhaseIndex(PhaseID);

  if (NewPhaseLeftDuration > 0)
    TL.setPhaseLeftDuration(NewPhaseLeftDuration);
  else
    TL.setPhaseLeftDuration(0);

  CurrSouthInputFlow = 0;
  CurrNorthInputFlow = 0;
  CurrWestInputFlow = 0;
  CurrEastInputFlow = 0;

  return;
}

void TraCIDemoRSU11p::computeTrafficFlows(TraCICommandInterface *traci) {
  // Count the number of vehicles in horizontal and vertical directions
  for (int TLId = 0; TLId < NumIntersections; ++TLId) {
    auto &TLNorthInputFlow = NorthInputFlow[TLId];
    auto &TLSouthInputFlow = SouthInputFlow[TLId];
    auto &TLWestInputFlow = WestInputFlow[TLId];
    auto &TLEastInputFlow = EastInputFlow[TLId];
    auto &TLNorthVehicles = PrevNorthVehicles[TLId];
    auto &TLSouthVehicles = PrevSouthVehicles[TLId];
    auto &TLWestVehicles = PrevWestVehicles[TLId];
    auto &TLEastVehicles = PrevEastVehicles[TLId];

    std::set<std::string> CurrNorthVehicles;
    std::set<std::string> CurrSouthVehicles;
    std::set<std::string> CurrWestVehicles;
    std::set<std::string> CurrEastVehicles;

    for (const auto &ID : traci->getLaneAreaDetectorIds()) {
      // This is ad-hoc to tell the two traffic lights apart
      if (TLId != endsWith(ID, "_new"))
        continue;
      auto LaneDetector = traci->laneAreaDetector(ID);
      std::set<std::string> Vehicles = LaneDetector.getLastStepVehicleIDs();

      // This is ad-hoc to tell horizontal and vertical lanes apart.
      // It depends on the names given to the lanes objects in the
      // configuration files and it may change in future.
      assert(ID.length() > 4);
      char c = ID.at(4);
      switch (c) {
        case 'E':
          CurrEastVehicles.insert(Vehicles.begin(), Vehicles.end());
          break;
        case 'W':
          CurrWestVehicles.insert(Vehicles.begin(), Vehicles.end());
          break;
        case 'S':
          CurrSouthVehicles.insert(Vehicles.begin(), Vehicles.end());
          break;
        case 'N':
          CurrNorthVehicles.insert(Vehicles.begin(), Vehicles.end());
          break;
        default:
          abort();
          break;
      }
    }

    for (const auto &V : TLNorthVehicles)
      if (CurrNorthVehicles.count(V) != 0)
        TLNorthInputFlow += 1;

    for (const auto &V : TLSouthVehicles)
      if (CurrSouthVehicles.count(V) != 0)
        TLSouthInputFlow += 1;

    for (const auto &V : TLEastVehicles)
      if (CurrEastVehicles.count(V) != 0)
        TLEastInputFlow += 1;

    for (const auto &V : TLWestVehicles)
      if (CurrWestVehicles.count(V) != 0)
        TLWestInputFlow += 1;

    TLNorthVehicles = std::move(CurrNorthVehicles);
    TLSouthVehicles = std::move(CurrSouthVehicles);
    TLEastVehicles = std::move(CurrEastVehicles);
    TLWestVehicles = std::move(CurrWestVehicles);
  }
}

static void setBalanced(TraCICommandInterface::Trafficlight &TL) {
  assert(TL.hasTraCI());
  assert(TL.hasConnection());
  TL.setProgram("Balanced");
  TL.setPhaseIndex(0);
}

void TraCIDemoRSU11p::handleSelfMsg(cMessage *msg) {

  switch (msg->getKind()) {

  case INITIAL_EVT: {
    // Initialize member pointers
    // We only do it here because this message comes at the beginning of the
    // simulation
    manager = Veins::TraCIScenarioManagerAccess().get();
    assert(manager != nullptr);
    traci = manager->getCommandInterface();
    assert(traci != nullptr);

    // This event is supposed to be received only once at the beginning of
    // simulation time and it is used for two things:
    // 1 - setup the traffic light with the default balanced policy
    // 2 - setup the recurring timer event that will wake up the RSU every
    //     second to collect statistics about the vehicles in the various
    //     lanes, and to decide the new policies to adopt according to the
    //     traffic

    // Set the default balanced program for the traffic lights
    auto OldTL = traci->trafficlight("Crossing");
    auto NewTL = traci->trafficlight("Crossing_new");
    setBalanced(OldTL);
    setBalanced(NewTL);

    // Cleanup the event
    cancelEvent(initialEvt);

    // Setup the recurring timer event
    sendTimerEvt = new cMessage("Timer Event", RECURRING_TIMER_EVT);
    scheduleAt(simTime() + 1, sendTimerEvt);

  } break;

  case RECURRING_TIMER_EVT: {
    // First, handle the recurring timer event and set it up for the new
    // occurence
    cancelEvent(sendTimerEvt);
    scheduleAt(simTime() + 1, sendTimerEvt);

    // Compute horizontal and vertical input traffic flows
    computeTrafficFlows(traci);

  } break;

  case CHECK_TRAFFIC_EVT: {
    setOptimalProgram("Crossing");
    setOptimalProgram("Crossing_new");
    cancelEvent(checkTrafficEvt);
    scheduleAt(simTime() + 5, checkTrafficEvt);
  } break;

  default:
    // do nothing
    break;
  }
}

static inline void cleanupMsg(TraCIDemoRSU11p *RSU, cMessage **MsgPtr) {
  assert(MsgPtr != nullptr);
  cMessage *Msg = *MsgPtr;
  if (Msg != nullptr) {
    if (Msg->isScheduled())
      RSU->cancelEvent(Msg);
  }
  delete Msg;
  Msg = nullptr;
}

void TraCIDemoRSU11p::finish() {

  for (int TLID = 0; TLID < NumIntersections; ++TLID) {
    PrevNorthVehicles[TLID] = {};
    PrevSouthVehicles[TLID] = {};
    PrevEastVehicles[TLID] = {};
    PrevWestVehicles[TLID] = {};
    NorthInputFlow[TLID] = 0;
    SouthInputFlow[TLID] = 0;
    EastInputFlow[TLID] = 0;
    WestInputFlow[TLID] = 0;
  }

  cleanupMsg(this, &sendTimerEvt);
  cleanupMsg(this, &checkTrafficEvt);
  cleanupMsg(this, &initialEvt);

  manager = nullptr;

  BaseWaveApplLayer::finish();
}
