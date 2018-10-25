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

#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"

/**
 * Small RSU Demo using 11p
 */
class TraCIDemoRSU11p : public BaseWaveApplLayer {
private:
  cMessage *sendTimerEvt = nullptr;
  cMessage *checkTrafficEvt = nullptr;
  cMessage *initialEvt = nullptr;
  Veins::TraCIScenarioManager *manager = nullptr;

  static const constexpr int NumIntersections = 2;

public:
  virtual void initialize(int stage) override;
  virtual void finish() override;

protected:
  virtual void handleSelfMsg(cMessage *msg);
  void setOptimalProgram(const std::string &TLName);
  void computeTrafficFlows(TraCICommandInterface *traci);

  std::set<std::string> PrevHorVehicles[NumIntersections] = { {}, {} };
  std::set<std::string> PrevVerVehicles[NumIntersections] = { {}, {} };
  int HorInputFlow[NumIntersections] = { 0 , 0 };
  int VerInputFlow[NumIntersections] = { 0 , 0 };
};

#endif
