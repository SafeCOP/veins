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

#include "veins/modules/application/ieee80211p/BaseWaveApplLayer.h"

/**
 * Small RSU Demo using 11p
 */
class TraCIDemoRSU11p : public BaseWaveApplLayer {
    private:
        cMessage* sendTimerEvt;
        cMessage* checkTrafficEvt;
	public:
		virtual void initialize(int stage);
	protected:
		int currentSubscribedServiceId=-1;
		virtual void onWSM(WaveShortMessage* wsm);
		virtual void onWSA(WaveServiceAdvertisment* wsa);
		virtual void handleSelfMsg(cMessage* msg);
		virtual void handleOptimalTraffic(TraCICommandInterface*);  //method for applying optimal control algorithm
		virtual void handleVerCongestion(TraCICommandInterface*);     //method to handle vertical traffic congestion controlled at each simTime
		virtual void handleHorCongestion(TraCICommandInterface*);     //method to handle horizontal traffic congestion controlled at each simTime
		virtual void computePerformanceMetric(double, double);

		int g[3] = {41, 78, 4};   //find a better way to query SUMO via TraCI for knowing the duration of the green
		int horTrafficHist=0;
		int verTrafficHist=0;
		int bufHor=0;
		int bufVer=0;
		std::string programUsed = "0";
};

#endif
