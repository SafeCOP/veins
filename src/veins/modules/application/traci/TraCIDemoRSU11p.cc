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
#define MYDEBUG EV
#define SEND_TIMER_EVT 2
#define CHECK_TRAFFIC_EVT 3

#define NUM_PROGRAMS_TL 3
#define NUM_MAX_CROSSINGVEHICLES 120
#define TL_CYCLE 90

#include "veins/modules/application/traci/TraCIDemoRSU11p.h"
#include "veins/modules/mobility/traci/TraCIMobility.h"
#include "veins/modules/mobility/traci/TraCICommandInterface.h"
#include "veins/modules/mobility/traci/TraCIScenarioManager.h"
#include <fstream>
#include <iostream>

Define_Module(TraCIDemoRSU11p);

void TraCIDemoRSU11p::initialize(int stage) {
    BaseWaveApplLayer::initialize(stage);

    std::ofstream f_perf;
    f_perf.open("perfMetrics.txt");
    if (f_perf.is_open()) {
        f_perf << "Performance metric for traffic light optimal control\n\n" << "not optimized\toptimized\ttl program\n";
    }
    f_perf.close();

    sendTimerEvt = new cMessage("timer evt", SEND_TIMER_EVT);
    scheduleAt(simTime()+10, sendTimerEvt);

    checkTrafficEvt = new cMessage("OptimalTraffic evt", CHECK_TRAFFIC_EVT);
    scheduleAt(simTime()+10, checkTrafficEvt);

    if (stage == 0) {
        currentSubscribedServiceId = -1;
		if (traci==0) {
			MYDEBUG << "No traci variable yet..." << std::endl;
			return;
		}
	MYDEBUG << "Traci available, setting program for C" << std::endl;
        traci->trafficlight("C").setProgram("0");
    }
}

void TraCIDemoRSU11p::computePerformanceMetric(double capacity, double maxCapacity) {
    //double metric;
    std::ofstream f_perf;

    f_perf.open("perfMetrics.txt", std::ios::app);
    if (f_perf.is_open()) {
        //metric = maxCapacity / capacity;
        f_perf << capacity << "\t\t" << maxCapacity << "\t\t" << this->programUsed << "\n";
    }
    else
        std::cout << "Oops, something went wrong.\n";

    f_perf.close();


}

void TraCIDemoRSU11p::handleVerCongestion(TraCICommandInterface* traci)  {
    if (this->programUsed.compare("1") != 0) {
        traci->trafficlight("C").setProgram("1");
        traci->trafficlight("C").setPhaseIndex(3);
        this->programUsed = "1";
    }
    cancelEvent(checkTrafficEvt);
    scheduleAt(simTime()+10, checkTrafficEvt);
}

void TraCIDemoRSU11p::handleHorCongestion(TraCICommandInterface* traci)  {
    if (this->programUsed.compare("2") != 0) {
        traci->trafficlight("C").setProgram("2");
        traci->trafficlight("C").setPhaseIndex(1);
        this->programUsed = "2";
    }
    cancelEvent(checkTrafficEvt);
    scheduleAt(simTime()+10, checkTrafficEvt);
}

void TraCIDemoRSU11p::handleOptimalTraffic(TraCICommandInterface* traci) {
    double capacity[NUM_PROGRAMS_TL]={0, 0, 0}, capacityHor = 0, capacityVer = 0, maxCapacity = 0.0;
    int idx_maxCapacity=0, phase =-1;
    std::string program;

    for (int i=0; i < NUM_PROGRAMS_TL; ++i)  {
        if (verTrafficHist != 0 && horTrafficHist != 0) {
            capacityVer = (NUM_MAX_CROSSINGVEHICLES * (this->g)[i]) / (verTrafficHist*TL_CYCLE);
            capacityHor = (NUM_MAX_CROSSINGVEHICLES * (TL_CYCLE - (this->g)[i] - 8)) / (horTrafficHist*TL_CYCLE);
            capacity[i] = capacityVer <= capacityHor ? capacityVer : capacityHor;
        }
        else if (verTrafficHist == 0 && horTrafficHist !=0) {
            capacityHor = (NUM_MAX_CROSSINGVEHICLES * (TL_CYCLE - (this->g)[i] - 8)) / (horTrafficHist*TL_CYCLE);
            capacity[i] = capacityHor;
        }
        else if (verTrafficHist != 0 && horTrafficHist ==0) {
            capacityVer = (NUM_MAX_CROSSINGVEHICLES * (this->g)[i]) / (verTrafficHist*TL_CYCLE);
            capacity[i] = capacityVer;
        }

        if (capacity[i] > maxCapacity) {
            idx_maxCapacity = i + 1;
            maxCapacity = capacity[i];
        }
    }
    //now we set the tl program according to the maximization of the capacity
    if (idx_maxCapacity) {
        idx_maxCapacity -= 1;
        program = std::to_string(idx_maxCapacity);
    }
    else {
            /*if (verTrafficHist)
                program = "1";
            else if (horTrafficHist)
                program = "2";
            else*/
        program = "0";
    }

    if (program.compare("1") == 0) {
        if (this->programUsed.compare(program) != 0) {
            phase = traci->trafficlight("C").getPhaseIndex();
            traci->trafficlight("C").setProgram(program);
            traci->trafficlight("C").setPhaseIndex(phase);
        }
        else {
            traci->trafficlight("C").setProgram(program);
            traci->trafficlight("C").setPhaseIndex(0);
        }
    }
    else if (program.compare("2") == 0) {
        if (this->programUsed.compare(program) != 0) {
            phase = traci->trafficlight("C").getPhaseIndex();
            traci->trafficlight("C").setProgram(program);
            traci->trafficlight("C").setPhaseIndex(phase);
        }
        else {
            traci->trafficlight("C").setProgram(program);
            traci->trafficlight("C").setPhaseIndex(2);
        }
    }
    else {
        phase = traci->trafficlight("C").getPhaseIndex();
        traci->trafficlight("C").setProgram(program);
        traci->trafficlight("C").setPhaseIndex(phase);
    }

    /*if (verTrafficHist) {
            capacityVer = (NUM_MAX_CROSSINGVEHICLES * (this->g)[i]) / (verTrafficHist*TL_CYCLE);
        }
        if (horTrafficHist) {
            capacityHor = (NUM_MAX_CROSSINGVEHICLES * (TL_CYCLE - (this->g)[i] - 8)) / (horTrafficHist*TL_CYCLE);
        }
        capacity[i] = capacityVer <= capacityHor ? capacityVer : capacityHor;
        if (capacity[i] > maxCapacity) {
            idx_maxCapacity = i + 1;
            maxCapacity = capacity[i];
        }*/

/*if (verTrafficHist > horTrafficHist) {
        if (this->programUsed.compare(program) != 0) {
            traci->trafficlight("C").setProgram(program);
            traci->trafficlight("C").setPhaseIndex(3);
        }
        else {
            traci->trafficlight("C").setProgram(program);
            traci->trafficlight("C").setPhaseIndex(0);
        }
    }
    else if (verTrafficHist < horTrafficHist) {
        if (this->programUsed.compare(program) != 0) {
            traci->trafficlight("C").setProgram(program);
            traci->trafficlight("C").setPhaseIndex(1);
        }
        else {
            traci->trafficlight("C").setProgram(program);
            traci->trafficlight("C").setPhaseIndex(2);
        }
    }
    else {
        phase = traci->trafficlight("C").getPhaseIndex();
        traci->trafficlight("C").setProgram(program);
        traci->trafficlight("C").setPhaseIndex(phase);
    }*/
    this->programUsed = program;
    this->computePerformanceMetric(capacity[0], maxCapacity);
    MYDEBUG << "Traci available, setting program for C " << program << std::endl;
    verTrafficHist = 0;
    horTrafficHist = 0;
    cancelEvent(checkTrafficEvt);
    scheduleAt(simTime()+10, checkTrafficEvt);
}

void TraCIDemoRSU11p::onWSA(WaveServiceAdvertisment* wsa) {
   /* MYDEBUG << "WSA Msg " << std::endl;
   if (currentSubscribedServiceId == -1) {
        mac->changeServiceChannel(wsa->getTargetChannel());
   }
     //if this RSU receives a WSA for service 42, it will tune to the chan
    if (wsa->getPsid() == 42) {
        mac->changeServiceChannel(wsa->getTargetChannel());
    } */
}

void TraCIDemoRSU11p::onWSM(WaveShortMessage* wsm) {
    /* MYDEBUG << "WSM Msg " << wsm->getWsmData() << std::endl;
    if (strcmp(wsm->getWsmData(),"Alert")==0){
        MYDEBUG << "Stop the traffic" << std::endl;
        Veins::TraCIScenarioManager *manager = Veins::TraCIScenarioManagerAccess().get();
        traci = manager->getCommandInterface();
        if(!traci) {
                MYDEBUG << "Failed to get the interface" << std::endl;
                return;
        }
        traci->trafficlight("C").setProgram("3");
    }
    wsm->setSenderAddress(myId);
    sendDelayedDown(wsm->dup(), 2 + uniform(0.01,0.2)); */
}

void TraCIDemoRSU11p::handleSelfMsg(cMessage* msg) {
    Veins::TraCIScenarioManager *manager = Veins::TraCIScenarioManagerAccess().get();
    traci = manager->getCommandInterface();
    if(!traci) {
        MYDEBUG << "Failed to get the interface" << std::endl;
        return;
    }

    if (msg->getKind() == CHECK_TRAFFIC_EVT) {
        //this->handleOptimalTraffic(traci);
    }
    if(msg->getKind() == SEND_TIMER_EVT)
    {
        int verticalTraffic = 0;
        int horizontalTraffic = 0;

        std::list<std::string> laneAreaDetectorIds = traci->getLaneAreaDetectorIds();

        for(std::list<std::string>::iterator it=laneAreaDetectorIds.begin(); it != laneAreaDetectorIds.end(); ++it) {
            if(it->c_str()[0] == 'E' || it->c_str()[0] == 'W')
                horizontalTraffic = horizontalTraffic + traci->laneAreaDetector(*it).getLastStepVehicleNumber();
            else
                verticalTraffic = verticalTraffic + traci->laneAreaDetector(*it).getLastStepVehicleNumber();
        }
        if (horizontalTraffic > bufHor) {
            horTrafficHist += (horizontalTraffic - bufHor);
        }
        if (verticalTraffic > bufVer) {
            verTrafficHist += (verticalTraffic - bufVer);
        }

        /*control on traffic congestion in a direction
         * threshold set to 14 vehicles in the vertical direction
         * threshold set to 18 vehicles in the horizontal direction
         * */
        if (verticalTraffic > 14 && this->programUsed.compare("1") != 0) {
            //this->handleVerCongestion(traci);
            verTrafficHist = verticalTraffic;
            //this->handleOptimalTraffic(traci);
        }
        if (horizontalTraffic > 18 && this->programUsed.compare("2") != 0) {
            //this->handleHorCongestion(traci);
            horTrafficHist = horizontalTraffic;
            //this->handleOptimalTraffic(traci);
        }

        bufHor = horizontalTraffic;
        bufVer = verticalTraffic;
	
        /*switch(traci->trafficlight("C").getPhaseIndex()) {
            case 0:    if(verticalTraffic > horizontalTraffic) {
                        traci->trafficlight("C").setPhaseIndex(1);
                       }
                       else if(verticalTraffic < horizontalTraffic) {
                        traci->trafficlight("C").setPhaseIndex(0);
                       }
                       break;
            case 1:    if(verticalTraffic < horizontalTraffic) {
                        traci->trafficlight("C").setPhaseIndex(2);
                       }
                       break;
            case 2:    if(verticalTraffic > horizontalTraffic) {
                        traci->trafficlight("C").setPhaseIndex(2);
                       }
                       else if(verticalTraffic < horizontalTraffic) {
                        traci->trafficlight("C").setPhaseIndex(3);
                       }
                       break;
            case 3:    if(verticalTraffic >= horizontalTraffic) {
                        traci->trafficlight("C").setPhaseIndex(0);
                       }
                       break;
            default:   ;
        }*/

        cancelEvent(sendTimerEvt);
        scheduleAt(simTime()+1, sendTimerEvt);
    }
    else
    {
        //BaseWaveApplLayer::handleSelfMsg(msg);
    } 
}
