/*
 * Copyright (c) 2019 - Adjacent Link LLC, Bridgewater, New Jersey
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * * Redistributions of source code must retain the above copyright
 *   notice, this list of conditions and the following disclaimer.
 * * Redistributions in binary form must reproduce the above copyright
 *   notice, this list of conditions and the following disclaimer in
 *   the documentation and/or other materials provided with the
 *   distribution.
 * * Neither the name of Adjacent Link LLC nor the names of its
 *   contributors may be used to endorse or promote products derived
 *   from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 * LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 * See toplevel COPYING for more information.
 */


#ifndef EMANELTE_CONFIGMANAGER_H
#define EMANELTE_CONFIGMANAGER_H

#include "emane/application/logger.h"

#include <string>
#include <sstream>
#include <map>

namespace EMANELTE
{
  namespace MHAL
  {
    class ConfigManager
     {
       public:

        static ConfigManager & getInstance()
         {
            static ConfigManager instance;
            return instance;
         }

         struct PlatformConfig {
           std::uint16_t id_;

           std::string   sControlPortEndpoint_;
           std::string   sAntennaGain_;
           std::string   sLogFileName_;
           std::string   sLogLevel_;
           std::string   sOtamManagerChannelEnable_;
           std::string   sOtaManagerGroup_;         
           std::string   sOtaManagerLoopback_;      
           std::string   sOtaManagerDevice_;        
           std::string   sEventServiceGroup_;       
           std::string   sEventServiceDevice_;
           std::string   sAntennaProfileManifest_;


           PlatformConfig() :
            id_{},
            sControlPortEndpoint_{"0.0.0.0:47000"},
            sAntennaGain_{"0.0"},
            sLogFileName_{"emanelte.log"},
            sLogLevel_{"0"},
            sOtamManagerChannelEnable_{"on"}, 
            sOtaManagerGroup_{"224.1.2.8:45702"}, 
            sOtaManagerLoopback_{"off"}, 
            sOtaManagerDevice_{"lo"}, 
            sEventServiceGroup_{"224.1.2.8:45703"}, 
            sEventServiceDevice_{"lo"},
            sAntennaProfileManifest_{""}
           { }

           std::string format()
            {
               std::stringstream ss;

               ss << "\n\tid="                        << id_; 
               ss << "\n\tcontrolportendpoint="       << sControlPortEndpoint_; 
               ss << "\n\tantennagain="               << sAntennaGain_;
               ss << "\n\tlogfile="                   << sLogFileName_;
               ss << "\n\tloglevel="                  << sLogLevel_;
               ss << "\n\totamanagerchannelenable="   << sOtamManagerChannelEnable_;
               ss << "\n\totamanagergroup="           << sOtaManagerGroup_;
               ss << "\n\totamanagerloopback="        << sOtaManagerLoopback_;
               ss << "\n\totamanagerdevice="          << sOtaManagerDevice_;
               ss << "\n\teventservicegroup="         << sEventServiceGroup_;
               ss << "\n\teventservicedevice="        << sEventServiceDevice_;
               ss << "\n\tantennaprofilemanifesturi=" << sAntennaProfileManifest_;

               return ss.str();
             }
          };

         struct PhyConfig {

           std::string   sAntennaGain_;
           std::string   sFixedAntennaGainEnable_;
           std::string   sNoiseMode_;
           std::string   sPropagationModel_;
           std::string   sSystemNoiseFigure_;
           std::string   sSubId_;


           PhyConfig() :
            sAntennaGain_{"0.0"},
            sFixedAntennaGainEnable_{"on"},
            sNoiseMode_{"all"},
            sPropagationModel_{"precomputed"},
            sSystemNoiseFigure_{"7.0"},
            sSubId_{"65533"}
           { }

           std::string format()
            {
               std::stringstream ss;

               ss << "\n\tantennagain="              << sAntennaGain_;
               ss << "\n\tfixedantennagain="         << sFixedAntennaGainEnable_;
               ss << "\n\tnoisemode="                << sNoiseMode_;
               ss << "\n\tpropagationmodel="         << sPropagationModel_;
               ss << "\n\tsystemnoisefigure="        << sSystemNoiseFigure_;
               ss << "\n\tsubid="                    << sSubId_;

               return ss.str();
             }
          };


         struct RadioModelConfig {
           std::string   sPcrCurveURI_;
           std::string   sMaxPropagationDelay_;
           std::string   sResourceBlockTxPower_;

           RadioModelConfig() :
            sPcrCurveURI_{},
            sMaxPropagationDelay_{"333"}, // in usec, ~100km is the max range supported by timing advance
            sResourceBlockTxPower_{"0.0"}
           { }

           std::string format()
            {
               std::stringstream ss;

               ss << "\n\tpcrcurveuri="             << sPcrCurveURI_; 
               ss << "\n\tmaxpropagationdelay="     << sMaxPropagationDelay_;
               ss << "\n\tresourceblocktxpower="    << sResourceBlockTxPower_;

               return ss.str();
             }
          };

         ConfigManager();

         ~ConfigManager();

         ConfigManager(ConfigManager const&)    = delete;

         void operator = (ConfigManager const&) = delete;

         void configure(const std::string & sFileName);

         void setLogger(EMANE::Application::Logger & logger);

         const PlatformConfig & getPlatformConfig() const;

         const RadioModelConfig & getRadioModelConfig() const;

         const PhyConfig & getPhyConfig() const;

       private:
         EMANE::Application::Logger logger_;

         void parseConfigFile_i(const std::string & sFileName);

         PlatformConfig platformConfig_;

         RadioModelConfig radioModelConfig_;

         PhyConfig phyConfig_;
    };
  }
}

#endif
