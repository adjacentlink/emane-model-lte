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


#include "configmanager.h"
#include "xmlvalidator.h"

#include "emane/configurationexception.h"
#include "emane/utils/parameterconvert.h"

#include <libxml/parser.h>
#include <libxml/xmlschemas.h>


namespace {
 const char * pzModuleName = "EMANELte";
 const char * pzFileName   = "ConfigManager";

 const std::string sExceptionPrefix = std::string{pzModuleName} + 
                                      std::string{":"}          +
                                      std::string{pzFileName};

 const char * pzSchema="\
  <xs:schema xmlns:xs='http://www.w3.org/2001/XMLSchema'>\
\
    <xs:simpleType name='NemIdType'>\
      <xs:restriction base='xs:unsignedShort'>\
        <xs:minInclusive value='1'/>\
        <xs:maxInclusive value='65534'/>\
      </xs:restriction>\
    </xs:simpleType>\
\
     <xs:element name='emanelte'>\
       <xs:complexType>\
         <xs:sequence>\
\
         <xs:element name='platform' minOccurs='1' maxOccurs='1'>\
           <xs:complexType>\
             <xs:sequence>\
\
               <xs:element name='radiomodel' minOccurs='1' maxOccurs='1'>\
                 <xs:complexType>\
                   <xs:attribute name='pcrcurveuri'             type='xs:string' use='required'/>\
                   <xs:attribute name='maxpropagationdelay'     type='xs:string' use='optional'/>\
                   <xs:attribute name='resourceblocktxpower'    type='xs:string' use='optional'/>\
                   <xs:attribute name='antenna'                 type='xs:string' use='optional'/>\
                 </xs:complexType>\
               </xs:element>\
\
               <xs:element name='phy' minOccurs='0' maxOccurs='1'>\
                 <xs:complexType>\
                   <xs:attribute name='fixedantennagain'        type='xs:string' use='optional'/>\
                   <xs:attribute name='fixedantennagainenable'  type='xs:string' use='optional'/>\
                   <xs:attribute name='noisemode'               type='xs:string' use='optional'/>\
                   <xs:attribute name='propagationmodel'        type='xs:string' use='optional'/>\
                   <xs:attribute name='systemnoisefigure'       type='xs:string' use='optional'/>\
                   <xs:attribute name='subid'                   type='xs:string' use='optional'/>\
                   <xs:attribute name='compatibilitymode'       type='xs:string' use='optional'/>\
                 </xs:complexType>\
               </xs:element>\
\
             </xs:sequence>\
             <xs:attribute name='id'                        type='NemIdType' use='required'/> \
             <xs:attribute name='statisticsendpoint'        type='xs:string' use='optional'/>\
             <xs:attribute name='controlportendpoint'       type='xs:string' use='optional'/>\
             <xs:attribute name='logfile'                   type='xs:string' use='optional'/>\
             <xs:attribute name='loglevel'                  type='xs:string' use='optional'/>\
             <xs:attribute name='otamanagerchannelenable'   type='xs:string' use='optional'/>\
             <xs:attribute name='otamanagergroup'           type='xs:string' use='optional'/>\
             <xs:attribute name='otamanagerloopback'        type='xs:string' use='optional'/>\
             <xs:attribute name='otamanagerdevice'          type='xs:string' use='optional'/>\
             <xs:attribute name='eventservicegroup'         type='xs:string' use='optional'/>\
             <xs:attribute name='eventservicedevice'        type='xs:string' use='optional'/>\
             <xs:attribute name='maxpropagationdelay'       type='xs:string' use='optional'/>\
             <xs:attribute name='antennaprofilemanifesturi' type='xs:string' use='optional'/>\
           </xs:complexType>\
         </xs:element>\
\
       </xs:sequence>\
     </xs:complexType>\
   </xs:element>\
 </xs:schema>";


  inline std::string getRequiredValue(xmlNodePtr pNode, const char * pzTag)
    {
      const auto xmlProp = xmlGetProp(pNode, BAD_CAST pzTag);

      if(xmlProp)
       {
         return std::string{(const char*)xmlProp};
       }
      else
       {
         const std::string sError = std::string{"invalid xml tag "} + pzTag;

         throw EMANE::ConfigurationException(sError.c_str());
       }
    } 

  inline std::string checkForValue(xmlNodePtr pNode, const char * pzTag, const std::string & defaultValue)
    {
      const auto xmlProp = xmlGetProp(pNode, BAD_CAST pzTag);

      if(xmlProp)
       {
         return std::string{(const char*)xmlProp};
       }
      else
       {
         return defaultValue;
       }
    } 
}



EMANELTE::MHAL::ConfigManager::ConfigManager()
{ }



EMANELTE::MHAL::ConfigManager::~ConfigManager()
{ }


void
EMANELTE::MHAL::ConfigManager::setLogger(EMANE::Application::Logger & logger)
 {
   logger_ = logger;
 }


void 
EMANELTE::MHAL::ConfigManager::configure(const std::string & sFileName)
{
  parseConfigFile_i(sFileName);
}


const EMANELTE::MHAL::ConfigManager::PlatformConfig & 
EMANELTE::MHAL::ConfigManager::getPlatformConfig() const
{
  return platformConfig_;
}

const EMANELTE::MHAL::ConfigManager::RadioModelConfig & 
EMANELTE::MHAL::ConfigManager::getRadioModelConfig() const
{
  return radioModelConfig_;
}


const EMANELTE::MHAL::ConfigManager::PhyConfig & 
EMANELTE::MHAL::ConfigManager::getPhyConfig() const
{
  return phyConfig_;
}


void EMANELTE::MHAL::ConfigManager::parseConfigFile_i(const std::string & sFileName)
{
  xmlDocPtr pSchemaDoc{};
  xmlDocPtr pDoc{};

  validateXml(sFileName, 
              pzSchema,
              "file:///emanelte.xsd",
              pSchemaDoc,
              pDoc);

  auto pRoot = xmlDocGetRootElement(pDoc);

  for(auto pEntryNode = pRoot->children;
      pEntryNode != nullptr;
      pEntryNode = pEntryNode->next)
    {
       if(!xmlStrcmp(pEntryNode->name, BAD_CAST "platform"))
         {
           // required
           platformConfig_.id_                     = EMANE::Utils::ParameterConvert(getRequiredValue(pEntryNode, "id")).toUINT16(1, 0xFFFE);

           // optional
           platformConfig_.sControlPortEndpoint_      = checkForValue(pEntryNode, "controlportendpoint",       platformConfig_.sControlPortEndpoint_);
           platformConfig_.sLogFileName_              = checkForValue(pEntryNode, "logfile",                   platformConfig_.sLogFileName_);
           platformConfig_.sLogLevel_                 = checkForValue(pEntryNode, "loglevel",                  platformConfig_.sLogLevel_);
           platformConfig_.sOtamManagerChannelEnable_ = checkForValue(pEntryNode, "otamanagerchannelenable",   platformConfig_.sOtamManagerChannelEnable_);
           platformConfig_.sOtaManagerGroup_          = checkForValue(pEntryNode, "otamanagergroup",           platformConfig_.sOtaManagerGroup_);
           platformConfig_.sOtaManagerLoopback_       = checkForValue(pEntryNode, "otamanagerloopback",        platformConfig_.sOtaManagerLoopback_);
           platformConfig_.sOtaManagerDevice_         = checkForValue(pEntryNode, "otamanagerdevice",          platformConfig_.sOtaManagerDevice_);
           platformConfig_.sEventServiceGroup_        = checkForValue(pEntryNode, "eventservicegroup",         platformConfig_.sEventServiceGroup_);
           platformConfig_.sEventServiceDevice_       = checkForValue(pEntryNode, "eventservicedevice",        platformConfig_.sEventServiceDevice_);
           platformConfig_.sAntennaProfileManifest_   = checkForValue(pEntryNode, "antennaprofilemanifesturi", platformConfig_.sAntennaProfileManifest_);

           LOGGER_STANDARD_LOGGING(logger_,
                                   EMANE::INFO_LEVEL,
                                   "%s %s:%s, section:platform %s",
                                   pzModuleName,
                                   pzFileName,
                                   __func__,
                                   platformConfig_.format().c_str());

           for(auto pLayerNode = pEntryNode->children;
               pLayerNode != nullptr;
               pLayerNode = pLayerNode->next)
             {
               if(!xmlStrcmp(pLayerNode->name, BAD_CAST "radiomodel"))
                 {
                   // required
                   radioModelConfig_.sPcrCurveURI_         = getRequiredValue(pLayerNode, "pcrcurveuri");

                   // optional
                   radioModelConfig_.sMaxPropagationDelay_  = checkForValue(pLayerNode, "maxpropagationdelay",     radioModelConfig_.sMaxPropagationDelay_);
                   radioModelConfig_.sResourceBlockTxPower_ = checkForValue(pLayerNode, "resourceblocktxpower",    radioModelConfig_.sResourceBlockTxPower_);
                   radioModelConfig_.sAntenna_              = checkForValue(pLayerNode, "antenna",                 radioModelConfig_.sAntenna_);

                   LOGGER_STANDARD_LOGGING(logger_,
                                           EMANE::INFO_LEVEL,
                                           "%s %s:%s, section:radiomodel %s",
                                           pzModuleName,
                                           pzFileName,
                                           __func__,
                                           radioModelConfig_.format().c_str());
                 }
               else if(!xmlStrcmp(pLayerNode->name, BAD_CAST "phy"))
                 {
                   // optional
                   phyConfig_.sAntennaGain_              = checkForValue(pLayerNode, "fixedantennagain",        phyConfig_.sAntennaGain_);
                   phyConfig_.sFixedAntennaGainEnable_   = checkForValue(pLayerNode, "fixedantennagainenable",  phyConfig_.sFixedAntennaGainEnable_);
                   phyConfig_.sNoiseMode_                = checkForValue(pLayerNode, "noisemode",               phyConfig_.sNoiseMode_);
                   phyConfig_.sPropagationModel_         = checkForValue(pLayerNode, "propagationmodel",        phyConfig_.sPropagationModel_);
                   phyConfig_.sSystemNoiseFigure_        = checkForValue(pLayerNode, "systemnoisefigure",       phyConfig_.sSystemNoiseFigure_);
                   phyConfig_.sSubId_                    = checkForValue(pLayerNode, "subid",                   phyConfig_.sSubId_);
                   phyConfig_.sCompatibilityMode_        = checkForValue(pLayerNode, "compatibilitymode",       phyConfig_.sCompatibilityMode_);

                   LOGGER_STANDARD_LOGGING(logger_,
                                           EMANE::INFO_LEVEL,
                                           "%s %s:%s, section:phy %s",
                                           pzModuleName,
                                           pzFileName,
                                           __func__,
                                           phyConfig_.format().c_str());
                 }
             }
         }
    }

  for(auto pEntryNode = pRoot->children;
      pEntryNode != nullptr;
      pEntryNode = pEntryNode->next)
    {
    }

   xmlFreeDoc(pSchemaDoc);
   xmlFreeDoc(pDoc);
}
