/*
 * Copyright (c) 2015 - Adjacent Link LLC, Bridgewater, New Jersey
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
 */

#include "pormanager.h"
#include "emane/configurationexception.h"
#include "emane/utils/parameterconvert.h"
#include <cmath>
#include <cstdlib>
#include <limits>
#include <libxml/parser.h>
#include <libxml/xmlschemas.h>

namespace
{
  const char * pzSchema="\
    <xs:schema xmlns:xs='http://www.w3.org/2001/XMLSchema'>\
    <xs:simpleType name='ModulationType'>\
      <xs:restriction base='xs:string'>\
        <xs:enumeration value='BPSK' />\
        <xs:enumeration value='QPSK' />\
        <xs:enumeration value='16QAM' />\
        <xs:enumeration value='64QAM' />\
      </xs:restriction>\
    </xs:simpleType>\
    \
    <xs:element name='ltemodel-pcr'>\
      <xs:complexType>\
      <xs:sequence>\
      <xs:element name='dedicated_channel_table' minOccurs='1' maxOccurs='1'>\
        <xs:complexType>\
        <xs:sequence>\
        <xs:element name='curve' minOccurs='4' maxOccurs='4'>\
          <xs:complexType>\
          <xs:sequence>\
          <xs:element name='row' minOccurs='2' maxOccurs='unbounded'>\
            <xs:complexType>\
            <xs:sequence>\
            </xs:sequence>\
            <xs:attribute name='sinr' type='xs:float' use='required'/>\
            <xs:attribute name='por' type='xs:float' use='required'/>\
            </xs:complexType>\
          </xs:element>\
          </xs:sequence>\
          <xs:attribute name='modulation' type='ModulationType' use='required'/>\
          </xs:complexType>\
        </xs:element>\
        </xs:sequence>\
        </xs:complexType>\
      </xs:element>\
      </xs:sequence>\
      </xs:complexType>\
    </xs:element>\
    </xs:schema>";


  std::string scaleFloatToInteger(const std::string & sValue)
  {
    const int iScaleFactor{2};
    std::string sTmpParameter{sValue};

    // location of decimal point, if exists
    std::string::size_type indexPoint =  sTmpParameter.find(".",0);

    if(indexPoint != std::string::npos)
      {
        std::string::size_type numberOfDigitsAfterPoint = sTmpParameter.size() - indexPoint - 1;

        if(numberOfDigitsAfterPoint > iScaleFactor)
          {
            // need to move the decimal point, enough digits are present
            sTmpParameter.insert(sTmpParameter.size() - (numberOfDigitsAfterPoint - iScaleFactor), ".");
          }
        else
          {
            // need to append 0s
            sTmpParameter.append(iScaleFactor - numberOfDigitsAfterPoint,'0');
          }

        // remove original decimal point
        sTmpParameter.erase(indexPoint,1);
      }
    else
      {
        // need to append 0s
        sTmpParameter.append(iScaleFactor,'0');
      }

    return sTmpParameter;
  }

  EMANELTE::MHAL::MOD_TYPE stringToModulationType(const std::string & modulationString)
  {
    if(modulationString == "BPSK")
      {
        return EMANELTE::MHAL::MOD_BPSK;
      }
    else if(modulationString == "QPSK")
      {
        return EMANELTE::MHAL::MOD_QPSK;
      }
    else if(modulationString == "16QAM")
      {
        return EMANELTE::MHAL::MOD_16QAM;
      }
    else
      {
        // relying on dtd to enforce "64QAM" is the only other valid value
        return EMANELTE::MHAL::MOD_64QAM;
      }
  }
}

EMANE::Models::LTE::PORManager::PORManager(){}

void EMANE::Models::LTE::PORManager::load(const std::string & sPCRFileName)
{
  xmlDocPtr pSchemaDoc{xmlReadMemory(pzSchema,
                                     strlen(pzSchema),
                                     "file:///emanelteemodelpcr.xsd",
                                     NULL,
                                     0)};
  if(!pSchemaDoc)
    {
      throw EMANE::makeException<EMANE::ConfigurationException>("unable to open schema");
    }

  xmlSchemaParserCtxtPtr pParserContext{xmlSchemaNewDocParserCtxt(pSchemaDoc)};

  if(!pParserContext)
    {
      throw EMANE::makeException<EMANE::ConfigurationException>("bad schema context");
    }

  xmlSchemaPtr pSchema{xmlSchemaParse(pParserContext)};

  if(!pSchema)
    {
      throw EMANE::makeException<EMANE::ConfigurationException>("bad schema parser");
    }

  xmlSchemaValidCtxtPtr pSchemaValidCtxtPtr{xmlSchemaNewValidCtxt(pSchema)};

  if(!pSchemaValidCtxtPtr)
    {
      throw EMANE::makeException<EMANE::ConfigurationException>("bad schema valid context");
    }

  xmlSchemaSetValidOptions(pSchemaValidCtxtPtr,XML_SCHEMA_VAL_VC_I_CREATE);

  xmlDocPtr pDoc = xmlReadFile(sPCRFileName.c_str(),nullptr,0);

  if(xmlSchemaValidateDoc(pSchemaValidCtxtPtr, pDoc))
    {
      throw EMANE::makeException<EMANE::ConfigurationException>("invalid document: %s",
                                                  sPCRFileName.c_str());
    }

  xmlNodePtr pRoot = xmlDocGetRootElement(pDoc);

  for(xmlNodePtr pDedicatedChannelTableNode = pRoot->children;
      pDedicatedChannelTableNode != nullptr;
      pDedicatedChannelTableNode = pDedicatedChannelTableNode->next)
    {
      if(!xmlStrcmp(pDedicatedChannelTableNode->name, BAD_CAST "dedicated_channel_table"))
        {
          for(xmlNodePtr pCurveNode = pDedicatedChannelTableNode->children;
              pCurveNode != nullptr;
              pCurveNode = pCurveNode->next)
            {
              if(!xmlStrcmp(pCurveNode->name, BAD_CAST "curve"))
                {
                  Curve curve{};
                  std::int32_t i32MinScaledSINR{std::numeric_limits<std::int32_t>::max()};
                  std::int32_t i32MaxScaledSINR{std::numeric_limits<std::int32_t>::lowest()};

                  xmlChar * pModulation = xmlGetProp(pCurveNode,BAD_CAST "modulation");

                  std::string sModulation(reinterpret_cast<char *>(pModulation));

                  xmlFree(pModulation);

                  for(xmlNodePtr pEntryNode = pCurveNode->children;
                      pEntryNode != nullptr;
                      pEntryNode = pEntryNode->next)
                    {
                      if(!xmlStrcmp(pEntryNode->name, BAD_CAST "row"))
                        {
                          xmlChar * pSINR = xmlGetProp(pEntryNode,BAD_CAST "sinr");
                          xmlChar * pPOR  = xmlGetProp(pEntryNode,BAD_CAST "por");

                          const auto i32ScaledSINR =
                            EMANE::Utils::ParameterConvert(scaleFloatToInteger(reinterpret_cast<const char *>(pSINR))).toINT32();

                          auto ret =
                            curve.insert(std::make_pair(i32ScaledSINR,
                                                        EMANE::Utils::ParameterConvert(reinterpret_cast<const char *>(pPOR)).toFloat()/100));

                          if(!ret.second)
                            {
                              throw EMANE::makeException<EMANE::ConfigurationException>("duplicate PCR SINR value for datarate: %zu sinr: %s in %s",
                                                                                        stringToModulationType(sModulation),
                                                                                        reinterpret_cast<const char *>(pSINR),
                                                                                        sPCRFileName.c_str());
                            }

                          xmlFree(pSINR);
                          xmlFree(pPOR);

                          i32MinScaledSINR = std::min(i32ScaledSINR,i32MinScaledSINR);
                          i32MaxScaledSINR = std::max(i32ScaledSINR,i32MaxScaledSINR);
                        }
                    }

                    Curve interpolation{};

                    auto iter = curve.begin();

                    while(iter != curve.end())
                      {
                        const auto x0 = iter->first;
                        const auto y0 = iter->second;

                        ++iter;

                        if(iter != curve.end())
                          {
                            const auto x1 = iter->first;
                            const auto y1 = iter->second;

                            const auto slope = (y1 - y0) / (x1 - x0);

                            for(auto i = x0; i < x1; ++i)
                              {
                                interpolation.insert({i,y1 - (x1 - i) * slope});
                              }
                          }
                      }

                    curve.insert(interpolation.begin(),interpolation.end());

                    auto ret = dedicatedChannelTable_.insert(std::make_pair(stringToModulationType(sModulation),
                                                                            std::make_tuple(i32MinScaledSINR,
                                                                                            i32MaxScaledSINR,
                                                                                            curve)));

                    if(!ret.second)
                      {
                        throw EMANE::makeException<EMANE::ConfigurationException>("duplicate PCR dedicated channel table modulation: '%s' in %s",
                                                                                  sModulation.c_str(),
                                                                                  sPCRFileName.c_str());
                      }
                }
            }
        }
    }

  xmlFreeDoc(pSchemaDoc);

  xmlFreeDoc(pDoc);
}


float
EMANE::Models::LTE::PORManager::getDedicatedChannelPOR(EMANELTE::MHAL::MOD_TYPE modulationType,
                                                       float fSINR)
{
  auto iter = dedicatedChannelTable_.find(modulationType);

  if(iter == dedicatedChannelTable_.end())
    {
      throw EMANE::makeException<EMANE::ConfigurationException>("No PCR modulationType: '%d' in dedicated_channel_table",
                                                                modulationType);
    }

  const std::int32_t i32Scaled{static_cast<std::int32_t>(fSINR * 100)};

  if(i32Scaled < std::get<0>(iter->second))
    {
      return 0;
    }

  if(i32Scaled > std::get<1>(iter->second))
    {
      return 1;
    }

  Curve & curve = std::get<2>(iter->second);

  auto curveIter = curve.find(i32Scaled);

  if(curveIter != curve.end())
    {
      auto por = curveIter->second;

      return por;
    }

  return 0;
}


EMANE::Models::LTE::PORManager::CurveDumps
EMANE::Models::LTE::PORManager::PORManager::dump()
{
  CurveDumps ret{};

  for(const auto & dedicatedChannelEntry : dedicatedChannelTable_)
    {
      CurveDump entries{};

      for(const auto & porEntry : std::get<2>(dedicatedChannelEntry.second))
        {
          entries.insert({porEntry.first / 100.0, porEntry.second});
        }

      ret.insert({dedicatedChannelEntry.first,entries});
    }

  return ret;
}
