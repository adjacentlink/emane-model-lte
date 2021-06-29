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


#include "uemessageprocessor.h"



void
EMANE::Models::LTE::UEMessageProcessor::loadPCRFile(const std::string & sPCRFileName)
{
  porManager_.load(sPCRFileName);
}


void
EMANE::Models::LTE::UEMessageProcessor::setTxPower(float resourceBlockTxPowerdBm)
{
  resourceBlockTxPowerdBm_ = resourceBlockTxPowerdBm;
}


void
EMANE::Models::LTE::UEMessageProcessor::swapFrequencyMaps(EMANELTE::FrequencyResourceBlockMap & rxFreqToRBMap,
                                                          EMANELTE::FrequencyResourceBlockMap & txFreqToRBMap)
{
  downlinkMap_.clear();

  size_t numResourceBlocks{rxFreqToRBMap.size()};

  downlinkMap_.emplace(numResourceBlocks, DownlinkParams(rxFreqToRBMap, new DownlinkResourceGridParams(rxFreqToRBMap.size(), 7)));

  txFreqToRBMap_.swap(txFreqToRBMap);
}


void
EMANE::Models::LTE::UEMessageProcessor::swapSearchFrequencyMaps(EMANELTE::FrequencyResourceBlockMap & rxEvenFreqToRBMap,
                                                                EMANELTE::FrequencyResourceBlockMap & rxOddFreqToRBMap,
                                                                EMANELTE::FrequencyResourceBlockMap & txFreqToRBMap)
{
  downlinkMap_.clear();

  size_t numResourceBlocks{rxEvenFreqToRBMap.size()};

  downlinkMap_.emplace(numResourceBlocks, DownlinkParams(rxEvenFreqToRBMap, new DownlinkResourceGridParams(numResourceBlocks, 7)));

  numResourceBlocks = rxOddFreqToRBMap.size();

  downlinkMap_.emplace(numResourceBlocks, DownlinkParams(rxOddFreqToRBMap, new DownlinkResourceGridParams(numResourceBlocks, 7)));

  txFreqToRBMap_.swap(txFreqToRBMap);
}


void
EMANE::Models::LTE::UEMessageProcessor::addTxSegments(const EMANELTE::MHAL::ChannelMessage & channel_msg,
                                                      const std::uint32_t tti_tx)
{
  size_t sfIdx  = tti_tx % 10;

  size_t slot1 = 2 * sfIdx;
  
  size_t slot2 = slot1 + 1;
  
  EMANELTE::MHAL::CHANNEL_TYPE type{channel_msg.channel_type()};

  UplinkChannelRBParams & chanParams(pUplinkRBParams_->params.find(type)->second);

  const ResourceBlockParams & rbParams{chanParams.rbParams()};

  float fSegmentPowerdBm = resourceBlockTxPowerdBm_ + channel_msg.tx_power_scale_db();

  for(int j=0; j<channel_msg.resource_block_frequencies_slot1_size(); ++j)
    {
      EMANELTE::FrequencyHz freq{channel_msg.resource_block_frequencies_slot1(j)};

      segmentBuilder_.insert(FrequencySegmentKey(freq, slot1, rbParams.first_, rbParams.last_), fSegmentPowerdBm);
    }

  for(int j=0; j<channel_msg.resource_block_frequencies_slot2_size(); ++j)
    {
      EMANELTE::FrequencyHz freq{channel_msg.resource_block_frequencies_slot2(j)};

      segmentBuilder_.insert(FrequencySegmentKey(freq, slot2, rbParams.first_, rbParams.last_), fSegmentPowerdBm);
    }
}


EMANE::FrequencySegments
EMANE::Models::LTE::UEMessageProcessor::buildFrequencySegments(EMANELTE::MHAL::TxControlMessage & txControl,
                                                               uint64_t frequencyHz)
{
  std::uint32_t tti_tx = txControl.tti_tx();

  const EMANE::Microseconds sfDuration{txControl.subframe_duration_microsecs()};

  const EMANE::Microseconds slotDuration = sfDuration/2;

  EMANE::FrequencySegments result;

  for(const auto & carrier : txControl.carriers())
   {
     if(carrier.frequency_hz() == frequencyHz)
      {       
        statisticManager_.updateTxTableCounts(txControl);

        if(carrier.uplink().has_prach())
         {
           addTxSegments(carrier.uplink().prach(), tti_tx);
         }

        for(int i = 0; i < carrier.uplink().pucch_size(); ++i)
         {
           addTxSegments(carrier.uplink().pucch(i), tti_tx);
         }

        for(int i = 0; i < carrier.uplink().pusch_size(); ++i)
         {
           addTxSegments(carrier.uplink().pusch(i), tti_tx);
         }

        result = segmentBuilder_.build(slotDuration);

        break;
      }
   }

  return result;
}


bool
EMANE::Models::LTE::UEMessageProcessor::noiseTestChannelMessage(const EMANELTE::MHAL::TxControlMessage & txControl,
                                                                const EMANELTE::MHAL::ChannelMessage & channel_message,
                                                                EMANE::Models::LTE::SegmentMap & segmentCache,
                                                                std::uint64_t frequencyHz)
{
  for(const auto & carrier : txControl.carriers())
   {
     if(carrier.frequency_hz() == frequencyHz)
      {
        const size_t sfIdx{txControl.tti_tx() % 10};

        const size_t slot1{2 * sfIdx};

        const size_t slot2{slot1 + 1};

        const std::uint32_t cfi{carrier.downlink().cfi()};

        const std::uint32_t numResourceBlocks{carrier.downlink().num_resource_blocks()};

        const EMANE::Microseconds sfDuration{txControl.subframe_duration_microsecs()};

        const EMANE::Microseconds slotDuration = sfDuration/2;

        EMANELTE::MHAL::CHANNEL_TYPE type{channel_message.channel_type()};

        EMANELTE::MHAL::MOD_TYPE modType{channel_message.modulation_type()};

        const std::uint32_t numberOfBits{channel_message.number_of_bits()};

        const std::uint32_t numberInfoREs{numberOfBits/modType};

        std::uint32_t numberMessageREs{0};

        std::uint32_t numberReceivedREs{0};

        EMANE::Microseconds offset;

        EMANE::Microseconds duration;

        std::uint32_t rxNumResourceBlocks{numResourceBlocks};

        auto bwIter = downlinkMap_.find(rxNumResourceBlocks);

        if(bwIter == downlinkMap_.end())
         {
          if(rxNumResourceBlocks % 2)
           {
             bwIter = downlinkMap_.find(75);

             if(bwIter == downlinkMap_.end())
              {
                LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                        EMANE::ERROR_LEVEL,
                                        "MACI %03hu %s::%s: FAIL cannot find rb map for numResourceBlocks=%d",
                                        id_,
                                        "UEMessageProcessor",
                                        __func__,
                                        75);

                return false;
              }
          }
         else
          {
            bwIter = downlinkMap_.find(100);

            if(bwIter == downlinkMap_.end())
              {
                LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                        EMANE::ERROR_LEVEL,
                                        "MACI %03hu %s::%s: FAIL cannot find rb map for numResourceBlocks=%d",
                                        id_,
                                        "UEMessageProcessor",
                                        __func__,
                                        100);

                return false;
              }
          }
       }

      DownlinkParams & downlinkParams(bwIter->second);

      DownlinkChannelRBParams & chanParams(downlinkParams.pRBParams_->params.find(type)->second);

      SlotRBParams & slot1Params(chanParams.slotParams(cfi, slot1));
      SlotRBParams & slot2Params(chanParams.slotParams(cfi, slot2));

      // determine pass/fail on channel_message
      // 1. figure out how many segments are needed to be successfully
      //    received based on channel_message.number_of_bits and by
      //    the number of resource elements contained in the resource
      //    blocks occupied by the message (based on type)
      // 2. determine pass/fail on each message segment based on
      //    por test and channel_message.modulation_type. Use
      //    the segment cache for the noise floor for the resource
      //    block noise information, also, store information there.
      // 3. Message passes if 2 >= 1

     for(int j=0; j<channel_message.resource_block_frequencies_slot1_size(); ++j)
      {
        EMANELTE::FrequencyHz freq{channel_message.resource_block_frequencies_slot1(j)};

        auto freqIter = downlinkParams.rxFreqToMap_.find(freq);

        // frequency is not found, should not happen
        if(freqIter == downlinkParams.rxFreqToMap_.end())
         {
           LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                   EMANE::ERROR_LEVEL,
                                   "MACI %03hu %s::%s: type %d, cannot find resource block for frequency %lu",
                                   id_,
                                   "UEMessageProcessor",
                                   __func__,
                                   type,
                                   freq);

           continue;
         }

        uint32_t rb{freqIter->second};

        ResourceBlockParams & rbParams(slot1Params[rb]);

        numberMessageREs += rbParams.res_;

        std::tie(offset, duration) = segmentBuilder_.calcSegmentBoundary(slot1, rbParams.first_, rbParams.last_, slotDuration);

        auto segmentIter = segmentCache.find(SegmentKey{freq, offset, duration});

        if(segmentIter == segmentCache.end())
         {
           LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                   EMANE::INFO_LEVEL,
                                   "MACI %03hu %s::%s: "
                                   "type %d, "
                                   "slot1 segment cache miss, "
                                   "slotDuration=%lu, "
                                   "freq=%lu, "
                                   "startSymb=%d, "
                                   "stopSymb=%d, "
                                   "offs=%lu, "
                                   "dur=%lu, "
                                   "rb=%d",
                                   id_,
                                   "UEMessageProcessor",
                                   __func__,
                                   type,
                                   slotDuration.count(),
                                   freq,
                                   rbParams.first_,
                                   rbParams.last_,
                                   offset.count(),
                                   duration.count(),
                                   rb);

            continue;
          }

        const float sinr_dB = segmentIter->second;

        const float por = porManager_.getDedicatedChannelPOR(modType, sinr_dB);

        const float fRandomValue{RNDZeroToOne_()};

        LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                               EMANE::INFO_LEVEL,
                               "MACI %03hu %s::%s: modType %d, sinr_dB %0.1f, por %0.1f, rand %0.1f, rbParams.res_ %d",
                               id_,
                               "UEMessageProcessor",
                               __func__,
                               modType,
                               sinr_dB,
                               por,
                               fRandomValue,
                               rbParams.res_);

        if(por >= fRandomValue)
         {
           numberReceivedREs += rbParams.res_;

           statisticManager_.updateRxFrequencyPass(freq);
         }
        else
         {
           statisticManager_.updateRxFrequencyDrop(freq);
         }
       }

      for(int j=0; j<channel_message.resource_block_frequencies_slot2_size(); ++j)
       {
         EMANELTE::FrequencyHz freq{channel_message.resource_block_frequencies_slot2(j)};

         auto freqIter = downlinkParams.rxFreqToMap_.find(freq);

         // frequency is not found, should not happen
         if(freqIter == downlinkParams.rxFreqToMap_.end())
          {
            continue;
          }

         uint32_t rb{freqIter->second};
  
         ResourceBlockParams & rbParams(slot2Params[rb]);

         numberMessageREs += rbParams.res_;

         std::tie(offset, duration) = segmentBuilder_.calcSegmentBoundary(slot2, rbParams.first_, rbParams.last_, slotDuration);

         auto segmentIter = segmentCache.find(SegmentKey(freq, offset, duration));

         if(segmentIter == segmentCache.end())
          {
#if 0
            LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                    EMANE::INFO_LEVEL,
                                    "MACI %03hu %s::%s: "
                                    "type %d, "
                                    "slot2 segment cache miss, "
                                    "slotDuration=%lu, "
                                    "freq=%lu, "
                                    "startSymb=%d, "
                                    "stopSymb=%d, "
                                    "offs=%lu, "
                                    "dur=%lu, "
                                    "rb=%d",
                                    id_,
                                    "UEMessageProcessor",
                                    __func__,
                                    type,
                                    slotDuration.count(),
                                    freq,
                                    rbParams.first_,
                                    rbParams.last_,
                                    offset.count(),
                                    duration.count(),
                                    rb);
#endif

            continue;
          }

         const float sinr_dB = segmentIter->second;

         const float por = porManager_.getDedicatedChannelPOR(modType, sinr_dB);

         const float fRandomValue{RNDZeroToOne_()};

#if 0
         LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                EMANE::INFO_LEVEL,
                                "MACI %03hu %s::%s: modType %d, sinr_dB %0.1f, por %0.1f, rand %0.1f, rbParams.res_ %d",
                                id_,
                                "UEMessageProcessor",
                                __func__,
                                modType,
                                sinr_dB,
                                por,
                                fRandomValue,
                                rbParams.res_);
#endif

         if(por >= fRandomValue)
          {
            numberReceivedREs += rbParams.res_;
          }
       } // end for

      const uint32_t numberChannelCodeREs{numberMessageREs - numberInfoREs};

      const bool messageReceived{numberReceivedREs > (numberInfoREs + numberChannelCodeREs/2)};

#if 0
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              EMANE::INFO_LEVEL,
                              "MACI %03hu %s::%s: %s sfIdx %zu, type %d, modType %d, messageREs %d, infoREs %d, rcvedREs %d",
                              id_,
                              "UEMessageProcessor",
                              __func__,
                              messageReceived ? "PASS" : "FAIL",
                              sfIdx,
                              type,
                              modType,
                              numberMessageREs,
                              numberInfoREs,
                              numberReceivedREs);
#endif
      return messageReceived;
    }
   else
    {
#if 0
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              EMANE::INFO_LEVEL,
                              "MACI %03hu %s::%s: carrieFrequency %lu != frequency %lu",
                              id_,
                              "UEMessageProcessor",
                              __func__,
                              carrier.frequency_hz(),
                              frequencyHz);
#endif
    }

  } // end for each carrier

  return false;
}
