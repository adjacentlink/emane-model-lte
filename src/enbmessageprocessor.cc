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

#include "enbmessageprocessor.h"



void
EMANE::Models::LTE::ENBMessageProcessor::loadPCRFile(const std::string & sPCRFileName)
{
  porManager_.load(sPCRFileName);
}


void
EMANE::Models::LTE::ENBMessageProcessor::setTxPower(float resourceBlockTxPowerdBm)
{
  resourceBlockTxPowerdBm_ = resourceBlockTxPowerdBm;
}


void
EMANE::Models::LTE::ENBMessageProcessor::swapFrequencyMaps(EMANELTE::FrequencyResourceBlockMap & rxFreqToRBMap,
                                                           EMANELTE::FrequencyResourceBlockMap & txFreqToRBMap)
{
  rxFreqToRBMap_.swap(rxFreqToRBMap);

  txFreqToRBMap_.swap(txFreqToRBMap);

  pDownlinkRBParams_.reset(new DownlinkResourceGridParams(txFreqToRBMap_.size(), 7));
}


void
EMANE::Models::LTE::ENBMessageProcessor::swapSearchFrequencyMaps(EMANELTE::FrequencyResourceBlockMap &,
                                                                 EMANELTE::FrequencyResourceBlockMap &,
                                                                 EMANELTE::FrequencyResourceBlockMap & )
{
}


void
EMANE::Models::LTE::ENBMessageProcessor::addTxSegments(const EMANELTE::MHAL::ChannelMessage & channel_msg,
                                                       const std::uint32_t tti_tx,
                                                       const EMANE::Microseconds sfDuration,
                                                       const std::uint32_t cfi)
{
  size_t sfIdx  = tti_tx % 10;

  size_t slot1 = 2 * sfIdx;

  size_t slot2 = slot1 + 1;

  EMANELTE::MHAL::CHANNEL_TYPE type{channel_msg.channel_type()};

  DownlinkChannelRBParams & chanParams(pDownlinkRBParams_->params.find(type)->second);

  float fSegmentPowerdBm = resourceBlockTxPowerdBm_ + channel_msg.tx_power_scale_db();

  LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                         EMANE::DEBUG_LEVEL,
                         "%s %03hu %s: "
                         "tti_tx=%d "
                         "type=%d "
                         "cfi=%d "
                         "sfDuration=%lu "
                         "segpower=%0.1f",
                         "RadioModel",
                         id_,
                         __func__,
                         tti_tx,
                         type,
                         cfi,
                         sfDuration.count(),
                         fSegmentPowerdBm);

  // segment duration parameters for slot1 and slot2
  SlotRBParams & slot1Params(chanParams.slotParams(cfi, slot1));
  SlotRBParams & slot2Params(chanParams.slotParams(cfi, slot2));

  // on a subframe where ue resource block allocation is different on each slot, create
  // a half slot segment for each slot.
  for(int j=0; j<channel_msg.resource_block_frequencies_slot1_size(); ++j)
    {
      EMANELTE::FrequencyHz freq{channel_msg.resource_block_frequencies_slot1(j)};

      uint32_t rb{txFreqToRBMap_[freq]};

      ResourceBlockParams & rbParams(slot1Params[rb]);

      // ignore empty
      if(rbParams.first_ >= rbParams.last_)
        {
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  EMANE::ERROR_LEVEL,
                                  "%s %03hu %s: "
                                  "Unexpected segment on empty downlink RB: chantype=%d slot=%lu rb=%d freq=%lu",
                                  "RadioModel",
                                  id_,
                                  __func__,
                                  type,
                                  slot1,
                                  rb,
                                  freq);
          continue;
        }

      segmentBuilder_.insert(FrequencySegmentKey(freq, slot1, rbParams.first_, rbParams.last_), fSegmentPowerdBm);
    }

  for(int j=0; j<channel_msg.resource_block_frequencies_slot2_size(); ++j)
    {
      EMANELTE::FrequencyHz freq{channel_msg.resource_block_frequencies_slot2(j)};

      uint32_t rb{txFreqToRBMap_[freq]};

      ResourceBlockParams & rbParams(slot2Params[rb]);

      // ignore empty
      if(rbParams.first_ >= rbParams.last_)
        {
          LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                                  EMANE::ERROR_LEVEL,
                                  "%s %03hu %s: "
                                  "Unexpected segment on empty downlink RB: chantype=%d slot=%lu rb=%d freq=%lu",
                                  "RadioModel",
                                  id_,
                                  __func__,
                                  type,
                                  slot2,
                                  rb,
                                  freq);
          continue;
        }

      segmentBuilder_.insert(FrequencySegmentKey(freq, slot2, rbParams.first_, rbParams.last_), fSegmentPowerdBm);
    }
}


EMANE::FrequencySegments
EMANE::Models::LTE::ENBMessageProcessor::buildFrequencySegments(EMANELTE::MHAL::TxControlMessage & txControl,
                                                                uint32_t)
{
  std::uint32_t tti_tx = txControl.tti_tx();

  std::uint32_t cfi = txControl.downlink().cfi();

  const EMANE::Microseconds sfDuration{txControl.subframe_duration_microsecs()};

  const EMANE::Microseconds slotDuration = sfDuration / 2;

  statisticManager_.updateTxTableCounts(txControl);

  addTxSegments(txControl.downlink().pcfich(), tti_tx, sfDuration, cfi);

  if(txControl.downlink().has_pbch())
    {
      addTxSegments(txControl.downlink().pbch(), tti_tx, sfDuration, cfi);
    }

  for(int i = 0; i < txControl.downlink().phich_size(); ++i)
    {
      addTxSegments(txControl.downlink().phich(i), tti_tx, sfDuration, cfi);
    }

  for(int i = 0; i < txControl.downlink().pdcch_size(); ++i)
    {
      addTxSegments(txControl.downlink().pdcch(i), tti_tx, sfDuration, cfi);
    }

  for(int i = 0; i < txControl.downlink().pdsch_size(); ++i)
    {
      addTxSegments(txControl.downlink().pdsch(i), tti_tx, sfDuration, cfi);
    }

  return segmentBuilder_.build(slotDuration);
}


bool
EMANE::Models::LTE::ENBMessageProcessor::noiseTestChannelMessage(const EMANELTE::MHAL::TxControlMessage & txControl,
                                                                          const EMANELTE::MHAL::ChannelMessage & channel_message,
                                                                          EMANE::Models::LTE::SegmentMap & segmentCache)
{
  size_t sfIdx{txControl.tti_tx() % 10};

  size_t slot1{2 * sfIdx};

  size_t slot2{slot1 + 1};

  const EMANE::Microseconds sfDuration{txControl.subframe_duration_microsecs()};

  const EMANE::Microseconds slotDuration = sfDuration/2;

  EMANELTE::MHAL::CHANNEL_TYPE type{channel_message.channel_type()};

  EMANELTE::MHAL::MOD_TYPE modType{channel_message.modulation_type()};

  std::uint32_t numberOfBits{channel_message.number_of_bits()};

  std::uint32_t numberInfoREs{numberOfBits/modType};

  std::uint32_t numberMessageREs{0};

  std::uint32_t numberReceivedREs{0};

  EMANE::Microseconds offset;

  EMANE::Microseconds duration;

  auto rbParamsIter = pUplinkRBParams_->params.find(type);

  if(rbParamsIter == pUplinkRBParams_->params.end())
    {
      LOGGER_STANDARD_LOGGING(pPlatformService_->logService(),
                              EMANE::DEBUG_LEVEL,
                              "MACI %03hu %s::%s: "
                              "no uplink parameter match for channel type %d.",
                              id_,
                              "ENBMessageProcessor",
                              __func__,
                              type);

      return false;
    }

  const ResourceBlockParams & rbParams{rbParamsIter->second.rbParams()};

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

      numberMessageREs += rbParams.res_;

      std::tie(offset, duration) = segmentBuilder_.calcSegmentBoundary(slot1, rbParams.first_, rbParams.last_, slotDuration);

      auto segmentIter = segmentCache.find(SegmentKey(freq, offset, duration));

      if(segmentIter == segmentCache.end())
        {
          LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                 EMANE::DEBUG_LEVEL,
                                 "MACI %03hu %s::%s: "
                                 "type %d, "
                                 "slot1 segment cache miss, "
                                 "slotDuration=%lu, "
                                 "freq=%lu, "
                                 "startSymb=%d, "
                                 "stopSymb=%d, "
                                 "offs=%lu, "
                                 "dur=%lu.",
                                 id_,
                                 "ENBMessageProcessor",
                                 __func__,
                                 type,
                                 slotDuration.count(),
                                 freq,
                                 rbParams.first_,
                                 rbParams.last_,
                                 offset.count(),
                                 duration.count());

          continue;
        }

      float sinr_dB = segmentIter->second;

      float por = porManager_.getDedicatedChannelPOR(modType, sinr_dB);

      float fRandomValue{RNDZeroToOne_()};

      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                             EMANE::DEBUG_LEVEL,
                             "MACI %03hu %s::%s: "
                             "slot=%zu "
                             "freq %lu, "
                             "offset %lu, "
                             "duration %lu, "
                             "type %d, "
                             "modType %d, "
                             "sinr_dB %0.1f, "
                             "por %0.3f, "
                             "rand %0.3f, "
                             "rbParams.res_ %d",
                             id_,
                             "ENBMessageProcessor",
                             __func__,
                             slot1,
                             freq,
                             offset.count(),
                             duration.count(),
                             type,
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

      numberMessageREs += rbParams.res_;

      std::tie(offset, duration) = segmentBuilder_.calcSegmentBoundary(slot2, rbParams.first_, rbParams.last_, slotDuration);

      auto segmentIter = segmentCache.find(SegmentKey(freq, offset, duration));

      if(segmentIter == segmentCache.end())
        {
          LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                                 EMANE::DEBUG_LEVEL,
                                 "MACI %03hu %s::%s: "
                                 "type %d, "
                                 "slot2 segment cache miss, "
                                 "slotDuration=%lu, "
                                 "freq=%lu, "
                                 "startSymb=%d, "
                                 "stopSymb=%d, "
                                 "offs=%lu, "
                                 "dur=%lu.",
                                 id_,
                                 "ENBMessageProcessor",
                                 __func__,
                                 type,
                                 slotDuration.count(),
                                 freq,
                                 rbParams.first_,
                                 rbParams.last_,
                                 offset.count(),
                                 duration.count());

          continue;
        }

      float sinr_dB = segmentIter->second;

      float por = porManager_.getDedicatedChannelPOR(modType, sinr_dB);

      float fRandomValue{RNDZeroToOne_()};

      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                             EMANE::DEBUG_LEVEL,
                             "MACI %03hu %s::%s: "
                             "slot=%zu "
                             "freq %lu, "
                             "offset %lu, "
                             "duration %lu, "
                             "type=%d, "
                             "modType %d, "
                             "sinr_dB %0.1f, "
                             "por %0.3f, "
                             "rand %0.3f, "
                             "rbParams.res_ %d",
                             id_,
                             "ENBMessageProcessor",
                             __func__,
                             slot2,
                             freq,
                             offset.count(),
                             duration.count(),
                             type,
                             modType,
                             sinr_dB,
                             por,
                             fRandomValue,
                             rbParams.res_);

      if(por >= fRandomValue)
        {
          numberReceivedREs += rbParams.res_;
        }
    }

  std::uint32_t numberChannelCodeREs{numberMessageREs - numberInfoREs};

  bool messageReceived{numberReceivedREs > (numberInfoREs + numberChannelCodeREs/2)};

  if(!messageReceived)
    {
      LOGGER_VERBOSE_LOGGING(pPlatformService_->logService(),
                             EMANE::INFO_LEVEL,
                             "MACI %03hu %s::%s: %s sfIdx %zu, type %d, modType %d, numberOfBits %d, messageREs %d, infoREs %d, rcvedREs %d",
                             id_,
                             "ENBMessageProcessor",
                             __func__,
                             messageReceived ? "PASS" : "FAIL",
                             sfIdx,
                             type,
                             modType,
                             numberOfBits,
                             numberMessageREs,
                             numberInfoREs,
                             numberReceivedREs);
    }

  return messageReceived;
}
