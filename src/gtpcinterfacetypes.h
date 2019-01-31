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


#ifndef EMANELTE_GTPC_INTERFACE_TYPES_H
#define EMANELTE_GTPC_INTERFACE_TYPES_H

 // see lib/include/srslte/asn1/gtpc_ies.h
const char * gtpc_interface_type_string [] =
{ 
  "S1_U_ENODEB_GTP_U_INTERFACE",
  "S1_U_SGW_GTP_U_INTERFACE",
  "S12_RNC_GTP_U_INTERFACE",
  "S12_SGW_GTP_U_INTERFACE",
  "S5_S8_SGW_GTP_U_INTERFACE",
  "S5_S8_PGW_GTP_U_INTERFACE",
  "S5_S8_SGW_GTP_C_INTERFACE",
  "S5_S8_PGW_GTP_C_INTERFACE",
  "S5_S8_SGW_PMIPV6_INTERFACE",
  "S5_S8_PGW_PMIPV6_INTERFACE",
  "S11_MME_GTP_C_INTERFACE",
  "S11_S4_SGW_GTP_C_INTERFACE", 
  "S10_MME_GTP_C_INTERFACE",
  "S3_MME_GTP_C_INTERFACE",
  "S3_SGSN_GTP_C_INTERFACE",
  "S4_SGSN_GTP_U_INTERFACE",
  "S4_SGW_GTP_U_INTERFACE",
  "S4_SGSN_GTP_C_INTERFACE",
  "S16_SGSN_GTP_C_INTERFACE",
  "ENODEB_GTP_U_INTERFACE_FOR_DL_DATA_FORWARDING",
  "ENODEB_GTP_U_INTERFACE_FOR_UL_DATA_FORWARDING",
  "RNC_GTP_U_INTERFACE_FOR_DATA_FORWARDING",
  "SGSN_GTP_U_INTERFACE_FOR_DATA_FORWARDING",
  "SGW_GTP_U_INTERFACE_FOR_DL_DATA_FORWARDING",
  "SM_MBMS_GW_GTP_C_INTERFACE",
  "SN_MBMS_GW_GTP_C_INTERFACE",
  "SM_MME_GTP_C_INTERFACE",
  "SN_SGSN_GTP_C_INTERFACE",
  "SGW_GTP_U_INTERFACE_FOR_UL_DATA_FORWARDING",
  "SN_SGSN_GTP_U_INTERFACE",
  "S2B_EPDG_GTP_C_INTERFACE",
  "S2B_U_EPDG_GTP_U_INTERFACE",
  "S2B_PGW_GTP_C_INTERFACE",
  "S2B_U_PGW_GTP_U_INTERFACE"
};


#endif
