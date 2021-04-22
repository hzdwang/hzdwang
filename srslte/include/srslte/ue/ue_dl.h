/**
 *
 * \section COPYRIGHT
 *
 * Copyright 2013-2015 Software Radio Systems Limited
 * Copyright 2016 IMDEA Networks Institute
 *
 * \section LICENSE
 *
 * This file is part of OWL, which extends the srsLTE library.
 *
 * srsLTE is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsLTE is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

/******************************************************************************
 *  File:         ue_dl.h
 *
 *  Description:  UE downlink object.
 *
 *                This module is a frontend to all the downlink data and control
 *                channel processing modules.
 *
 *  Reference:
 *****************************************************************************/

#ifndef UEDL_H
#define UEDL_H

#include <stdbool.h>

#include "srslte/ch_estimation/chest_dl.h"
#include "srslte/dft/ofdm.h"
#include "srslte/common/phy_common.h"

#include "srslte/phch/dci.h"
#include "srslte/phch/pcfich.h"
#include "srslte/phch/pdcch.h"
#include "srslte/phch/pdsch.h"
#include "srslte/phch/pdsch_cfg.h"
#include "srslte/phch/phich.h"
#include "srslte/phch/ra.h"
#include "srslte/phch/regs.h"

#include "srslte/sync/cfo.h"

#include "srslte/utils/vector.h"
#include "srslte/utils/debug.h"

#include "srslte/config.h"

#define MAXACTIVERNTI 20
// CRNTI+ 10+2 = All RNTI
#define MAXACTIVECRNTI 12

typedef struct SRSLTE_API {
  srslte_pcfich_t pcfich;
  srslte_pdcch_t pdcch;
  srslte_pdsch_t pdsch;
  srslte_phich_t phich; 
  srslte_regs_t regs;
  srslte_ofdm_t fft;
  srslte_chest_dl_t chest;
  
  srslte_cfo_t sfo_correct; 
  
  srslte_pdsch_cfg_t pdsch_cfg; 
  srslte_softbuffer_rx_t softbuffer;
  srslte_ra_dl_dci_t dl_dci;
  srslte_cell_t cell;

  cf_t *sf_symbols; 
  cf_t *ce[SRSLTE_MAX_PORTS];
  
  srslte_dci_format_t dci_format;
  uint32_t cfi;
  uint64_t pkt_errors; 
  uint64_t pkts_total;
  uint64_t nof_detected; 

  uint16_t current_rnti;
  //增加变量,当前系统子帧可能活跃的rnti值，假设最多为64.
  // 这里只是记录C-RNTI，从RAR消息中获取的
  uint16_t rnti_active[64];
  uint32_t rnti_activeCount;
  // 记录通过功率的方式获取的临时RNTI列表
  
  uint16_t rnti_active_temp[64];
  uint32_t rnti_activetempCount;
  //用于统计C-rnti的数量
  uint32_t cRNTICount ;
  // 如何维持队列的大小
  uint16_t listSize;
  uint8_t rnti_list[65536];
  uint16_t rnti_use_count[65536];
  uint32_t totRBup, totRBdw, totBWup, totBWdw;
  uint16_t nof_rnti;
  uint32_t last_n_cce; 
  srslte_dci_location_t last_location;
  
  srslte_dci_msg_t pending_ul_dci_msg; 
  uint16_t pending_ul_dci_rnti; 
  
  float sample_offset; 
}srslte_ue_dl_t;

SRSLTE_API void srslte_ue_dl_set_active_crnti(srslte_ue_dl_t *q, uint16_t user);
SRSLTE_API void srslte_ue_dl_reset_rnti_userPWH(srslte_ue_dl_t *q, uint16_t user) ;
SRSLTE_API void srslte_ue_dl_reset_rnti_userPWH_clear(srslte_ue_dl_t *q) ;
//因为解码正确，直接转化为真实的RNTI
SRSLTE_API void srslte_ue_dl_reset_rnti_userPWHToRAR(srslte_ue_dl_t *q, uint16_t user) ;

SRSLTE_API void srslte_ue_dl_reset_rnti_userRAR(srslte_ue_dl_t *q, uint16_t user) ;

/* This function shall be called just after the initial synchronization */
SRSLTE_API int srslte_ue_dl_init(srslte_ue_dl_t *q, 
                                 srslte_cell_t cell);

SRSLTE_API void srslte_ue_dl_free(srslte_ue_dl_t *q);

SRSLTE_API int srslte_ue_dl_decode_fft_estimate(srslte_ue_dl_t *q, 
                                                cf_t *input, 
                                                uint32_t sf_idx, 
                                                uint32_t *cfi); 

SRSLTE_API int srslte_ue_dl_decode_estimate(srslte_ue_dl_t *q, 
                                            uint32_t sf_idx, 
                                            uint32_t *cfi); 

SRSLTE_API int srslte_ue_dl_cfg_grant(srslte_ue_dl_t *q, 
                                      srslte_ra_dl_grant_t *grant, 
                                      uint32_t cfi, 
                                      uint32_t sf_idx, 
                                      uint32_t rvidx); 

SRSLTE_API int srslte_ue_dl_find_ul_dci(srslte_ue_dl_t *q, 
                                        uint32_t cfi, 
                                        uint32_t sf_idx, 
                                        uint16_t rnti, 
                                        srslte_dci_msg_t *dci_msg); 

SRSLTE_API int srslte_ue_dl_find_dl_dci(srslte_ue_dl_t *q, 
                                        uint32_t cfi, 
                                        uint32_t sf_idx, 
                                        uint16_t rnti, 
                                        srslte_dci_msg_t *dci_msg); 

SRSLTE_API int srslte_ue_dl_find_dl_dci_type(srslte_ue_dl_t *q, 
                                             uint32_t cfi, 
                                             uint32_t sf_idx, 
                                             uint16_t rnti, 
                                             srslte_rnti_type_t rnti_type, 
                                             srslte_dci_msg_t *dci_msg);

SRSLTE_API uint32_t srslte_ue_dl_get_ncce(srslte_ue_dl_t *q);

SRSLTE_API void srslte_ue_dl_set_sample_offset(srslte_ue_dl_t * q, 
                                               float sample_offset); 

SRSLTE_API int srslte_ue_dl_decode(srslte_ue_dl_t * q, 
                                   cf_t *input, 
                                   uint8_t *data,
                                   uint32_t sf_idx);
SRSLTE_API int srslte_ue_dl_decode2(srslte_ue_dl_t * q, 
                                   cf_t *input, 
                                   uint8_t *data,
                                   uint32_t sf_idx,
                                   uint32_t sfn);
SRSLTE_API int srslte_ue_dl_decode_rnti(srslte_ue_dl_t * q, 
                                        cf_t *input, 
                                        uint8_t *data,
                                        uint32_t sf_idx,
                                        uint16_t rnti);

SRSLTE_API int srslte_ue_dl_get_control_cc(srslte_ue_dl_t *q,
					  cf_t *input,
					  uint8_t *data,
					  uint32_t sf_idx,
					  uint32_t rvidx,
					  uint32_t sfn);

SRSLTE_API float srslte_ue_dl_fix_location_ra(srslte_ue_dl_t *q,
					srslte_dci_msg_t *dci_msg,
					uint32_t cfi,
					uint32_t sf_idx,
					srslte_rnti_type_t rnti_type,
					uint32_t sfn,
					uint32_t ncce,
					uint32_t L,
					uint8_t print);

SRSLTE_API float srslte_ue_dl_fix_control_ra(srslte_ue_dl_t *q,
					  cf_t *input,
					  uint8_t *data,
					  uint32_t sf_idx,
					  uint32_t rvidx,
					  uint32_t sfn,
					  uint32_t ncce,
					  uint32_t L,
					  uint32_t cfi,
					  uint8_t print);

SRSLTE_API int srslte_ue_dl_decode_rnti_rv(srslte_ue_dl_t * q, 
                                           cf_t *input, 
                                           uint8_t * data,
                                           uint32_t sf_idx, 
                                           uint16_t rnti, 
                                           uint32_t rvidx); 

SRSLTE_API bool srslte_ue_dl_decode_phich(srslte_ue_dl_t *q, 
                                          uint32_t sf_idx, 
                                          uint32_t n_prb_lowest, 
                                          uint32_t n_dmrs); 

SRSLTE_API int srslte_ue_dl_decode_broad(srslte_ue_dl_t *q,
										cf_t *input,
										uint8_t *data,
										uint32_t sf_idx,
										uint16_t rnti);

SRSLTE_API void srslte_ue_dl_set_rnti(srslte_ue_dl_t *q, 
                                      uint16_t rnti);

SRSLTE_API void srslte_ue_dl_save_signal(srslte_ue_dl_t *q, 
                                         srslte_softbuffer_rx_t *softbuffer, 
                                         uint32_t tti, 
                                         uint32_t rv_idx, 
                                         uint16_t rnti, 
                                         uint32_t cfi); 

SRSLTE_API void srslte_ue_dl_reset_rnti_list(srslte_ue_dl_t *q);

SRSLTE_API void srslte_ue_dl_update_rnti_list(srslte_ue_dl_t *q);

SRSLTE_API void srslte_ue_dl_reset_rnti_user(srslte_ue_dl_t *q, uint16_t user);

SRSLTE_API void srslte_ue_dl_reset_rnti_user_to(srslte_ue_dl_t *q, uint16_t user, uint16_t val);

SRSLTE_API int rnti_in_list(srslte_ue_dl_t *q, uint16_t check);

#endif
