/**
 *
 * \section COPYRIGHT
 *
 * Copyright 2013-2020 Software Radio Systems Limited
 *
 * By using this file, you agree to the terms and conditions set
 * forth in the LICENSE file which can be found at the top level of
 * the distribution.
 *
 */

#ifndef SRSUE_UE_STACK_NR_H
#define SRSUE_UE_STACK_NR_H

#include <functional>
#include <pthread.h>
#include <stdarg.h>
#include <string>

#include "mac_nr/mac_nr.h"
#include "rrc/rrc_nr.h"
#include "srslte/radio/radio.h"
#include "srslte/upper/pdcp.h"
#include "srslte/upper/rlc.h"
#include "upper/nas.h"
#include "upper/usim.h"

#include "srslte/common/buffer_pool.h"
#include "srslte/common/mac_pcap.h"
#include "srslte/common/multiqueue.h"
#include "srslte/common/thread_pool.h"
#include "srslte/interfaces/ue_nr_interfaces.h"

#include "srsue/hdr/ue_metrics_interface.h"
#include "ue_stack_base.h"

namespace srsue {

/** \brief L2/L3 stack class for 5G/NR UEs.
 *
 *  This class wraps all L2/L3 blocks and provides a single interface towards the PHY.
 */

class ue_stack_nr final : public ue_stack_base,
                          public stack_interface_phy_nr,
                          public stack_interface_gw,
                          public stack_interface_rrc,
                          public srslte::thread
{
public:
  ue_stack_nr();
  ~ue_stack_nr();

  std::string get_type() final;

  int  init(const stack_args_t& args_);
  int  init(const stack_args_t& args_, phy_interface_stack_nr* phy_, gw_interface_stack* gw_);
  bool switch_on() final;
  bool switch_off() final;
  void stop();

  // GW srsue stack_interface_gw dummy interface
  bool is_registered() { return true; };
  bool start_service_request() { return true; };

  bool get_metrics(stack_metrics_t* metrics);
  bool is_rrc_connected();

  // RRC interface for PHY
  void in_sync() final;
  void out_of_sync() final;
  void run_tti(uint32_t tti) final;

  // MAC interface for PHY
  sched_rnti_t get_dl_sched_rnti_nr(const uint32_t tti) final { return mac->get_dl_sched_rnti_nr(tti); }
  sched_rnti_t get_ul_sched_rnti_nr(const uint32_t tti) final { return mac->get_ul_sched_rnti_nr(tti); }
  int          sf_indication(const uint32_t tti)
  {
    run_tti(tti);
    return SRSLTE_SUCCESS;
  }
  void tb_decoded(const uint32_t cc_idx, mac_nr_grant_dl_t& grant) final { mac->tb_decoded(cc_idx, grant); }
  void new_grant_ul(const uint32_t cc_idx, const mac_nr_grant_ul_t& grant, tb_action_ul_t* action) final
  {
    mac->new_grant_ul(cc_idx, grant, action);
  }
  void prach_sent(uint32_t tti, uint32_t s_id, uint32_t t_id, uint32_t f_id, uint32_t ul_carrier_id)
  {
    mac->prach_sent(tti, s_id, t_id, f_id, ul_carrier_id);
  }

  // Interface for GW
  void write_sdu(uint32_t lcid, srslte::unique_byte_buffer_t sdu) final;
  bool is_lcid_enabled(uint32_t lcid) final { return pdcp->is_lcid_enabled(lcid); }

  // Interface for RRC
  srslte::tti_point get_current_tti() { return srslte::tti_point{0}; };

private:
  void run_thread() final;
  void run_tti_impl(uint32_t tti);
  void stop_impl();

  bool                running = false;
  srsue::stack_args_t args    = {};

  // task scheduler
  srslte::task_scheduler                task_sched;
  srslte::task_multiqueue::queue_handle sync_task_queue, ue_task_queue, gw_task_queue;

  // UE stack logging
  srslog::basic_logger& mac_logger;
  srslog::basic_logger& rlc_logger;
  srslog::basic_logger& pdcp_logger;

  // stack components
  std::unique_ptr<mac_nr>       mac;
  std::unique_ptr<rrc_nr>       rrc;
  std::unique_ptr<srslte::rlc>  rlc;
  std::unique_ptr<srslte::pdcp> pdcp;

  // RAT-specific interfaces
  phy_interface_stack_nr* phy = nullptr;
  gw_interface_stack*     gw  = nullptr;

  // Thread
  static const int STACK_MAIN_THREAD_PRIO = 4;
};

} // namespace srsue

#endif // SRSUE_UE_STACK_NR_H
