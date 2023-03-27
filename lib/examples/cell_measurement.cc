/**
 * Copyright 2013-2022 Software Radio Systems Limited
 *
 * This file is part of srsRAN.
 *
 * srsRAN is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Affero General Public License as
 * published by the Free Software Foundation, either version 3 of
 * the License, or (at your option) any later version.
 *
 * srsRAN is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Affero General Public License for more details.
 *
 * A copy of the GNU Affero General Public License can be found in
 * the LICENSE file in the top-level directory of this distribution
 * and at http://www.gnu.org/licenses/.
 *
 */

#include <assert.h>
#include <iomanip>
#include <math.h>
#include <signal.h>
#include <sstream>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <strings.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <sys/un.h>
#include <thread>
#include <unistd.h>
#include <vector>

#define ENABLE_AGC_DEFAULT

extern "C" {
#include "srsran/common/crash_handler.h"
#include "srsran/phy/rf/rf.h"
#include "srsran/phy/rf/rf_utils.h"
#include "srsran/srsran.h"
}
#include "srsran/asn1/rrc/rrc_asn1.h"
#include "srsran/common/bcd_helpers.h"

#define MHZ 1000000
#define SAMP_FREQ 1920000
#define FLEN 9600
#define FLEN_PERIOD 0.005

#define MAX_EARFCN 1000

cell_search_cfg_t cell_detect_config = {.max_frames_pbch      = SRSRAN_DEFAULT_MAX_FRAMES_PBCH,
                                        .max_frames_pss       = SRSRAN_DEFAULT_MAX_FRAMES_PSS,
                                        .nof_valid_pss_frames = SRSRAN_DEFAULT_NOF_VALID_PSS_FRAMES,
                                        .init_agc             = 0,
                                        .force_tdd            = false};
struct cells {
  srsran_cell_t cell;
  float         freq;
  int           dl_earfcn;
  float         power;
};
struct cells results[1024];

/**********************************************************************
 *  Program arguments processing
 ***********************************************************************/
typedef struct {
  int                   nof_subframes;
  bool                  disable_plots;
  int                   force_N_id_2;
  char*                 rf_args;
  float                 rf_freq;
  float                 rf_gain;
  int                   band;
  int                   earfcn_start;
  int                   earfcn_end;
  std::string           earfcn_list;
  std::vector<uint32_t> earfcn_vector;
} prog_args_t;

void args_default(prog_args_t* args)
{
  args->nof_subframes = -1;
  args->force_N_id_2  = -1; // Pick the best
  args->earfcn_list   = "";
  args->rf_args       = "";
  args->rf_freq       = -1.0;
  args->band          = -1;
  args->earfcn_start  = -1;
  args->earfcn_end    = -1;

#ifdef ENABLE_AGC_DEFAULT
  args->rf_gain = -1;
#else
  args->rf_gain = 50;
#endif
}

void usage(prog_args_t* args, char* prog)
{
  printf("Usage: %s [agselnv] -b band\n", prog);
  printf("\t-z earfcn_list comma separated list of EARFCNs [empty by default]\n");
  printf("\t-a RF args [Default %s]\n", args->rf_args);
  printf("\t-g RF RX gain [Default %.2f dB]\n", args->rf_gain);
  printf("\t-s earfcn_start [Default All]\n");
  printf("\t-e earfcn_end [Default All]\n");
  printf("\t-l Force N_id_2 [Default best]\n");
  printf("\t-n nof_subframes [Default %d]\n", args->nof_subframes);
  printf("\t-v [set srsran_verbose to debug, default none]\n");
}

int parse_args(prog_args_t* args, int argc, char** argv)
{
  int opt;
  args_default(args);
  while ((opt = getopt(argc, argv, "aglnvfsebz")) != -1) {
    switch (opt) {
      case 'a':
        args->rf_args = argv[optind];
        break;
      case 'g':
        args->rf_gain = atof(argv[optind]);
        break;
      case 'f':
        args->rf_freq = atof(argv[optind]);
        break;
      case 'n':
        args->nof_subframes = atoi(argv[optind]);
        break;
      case 'z':
        args->earfcn_list = argv[optind];
        if (args->earfcn_list != "") {
          std::stringstream ss(args->earfcn_list);
          while (ss.good()) {
            std::string substr;
            getline(ss, substr, ',');
            const int earfcn = atoi(substr.c_str());
            args->earfcn_vector.push_back(earfcn);
          }
        } else {
          printf("Error: earfcn list is empty\n");
          return false;
        }
        break;
      case 'l':
        args->force_N_id_2 = atoi(argv[optind]);
        break;
      case 'b':
        args->band = atoi(argv[optind]);
        break;
      case 's':
        args->earfcn_start = atoi(argv[optind]);
        break;
      case 'e':
        args->earfcn_end = atoi(argv[optind]);
        break;
      case 'v':
        srsran_verbose++;
        break;
      default:
        usage(args, argv[0]);
        return -1;
    }
  }
  if (args->earfcn_list == "") {
    usage(args, argv[0]);
    return -1;
  }
  return 0;
}
/**********************************************************************/

/* TODO: Do something with the output data */
uint8_t* data[SRSRAN_MAX_CODEWORDS];

bool go_exit = false;
void sig_int_handler(int signo)
{
  printf("SIGINT received. Exiting...\n");
  if (signo == SIGINT) {
    go_exit = true;
  }
}

double srsran_rf_set_rx_gain_wrapper(void* h, double f)
{
  return srsran_rf_set_rx_gain((srsran_rf_t*)h, f);
}

int srsran_rf_recv_wrapper(void* h, cf_t* data[SRSRAN_MAX_PORTS], uint32_t nsamples, srsran_timestamp_t* q)
{
  DEBUG(" ----  Receive %d samples  ---- \n", nsamples);

  return srsran_rf_recv((srsran_rf_t*)h, data[0], nsamples, 1);
}

enum receiver_state { DECODE_MIB, DECODE_SIB, MEASURE } state;

#define MAX_SINFO 10
#define MAX_NEIGHBOUR_CELLS 128

typedef struct {
  uint16_t    mcc;
  uint16_t    mnc;
  uint16_t    tac;
  uint32_t    cid;
  uint16_t    phyid;
  uint16_t    earfcn;
  double      rssi;
  double      frequency;
  uint32_t    enodeb_id;
  uint16_t    sector_id;
  double      cfo;
  double      rsrq;
  double      snr;
  double      rsrp;
  double      tx_pwr;
  std::string raw_sib1;
} tower_info_t;

static int write_sib1_data(tower_info_t tower)
{
  std::ostringstream os;
  long               seconds = (unsigned long)time(NULL);

  os << tower.mcc << "," << tower.mnc << "," << tower.tac << "," << tower.cid << "," << tower.phyid << ","
     << tower.earfcn << "," << tower.rssi << "," << tower.frequency << "," << tower.enodeb_id << "," << tower.sector_id
     << "," << tower.cfo << "," << tower.rsrq << "," << tower.snr << "," << tower.rsrp << "," << tower.tx_pwr << ","
     << tower.raw_sib1 << "," << seconds;
  // https://stackoverflow.com/questions/1374468/stringstream-string-and-char-conversion-confusion

  const std::string& tmp    = os.str();
  const char*        packet = tmp.c_str();

  printf("**** sending packet: <%s>\n", packet);

  const char*        socket_path = "/tmp/croc.sock";
  struct sockaddr_un addr;
  int                fd;
  if ((fd = socket(AF_UNIX, SOCK_STREAM, 0)) == -1) {
    perror("socket error");
    exit(-1);
  }
  memset(&addr, 0, sizeof(addr));
  addr.sun_family = AF_UNIX;
  if (*socket_path == '\0') {
    *addr.sun_path = '\0';
    strncpy(addr.sun_path + 1, socket_path + 1, sizeof(addr.sun_path) - 2);
  } else {
    strncpy(addr.sun_path, socket_path, sizeof(addr.sun_path) - 1);
  }

  if (connect(fd, (struct sockaddr*)&addr, sizeof(addr)) == -1) {
    perror("connect error");
    exit(-1);
  }

  ssize_t w = write(fd, packet, strlen(packet));
  return (int)w;
}

void get_fd_for_earfcn_vector(std::vector<uint32_t> earfcn_vector, srsran_earfcn_t* channels)
{
  for (int j = 0; j < earfcn_vector.size(); j++) {
    channels[j].id = earfcn_vector[j];
    channels[j].fd = srsran_band_fd(earfcn_vector[j]);
  }
}

int main(int argc, char** argv)
{
  int               ret;
  cf_t*             sf_buffer[SRSRAN_MAX_PORTS] = {NULL, NULL};
  prog_args_t       prog_args;
  srsran_cell_t     cell;
  int64_t           sf_cnt;
  srsran_rf_t       rf;
  srsran_ofdm_t     fft;
  srsran_chest_dl_t chest;
  uint32_t          nframes    = 0;
  uint32_t          nof_trials = 0;
  uint32_t          max_trials = 16;
  uint32_t          sfn        = 0; // system frame number
  int               n;
  int               sfn_offset;
  cf_t*             ce[SRSRAN_MAX_PORTS];
  float             cfo = 0;

  srsran_ue_cellsearch_t        cs;
  srsran_ue_cellsearch_result_t found_cells[3];
  int32_t                       nof_freqs;
  int32_t                       freq;
  uint32_t                      n_found_cells = 0;

  srsran_debug_handle_crash(argc, argv);

  if (parse_args(&prog_args, argc, argv)) {
    exit(-1);
  }
  printf("- Scanning %d EARFCNs\n", prog_args.earfcn_vector.size());
  srsran_earfcn_t channels[prog_args.earfcn_vector.size()];

  printf("Opening RF device...\n");
  if (srsran_rf_open(&rf, prog_args.rf_args)) {
    fprintf(stderr, "Error opening rf\n");
    exit(-1);
  }
  if (prog_args.rf_gain > 0) {
    srsran_rf_set_rx_gain(&rf, prog_args.rf_gain);
  } else {
    printf("Starting AGC thread...\n");
    if (srsran_rf_start_gain_thread(&rf, false)) {
      fprintf(stderr, "Error opening rf\n");
      exit(-1);
    }
    srsran_rf_set_rx_gain(&rf, 50);
  }

  sf_buffer[0] = (cf_t*)srsran_vec_malloc(3 * sizeof(cf_t) * SRSRAN_SF_LEN_PRB(100));
  for (int i = 0; i < SRSRAN_MAX_CODEWORDS; i++) {
    data[i] = (uint8_t*)srsran_vec_malloc(sizeof(uint8_t) * 1500 * 8);
  }

  sigset_t sigset;
  sigemptyset(&sigset);
  sigaddset(&sigset, SIGINT);
  sigprocmask(SIG_UNBLOCK, &sigset, NULL);
  signal(SIGINT, sig_int_handler);

  srsran_rf_set_master_clock_rate(&rf, 30.72e6);

  // Supress RF messages
  srsran_rf_suppress_stdout(&rf);

  nof_freqs = prog_args.earfcn_vector.size();

  get_fd_for_earfcn_vector(prog_args.earfcn_vector, channels);

  // srsran_band_get_fd_band(prog_args.band, channels, prog_args.earfcn_start, prog_args.earfcn_end, MAX_EARFCN);
  if (nof_freqs < 0) {
    fprintf(stderr, "Error getting EARFCN list\n");
    exit(-1);
  }

  fprintf(stderr, "******* first init multi\n");
  if (srsran_ue_cellsearch_init_multi(&cs, cell_detect_config.max_frames_pss, srsran_rf_recv_wrapper, 1, (void*)&rf)) {
    fprintf(stderr, "Error initiating UE cell detect\n");
    exit(-1);
  }
  fprintf(stderr, "******* first init multi done \n");

  if (cell_detect_config.max_frames_pss) {
    srsran_ue_cellsearch_set_nof_valid_frames(&cs, cell_detect_config.nof_valid_pss_frames);
  }
  if (cell_detect_config.init_agc) {
    srsran_rf_info_t* rf_info = srsran_rf_get_info(&rf);
    srsran_ue_sync_start_agc(&cs.ue_sync,
                             srsran_rf_set_rx_gain_wrapper,
                             rf_info->min_rx_gain,
                             rf_info->max_rx_gain,
                             cell_detect_config.init_agc);
  }

  /* begin cell search loop */
  freq = -1;
  bzero(found_cells, 3 * sizeof(srsran_ue_cellsearch_result_t));
  while (!go_exit) {
    srsran_ue_sync_t ue_sync;
    srsran_ue_mib_t  ue_mib;
    srsran_ue_dl_t   ue_dl;
    bool             acks[SRSRAN_MAX_CODEWORDS] = {false};
    /* set rf_freq */
    freq++;
    if (freq == nof_freqs) {
      freq = 0; // continue loop at the beginning
    }
    float rx_freq = channels[freq].fd * MHZ;
    srsran_rf_set_rx_freq(&rf, (double)rx_freq);
    srsran_rf_rx_wait_lo_locked(&rf);
    INFO("Set rf_freq to %.3f MHz\n", (double)rx_freq / 1000000);

    printf(
        "[%3d/%d]: EARFCN %d Freq. %.2f MHz looking for PSS.\n", freq, nof_freqs, channels[freq].id, channels[freq].fd);
    fflush(stdout);

    if (SRSRAN_VERBOSE_ISINFO()) {
      printf("\n");
    }

    INFO("Setting sampling frequency %.2f MHz for PSS search\n", SRSRAN_CS_SAMP_FREQ / 1000000);
    srsran_rf_set_rx_srate(&rf, SRSRAN_CS_SAMP_FREQ);
    INFO("Starting receiver...\n");
    srsran_rf_start_rx_stream(&rf, false);

    n                 = srsran_ue_cellsearch_scan(&cs, found_cells, NULL);
    int           ret = SRSRAN_UE_MIB_NOTFOUND;
    srsran_cell_t cell;
    if (n < 0) {
      fprintf(stderr, "Error searching cell\n");
      exit(-1);
    } else if (n > 0) {
      for (int i = 0; i < 3; i++) {
        if (found_cells[i].psr > 10.0) {
          cell.id   = found_cells[i].cell_id;
          cell.cp   = found_cells[i].cp;
          cell.peak = 20 * log10(found_cells[i].peak * 1000);

          fprintf(stderr, "******* rf_mib_decoder\n");
          ret = rf_mib_decoder(&rf, 1, &cell_detect_config, &cell, NULL);
          fprintf(stderr, "******* rf_mib_decoder_done\n");
          if (ret < 0) {
            fprintf(stderr, "Error decoding MIB\n");
            continue;
          }
        }
      }
      if (ret == SRSRAN_UE_MIB_NOTFOUND) {
        continue;
      }

      if (ret == SRSRAN_UE_MIB_FOUND) {
        printf("Found CELL ID %d. %d PRB, %d ports\n", cell.id, cell.nof_prb, cell.nof_ports);
      }

      /* set receiver frequency */
      printf("Tunning receiver to %.3f MHz\n", (double)rx_freq / 1000000);

      cell_detect_config.init_agc = (prog_args.rf_gain < 0);

      uint32_t  ntrial       = 0;
      const int MAX_ATTEMPTS = 1;
      /*
      do {
            fprintf(stderr, "******* rf_search_and_decode_mib\n");
        ret = rf_search_and_decode_mib(&rf, 1, &cell_detect_config, prog_args.force_N_id_2, &cell, &cfo);
            fprintf(stderr, "******* rf_search_and_decode_mib_done\n");
        if (ret < 0) {
          fprintf(stderr, "Error searching for cell\n");
          exit(-1);
        } else if (ret == 0 && !go_exit) {
          printf("Cell not found after %d trials. Trying again (Press Ctrl+C to exit)\n", ntrial++);
        }
      } while (ret == 0 && !go_exit && ntrial < MAX_ATTEMPTS);

      if (go_exit) {
        exit(0);
      }

      /* set sampling frequency */
      int srate = srsran_sampling_freq_hz(cell.nof_prb);
      if (srate != -1) {
        if (srate < 10e6) {
          srsran_rf_set_master_clock_rate(&rf, 4 * srate);
        } else {
          srsran_rf_set_master_clock_rate(&rf, srate);
        }
        printf("Setting sampling rate %.2f MHz\n", (float)srate / 1000000);
        float srate_rf = srsran_rf_set_rx_srate(&rf, (double)srate);
        if (srate_rf != srate) {
          fprintf(stderr, "Could not set sampling rate\n");
          exit(-1);
        }
      } else {
        fprintf(stderr, "Invalid number of PRB %d\n", cell.nof_prb);
        continue;
      }

      /*
    printf("Stopping RF and flushing buffer...\n");
    srsran_rf_stop_rx_stream(&rf);
    srsran_rf_flush_buffer(&rf);
    printf("DONE Stopping RF and flushing buffer...\n");
    */

      fprintf(stderr, "******* init multi\n");
      if (srsran_ue_sync_init_multi(&ue_sync, cell.nof_prb, cell.id == 1000, srsran_rf_recv_wrapper, 1, (void*)&rf)) {
        fprintf(stderr, "Error initiating ue_sync\n");
        return -1;
      }
      if (srsran_ue_sync_set_cell(&ue_sync, cell)) {
        fprintf(stderr, "Error initiating ue_sync\n");
        return -1;
      }
      if (srsran_ue_dl_init(&ue_dl, sf_buffer, cell.nof_prb, 1)) {
        fprintf(stderr, "Error initiating UE downlink processing module\n");
        return -1;
      }
      if (srsran_ue_dl_set_cell(&ue_dl, cell)) {
        fprintf(stderr, "Error initiating UE downlink processing module\n");
        return -1;
      }
      if (srsran_ue_mib_init(&ue_mib, sf_buffer, cell.nof_prb)) {
        fprintf(stderr, "Error initaiting UE MIB decoder\n");
        return -1;
      }
      if (srsran_ue_mib_set_cell(&ue_mib, cell)) {
        fprintf(stderr, "Error initaiting UE MIB decoder\n");
        return -1;
      }

      /* Configure downlink receiver for the SI-RNTI since will be the only one we'll use */
      srsran_ue_dl_set_rnti(&ue_dl, SRSRAN_SIRNTI);

      /* Initialize subframe counter */
      sf_cnt = 0;

      int sf_re = SRSRAN_SF_LEN_RE(cell.nof_prb, cell.cp);

      cf_t* sf_symbols = (cf_t*)srsran_vec_malloc(sf_re * sizeof(cf_t));

      for (int i = 0; i < SRSRAN_MAX_PORTS; i++) {
        ce[i] = (cf_t*)srsran_vec_malloc(sizeof(cf_t) * sf_re);
      }

      if (srsran_ofdm_rx_init(&fft, cell.cp, sf_buffer[0], sf_symbols, cell.nof_prb)) {
        fprintf(stderr, "Error initiating FFT\n");
        return -1;
      }
      if (srsran_chest_dl_init(&chest, cell.nof_prb)) {
        fprintf(stderr, "Error initiating channel estimator\n");
        return -1;
      }
      if (srsran_chest_dl_set_cell(&chest, cell)) {
        fprintf(stderr, "Error initiating channel estimator\n");
        return -1;
      }

      fprintf(stderr, "******* start rx stream\n");
      srsran_rf_start_rx_stream(&rf, false);
      fprintf(stderr, "******* start rx stream done\n");

      float rx_gain_offset = 0;
      fprintf(stderr, "******* Begin SIB Decoding Loop");

      /* Main loop */
      bool         exit_decode_loop = false;
      tower_info_t tower;
      tower.frequency = channels[freq].fd;
      tower.earfcn    = channels[freq].id;
      int   mib_tries = 0;
      float rssi_utra = 0, rssi = 0, rsrp = 0, rsrq = 0, snr = 0;
      state = DECODE_MIB;
      while ((sf_cnt < prog_args.nof_subframes || prog_args.nof_subframes == -1) && !go_exit && !exit_decode_loop) {
        uint8_t bch_payload[SRSRAN_BCH_PAYLOAD_LEN];

        ret = srsran_ue_sync_zerocopy_multi(&ue_sync, sf_buffer);
        if (ret < 0) {
          fprintf(stderr, "Error calling srsran_ue_sync_work()\n");
        }

        /* srsran_ue_sync_get_buffer returns 1 if successfully read 1 aligned subframe */
        if (ret == 1) {
          switch (state) {
            case DECODE_MIB:
              mib_tries++;
              fprintf(stderr, "******* Decoding MIB try %i\n", mib_tries);
              if (mib_tries > 20) {
                exit_decode_loop = true;
                break;
              }
              if (srsran_ue_sync_get_sfidx(&ue_sync) == 0) {
                srsran_pbch_decode_reset(&ue_mib.pbch);
                n = srsran_ue_mib_decode(&ue_mib, bch_payload, NULL, &sfn_offset);
                if (n < 0) {
                  fprintf(stderr, "Error decoding UE MIB\n");
                  break;
                } else if (n == SRSRAN_UE_MIB_FOUND) {
                  srsran_pbch_mib_unpack(bch_payload, &cell, &sfn);
                  printf("Decoded MIB. SFN: %d, offset: %d\n", sfn, sfn_offset);
                  tower.phyid  = cell.id;
                  tower.tx_pwr = cell.peak;
                  sfn          = (sfn + sfn_offset) % 1024;
                  state        = DECODE_SIB;
                }
              }
              break;
            case DECODE_SIB:
              /* We are looking for SI Blocks, search only in appropiate places */
              if ((srsran_ue_sync_get_sfidx(&ue_sync) == 5 && (sfn % 2) == 0)) {
                n = srsran_ue_dl_decode(&ue_dl, data, 0, sfn * 10 + srsran_ue_sync_get_sfidx(&ue_sync), acks);
                if (n < 0) {
                  fprintf(stderr, "Error decoding UE DL\n");
                  fflush(stdout);
                  exit_decode_loop = true;
                  break;
                } else if (n == 0) {
                  printf("CFO: %+6.4f kHz, SFO: %+6.4f kHz, PDCCH-Det: %.3f\r",
                         srsran_ue_sync_get_cfo(&ue_sync) / 1000,
                         srsran_ue_sync_get_sfo(&ue_sync) / 1000,
                         (float)ue_dl.nof_detected / nof_trials);
                  nof_trials++;
                  if (nof_trials > max_trials) {
                    fprintf(stderr, "Error decoding UE DL\n");
                    fflush(stdout);
                    exit_decode_loop = true;
                    break;
                  }
                } else {
                  printf("Decoded SIB1. Payload: ");
                  srsran_vec_fprint_byte(stdout, data[0], n / 8);
                  ;
                  asn1::rrc::bcch_dl_sch_msg_s dlsch_msg;
                  std::ostringstream           sib1;
                  sib1 << std::hex;
                  for (int i = 0; i < n / 8; i++) {
                    sib1 << std::setfill('0') << std::setw(2) << (int)data[0][i];
                  }
                  tower.raw_sib1 = sib1.str();

                  asn1::bit_ref     bref(data[0], n / 8);
                  asn1::SRSASN_CODE unpackResult = dlsch_msg.unpack(bref);

                  if (unpackResult == asn1::SRSASN_SUCCESS) {
                    int msgTypeValue = dlsch_msg.msg.type().value;
                    if (msgTypeValue == 0) {
                      if (dlsch_msg.msg.c1().type().value ==
                          asn1::rrc::bcch_dl_sch_msg_type_c::c1_c_::types::sib_type1) {
                        //                                        printf("Accessing dlsch_msg.msg.c1().sib_type1()\n");
                        asn1::rrc::sib_type1_s* sib1 = &dlsch_msg.msg.c1().sib_type1();
                        //                                      printf("Accessing
                        //                                      sib1->cell_access_related_info.plmn_id_list[0].plmn_id\n");
                        asn1::rrc::plmn_id_s plmn = sib1->cell_access_related_info.plmn_id_list[0].plmn_id;

                        std::string plmn_string = srsran::plmn_id_to_string(plmn);
                        // If we were using C++11 we could just use stoi
                        tower.mcc = atoi(plmn_string.substr(0, 3).c_str());
                        tower.mnc = atoi(plmn_string.substr(3, plmn_string.length() - 3).c_str());
                        // srsran::bytes_to_mcc(&plmn.mcc[0], &mcc);
                        // srsran::bytes_to_mnc(&plmn.mnc[0], &mnc, plmn.mnc.size());
                        tower.tac       = (uint16_t)sib1->cell_access_related_info.tac.to_number();
                        tower.cid       = (uint32_t)sib1->cell_access_related_info.cell_id.to_number();
                        tower.enodeb_id = tower.cid >> 8;
                        tower.sector_id = tower.cid & 255;

                        printf("MCC=%d, MNC=%d, PID=%d, TAC=%d, CID=%d\n",
                               tower.mcc,
                               tower.mnc,
                               tower.phyid,
                               tower.tac,
                               tower.cid);
                        if ((tower.mnc != 0) && (tower.mcc != 0)) {
                          state = MEASURE;
                          // exit_decode_loop = true;
                        }
                      }
                    }
                  }
                  state = MEASURE;
                  // exit_decode_loop = true;
                }
              }
              break;

            case MEASURE:

              if (srsran_ue_sync_get_sfidx(&ue_sync) == 5) {
                /* Run FFT for all subframe data */
                srsran_ofdm_rx_sf(&fft);

                srsran_chest_dl_estimate(&chest, sf_symbols, ce, srsran_ue_sync_get_sfidx(&ue_sync));

                rssi = SRSRAN_VEC_EMA(
                    srsran_vec_avg_power_cf(sf_buffer[0], SRSRAN_SF_LEN(srsran_symbol_sz(cell.nof_prb))), rssi, 0.05);
                rssi_utra = SRSRAN_VEC_EMA(srsran_chest_dl_get_rssi(&chest), rssi_utra, 0.05);
                rsrq      = SRSRAN_VEC_EMA(srsran_chest_dl_get_rsrq(&chest), rsrq, 0.05);
                rsrp      = SRSRAN_VEC_EMA(srsran_chest_dl_get_rsrp(&chest), rsrp, 0.05);
                snr       = SRSRAN_VEC_EMA(srsran_chest_dl_get_snr(&chest), snr, 0.05);

                nframes++;
              }

              if ((nframes % 100) == 0 || rx_gain_offset == 0) {
                if (srsran_rf_has_rssi(&rf)) {
                  rx_gain_offset = 30 + 10 * log10(rssi * 1000) - srsran_rf_get_rssi(&rf);
                } else {
                  rx_gain_offset = srsran_rf_get_rx_gain(&rf);
                }
              }

              // Plot and Printf
              if ((nframes % 10) == 0) {
                printf("CFO: %+8.4f kHz, SFO: %+8.4f Hz, RSSI: %5.1f dBm, RSSI/ref-symbol: %+5.1f dBm, "
                       "RSRP: %+5.1f dBm, RSRQ: %5.1f dB, SNR: %5.1f dB\r",
                       srsran_ue_sync_get_cfo(&ue_sync) / 1000,
                       srsran_ue_sync_get_sfo(&ue_sync),
                       10 * log10(rssi * 1000) - rx_gain_offset,
                       10 * log10(rssi_utra * 1000) - rx_gain_offset,
                       10 * log10(rsrp * 1000) - rx_gain_offset,
                       10 * log10(rsrq),
                       10 * log10(snr));
                if (srsran_verbose != SRSRAN_VERBOSE_NONE) {
                  printf("\n");
                }
                tower.rssi = 10 * log10(rssi * 1000) - rx_gain_offset;
                tower.rsrq = 10 * log10(rsrq);
                tower.snr  = 10 * log10(snr);
                tower.rsrp = 10 * log10(rsrp * 1000) - rx_gain_offset;
                if (tower.rssi > -200 && tower.rssi < 200) {
                  // If you know of a better way to test that it's a real number (not NaN or ∞,) I'd like to hear it.
                  tower.cfo = srsran_ue_sync_get_cfo(&ue_sync) / 1000;
                  std::thread thread_socket(write_sib1_data, tower);
                  thread_socket.detach();
                  exit_decode_loop = true;
                }
              }
          }
          if (srsran_ue_sync_get_sfidx(&ue_sync) == 9) {
            sfn++;
            if (sfn == 1024) {
              sfn = 0;
            }
          }
        } else if (ret == 0) {
          printf("Finding PSS... Peak: %8.1f, FrameCnt: %d, State: %d\r",
                 srsran_sync_get_peak_value(&ue_sync.sfind),
                 ue_sync.frame_total_cnt,
                 ue_sync.state);
          if (sf_cnt > 1000) {
            exit_decode_loop = true;
          }
        }

        sf_cnt++;
        if (exit_decode_loop) {
          sf_cnt = 0;
          break;
        }
      } // Decoding Loop
      free(sf_symbols);
      for (int i = 0; i < SRSRAN_MAX_CODEWORDS; i++) {
        if (ce[i]) {
          free(ce[i]);
        }
      }
      srsran_ue_dl_free(&ue_dl);
      srsran_ue_sync_free(&ue_sync);
      srsran_ue_mib_free(&ue_mib);

    } // if found cell
    // if (freq == nof_freqs) {
    //   freq = -1; //continue loop at the beginning
    // }
  } // Search loop

  for (int i = 0; i < SRSRAN_MAX_CODEWORDS; i++) {
    if (data[i]) {
      free(data[i]);
    }
  }
  for (int i = 0; i < SRSRAN_MAX_PORTS; i++) {
    if (sf_buffer[i]) {
      free(sf_buffer[i]);
    }
  }

  srsran_rf_close(&rf);
  printf("\nBye\n");
  exit(0);
}
