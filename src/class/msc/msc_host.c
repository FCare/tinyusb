/*
 * The MIT License (MIT)
 *
 * Copyright (c) 2019 Ha Thach (tinyusb.org)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 *
 * This file is part of the TinyUSB stack.
 */

#include "tusb_option.h"

#if CFG_TUH_ENABLED && CFG_TUH_MSC

#include "host/usbh.h"
#include "host/usbh_classdriver.h"

#include "msc_host.h"

//--------------------------------------------------------------------+
// MACRO CONSTANT TYPEDEF
//--------------------------------------------------------------------+
enum
{
  MSC_STAGE_IDLE = 0,
  MSC_STAGE_CMD,
  MSC_STAGE_DATA,
  MSC_STAGE_STATUS,
};

typedef struct
{
  uint8_t itf_num;
  uint8_t ep_in;
  uint8_t ep_out;

  uint8_t max_lun;

  volatile bool configured; // Receive SET_CONFIGURE
  volatile bool mounted;    // Enumeration is complete

  struct {
    uint32_t block_size;
    uint32_t block_count;
  } capacity[CFG_TUH_MSC_MAXLUN];

  //------------- SCSI -------------//
  uint8_t stage;
  void*   buffer;
  tuh_msc_complete_cb_t complete_cb;
  bool done;
  bool clear_done;
  uint retry;

  msc_cbw_t cbw;
  msc_csw_t csw;
}msch_interface_t;

CFG_TUSB_MEM_SECTION static msch_interface_t _msch_itf[CFG_TUH_DEVICE_MAX];

// buffer used to read scsi information when mounted
// largest response data currently is inquiry TODO Inquiry is not part of enum anymore
CFG_TUSB_MEM_SECTION TU_ATTR_ALIGNED(4)
static uint8_t _msch_buffer[sizeof(scsi_inquiry_resp_t)];

TU_ATTR_ALWAYS_INLINE
static inline msch_interface_t* get_itf(uint8_t dev_addr)
{
  return &_msch_itf[dev_addr-1];
}

//--------------------------------------------------------------------+
// PUBLIC API
//--------------------------------------------------------------------+
uint8_t tuh_msc_get_maxlun(uint8_t dev_addr)
{
  msch_interface_t* p_msc = get_itf(dev_addr);
  return p_msc->max_lun;
}

uint32_t tuh_msc_get_block_count(uint8_t dev_addr, uint8_t lun)
{
  msch_interface_t* p_msc = get_itf(dev_addr);
  return p_msc->capacity[lun].block_count;
}

uint32_t tuh_msc_get_block_size(uint8_t dev_addr, uint8_t lun)
{
  msch_interface_t* p_msc = get_itf(dev_addr);
  return p_msc->capacity[lun].block_size;
}

bool tuh_msc_mounted(uint8_t dev_addr)
{
  msch_interface_t* p_msc = get_itf(dev_addr);
  return p_msc->mounted;
}

bool tuh_msc_ready(uint8_t dev_addr)
{
  msch_interface_t* p_msc = get_itf(dev_addr);
  return p_msc->mounted && !usbh_edpt_busy(dev_addr, p_msc->ep_in);
}

static uint32_t tag = 0;
//--------------------------------------------------------------------+
// PUBLIC API: SCSI COMMAND
//--------------------------------------------------------------------+
static inline void cbw_init(msc_cbw_t *cbw, uint8_t lun)
{
  tu_memclr(cbw, sizeof(msc_cbw_t));
  cbw->signature = MSC_CBW_SIGNATURE;
  cbw->tag       = tag++; // TUSB
  cbw->lun       = lun;
}

static bool setSync(uint8_t dev_addr, msc_cbw_t const* cbw, msc_csw_t const* csw) {
  msch_interface_t* p_msc = get_itf(dev_addr);
  p_msc->done = true;
  return true;
}

static bool tuh_msc_clear_feature(uint8_t dev_addr, uint8_t ep_addr, uint8_t feature, tuh_control_complete_cb_t complete_cb)
{
  TU_LOG3("tuh_msc_clear_feature %x %x %x\n", dev_addr, ep_addr, feature);
  tusb_control_request_t const request =
  {
    .bmRequestType_bit =
    {
      .recipient = TUSB_REQ_RCPT_ENDPOINT,
      .type      = TUSB_REQ_TYPE_STANDARD,
      .direction = TUSB_DIR_OUT
    },
    .bRequest = HOST_REQUEST_CLEAR_FEATURE,
    .wValue   = feature,
    .wIndex   = ep_addr, //genre 129
    .wLength  = 0
  };

  TU_ASSERT( tuh_control_xfer_bypass(dev_addr, &request, NULL, complete_cb) );
  return true;
}

static bool clear_feature_complete(uint8_t dev_addr, tusb_control_request_t const * request, xfer_result_t result)
{
  msch_interface_t* p_msc = get_itf(dev_addr);
  p_msc->clear_done = true;
  return true;
}

static msc_cbw_t* nextToFree = NULL;
static void msch_event_start(hcd_event_t *event, bool start) {
  if (start) {
    msch_interface_t* p_msc = get_itf(event->dev_addr);

    nextToFree = (msc_cbw_t*)event->request;
    p_msc->cbw = *(nextToFree);
    p_msc->stage = MSC_STAGE_CMD;
    p_msc->buffer = event->buffer;
    p_msc->retry = 4;
    p_msc->complete_cb = (tuh_msc_complete_cb_t)event->complete_cb;

    TU_LOG2("Scsi Cmd: \n");
    TU_LOG2_MEM(p_msc->cbw.command, p_msc->cbw.cmd_len, 2);
  }
}

static bool tuh_msc_command(uint8_t dev_addr, msc_cbw_t const* cbw, void* data, tuh_msc_complete_cb_t complete_cb)
{
  msch_interface_t* p_msc = get_itf(dev_addr);
  TU_VERIFY(p_msc->configured);


  msc_cbw_t *new_cbw = (msc_cbw_t *)malloc(sizeof(msc_cbw_t));
  memcpy(new_cbw, cbw, sizeof(msc_cbw_t));
  hcd_event_t event =
  {
    .dev_addr = dev_addr,
    .ep_addr  = p_msc->ep_out,
    .id = getCmdId(),
    .event_id = HCD_EVENT_HOST_COMMAND,
    .buffer   = data,
    .request  = (void *)new_cbw,
    .request_size = sizeof(msc_cbw_t),
    .complete_cb = (void *)complete_cb,
    .event_cb = msch_event_start,
  };

  TU_LOG2("Stack MSC Cmd: 0x%x\n", event.id);
  TU_LOG2_MEM(cbw->command, cbw->cmd_len, 2);

  hcd_event_handler(&event, false);

  return true;
}

bool tuh_msc_scsi_command(uint8_t dev_addr, msc_cbw_t const* cbw, void* data, tuh_msc_complete_cb_t complete_cb)
{

  return tuh_msc_command(dev_addr, cbw, data, complete_cb);
  // bool ret = false;
  // msc_cbw_t resp_cbw;
  // msc_csw_t resp_csw;
  //
  //
  // msch_interface_t* p_msc = get_itf(dev_addr);
  // TU_VERIFY(p_msc->configured);
  //
  // // TODO claim endpoint
  //
  // TU_LOG2("Stack Scsi Cmd: \n");
  // TU_LOG2_MEM(cbw->command, cbw->cmd_len, 2);
  //
  // p_msc->cbw = *cbw;
  // p_msc->stage = MSC_STAGE_CMD;
  // p_msc->buffer = data;
  // p_msc->complete_cb = complete_cb;
  //
  // TU_ASSERT(usbh_edpt_xfer(dev_addr, p_msc->ep_out, (uint8_t*) &p_msc->cbw, sizeof(msc_cbw_t)));
  // return true;
}

bool tuh_msc_read_capacity(uint8_t dev_addr, uint8_t lun, scsi_read_capacity10_resp_t* response, tuh_msc_complete_cb_t complete_cb)
{
   msch_interface_t* p_msc = get_itf(dev_addr);
   TU_VERIFY(p_msc->configured);

  TU_LOG1("Read Capacity\r\n");

  msc_cbw_t cbw;
  cbw_init(&cbw, lun);

  cbw.total_bytes = sizeof(scsi_read_capacity10_resp_t);
  cbw.dir        = TUSB_DIR_IN_MASK;
  cbw.cmd_len    = sizeof(scsi_read_capacity10_t);

  scsi_read_capacity10_t const cmd_read_capacity =
  {
    .cmd_code     = SCSI_CMD_READ_CAPACITY_10,
  };
  memcpy(cbw.command, &cmd_read_capacity, cbw.cmd_len);

  return tuh_msc_scsi_command(dev_addr, &cbw, response, complete_cb);
}

bool tuh_msc_inquiry(uint8_t dev_addr, uint8_t lun, scsi_inquiry_resp_t* response, tuh_msc_complete_cb_t complete_cb)
{
  msch_interface_t* p_msc = get_itf(dev_addr);

  // TU_VERIFY(p_msc->mounted);
  msc_cbw_t cbw;
  cbw_init(&cbw, lun);

  cbw.total_bytes = sizeof(scsi_inquiry_resp_t);
  cbw.dir         = TUSB_DIR_IN_MASK;
  cbw.cmd_len     = sizeof(scsi_inquiry_t);

  scsi_inquiry_t const cmd_inquiry =
  {
    .cmd_code     = SCSI_CMD_INQUIRY,
    .alloc_length = tu_htons(sizeof(scsi_inquiry_resp_t))
  };
  memcpy(cbw.command, &cmd_inquiry, cbw.cmd_len);

  return tuh_msc_scsi_command(dev_addr, &cbw, response, complete_cb);
}

bool tuh_msc_test_unit_ready(uint8_t dev_addr, uint8_t lun, tuh_msc_complete_cb_t complete_cb)
{
  msch_interface_t* p_msc = get_itf(dev_addr);
  TU_VERIFY(p_msc->configured);

  TU_LOG1("Test Unit Ready\n");

  msc_cbw_t cbw;
  cbw_init(&cbw, lun);

  cbw.total_bytes = 0;
  cbw.dir         = TUSB_DIR_OUT;
  cbw.cmd_len     = sizeof(scsi_test_unit_ready_t);
  cbw.command[0]  = SCSI_CMD_TEST_UNIT_READY;
  cbw.command[1]  = lun; // according to wiki TODO need verification

  return tuh_msc_scsi_command(dev_addr, &cbw, NULL, complete_cb);
}

bool tuh_msc_mode_sense(uint8_t dev_addr, uint8_t lun, uint8_t page_code, uint8_t dbd,  uint8_t llba, uint16_t alloc_length, uint8_t* response, tuh_msc_complete_cb_t complete_cb)
{
   msch_interface_t* p_msc = get_itf(dev_addr);
   TU_VERIFY(p_msc->configured);

  TU_LOG1("Mode sense\r\n");

  msc_cbw_t cbw;
  cbw_init(&cbw, lun);

  cbw.total_bytes = alloc_length;
  cbw.dir        = TUSB_DIR_IN_MASK;
  cbw.cmd_len    = sizeof(scsi_mode_sense10_t);

  scsi_mode_sense10_t const cmd_mode_sense =
  {
    .cmd_code     = SCSI_CMD_MODE_SENSE_10,
    .dbd          = dbd,
    .llba         = llba,
    .page_code    = page_code,
    .alloc_length = alloc_length,
  };
  memcpy(cbw.command, &cmd_mode_sense, cbw.cmd_len);

  return tuh_msc_scsi_command(dev_addr, &cbw, response, complete_cb);
}


bool tuh_msc_request_sense(uint8_t dev_addr, uint8_t lun, void *resposne, tuh_msc_complete_cb_t complete_cb)
{
  msc_cbw_t cbw;
  bool ret = false;
  cbw_init(&cbw, lun);

  TU_LOG1("Request sense LUN = %d\n", lun);

  cbw.total_bytes = 18; // TODO sense response
  cbw.dir         = TUSB_DIR_IN_MASK;
  cbw.cmd_len     = sizeof(scsi_request_sense_t);

  scsi_request_sense_t const cmd_request_sense =
  {
    .cmd_code     = SCSI_CMD_REQUEST_SENSE,
    .alloc_length = tu_htons(18)
  };

  memcpy(&cbw.command[0], &cmd_request_sense, cbw.cmd_len);
  ret = tuh_msc_command(dev_addr, &cbw, resposne, complete_cb);
  return true;
}

bool tuh_msc_read10(uint8_t dev_addr, uint8_t lun, void * buffer, uint32_t lba, uint16_t block_count, tuh_msc_complete_cb_t complete_cb)
{
  msch_interface_t* p_msc = get_itf(dev_addr);
  TU_VERIFY(p_msc->mounted);

  msc_cbw_t cbw;
  cbw_init(&cbw, lun);

  cbw.total_bytes = block_count*p_msc->capacity[lun].block_size;
  cbw.dir         = TUSB_DIR_IN_MASK;
  cbw.cmd_len     = sizeof(scsi_read10_t);

  scsi_read10_t const cmd_read10 =
  {
    .cmd_code    = SCSI_CMD_READ_10,
    .lba         = tu_htonl(lba),
    .block_count = tu_htons(block_count)
  };

  memcpy(cbw.command, &cmd_read10, cbw.cmd_len);

  return tuh_msc_scsi_command(dev_addr, &cbw, buffer, complete_cb);
}

bool tuh_msc_read10_sync(uint8_t dev_addr, uint8_t lun, void * buffer, uint32_t lba, uint16_t block_count, tuh_msc_complete_cb_t complete_cb)
{
  msch_interface_t* p_msc = get_itf(dev_addr);
  TU_VERIFY(p_msc->mounted);

  msc_cbw_t cbw;
  cbw_init(&cbw, lun);

  cbw.total_bytes = block_count*p_msc->capacity[lun].block_size;
  cbw.dir         = TUSB_DIR_IN_MASK;
  cbw.cmd_len     = sizeof(scsi_read10_t);

  scsi_read10_t const cmd_read10 =
  {
    .cmd_code    = SCSI_CMD_READ_10,
    .lba         = tu_htonl(lba),
    .block_count = tu_htons(block_count)
  };

  memcpy(cbw.command, &cmd_read10, cbw.cmd_len);

  return tuh_msc_scsi_command(dev_addr, &cbw, buffer, complete_cb);
}

bool  tuh_msc_read_cd(uint8_t dev_addr, uint8_t lun, void * buffer, uint32_t lba, uint32_t block_count, bool subQ, tuh_msc_complete_cb_t complete_cb)
{
  msch_interface_t* p_msc = get_itf(dev_addr);
  TU_VERIFY(p_msc->mounted);

  msc_cbw_t cbw;
  cbw_init(&cbw, lun);

  uint32_t block_size = p_msc->capacity[lun].block_size;

  block_size = 2352;
  if (subQ) block_size += 96;

  cbw.total_bytes = block_count*block_size; // arevoir
  cbw.dir         = TUSB_DIR_IN_MASK;
  cbw.cmd_len     = sizeof(scsi_read_cd_t);

  scsi_read_cd_t const cmd_read_cd =
  {
    .cmd_code    = SCSI_CMD_READ_CD,
    .lba         = tu_htonl(lba),
    .block_count[0] = (block_count>>16)&0xFF, //MSB
    .block_count[1] = (block_count>>8)&0xFF,
    .block_count[2] = block_count&0xFF, //LSB
    .sector_type = 0b000,
    .sync = 1,
    .header_code = 0b11,
    .user_data = 1,
    .edc_ecc = 1,
    .sub_channel = subQ?0b001:0b000,
  };

  memcpy(cbw.command, &cmd_read_cd, cbw.cmd_len);

  return tuh_msc_scsi_command(dev_addr, &cbw, buffer, complete_cb);
}


bool tuh_msc_read_sub_channel(uint8_t dev_addr, uint8_t lun, void * buffer, tuh_msc_complete_cb_t complete_cb)
{
  msch_interface_t* p_msc = get_itf(dev_addr);
  TU_VERIFY(p_msc->mounted);

  msc_cbw_t cbw;
  cbw_init(&cbw, lun);

  cbw.total_bytes = 12+4; // arevoir
  cbw.dir         = TUSB_DIR_IN_MASK;
  cbw.cmd_len     = sizeof(scsi_read_sub_channel_t);

  scsi_read_sub_channel_t const cmd_read_sub_channel=
  {
    .cmd_code    = SCSI_CMD_READ_SUB_CHANNEL,
    .subq        = 1,
    .msf         = 1,
    .parameter   = 0x01,
    .alloc_length = tu_htons(12+4),
  };

  memcpy(cbw.command, &cmd_read_sub_channel, cbw.cmd_len);

  return tuh_msc_scsi_command(dev_addr, &cbw, buffer, complete_cb);
}

bool tuh_msc_read_header(uint8_t dev_addr, uint8_t lun, void * buffer, uint32_t lba, tuh_msc_complete_cb_t complete_cb)
{
  msch_interface_t* p_msc = get_itf(dev_addr);
  TU_VERIFY(p_msc->mounted);

  msc_cbw_t cbw;
  cbw_init(&cbw, lun);

  cbw.total_bytes = 8;
  cbw.dir         = TUSB_DIR_IN_MASK;
  cbw.cmd_len     = sizeof(scsi_read_header_t);

  scsi_read_header_t const cmd_read_header=
  {
    .cmd_code    = SCSI_CMD_READ_HEADER,
    .msf         = 1,
    .lba         = tu_htonl(lba),
    .alloc_length = tu_htons(8),
  };

  memcpy(cbw.command, &cmd_read_header, cbw.cmd_len);

  return tuh_msc_scsi_command(dev_addr, &cbw, buffer, complete_cb);
}

bool tuh_msc_start_stop(uint8_t dev_addr, uint8_t lun, bool start, bool load_eject, tuh_msc_complete_cb_t complete_cb)
{
  msch_interface_t* p_msc = get_itf(dev_addr);
  TU_VERIFY(p_msc->configured);

  msc_cbw_t cbw;
  cbw_init(&cbw, lun);

  cbw.total_bytes = 0;
  cbw.dir         = TUSB_DIR_IN_MASK;
  cbw.cmd_len     = sizeof(scsi_start_stop_unit_t);

  scsi_start_stop_unit_t const cmd_start_stop =
  {
    .cmd_code        = SCSI_CMD_START_STOP_UNIT,
    .start           = start,
    .load_eject      = load_eject,
    .lun = lun,
  };

  p_msc->mounted = false;

  memcpy(cbw.command, &cmd_start_stop, cbw.cmd_len);

  return tuh_msc_scsi_command(dev_addr, &cbw, NULL, complete_cb);
}

bool tuh_msc_write10(uint8_t dev_addr, uint8_t lun, void const * buffer, uint32_t lba, uint16_t block_count, tuh_msc_complete_cb_t complete_cb)
{
  msch_interface_t* p_msc = get_itf(dev_addr);
  TU_VERIFY(p_msc->mounted);

  msc_cbw_t cbw;
  cbw_init(&cbw, lun);

  cbw.total_bytes = block_count*p_msc->capacity[lun].block_size;
  cbw.dir         = TUSB_DIR_OUT;
  cbw.cmd_len     = sizeof(scsi_write10_t);

  scsi_write10_t const cmd_write10 =
  {
    .cmd_code    = SCSI_CMD_WRITE_10,
    .lba         = tu_htonl(lba),
    .block_count = tu_htons(block_count)
  };

  memcpy(cbw.command, &cmd_write10, cbw.cmd_len);

  return tuh_msc_scsi_command(dev_addr, &cbw, (void*)(uintptr_t) buffer, complete_cb);
}
bool tuh_msc_write10_sync(uint8_t dev_addr, uint8_t lun, void const * buffer, uint32_t lba, uint16_t block_count, tuh_msc_complete_cb_t complete_cb)
{
  msch_interface_t* p_msc = get_itf(dev_addr);
  TU_VERIFY(p_msc->mounted);

  msc_cbw_t cbw;
  cbw_init(&cbw, lun);

  cbw.total_bytes = block_count*p_msc->capacity[lun].block_size;
  cbw.dir         = TUSB_DIR_OUT;
  cbw.cmd_len     = sizeof(scsi_write10_t);

  scsi_write10_t const cmd_write10 =
  {
    .cmd_code    = SCSI_CMD_WRITE_10,
    .lba         = tu_htonl(lba),
    .block_count = tu_htons(block_count)
  };

  memcpy(cbw.command, &cmd_write10, cbw.cmd_len);

  return tuh_msc_scsi_command(dev_addr, &cbw, (void*)(uintptr_t) buffer, complete_cb);
}

bool tuh_msc_read_toc(uint8_t dev_addr, uint8_t lun, void * buffer, uint8_t msf, uint8_t starting_track, uint8_t nb_tracks, tuh_msc_complete_cb_t complete_cb)
{
  msch_interface_t* p_msc = get_itf(dev_addr);
  TU_VERIFY(p_msc->mounted);

  msc_cbw_t cbw;
  cbw_init(&cbw, lun);

  cbw.total_bytes = 4 + 8*nb_tracks;
  cbw.dir         = TUSB_DIR_IN_MASK;
  cbw.cmd_len     = sizeof(scsi_read_toc_t);

  scsi_read_toc_t const cmd_read_toc =
  {
    .cmd_code    = SCSI_CMD_READ_TOC,
    .msf         = msf,
    .starting_track = starting_track,
    .alloc_length = tu_htons(4 + 8*nb_tracks), //MAXIMUM TOC length
  };

  memcpy(cbw.command, &cmd_read_toc, cbw.cmd_len);

  return tuh_msc_scsi_command(dev_addr, &cbw, buffer, complete_cb);
}

bool tuh_msc_set_speed(uint8_t dev_addr, uint8_t lun, uint16_t read_speed, uint16_t write_speed, tuh_msc_complete_cb_t complete_cb)
{
  msch_interface_t* p_msc = get_itf(dev_addr);
  TU_VERIFY(p_msc->mounted);

  msc_cbw_t cbw;
  cbw_init(&cbw, lun);

  cbw.total_bytes = 0;
  cbw.dir         = TUSB_DIR_IN_MASK;
  cbw.cmd_len     = sizeof(scsi_set_speed_t);

  scsi_set_speed_t const cmd_set_speed =
  {
    .cmd_code    = SCSI_CMD_SET_SPEED,
    .read_speed  = tu_htons(read_speed),
    .write_speed = tu_htons(write_speed),
  };

  memcpy(cbw.command, &cmd_set_speed, cbw.cmd_len);

  return tuh_msc_scsi_command(dev_addr, &cbw, NULL, complete_cb);
}

#if 0
// MSC interface Reset (not used now)
bool tuh_msc_reset(uint8_t dev_addr)
{
  tusb_control_request_t const new_request =
  {
    .bmRequestType_bit =
    {
      .recipient = TUSB_REQ_RCPT_INTERFACE,
      .type      = TUSB_REQ_TYPE_CLASS,
      .direction = TUSB_DIR_OUT
    },
    .bRequest = MSC_REQ_RESET,
    .wValue   = 0,
    .wIndex   = p_msc->itf_num,
    .wLength  = 0
  };
  TU_ASSERT( usbh_control_xfer( dev_addr, &new_request, NULL ) );
}
#endif

//--------------------------------------------------------------------+
// CLASS-USBH API
//--------------------------------------------------------------------+
void msch_init(void)
{
  tu_memclr(_msch_itf, sizeof(_msch_itf));
}

void msch_close(uint8_t dev_addr)
{
  TU_VERIFY(dev_addr <= CFG_TUH_DEVICE_MAX, );

  msch_interface_t* p_msc = get_itf(dev_addr);

  // invoke Application Callback
  if (p_msc->mounted && tuh_msc_umount_cb) tuh_msc_umount_cb(dev_addr);

  tu_memclr(p_msc, sizeof(msch_interface_t));
}

bool msch_xfer_cb(uint8_t dev_addr, uint8_t ep_addr, xfer_result_t event, uint32_t xferred_bytes)
{
  msch_interface_t* p_msc = get_itf(dev_addr);
  msc_cbw_t const * cbw = &p_msc->cbw;
  msc_csw_t       * csw = &p_msc->csw;
  bool finished = false;

  if (nextToFree != NULL) {
    free(nextToFree);
    nextToFree = NULL;
  }

  TU_LOG2("MSC Xfer stage %d %x %x Transfert:%x\n", p_msc->stage, cbw->total_bytes, p_msc->buffer, event);
  if (event != XFER_RESULT_SUCCESS) {
      if (event == XFER_RESULT_STALLED) {
        if (p_msc->retry != 0) {
          p_msc->retry--;
          TU_LOG1("Stalled on phase %d\n", p_msc->stage);
          p_msc->clear_done = false;
          TU_LOG1("Stalled: Clear endpoint halt\n");
          uint8_t const ep_data = (cbw->dir & TUSB_DIR_IN_MASK) ? p_msc->ep_in : p_msc->ep_out;
          tuh_msc_clear_feature(dev_addr, ep_addr ,HOST_ENDPOINT_HALT, clear_feature_complete);
          while(!p_msc->clear_done) {tuh_task();};
          osal_task_delay(80);
          usbh_reset_endpoint_pid(dev_addr, ep_data);
          TU_LOG1("Stalled: Retry last transfer\n");
          switch (p_msc->stage) {
            case MSC_STAGE_DATA:
            {
              TU_LOG1("Restart the data stage\n");
              TU_ASSERT(usbh_edpt_xfer(dev_addr, ep_data, p_msc->buffer, cbw->total_bytes));
            }
            break;
            case MSC_STAGE_STATUS:
            TU_LOG1("Restart the status stage\n");
            TU_ASSERT(usbh_edpt_xfer(dev_addr, p_msc->ep_in, (uint8_t*) &p_msc->csw, sizeof(msc_csw_t)));
            break;
            default:
            TU_LOG1("Stalled: unknow last transfert\n");
          }
        } else {
          csw->status = MSC_CSW_STATUS_CHECK_CONDITION;
          usbh_can_accept_cmd();
          if (p_msc->complete_cb) p_msc->complete_cb(dev_addr, cbw, csw);
          finished = true;
        }
      } else {
        TU_LOG1("Stalled: still blocked - abort\n");
        csw->status = MSC_CSW_STATUS_CHECK_CONDITION;
        usbh_can_accept_cmd();
        if (p_msc->complete_cb) p_msc->complete_cb(dev_addr, cbw, csw);
        finished = true;
      }
    return finished;
  }
  switch (p_msc->stage)
  {
    case MSC_STAGE_CMD:
      // Must be Command Block
      TU_LOG3("%d=%d %d=%d %d=%d\n",ep_addr,p_msc->ep_out,event,XFER_RESULT_SUCCESS,xferred_bytes,sizeof(msc_cbw_t));
      TU_ASSERT(ep_addr == p_msc->ep_out &&  event == XFER_RESULT_SUCCESS && xferred_bytes == sizeof(msc_cbw_t));

      if ( cbw->total_bytes && p_msc->buffer )
      {
        // Data stage if any
        p_msc->stage = MSC_STAGE_DATA;

        uint8_t const ep_data = (cbw->dir & TUSB_DIR_IN_MASK) ? p_msc->ep_in : p_msc->ep_out;
        TU_ASSERT(usbh_edpt_xfer(dev_addr, ep_data, p_msc->buffer, cbw->total_bytes));
      }else
      {
        // Status stage
        p_msc->stage = MSC_STAGE_STATUS;
        TU_ASSERT(usbh_edpt_xfer(dev_addr, p_msc->ep_in, (uint8_t*) &p_msc->csw, sizeof(msc_csw_t)));
      }
    break;

    case MSC_STAGE_DATA:
      // Status stage
      p_msc->stage = MSC_STAGE_STATUS;
      TU_ASSERT(usbh_edpt_xfer(dev_addr, p_msc->ep_in, (uint8_t*) &p_msc->csw, sizeof(msc_csw_t)));
    break;

    case MSC_STAGE_STATUS:
      // SCSI op is complete
      {
        csw->status = csw->status<<1;
        TU_LOG2("SCSI status %x\n", csw->status);
        p_msc->stage = MSC_STAGE_IDLE;
        usbh_can_accept_cmd();
        if (p_msc->complete_cb) p_msc->complete_cb(dev_addr, cbw, csw);
        finished = true;
      }
    break;

    // unknown state
    default: break;
  }

  return finished;
}

//--------------------------------------------------------------------+
// MSC Enumeration
//--------------------------------------------------------------------+

static bool config_get_maxlun_complete (uint8_t dev_addr, tusb_control_request_t const * request, xfer_result_t result);
static bool config_test_unit_ready_complete(uint8_t dev_addr, msc_cbw_t const* cbw, msc_csw_t const* csw);
static bool config_request_sense_complete(uint8_t dev_addr, msc_cbw_t const* cbw, msc_csw_t const* csw);
static bool config_read_capacity_complete(uint8_t dev_addr, msc_cbw_t const* cbw, msc_csw_t const* csw);

bool msch_open(uint8_t rhport, uint8_t dev_addr, tusb_desc_interface_t const *desc_itf, uint16_t max_len)
{
  TU_VERIFY (MSC_SUBCLASS_SCSI == desc_itf->bInterfaceSubClass &&
             MSC_PROTOCOL_BOT  == desc_itf->bInterfaceProtocol);

  // msc driver length is fixed
  uint16_t const drv_len = sizeof(tusb_desc_interface_t) + desc_itf->bNumEndpoints*sizeof(tusb_desc_endpoint_t);
  TU_ASSERT(drv_len <= max_len);

  msch_interface_t* p_msc = get_itf(dev_addr);
  tusb_desc_endpoint_t const * ep_desc = (tusb_desc_endpoint_t const *) tu_desc_next(desc_itf);

  for(uint32_t i=0; i<2; i++)
  {
    TU_ASSERT(TUSB_DESC_ENDPOINT == ep_desc->bDescriptorType && TUSB_XFER_BULK == ep_desc->bmAttributes.xfer);
    TU_ASSERT(usbh_edpt_open(rhport, dev_addr, ep_desc));

    if ( tu_edpt_dir(ep_desc->bEndpointAddress) == TUSB_DIR_IN )
    {
      p_msc->ep_in = ep_desc->bEndpointAddress;
    }else
    {
      p_msc->ep_out = ep_desc->bEndpointAddress;
    }

    ep_desc = (tusb_desc_endpoint_t const *) tu_desc_next(ep_desc);
  }

  p_msc->itf_num = desc_itf->bInterfaceNumber;

  return true;
}

bool msch_set_config(uint8_t dev_addr, uint8_t itf_num)
{
  msch_interface_t* p_msc = get_itf(dev_addr);
  TU_ASSERT(p_msc->itf_num == itf_num);

  p_msc->configured = true;

  //------------- Get Max Lun -------------//
  TU_LOG1("MSC Get Max Lun %d\r\n", itf_num);
  tusb_control_request_t request =
  {
    .bmRequestType_bit =
    {
      .recipient = TUSB_REQ_RCPT_INTERFACE,
      .type      = TUSB_REQ_TYPE_CLASS,
      .direction = TUSB_DIR_IN
    },
    .bRequest = MSC_REQ_GET_MAX_LUN,
    .wValue   = 0,
    .wIndex   = itf_num,
    .wLength  = 1
  };
  TU_ASSERT(tuh_control_xfer(dev_addr, &request, &p_msc->max_lun, config_get_maxlun_complete));

  return true;
}

static bool config_get_maxlun_complete (uint8_t dev_addr, tusb_control_request_t const * request, xfer_result_t result)
{
  (void) request;

  msch_interface_t* p_msc = get_itf(dev_addr);

  // STALL means zero
  p_msc->max_lun = (XFER_RESULT_SUCCESS == result) ? _msch_buffer[0] : 0;
  p_msc->max_lun++; // MAX LUN is minus 1 by specs

  if (tuh_msc_enumerate_cb) tuh_msc_enumerate_cb(dev_addr);
  // notify usbh that driver enumeration is complete
  usbh_driver_set_config_complete(dev_addr, p_msc->itf_num);

  // // TODO multiple LUN support
  // if (tuh_msc_enumerated_cb) tuh_msc_enumerated_cb(dev_addr);
  // else {
  //   uint8_t const lun = 0;
  //   checkForMedia(dev_addr, lun);
  // }

  return true;
}

bool checkForMedia(uint8_t dev_addr, uint8_t lun) {
  return tuh_msc_test_unit_ready(dev_addr, lun, config_test_unit_ready_complete);
}

static bool config_test_unit_ready_complete(uint8_t dev_addr, msc_cbw_t const* cbw, msc_csw_t const* csw)
{
    TU_LOG2("Test Unit ready Complete\n");
  if (tuh_msc_ready_cb) tuh_msc_ready_cb(dev_addr, csw->status == MSC_CSW_STATUS_GOOD);
  if (csw->status == MSC_CSW_STATUS_GOOD)
  {
    // Unit is ready, read its capacity
    tuh_msc_read_capacity(dev_addr, cbw->lun, (scsi_read_capacity10_resp_t*) ((void*) _msch_buffer), config_read_capacity_complete);
  }
  return true;
}

static bool config_request_sense_complete(uint8_t dev_addr, msc_cbw_t const* cbw, msc_csw_t const* csw)
{
  TU_LOG1("Sense complete %x\n", csw->status);

  TU_ASSERT(csw->status == MSC_CSW_STATUS_GOOD);
  if (!tuh_msc_enumerated_cb) TU_ASSERT(tuh_msc_test_unit_ready(dev_addr, cbw->lun, config_test_unit_ready_complete));
  else tuh_msc_enumerated_cb(dev_addr);
  return true;
}

static bool config_read_capacity_complete(uint8_t dev_addr, msc_cbw_t const* cbw, msc_csw_t const* csw)
{
  TU_LOG1("Read Capacity complete %x\r\n", csw->status);
  if (csw->status != MSC_CSW_STATUS_GOOD) {
    // device is not ready yet..
    return true;
  }

  msch_interface_t* p_msc = get_itf(dev_addr);

  // Capacity response field: Block size and Last LBA are both Big-Endian
  scsi_read_capacity10_resp_t* resp = (scsi_read_capacity10_resp_t*) ((void*) _msch_buffer);
  p_msc->capacity[cbw->lun].block_count = tu_ntohl(resp->last_lba) + 1;
  p_msc->capacity[cbw->lun].block_size = tu_ntohl(resp->block_size);

  // Mark enumeration is complete
  p_msc->mounted = true;
  if (tuh_msc_mount_cb) tuh_msc_mount_cb(dev_addr);

  return true;
}

#endif
