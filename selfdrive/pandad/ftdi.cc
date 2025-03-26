
#include "selfdrive/pandad/panda.h"  // for  _USE_FLEXRAY_HARNESS_

#ifdef _USE_FLEXRAY_HARNESS_

#include "selfdrive/pandad/libftdi.h"
#include "selfdrive/pandad/libftdi.c"
#include <sys/file.h>
#include <sys/ioctl.h>

#include <cassert>
#include <cmath>
#include <cstring>
#include <iomanip>
#include <sstream>

#include "common/util.h"
#include "common/timing.h"
#include "common/swaglog.h"
#include "panda/board/comms_definitions.h"
#include "selfdrive/pandad/panda_comms.h"

#define FTDI_DEVICE_ID  0x0403
#define FTDI_PRODUCT_ID 0x6010

PandaFtdiHandle::PandaFtdiHandle(std::string serial) : PandaCommsHandle(serial) {
  char serial_no[128];

  if((ftdi_ctx = ftdi_new()) == 0) {
    LOGW("FTDI : fail ftdi_new");
    return;
  }

  if(ftdi_set_interface(ftdi_ctx, INTERFACE_B) < 0) {
    LOGW("FTDI : fail ftdi_set_interface");
    goto fail;
  }


  if (ftdi_usb_open_desc(ftdi_ctx, FTDI_DEVICE_ID, FTDI_PRODUCT_ID, NULL, serial.c_str()) < 0) {
    LOGW("FTDI : Can't open ftdi\n");
    goto fail;
  }

  if (ftdi_set_bitmode(ftdi_ctx,  0xff, BITMODE_RESET) < 0)
  {
    LOGW("Can't set synchronous fifo mode, Error %s",
          ftdi_get_error_string(ftdi_ctx));
    goto fail;
  }

  if (ftdi_set_latency_timer(ftdi_ctx, 2)) {
    LOGW("Can't set latency : (%s)", ftdi_get_error_string(ftdi_ctx));
    goto fail;
  }

  sprintf(serial_no, "%04x%04x", FTDI_DEVICE_ID, FTDI_PRODUCT_ID);

  hw_serial =  serial_no;

  #if 0
  if (!serial.empty() && (serial != hw_serial)) {
    LOGW("FTDI : not match serial");
    goto fail;
  }
  #endif

  return;


fail:
  LOGW("FTDI : panda() fail");
  cleanup();
  throw std::runtime_error("Error connecting to flexray panda");
}


PandaFtdiHandle::~PandaFtdiHandle() {
  std::lock_guard lk(hw_lock);
  cleanup();
  connected = false;
}

void PandaFtdiHandle::cleanup() {
  if (ftdi_ctx) {
    ftdi_usb_close(ftdi_ctx);
    ftdi_free(ftdi_ctx);
    ftdi_ctx = NULL;
  }
}



int PandaFtdiHandle::control_write(uint8_t request, uint16_t param1, uint16_t param2, unsigned int timeout) {

  return 0;
}

int PandaFtdiHandle::control_read(uint8_t request, uint16_t param1, uint16_t param2, unsigned char *data, uint16_t length, unsigned int timeout) {

  switch(request) {
    case 0xc1 :
      *data = ((int)cereal::PandaState::PandaType::FLEXRAY_PANDA);
      break;

    // state health
    case 0xd2 :
      break;

    // can health
    case 0xc2 :
      break;

    // firmware
    case 0xd3 :
      break;
    case 0xd4:
      break;

    default:
      break;
  }

  return 0;
}

int PandaFtdiHandle::bulk_write(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout) {

  return 0;
}

int PandaFtdiHandle::bulk_read(unsigned char endpoint, unsigned char* data, int length, unsigned int timeout) {
  int recv;

  if(!ftdi_ctx) return -1;

  std::lock_guard lk(hw_lock);

  ftdi_ctx->usb_read_timeout = timeout;
  recv = ftdi_read_data(ftdi_ctx, data, length);

  if(recv < 0)
  {
    LOGW("FTDI : fail read_data %d", recv);
    comms_healthy = false;
  }

  return recv;
}


std::vector<std::string> PandaFtdiHandle::list() {
  std::vector<std::string> serials;

  int ret, i;
  struct ftdi_context *ftdi;
  struct ftdi_device_list *devlist, *curdev;
  char sn[128];

  if ((ftdi = ftdi_new()) == 0)
  {
      goto finish;
  }

  if ((ret = ftdi_usb_find_all(ftdi, &devlist, FTDI_DEVICE_ID, FTDI_PRODUCT_ID)) < 0)
  {
        goto finish;
  }

  i = 0;
  for (curdev = devlist; curdev != NULL; i++)
  {
    if ((ret=ftdi_usb_get_strings(ftdi, curdev->dev, NULL, 0, NULL, 0, sn, 128)) < 0)
    {
        goto done;
    }

    //LOGW("FTDI serial number  %s", sn);
    serials.push_back(std::string((char *)sn, strlen(sn)).c_str());
    curdev = curdev->next;
  }


done:
  ftdi_list_free(&devlist);

finish:

  if(ftdi) ftdi_free(ftdi);

  return serials;
}


#endif
