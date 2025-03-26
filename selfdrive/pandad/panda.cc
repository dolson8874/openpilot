#include "selfdrive/pandad/panda.h"

#include <unistd.h>

#include <cassert>
#include <stdexcept>
#include <vector>

#include "cereal/messaging/messaging.h"
#include "common/swaglog.h"
#include "common/util.h"

const bool PANDAD_MAXOUT = getenv("PANDAD_MAXOUT") != nullptr;

Panda::Panda(std::string serial, uint32_t bus_offset) : bus_offset(bus_offset) {
#ifdef _USE_FLEXRAY_HARNESS_
  // try USB first, then SPI, then FTDI
  try {
      handle = std::make_unique<PandaUsbHandle>(serial);
  } catch (std::exception &e) {

      #ifndef __APPLE__
      if(serial.compare(0, 3, "ARA") == 0)
        handle = std::make_unique<PandaFtdiHandle>(serial);
      else
        handle = std::make_unique<PandaSpiHandle>(serial);
      #endif
  }
#else //_USE_FLEXRAY_HARNESS_
  // try USB first, then SPI
  try {
    handle = std::make_unique<PandaUsbHandle>(serial);
    LOGW("connected to %s over USB", serial.c_str());
  } catch (std::exception &e) {
#ifndef __APPLE__
    handle = std::make_unique<PandaSpiHandle>(serial);
    LOGW("connected to %s over SPI", serial.c_str());
#else
    throw e;
#endif
  }
#endif //_USE_FLEXRAY_HARNESS

  hw_type = get_hw_type();
  can_reset_communications();
}

bool Panda::connected() {
  return handle->connected;
}

bool Panda::comms_healthy() {
  return handle->comms_healthy;
}

std::string Panda::hw_serial() {
  return handle->hw_serial;
}

std::vector<std::string> Panda::list(bool usb_only) {
  std::vector<std::string> serials = PandaUsbHandle::list();

#ifdef _USE_FLEXRAY_HARNESS_
  if (!usb_only) {
    for (auto s : PandaFtdiHandle::list()) {
      if (std::find(serials.begin(), serials.end(), s) == serials.end()) {
        serials.push_back(s);
      }
    }
  }
#else // _USE_FLEXRAY_HARNESS_

#ifndef __APPLE__
  if (!usb_only) {
    for (const auto &s : PandaSpiHandle::list()) {
      if (std::find(serials.begin(), serials.end(), s) == serials.end()) {
        serials.push_back(s);
      }
    }
  }
#endif

#endif // _USE_FLEXRAY_HARNESS_

  return serials;
}

void Panda::set_safety_model(cereal::CarParams::SafetyModel safety_model, uint16_t safety_param) {
  handle->control_write(0xdc, (uint16_t)safety_model, safety_param);
}

void Panda::set_alternative_experience(uint16_t alternative_experience) {
  handle->control_write(0xdf, alternative_experience, 0);
}

std::string Panda::serial_read(int port_number) {
  std::string ret;
  char buffer[USBPACKET_MAX_SIZE] = {};

  while (true) {
    int bytes_read = handle->control_read(0xe0, port_number, 0, (unsigned char *)buffer, USBPACKET_MAX_SIZE);
    if (bytes_read <= 0) {
      break;
    }
    ret.append(buffer, bytes_read);
  }

  return ret;
}

void Panda::set_uart_baud(int uart, int rate) {
  handle->control_write(0xe4, uart, int(rate / 300));
}

cereal::PandaState::PandaType Panda::get_hw_type() {
  unsigned char hw_query[1] = {0};

  handle->control_read(0xc1, 0, 0, hw_query, 1);
  return (cereal::PandaState::PandaType)(hw_query[0]);
}

void Panda::set_fan_speed(uint16_t fan_speed) {
  handle->control_write(0xb1, fan_speed, 0);
}

uint16_t Panda::get_fan_speed() {
  uint16_t fan_speed_rpm = 0;
  handle->control_read(0xb2, 0, 0, (unsigned char*)&fan_speed_rpm, sizeof(fan_speed_rpm));
  return fan_speed_rpm;
}

void Panda::set_ir_pwr(uint16_t ir_pwr) {
  handle->control_write(0xb0, ir_pwr, 0);
}

std::optional<health_t> Panda::get_state() {
  health_t health {0};
  int err = handle->control_read(0xd2, 0, 0, (unsigned char*)&health, sizeof(health));
  return err >= 0 ? std::make_optional(health) : std::nullopt;
}

std::optional<can_health_t> Panda::get_can_state(uint16_t can_number) {
  can_health_t can_health {0};
  int err = handle->control_read(0xc2, can_number, 0, (unsigned char*)&can_health, sizeof(can_health));
  return err >= 0 ? std::make_optional(can_health) : std::nullopt;
}

void Panda::set_loopback(bool loopback) {
  handle->control_write(0xe5, loopback, 0);
}

std::optional<std::vector<uint8_t>> Panda::get_firmware_version() {
  std::vector<uint8_t> fw_sig_buf(128);
  int read_1 = handle->control_read(0xd3, 0, 0, &fw_sig_buf[0], 64);
  int read_2 = handle->control_read(0xd4, 0, 0, &fw_sig_buf[64], 64);
  return ((read_1 == 64) && (read_2 == 64)) ? std::make_optional(fw_sig_buf) : std::nullopt;
}

std::optional<std::string> Panda::get_serial() {
  char serial_buf[17] = {'\0'};
  int err = handle->control_read(0xd0, 0, 0, (uint8_t*)serial_buf, 16);
  return err >= 0 ? std::make_optional(serial_buf) : std::nullopt;
}

bool Panda::up_to_date() {

#if defined(_USE_FLEXRAY_HARNESS_)
  // skip ftdi panda for flexray log
  if(hw_type == cereal::PandaState::PandaType::FLEXRAY_PANDA) {
    return true;
  }
#endif

  if (auto fw_sig = get_firmware_version()) {
    for (auto fn : { "panda.bin.signed", "panda_h7.bin.signed" }) {
      auto content = util::read_file(std::string("../../panda/board/obj/") + fn);
      if (content.size() >= fw_sig->size() &&
          memcmp(content.data() + content.size() - fw_sig->size(), fw_sig->data(), fw_sig->size()) == 0) {
        return true;
      }
    }
  }
  return false;
}

void Panda::set_power_saving(bool power_saving) {
  handle->control_write(0xe7, power_saving, 0);
}

void Panda::enable_deepsleep() {
  handle->control_write(0xfb, 0, 0);
}

void Panda::send_heartbeat(bool engaged) {
  handle->control_write(0xf3, engaged, 0);
}

void Panda::set_can_speed_kbps(uint16_t bus, uint16_t speed) {
  handle->control_write(0xde, bus, (speed * 10));
}

void Panda::set_can_fd_auto(uint16_t bus, bool enabled) {
  handle->control_write(0xe8, bus, enabled);
}

void Panda::set_data_speed_kbps(uint16_t bus, uint16_t speed) {
  handle->control_write(0xf9, bus, (speed * 10));
}

void Panda::set_canfd_non_iso(uint16_t bus, bool non_iso) {
  handle->control_write(0xfc, bus, non_iso);
}

static uint8_t len_to_dlc(uint8_t len) {
  if (len <= 8) {
    return len;
  }
  if (len <= 24) {
    return 8 + ((len - 8) / 4) + ((len % 4) ? 1 : 0);
  } else {
    return 11 + (len / 16) + ((len % 16) ? 1 : 0);
  }
}

void Panda::pack_can_buffer(const capnp::List<cereal::CanData>::Reader &can_data_list,
                            std::function<void(uint8_t *, size_t)> write_func) {
  int32_t pos = 0;
  uint8_t send_buf[2 * USB_TX_SOFT_LIMIT];

  for (const auto &cmsg : can_data_list) {
    // check if the message is intended for this panda
    uint8_t bus = cmsg.getSrc();
    if (bus < bus_offset || bus >= (bus_offset + PANDA_BUS_OFFSET)) {
      continue;
    }
    auto can_data = cmsg.getDat();
    uint8_t data_len_code = len_to_dlc(can_data.size());
    assert(can_data.size() <= 64);
    assert(can_data.size() == dlc_to_len[data_len_code]);

    can_header header = {};
    header.addr = cmsg.getAddress();
    header.extended = (cmsg.getAddress() >= 0x800) ? 1 : 0;
    header.data_len_code = data_len_code;
    header.bus = bus - bus_offset;
    header.checksum = 0;

    memcpy(&send_buf[pos], (uint8_t *)&header, sizeof(can_header));
    memcpy(&send_buf[pos + sizeof(can_header)], (uint8_t *)can_data.begin(), can_data.size());
    uint32_t msg_size = sizeof(can_header) + can_data.size();

    // set checksum
    ((can_header *) &send_buf[pos])->checksum = calculate_checksum(&send_buf[pos], msg_size);

    pos += msg_size;

    if (pos >= USB_TX_SOFT_LIMIT) {
      write_func(send_buf, pos);
      pos = 0;
    }
  }

  // send remaining packets
  if (pos > 0) write_func(send_buf, pos);
}

void Panda::can_send(const capnp::List<cereal::CanData>::Reader &can_data_list) {
  pack_can_buffer(can_data_list, [=](uint8_t* data, size_t size) {
    handle->bulk_write(3, data, size, 5);
  });
}

bool Panda::can_receive(std::vector<can_frame>& out_vec) {
  // Check if enough space left in buffer to store RECV_SIZE data
  assert(receive_buffer_size + RECV_SIZE <= sizeof(receive_buffer));

  int recv = handle->bulk_read(0x81, &receive_buffer[receive_buffer_size], RECV_SIZE);
  if (!comms_healthy()) {
    return false;
  }

  if (PANDAD_MAXOUT) {
    static uint8_t junk[RECV_SIZE];
    handle->bulk_read(0xab, junk, RECV_SIZE - recv);
  }

  bool ret = true;
  if (recv > 0) {
    receive_buffer_size += recv;
    ret = unpack_can_buffer(receive_buffer, receive_buffer_size, out_vec);
  }
  return ret;
}

void Panda::can_reset_communications() {
  handle->control_write(0xc0, 0, 0);
}

bool Panda::unpack_can_buffer(uint8_t *data, uint32_t &size, std::vector<can_frame> &out_vec) {
  int pos = 0;


#if defined(_USE_FLEXRAY_HARNESS_)
  while (pos <= size - sizeof(can_header)) {
    can_header header;

    uint16_t data_len;
    struct flexray_header *fheader;
    uint16_t frame_id;

    if(hw_type == cereal::PandaState::PandaType::FLEXRAY_PANDA) {

      if(data[pos] != 0xCA || data[pos+1] != 0xA0) {
        //LOGW(" %d(%02x)", pos, data[pos]);
        pos++;
        continue;
      }

      pos++;

      memcpy(&header, &data[pos], sizeof(can_header));

      /*
      LOGW("header : %02x %02x %02x %02x %02x %02x",
                      data[pos], data[pos+1], data[pos+2],
                      data[pos+3], data[pos+4], data[pos+5]);
      */

      fheader =  (struct flexray_header *)(&header);

      // flags(1) + counter (1) + data (len) + CRC (3)
      data_len =  fheader->length * 2 + 5;

      frame_id = fheader->frame_id | ((fheader->flagsid & 0x7) << 8);

      //LOGW("flex_len %d (%02x)", data_len, fheader->length);
    } else {
      memcpy(&header, &data[pos], sizeof(can_header));
      data_len = dlc_to_len[header.data_len_code];
    }


    if (pos + sizeof(can_header) + data_len > size) {
      // we don't have all the data for this message yet
      break;
    }

    can_frame &canData = out_vec.emplace_back();

    #if 0
    if(hw_type == cereal::PandaState::PandaType::FLEXRAY_PANDA)
    LOGW("flexray busoffset=%d len=%d res=%x bus=%x rej=%x ret=%x ext=%x fl=%x id=%x len=%0x crc=%x cnt=%x",
        bus_offset,
        data_len, fheader->reserved, fheader->bus,
        fheader->rejected, fheader->returned, fheader->extended,
        (fheader->flagsid & 0xf8)>>3, frame_id,
        fheader->length,
        ((fheader->crc_msb << 10)  | (fheader->crc << 2)| fheader->crc_lsb) , fheader->counter);
    #endif


    if(hw_type == cereal::PandaState::PandaType::FLEXRAY_PANDA
        &&  fheader->reserved == 1 ) {

        //LOGW("frame id = %x, counter=%d\n", frame_id, fheader->counter);

        //if(frame_id == 0x1f || frame_id == 0x27)
        {

          unsigned char sync_id;

          switch(frame_id)
          {
            // 200Hz
            case 0x7F:
              sync_id = 0;
              break;

            // 100Hz
            case 0x0f: // 250114 50->100hz
            case 0x12:
            case 0x17:
            case 0x18:
            case 0x1E:
            case 0x20: // 250114 50->100
            case 0x22:
            case 0x23:
            case 0x29: // 250114 50->100
            case 0x2B:
            case 0x2F:
            case 0x3A:
            case 0x3E: // 250114 50->100
              sync_id = fheader->counter % 2;
              break;

            case 0x3B:
              // 250113
              sync_id = fheader->counter % 2;
              if (sync_id == 0) {
                // 50Hz
                sync_id = fheader->counter % 4;
                sync_id = sync_id + 0x10;
              }
              break;

            case 0x32:
              sync_id = fheader->counter % 2;
              if (sync_id == 1)
              {
                sync_id = fheader->counter % 4;
                sync_id = sync_id + 0x10;
              }
              break;
            // 25Hz
            case 0x10: // 250112
            case 0x6: // 250112
              sync_id = fheader->counter % 8;
              if (fheader->counter == 1 || fheader->counter == 33)
                sync_id = 0x11;
              else if (sync_id == 2 && (fheader->counter % 16) == 2)
                sync_id = 0x22;
              else if (sync_id == 5 && fheader->counter == 5)
                sync_id = 0x55;

              break;

            case 0x30:
            case 0x2E:
            case 0x31: // 250111 50->25
              sync_id = fheader->counter % 8;

              // merge 0,4
              if (sync_id == 4) {
                sync_id = 0;

              // split 5, 13
              }else if (sync_id == 5) {
                  sync_id = fheader->counter % 16;
                  sync_id = sync_id + 0x50;
              }
              break;

            // 13Hz
            case 0x5:
            case 0x19:  // 250114 50->13hz
              sync_id = fheader->counter % 16;
              // 250112
              if (sync_id == 0xa)
                sync_id = 2;
              break;
            case 0x14: // 250111
              sync_id = fheader->counter % 16;
              break;
            case 0x24: // 250112
              sync_id = fheader->counter % 16;
              if (sync_id == 0 &&  (fheader->counter % 32))
                sync_id = 0x32;
              break;

            case 0xa : // 250114, 100-> 50hz
            case 0xb : // 250111, 100-> 50hz
            default:
              // 50Hz
              sync_id = fheader->counter % 4;

              if (frame_id == 0xa && sync_id == 2) // 250115 50->100
                sync_id = 0;

              if (frame_id == 0x1A)
              {
                if (sync_id != 0 && sync_id != 3)   // 250115, 250116 add 3
                {
                  sync_id = fheader->counter % 16;
                  sync_id = sync_id + 0x10;
                }
              }
              if (frame_id == 0x1B)
              {
                if (sync_id != 0 && sync_id != 2)
                {
                  sync_id = fheader->counter % 8;
                  // 250112
                  if (sync_id == 3 && (fheader->counter % 16) == 3)
                    sync_id = 0x33;
                }

                if (fheader->counter == 7)
                  sync_id = 0x77;
              }
              break;

          }

          canData.address = frame_id << 8 | (sync_id & 0xff);

        }
        canData.src = 1 + bus_offset;   // fix bus5
     } else {

      canData.address = header.addr;

      canData.src = header.bus + bus_offset;
      if (header.rejected) {
        canData.src += CAN_REJECTED_BUS_OFFSET;
      }
      if (header.returned) {
        canData.src += CAN_RETURNED_BUS_OFFSET;
      }
    }

    if(hw_type == cereal::PandaState::PandaType::FLEXRAY_PANDA) {
      #if 0
      if (calculate_flexray_checksum((uint8_t *) &header , frame_id) != 0) {
        //LOGE("Panda Flexray header checksum failed" );
        //size = 0;
        //return false;
      }
      #endif
    }
    else
    {
      if (calculate_checksum(&data[pos], sizeof(can_header) + data_len) != 0) {
        LOGE("Panda CAN checksum failed");
        size = 0;
        return false;
      }
    }

    canData.dat.assign((char *)&data[pos + sizeof(can_header)], data_len);

    pos += sizeof(can_header) + data_len;
  }

#else  // FLEXRAY_HARNESS


  while (pos <= size - sizeof(can_header)) {
    can_header header;
    memcpy(&header, &data[pos], sizeof(can_header));

    const uint8_t data_len = dlc_to_len[header.data_len_code];
    if (pos + sizeof(can_header) + data_len > size) {
      // we don't have all the data for this message yet
      break;
    }

    if (calculate_checksum(&data[pos], sizeof(can_header) + data_len) != 0) {
      LOGE("Panda CAN checksum failed");
      size = 0;
      can_reset_communications();
      return false;
    }

    can_frame &canData = out_vec.emplace_back();
    canData.address = header.addr;
    canData.src = header.bus + bus_offset;
    if (header.rejected) {
      canData.src += CAN_REJECTED_BUS_OFFSET;
    }
    if (header.returned) {
      canData.src += CAN_RETURNED_BUS_OFFSET;
    }

    canData.dat.assign((char *)&data[pos + sizeof(can_header)], data_len);

    pos += sizeof(can_header) + data_len;
  }

#endif  // FLEXRAY_HARNESS


  // move the overflowing data to the beginning of the buffer for the next round
  memmove(data, &data[pos], size - pos);
  size -= pos;

  return true;
}

uint8_t Panda::calculate_checksum(uint8_t *data, uint32_t len) {
  uint8_t checksum = 0U;
  for (uint32_t i = 0U; i < len; i++) {
    checksum ^= data[i];
  }
  return checksum;
}

#if defined(_USE_FLEXRAY_HARNESS_)
/*
 Generic CRC algorithm for any bit size and any data length. Used for
 11-bit header and 24-bit trailer. Not very efficient but at least it
 works for now.

 TODO:
 - use precalculated tables to increase performance.
 - Add support for reverse CRC calculations.

*/

uint8_t Panda::calculate_flexray_checksum(uint8_t *header, uint16_t fid) {
  const uint16_t polynom = 0x385;
  const uint16_t iv = 0x01A;
  const uint16_t xorval = 0;
  const uint8_t crc_len_bits = 11;
  const uint8_t data_len_bits = 20; // flags(2) + frame_id(11) + length(7)

  struct flexray_header *hdr = (struct flexray_header *) header;


  uint32_t data =   (((hdr->flagsid & 0x7) << 14)
                  | (hdr->frame_id) << 7
                  | hdr->length);

  uint16_t crc_org =  ((hdr->crc_msb << 10)  | (hdr->crc << 2)| hdr->crc_lsb);

  uint16_t reg = iv ^ xorval;

  for (int i = data_len_bits - 1; i >= 0; i--) {
      uint16_t bit = ((reg >> (crc_len_bits - 1)) & 0x1) ^ ((data >> i) & 0x1);
      reg <<= 1;
      if (bit) {
        reg ^= polynom;
      }
   }

   uint16_t mask = (1 << crc_len_bits) - 1;
   uint16_t crc = reg & mask;

   //LOGW("flexray header crc %x / %x counter=%d", (crc ^ xorval), crc_org, hdr->counter);

   return (crc ^ xorval) != crc_org;
}
#endif

