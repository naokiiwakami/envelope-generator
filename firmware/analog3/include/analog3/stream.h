/*
 * stream.h
 *
 *  Created on: Jul 28, 2025
 *      Author: naoki
 */

#ifndef INCLUDE_ANALOG3_STREAM_H_
#define INCLUDE_ANALOG3_STREAM_H_

#include <stdint.h>

#include <vector>

#include "analog3/definitions.h"
#include "analog3/property.h"
#include "analog3/stream.h"

namespace analog3 {

/**
 * Keeps track of a data stream over CAN bus.
 */
class Stream {
 private:
  /**
   * Current position in the property vector.
   */
  uint32_t prop_position_;

  /**
   * Current position in the data array of the current property.
   */
  uint32_t data_position_;

  /**
   * CAN ID that shares with the peer to exchange the data.
   */
  uint32_t wire_addr_;

  /**
   * Number of remaining properties in the stream.
   */
  uint8_t num_remaining_properties_;

  /**
   * Streamed data size
   */
  uint8_t data_size_;

 public:
  Stream();
  ~Stream() = default;

  inline uint32_t GetWireAddress() const {
    return wire_addr_;
  }

  /**
   * Initializes an admin wire for writes.
   *
   * @param wire_addr - wire ID to start
   * @param prop_start_index - starting index of the properties
   * @param num_props - number of properties to send
   */
  void InitiateAdminWrites(uint32_t wire_addr, int prop_start_index, int num_props);

  /**
   * Fill property values to the tx data payload.
   *
   * @param props - Source properties.
   * @param data - Payload to fill the data. The size is 8 byte (until we support CAN FD).
   * @param payload_index - Index that points the starting point of the payload.
   *
   * @returns The payload index for the next iteration.
   */
  int32_t FillPropertyData(const std::vector<Property> &props, uint8_t *data,
                           int32_t payload_index);

  inline bool IsDone() const {
    return num_remaining_properties_ == 0;
  }

 private:
  template <typename T>
  int32_t FillInt(const Property &prop, uint8_t *data, uint32_t payload_index) {
    size_t num_bytes = sizeof(T);
    if (data_position_ < 2) {
         data[payload_index++] = num_bytes;
         ++data_position_;
         if (payload_index == CAN_STD_DATA_LENGTH) {
             return payload_index;
         }
     }
     T value = *static_cast<const T *>(prop.data);
     uint8_t total_bytes = num_bytes + 2;
     uint8_t bytes_to_send = std::min(CAN_STD_DATA_LENGTH - payload_index, total_bytes - data_position_);
     value >>= (total_bytes - bytes_to_send - data_position_) * 8;
     for (uint8_t i = 0; i < bytes_to_send; ++i) {
         data[payload_index + bytes_to_send - i - 1] = value & 0xff;
         value >>= 8;
     }
     data_position_ += bytes_to_send;
     payload_index += bytes_to_send;
     CheckForTransferTermination(total_bytes);
     return payload_index;
  }

  int32_t FillString(const Property &prop, uint8_t *data, uint32_t payload_index);

  void CheckForTransferTermination(uint32_t property_data_length);
};

}  // namespace analog3

#endif /* INCLUDE_ANALOG3_STREAM_H_ */
