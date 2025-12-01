/*
 * stream.cpp
 *
 *  Created on: Jul 28, 2025
 *      Author: naoki
 */

#include <string.h>

#include <algorithm>
#include <string>

#include "analog3/stream.h"
#include "analog3/property.h"

namespace analog3 {

Stream::Stream() {
  prop_position_ = 0;
  data_position_ = 0;
  wire_addr_ = A3_ID_INVALID;
  num_remaining_properties_ = 0;
  data_size_ = 0;
}

void Stream::InitiateAdminWrites(uint32_t wire_addr, int prop_start_index, int num_props)
{
    wire_addr_ = wire_addr;
    prop_position_ = prop_start_index;
    num_remaining_properties_ = num_props;
}

int32_t Stream::FillPropertyData(const std::vector<Property>& props, uint8_t *data, int32_t payload_index) {
    if (prop_position_ >= props.size()) {
        return CAN_STD_DATA_LENGTH;
    }
    const Property& current_prop = props.at(prop_position_);
    if (data_position_ < 1) {
        data[payload_index++] = current_prop.id;
        ++data_position_;
        if (payload_index == CAN_STD_DATA_LENGTH) {
            return payload_index;
        }
    }
    switch (current_prop.value_type) {
    case A3_U8:
      return FillInt<uint8_t>(current_prop, data, payload_index);
    case A3_U16:
      return FillInt<uint16_t>(current_prop, data, payload_index);
    case A3_U32:
      return FillInt<uint32_t>(current_prop, data, payload_index);
    case A3_STRING:
      return FillString(current_prop, data, payload_index);
    case A3_VECTOR_U16P:
      return FillVectorP<uint16_t>(current_prop, data, payload_index);
        /*
    case A3_VECTOR_U8:
        return FillVectorU8(current_prop, data, payload_index);
        */
    }
    return CAN_STD_DATA_LENGTH;
}

int32_t Stream::FillString(const Property &prop, uint8_t *data, uint32_t payload_index) {
  auto text = static_cast<const std::string *>(prop.data);
    uint8_t length = static_cast<uint8_t>(text->size());
    if (data_position_ < 2) {
        data[payload_index++] = length;
        ++data_position_;
        if (payload_index == CAN_STD_DATA_LENGTH) {
            return payload_index;
        }
    }
    int data_index = data_position_ - 2;
    size_t data_size = std::min(static_cast<size_t>(CAN_STD_DATA_LENGTH - payload_index), static_cast<size_t>(length - data_index));
    memcpy(&data[payload_index], text->c_str() + data_index, data_size);
    data_position_ += data_size;
    payload_index += data_size;
    CheckForTransferTermination(length + 2);
    return payload_index;
}

void Stream::CheckForTransferTermination(uint32_t property_data_length)
{
    if (data_position_ == property_data_length) {
        // completed transferring the property. proceed to the next.
        data_position_ = 0;
        ++prop_position_;
        --num_remaining_properties_;
        if (num_remaining_properties_ == 0) {
            // entire transfer completed. clear the wire ID
            wire_addr_ = A3_ID_INVALID;
        }
    }
}

}  // namespace analog3
