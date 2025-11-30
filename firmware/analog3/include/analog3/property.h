/*
 * property.h
 *
 *  Created on: Jul 28, 2025
 *      Author: naoki
 */

#ifndef INCLUDE_ANALOG3_PROPERTY_H_
#define INCLUDE_ANALOG3_PROPERTY_H_

enum A3PropertyValueType {
  A3_U8,
  A3_U16,
  A3_U32,
  // A3_I8,
  // A3_I16,
  // A3_I32,
  A3_STRING,
  A3_VECTOR_U8
};

// Properties that are common among modules
#define A3_PROP_MODULE_UID 0
#define A3_TYPE_MODULE_UID A3_U32
#define A3_PROP_MODULE_TYPE 1
#define A3_TYPE_MODULE_TYPE A3_U16
#define A3_PROP_MODULE_NAME 2
#define A3_TYPE_MODULE_NAME A3_STRING

namespace analog3 {

typedef struct A3Vector {
  uint8_t size;
  void *data;
} a3_vector_t;

/**
 * Least set of parameters necessary for sharing config with the Mission Control.
 */
struct Property {
  uint8_t id;  // attribute ID
  uint8_t value_type;  // value type
  bool write_protected;  // indicates if the value is write-protected
  void *data;
  // function to incorporate data
  void (*incorporate)(Property*, const void *data, uint8_t len);
  // address to persist configuration data, 0xffff for a constant value
  uint32_t save_addr;

  Property(uint8_t id, uint8_t value_type, const void *data)
      :
      id(id),
      value_type(value_type),
      write_protected(true),
      data(const_cast<void*>(data)),
      incorporate(nullptr),
      save_addr(0xffff) {
  }

  Property(uint8_t id, uint8_t value_type, void *data, void (*incorporate)(Property*, const void*, uint8_t),
           uint16_t save_addr)
      :
      id(id),
      value_type(value_type),
      write_protected(false),
      data(data),
      incorporate(incorporate),
      save_addr(save_addr) {
  }
};

}  // analog3

#endif /* INCLUDE_ANALOG3_PROPERTY_H_ */
