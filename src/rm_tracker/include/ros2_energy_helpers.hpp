#pragma once

#include <cstdint>

inline char rm_msgs_int8_to_char(const std::int8_t value) {
  return static_cast<char>(static_cast<unsigned char>(value));
}

inline bool is_energy_mode(const std::int8_t shoot_flag) {
  return rm_msgs_int8_to_char(shoot_flag) == 'f';
}

inline bool is_armor_mode(const std::int8_t shoot_flag) {
  return rm_msgs_int8_to_char(shoot_flag) == 'a';
}

inline char camp_to_internal(const std::int8_t camp) {
  const char value = rm_msgs_int8_to_char(camp);
  if (value == 'b') {
    return 'r';
  }
  if (value == 'r') {
    return 'b';
  }
  return 'b';
}
