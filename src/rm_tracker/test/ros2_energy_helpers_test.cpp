#include <cstdint>
#include <cassert>

#include "ros2_energy_helpers.hpp"

int main() {
  assert(rm_msgs_int8_to_char(static_cast<std::int8_t>('f')) == 'f');
  assert(rm_msgs_int8_to_char(static_cast<std::int8_t>(-1)) == '\xff');

  assert(is_energy_mode(static_cast<std::int8_t>('f')));
  assert(!is_energy_mode(static_cast<std::int8_t>('a')));
  assert(is_armor_mode(static_cast<std::int8_t>('a')));
  assert(!is_armor_mode(static_cast<std::int8_t>('f')));
  assert(!is_energy_mode(static_cast<std::int8_t>('x')));
  assert(!is_armor_mode(static_cast<std::int8_t>('x')));

  assert(camp_to_internal(static_cast<std::int8_t>('b')) == 'r');
  assert(camp_to_internal(static_cast<std::int8_t>('r')) == 'b');
  assert(camp_to_internal(static_cast<std::int8_t>('x')) == 'b');
  return 0;
}
