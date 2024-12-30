# Copyright 2024 Ekumen, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.


PULSE_PROCESSOR_N_SENSORS = 4

DECK_LIGHTHOUSE_MAX_N_BS = 4

TIMESTAMP_COUNTER_MASK = (1 << 24) - 1

MIN_TICKS_BETWEEN_SLOW_BITS = (887000 // 2) * 8 // 10

OOTX_MAX_PAYLOAD_LENGTH = 43


# The cycle times from the Lighhouse base stations is expressed
# in a 48 MHz clock, we use 24 MHz, hence the / 2.
PERIODS = {
    1: 959000 / 2,
    2: 957000 / 2,
    3: 953000 / 2,
    4: 949000 / 2,
    5: 947000 / 2,
    6: 943000 / 2,
    7: 941000 / 2,
    8: 939000 / 2,
    9: 937000 / 2,
    10: 929000 / 2,
    11: 919000 / 2,
    12: 911000 / 2,
    13: 907000 / 2,
    14: 901000 / 2,
    15: 893000 / 2,
    16: 887000 / 2,
}


def timestamp_diff(a, b):
    """Calculate the difference between two timestamps, taking into account."""
    return (TIMESTAMP_COUNTER_MASK + 1 + a - b) & TIMESTAMP_COUNTER_MASK


def timestamp_sum(a, b):
    """Calculate the difference between two timestamps, taking into account."""
    return (a + b) & TIMESTAMP_COUNTER_MASK


def timestamp_abs_diff_larger_than(a, b, limit):
    """Check if the absolute difference between two timestamps is larger than a limit."""
    return timestamp_diff(a + limit, b) > (limit * 2)


def npoly_is_valid(npoly):
    """Check if the npoly value is valid."""
    return (npoly & 0x20) == 0


def npoly_channel(npoly):
    """Get import dataclasses the base_station_id from the npoly value."""
    return (npoly // 2) + 1


def npoly_slow_bit(npoly):
    """Get the slow bit from the npoly value."""
    return npoly & 1
