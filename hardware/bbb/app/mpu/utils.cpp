#include "utils.hpp"

#include "logger.hpp"

namespace {
/**
 * @brief simple struct hold the MSB and LSB of a word
 *
 */
struct HighLowBytes {
  /// @brief most significant bits (High 8-bits)
  uint8_t high{0};

  /// @brief least significant bits (Low 8-bits)
  uint8_t low{0};

  HighLowBytes(const uint8_t high_, const uint8_t low_)
    : high{high_}, low{low_} {
  }

  HighLowBytes(const int16_t word) {
    constexpr uint8_t bit_shift = 8;
    constexpr uint8_t mask = 0xFF;
    high = static_cast<uint8_t>((word >> bit_shift) & mask);
    low = static_cast<uint8_t>(word & mask);
  }

  int16_t ToWord() const {
    constexpr uint8_t bit_shift = 8;
    const int16_t msb = static_cast<int16_t>(high << bit_shift);
    const int16_t lsb = static_cast<int16_t>(low);
    const int16_t word = static_cast<int16_t>(msb | lsb);
    return word;
  }

  std::pair<uint8_t, uint8_t> ToLittleEndian() const {
    return {low, high};
  }

  std::pair<uint8_t, uint8_t> ToBigEndian() const {
    return {high, low};
  }
};

int16_t RegisterBytesToWordUsingLittleEndian(const uint8_t reg0,
                                             const uint8_t reg1) {
  return HighLowBytes(reg1, reg0).ToWord();
}

int16_t RegisterBytesToWordUsingBigEndian(const uint8_t reg0,
                                          const uint8_t reg1) {
  return HighLowBytes(reg0, reg1).ToWord();
}

}  // namespace

int16_t RegisterBytesToWord(const uint8_t reg0, const uint8_t reg1,
                            const EndianByteOrder order) {
  switch (order) {
    case EndianByteOrder::LITTLE:
      return RegisterBytesToWordUsingLittleEndian(reg0, reg1);
    case EndianByteOrder::BIG:
      return RegisterBytesToWordUsingBigEndian(reg0, reg1);
    default:
      SYS_LOG_WARN("RegisterBytesToWord, undefined EndianByteOrder");
      return 0;
  }
}

std::vector<int16_t> RegisterBytesToWords(const std::vector<uint8_t> bytes,
                                          const EndianByteOrder order) {
  if (bytes.size() % 2 != 0) {
    SYS_LOG_WARN(
      "WordToRegisterBytes, the size of bytes vector should be even");
    return {};
  }

  std::vector<int16_t> words(bytes.size() / 2);
  for (size_t i = 0; i < words.size(); i++) {
    const size_t idx{i * 2};
    const uint8_t reg0{bytes[idx]};
    const uint8_t reg1{bytes[idx + 1]};
    words[i] = RegisterBytesToWord(reg0, reg1, order);
  }
  return words;
}

std::pair<uint8_t, uint8_t> WordToRegisterBytes(const int16_t word,
                                                const EndianByteOrder order) {
  HighLowBytes hl_bytes{word};
  switch (order) {
    case EndianByteOrder::LITTLE:
      return hl_bytes.ToLittleEndian();
    case EndianByteOrder::BIG:
      return hl_bytes.ToBigEndian();
    default:
      SYS_LOG_WARN("WordToRegisterBytes, undefined EndianByteOrder");
      return {0, 0};
  }
}

std::vector<uint8_t> WordsToRegisterBytes(const std::vector<int16_t> words,
                                          const EndianByteOrder order) {
  std::vector<uint8_t> bytes(words.size() * 2);
  for (size_t i = 0; i < words.size(); i++) {
    const auto [reg0, reg1] = WordToRegisterBytes(words[i], order);
    const size_t idx{i * 2};
    bytes[idx] = reg0;
    bytes[idx + 1] = reg1;
  }
  return bytes;
}
