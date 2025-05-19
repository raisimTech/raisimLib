//
//  aes.h
//  Part of Mine crypto library
//
//  You should not use this file, use mine.h
//  instead which is automatically generated and includes this file
//  This is seperated to aid the development
//
//  Copyright (c) 2017-present Amrayn Web Services
//
//  This library is released under the Apache 2.0 license
//  https://github.com/amrayn/mine/blob/master/LICENSE
//
//  https://github.com/amrayn/mine
//  https://amrayn.com
//

#ifdef MINE_CRYPTO_H
#   error "Please use mine.h file. this file is only to aid the development"
#endif

#ifndef AES_H
#define AES_H

#include <string>
#include <array>
#include <unordered_map>
#include <vector>
#include <map>
#include <stdint.h>

#include "mine-common.h"

namespace mine {

///
/// \brief Provides AES crypto functionalities
///
/// This is validated against NIST test data and all
/// the corresponding tests under test/ directory
/// are from NIST themselves.
///
/// Please make sure to use public functions and do not
/// use private functions especially in production as
/// you may end up using them incorrectly. However
/// the source code for AES class is heavily commented for
/// verification on implementation.
///
class AES {
public:

    ///
    /// \brief A key is a byte array
    ///
    using Key = ByteArray;

    AES() = default;
    AES(const std::string& key);
    AES(const ByteArray& key);
    AES(const AES&);
    AES(const AES&&);
    AES& operator=(const AES&);
    virtual ~AES() = default;

    void setKey(const std::string& key);
    void setKey(const ByteArray& key);

    ///
    /// \brief Generates random key of valid length
    ///
    static std::string generateRandomKey(const std::size_t len);

    ///
    /// \brief Ciphers the input with specified hex key
    /// \param key Hex key
    /// \param inputEncoding the type of input. Defaults to Plain
    /// \param outputEncoding Type of encoding for cipher
    /// \param pkcs5Padding Defaults to true, if false non-standard zero-padding is used
    /// \return Base16 encoded cipher
    ///
    std::string encrypt(const std::string& input, const std::string& key, MineCommon::Encoding inputEncoding = MineCommon::Encoding::Raw, MineCommon::Encoding outputEncoding = MineCommon::Encoding::Base16, bool pkcs5Padding = true);

    ///
    /// \brief Ciphers the input with specified hex key using CBC mode
    /// \param key Hex key
    /// \param iv Initialization vector, passed by reference. If empty a random is generated and passed in
    /// \param inputEncoding the type of input. Defaults to Plain
    /// \param outputEncoding Type of encoding for cipher
    /// \param pkcs5Padding Defaults to true, if false non-standard zero-padding is used
    /// \return Base16 encoded cipher
    ///
    std::string encrypt(const std::string& input, const std::string& key, std::string& iv, MineCommon::Encoding inputEncoding = MineCommon::Encoding::Raw, MineCommon::Encoding outputEncoding = MineCommon::Encoding::Base16, bool pkcs5Padding = true);

    ///
    /// \brief Deciphers the input with specified hex key
    /// \param key Hex key
    /// \param inputEncoding the type of input. Defaults to base16
    /// \param outputEncoding Type of encoding for result
    /// \return Base16 encoded cipher
    ///
    std::string decrypt(const std::string& input, const std::string& key, MineCommon::Encoding inputEncoding = MineCommon::Encoding::Base16, MineCommon::Encoding outputEncoding = MineCommon::Encoding::Raw);

    ///
    /// \brief Deciphers the input with specified hex key using CBC mode
    /// \param key Hex key
    /// \param iv Initialization vector
    /// \param inputEncoding the type of input. Defaults to base16
    /// \param outputEncoding Type of encoding for result
    /// \return Base16 encoded cipher
    ///
    std::string decrypt(const std::string& input, const std::string& key, const std::string& iv, MineCommon::Encoding inputEncoding = MineCommon::Encoding::Base16, MineCommon::Encoding outputEncoding = MineCommon::Encoding::Raw);

    ///
    /// \brief Ciphers with ECB-Mode, the input can be as long as user wants
    /// \param input Plain input of any length
    /// \param key Pointer to a valid AES key
    /// \param pkcs5Padding Defaults to true, if false non-standard zero-padding is used
    /// \return Cipher text byte array
    ///
    ByteArray encrypt(const ByteArray& input, const Key* key, bool pkcs5Padding = true);

    ///
    /// \brief Deciphers with ECB-Mode, the input can be as long as user wants
    /// \param input Plain input of any length
    /// \param key Pointer to a valid AES key
    /// \return Cipher text byte array
    ///
    ByteArray decrypt(const ByteArray& input, const Key* key);

    ///
    /// \brief Ciphers with CBC-Mode, the input can be as long as user wants
    /// \param input Plain input of any length
    /// \param key Pointer to a valid AES key
    /// \param iv Initialization vector
    /// \param pkcs5Padding Defaults to true, if false non-standard zero-padding is used
    /// \return Cipher text byte array
    ///
    ByteArray encrypt(const ByteArray& input, const Key* key, ByteArray& iv, bool pkcs5Padding = true);

    ///
    /// \brief Deciphers with CBC-Mode, the input can be as long as user wants
    /// \param input Plain input of any length
    /// \param key Pointer to a valid AES key
    /// \param iv Initialization vector
    /// \return Cipher text byte array
    ///
    ByteArray decrypt(const ByteArray& input, const Key* key, ByteArray& iv);


    // cipher / decipher interface without keys

    std::string encr(const std::string& input, MineCommon::Encoding inputEncoding = MineCommon::Encoding::Raw, MineCommon::Encoding outputEncoding = MineCommon::Encoding::Base16, bool pkcs5Padding = true);

    std::string encr(const std::string& input, std::string& iv, MineCommon::Encoding inputEncoding = MineCommon::Encoding::Raw, MineCommon::Encoding outputEncoding = MineCommon::Encoding::Base16, bool pkcs5Padding = true);

    std::string decr(const std::string& input, MineCommon::Encoding inputEncoding = MineCommon::Encoding::Base16, MineCommon::Encoding outputEncoding = MineCommon::Encoding::Raw);

    std::string decr(const std::string& input, const std::string& iv, MineCommon::Encoding inputEncoding = MineCommon::Encoding::Base16, MineCommon::Encoding outputEncoding = MineCommon::Encoding::Raw);

    ByteArray encr(const ByteArray& input, bool pkcs5Padding = true);

    ByteArray decr(const ByteArray& input);

    ByteArray encr(const ByteArray& input, ByteArray& iv, bool pkcs5Padding = true);

    ByteArray decr(const ByteArray& input, ByteArray& iv);

private:

    ///
    /// \brief A word is array of 4 byte
    ///
    using Word = std::array<byte, 4>;

    ///
    /// \brief KeySchedule is linear array of 4-byte words
    /// \ref FIPS.197 Sec 5.2
    ///
    using KeySchedule = std::map<uint8_t, Word>;

    ///
    /// \brief State as described in FIPS.197 Sec. 3.4
    ///
    using State = std::array<Word, 4>;

    ///
    /// \brief AES works on 16 bit block at a time
    ///
    static const uint8_t kBlockSize = 16;

    ///
    /// \brief Defines the key params to it's size
    ///
    inline static const std::unordered_map<uint8_t, std::vector<uint8_t>>& kKeyParams() {
      static const std::unordered_map<uint8_t, std::vector<uint8_t>> kKeyParams= {
          { 16, {{ 4, 10 }} },
          { 24, {{ 6, 12 }} },
          { 32, {{ 8, 14 }} }
      };
      return kKeyParams;
    }

    ///
    /// \brief As defined in FIPS. 197 Sec. 5.1.1
    ///
    inline static const byte* kSBox() {
      static const byte kSBox[256] = {
          0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76,
          0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0,
          0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15,
          0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75,
          0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84,
          0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf,
          0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8,
          0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2,
          0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73,
          0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb,
          0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79,
          0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08,
          0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a,
          0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e,
          0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf,
          0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16
      };
      return &kSBox[0];
    }

    ///
    /// \brief As defined in FIPS. 197 Sec. 5.3.2
    ///
    static inline const byte* kSBoxInverse() {
      static const byte kSBoxInverse[256] = {
          0x52, 0x09, 0x6a, 0xd5, 0x30, 0x36, 0xa5, 0x38, 0xbf, 0x40, 0xa3, 0x9e, 0x81, 0xf3, 0xd7, 0xfb,
          0x7c, 0xe3, 0x39, 0x82, 0x9b, 0x2f, 0xff, 0x87, 0x34, 0x8e, 0x43, 0x44, 0xc4, 0xde, 0xe9, 0xcb,
          0x54, 0x7b, 0x94, 0x32, 0xa6, 0xc2, 0x23, 0x3d, 0xee, 0x4c, 0x95, 0x0b, 0x42, 0xfa, 0xc3, 0x4e,
          0x08, 0x2e, 0xa1, 0x66, 0x28, 0xd9, 0x24, 0xb2, 0x76, 0x5b, 0xa2, 0x49, 0x6d, 0x8b, 0xd1, 0x25,
          0x72, 0xf8, 0xf6, 0x64, 0x86, 0x68, 0x98, 0x16, 0xd4, 0xa4, 0x5c, 0xcc, 0x5d, 0x65, 0xb6, 0x92,
          0x6c, 0x70, 0x48, 0x50, 0xfd, 0xed, 0xb9, 0xda, 0x5e, 0x15, 0x46, 0x57, 0xa7, 0x8d, 0x9d, 0x84,
          0x90, 0xd8, 0xab, 0x00, 0x8c, 0xbc, 0xd3, 0x0a, 0xf7, 0xe4, 0x58, 0x05, 0xb8, 0xb3, 0x45, 0x06,
          0xd0, 0x2c, 0x1e, 0x8f, 0xca, 0x3f, 0x0f, 0x02, 0xc1, 0xaf, 0xbd, 0x03, 0x01, 0x13, 0x8a, 0x6b,
          0x3a, 0x91, 0x11, 0x41, 0x4f, 0x67, 0xdc, 0xea, 0x97, 0xf2, 0xcf, 0xce, 0xf0, 0xb4, 0xe6, 0x73,
          0x96, 0xac, 0x74, 0x22, 0xe7, 0xad, 0x35, 0x85, 0xe2, 0xf9, 0x37, 0xe8, 0x1c, 0x75, 0xdf, 0x6e,
          0x47, 0xf1, 0x1a, 0x71, 0x1d, 0x29, 0xc5, 0x89, 0x6f, 0xb7, 0x62, 0x0e, 0xaa, 0x18, 0xbe, 0x1b,
          0xfc, 0x56, 0x3e, 0x4b, 0xc6, 0xd2, 0x79, 0x20, 0x9a, 0xdb, 0xc0, 0xfe, 0x78, 0xcd, 0x5a, 0xf4,
          0x1f, 0xdd, 0xa8, 0x33, 0x88, 0x07, 0xc7, 0x31, 0xb1, 0x12, 0x10, 0x59, 0x27, 0x80, 0xec, 0x5f,
          0x60, 0x51, 0x7f, 0xa9, 0x19, 0xb5, 0x4a, 0x0d, 0x2d, 0xe5, 0x7a, 0x9f, 0x93, 0xc9, 0x9c, 0xef,
          0xa0, 0xe0, 0x3b, 0x4d, 0xae, 0x2a, 0xf5, 0xb0, 0xc8, 0xeb, 0xbb, 0x3c, 0x83, 0x53, 0x99, 0x61,
          0x17, 0x2b, 0x04, 0x7e, 0xba, 0x77, 0xd6, 0x26, 0xe1, 0x69, 0x14, 0x63, 0x55, 0x21, 0x0c, 0x7d
      };
      return &kSBoxInverse[0];
    }

    ///
    /// \brief Round constant is constant for each round
    /// it contains 10 values each defined in
    /// Appendix A of FIPS.197 in column Rcon[i/Nk] for
    /// each key size, we add all of them in one array for
    /// ease of access
    ///
    static inline const byte* kRoundConstant() {
      static const byte kRoundConstant[10] = {
          0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36
      };
      return &kRoundConstant[0];
    }

    ///
    /// \brief Nb
    /// \note we make it constant as FIPS.197 p.9 says
    /// "For this standard, Nb=4."
    ///
    static const uint8_t kNb = 4;


    /// rotateWord function is specified in FIPS.197 Sec. 5.2:
    ///      The function RotWord() takes a
    ///      word [a0,a1,a2,a3] as input, performs a cyclic permutation,
    ///      and returns the word [a1,a2,a3,a0]. The
    ///      round constant word array
    ///
    /// Our definition:
    ///      We swap the first byte
    ///      to last one causing it to shift to the left
    ///      i.e,
    ///           [a1]      [a2]
    ///           [a2]      [a3]
    ///           [a3]  =>  [a4]
    ///           [a4]      [a1]
    ///
    static void rotateWord(Word* w);

    /// this function is also specified in FIPS.197 Sec. 5.2:
    ///      SubWord() is a function that takes a four-byte
    ///      input word and applies the S-box
    ///      to each of the four bytes to produce an output word.
    ///
    /// Out definition:
    /// It's a simple substition with kSbox for corresponding byte
    /// index
    ///
    static void substituteWord(Word* w);

    ///
    /// \brief Key expansion function as described in FIPS.197
    ///
    static KeySchedule keyExpansion(const Key* key);

    ///
    /// \brief Adds round to the state using specified key schedule
    ///
    static void addRoundKey(State* state, KeySchedule* keySchedule, int round);

    ///
    /// \brief Substitution step for state
    /// \ref Sec. 5.1.1
    ///
    static void subBytes(State* state);

    ///
    /// \brief Shifting rows step for the state
    /// \ref Sec. 5.1.2
    ///
    static void shiftRows(State* state);

    ///
    /// \ref Sec. 4.2.1
    ///
    static byte xtime(byte x);

    ///
    /// \ref Sec. 4.2.1
    ///
    static byte multiply(byte x, byte y);

    ///
    /// \brief Mixing columns for the state
    /// \ref Sec. 5.1.3
    ///
    static void mixColumns(State* state);

    ///
    /// \brief Transformation in the Inverse Cipher
    /// that is the reverse of subBytes()
    /// \ref Sec. 5.3.2
    ///
    static void invSubBytes(State* state);

    ///
    /// \brief  Transformation in the Inverse Cipher that is
    /// the reverse of shiftRows()
    /// \ref Sec. 5.3.1
    ///
    static void invShiftRows(State* state);

    ///
    /// \brief Transformation in the Inverse Cipher
    /// that is the reverse of mixColumns()
    /// \ref Sec. 5.3.3
    ///
    static void invMixColumns(State* state);

    ///
    /// \brief Prints bytes in hex format in 4x4 matrix fashion
    ///
    static void printBytes(const ByteArray& b);

    ///
    /// \brief Prints state for debugging
    ///
    static void printState(const State*);

    ///
    /// \brief Initializes the state with input. This function
    /// also pads the input if needed (i.e, input is not block of 128-bit)
    ///
    static void initState(State* state, const ByteArray::const_iterator& begin);

    ///
    /// \brief Creates byte array from input based on input mode
    ///
    static ByteArray resolveInputMode(const std::string& input, MineCommon::Encoding inputMode);

    ///
    /// \brief Creates string from byte array based on convert mode
    ///
    static std::string resolveOutputMode(const ByteArray& input, MineCommon::Encoding outputMode);

    ///
    /// \brief Exclusive XOR with iter of range size as input
    ///
    static ByteArray* xorWithRange(ByteArray* input, const ByteArray::const_iterator& begin, const ByteArray::const_iterator& end);

    ///
    /// \brief Raw encryption function - not for public use
    /// \param input 128-bit plain input
    /// If array is bigger it's chopped and if it's smaller, it's padded
    /// please use alternative functions if your array is bigger. Those
    /// function will handle all the bytes correctly.
    /// \param key Pointer to a valid AES key
    /// \note This does not do any key or input validation
    /// \return 128-bit cipher text
    ///
    static ByteArray encryptSingleBlock(const ByteArray::const_iterator& range, const Key* key, KeySchedule* keySchedule);

    ///
    /// \brief Raw decryption function - not for public use
    /// \param input 128-bit cipher input
    /// If array is bigger it's chopped and if it's smaller, it's padded
    /// please use alternative functions if your array is bigger. Those
    /// function will handle all the bytes correctly.
    /// \param key Byte array of key
    /// \return 128-bit plain text
    ///
    static ByteArray decryptSingleBlock(const ByteArray::const_iterator& range, const Key* key, KeySchedule* keySchedule);

    ///
    /// \brief Converts 4x4 byte state matrix in to linear 128-bit byte array
    ///
    static ByteArray stateToByteArray(const State* state);

    ///
    /// \brief Get padding index for stripping the padding (unpadding)
    ///
    static std::size_t getPaddingIndex(const ByteArray& byteArr);

    Key m_key; // to keep track of key differences
    KeySchedule m_keySchedule;

    // for tests
    friend class AESTest_RawCipher_Test;
    friend class AESTest_RawCipherPlain_Test;
    friend class AESTest_RawCipherBase64_Test;
    friend class AESTest_RawSimpleCipher_Test;
    friend class AESTest_RawSimpleDecipher_Test;
    friend class AESTest_SubByte_Test;
    friend class AESTest_InvSubByte_Test;
    friend class AESTest_ShiftRows_Test;
    friend class AESTest_InvShiftRows_Test;
    friend class AESTest_MixColumns_Test;
    friend class AESTest_InvMixColumns_Test;
    friend class AESTest_KeyExpansion_Test;
    friend class AESTest_AddRoundKey_Test;
    friend class AESTest_CbcCipher_Test;
    friend class AESTest_Copy_Test;
};
} // end namespace mine

#endif // AES_H
