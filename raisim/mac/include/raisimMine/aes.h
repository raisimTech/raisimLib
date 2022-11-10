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
    static const std::unordered_map<uint8_t, std::vector<uint8_t>> kKeyParams;

    ///
    /// \brief As defined in FIPS. 197 Sec. 5.1.1
    ///
    static const byte kSBox[];

    ///
    /// \brief As defined in FIPS. 197 Sec. 5.3.2
    ///
    static const byte kSBoxInverse[];

    ///
    /// \brief Round constant is constant for each round
    /// it contains 10 values each defined in
    /// Appendix A of FIPS.197 in column Rcon[i/Nk] for
    /// each key size, we add all of them in one array for
    /// ease of access
    ///
    static const byte kRoundConstant[];

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
