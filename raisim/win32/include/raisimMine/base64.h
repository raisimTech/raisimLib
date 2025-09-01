//
//  base64.h
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

#ifndef Base64_H
#define Base64_H

#include <string>
#include <sstream>
#include <unordered_map>

// codecvt is not part of standard
// hence we leave it to user to enable/disable
// it depending on their
#ifdef MINE_WSTRING_CONVERSION
#   include <locale>
#   include <codecvt>
#endif
#include <iostream>
#include "mine/mine-common.h"

namespace mine {

///
/// \brief Provides base64 encoding / decoding implementation
///
/// This class also provides public interface to encode
/// the iterators for other containers like vector etc.
///
/// This also handles 16-bit, 24-bit and 32-bit characters
///
///
///

#ifdef RAISIM_STATIC_API
#undef RAISIM_STATIC_API
#endif

#ifdef WIN32
  #ifdef RAISIM_MINE_STATIC_MEMBER_EXPORT
    #define RAISIM_MINE_STATIC_API __declspec(dllexport)
  #else
    #define RAISIM_MINE_STATIC_API __declspec(dllimport)
  #endif
#else
  #define RAISIM_MINE_STATIC_API
#endif

class Base64 {
public:

    ///
    /// \brief List of valid base64 encoding characters
    ///
    RAISIM_MINE_STATIC_API static const char kValidChars[];

    inline static const std::unordered_map<byte, byte>& kDecodeMap() {
      static const std::unordered_map<byte, byte> kDecodeMap = {
          {0x41, 0x00}, {0x42, 0x01}, {0x43, 0x02}, {0x44, 0x03},
          {0x45, 0x04}, {0x46, 0x05}, {0x47, 0x06}, {0x48, 0x07},
          {0x49, 0x08}, {0x4A, 0x09}, {0x4B, 0x0A}, {0x4C, 0x0B},
          {0x4D, 0x0C}, {0x4E, 0x0D}, {0x4F, 0x0E}, {0x50, 0x0F},
          {0x51, 0x10}, {0x52, 0x11}, {0x53, 0x12}, {0x54, 0x13},
          {0x55, 0x14}, {0x56, 0x15}, {0x57, 0x16}, {0x58, 0x17},
          {0x59, 0x18}, {0x5A, 0x19}, {0x61, 0x1A}, {0x62, 0x1B},
          {0x63, 0x1C}, {0x64, 0x1D}, {0x65, 0x1E}, {0x66, 0x1F},
          {0x67, 0x20}, {0x68, 0x21}, {0x69, 0x22}, {0x6A, 0x23},
          {0x6B, 0x24}, {0x6C, 0x25}, {0x6D, 0x26}, {0x6E, 0x27},
          {0x6F, 0x28}, {0x70, 0x29}, {0x71, 0x2A}, {0x72, 0x2B},
          {0x73, 0x2C}, {0x74, 0x2D}, {0x75, 0x2E}, {0x76, 0x2F},
          {0x77, 0x30}, {0x78, 0x31}, {0x79, 0x32}, {0x7A, 0x33},
          {0x30, 0x34}, {0x31, 0x35}, {0x32, 0x36}, {0x33, 0x37},
          {0x34, 0x38}, {0x35, 0x39}, {0x36, 0x3A}, {0x37, 0x3B},
          {0x38, 0x3C}, {0x39, 0x3D}, {0x2B, 0x3E}, {0x2F, 0x3F},
          {0x3D, 0x40}
      };
      return kDecodeMap;
    }
    ///
    /// \brief Padding is must in mine implementation of base64
    ///
    static constexpr int kPadding = 64;

    ///
    /// \brief Encodes input of length to base64 encoding
    ///
    static std::string encode(const std::string& raw) noexcept
    {
        return encode(raw.begin(), raw.end());
    }

    ///
    /// \brief Encodes iterators
    ///
    template <class Iter>
    static std::string encode(const Iter& begin, const Iter& end) noexcept
    {
        std::string padding;
        std::stringstream ss;
        for (auto it = begin; it < end; it += 3) {

            //
            // we use example following example for implementation basis
            // Bits              01100001   01100010  01100011
            // 24-bit stream:    011000   010110   001001   100011
            // result indices     24        22       9        35
            //

            int c = static_cast<int>(*it & 0xff);
            ss << static_cast<char>(static_cast<char>(kValidChars[(c >> 2) & 0x3f])); // first 6 bits from first bitset
            if (it + 1 < end) {
                int c2 = static_cast<int>(*(it + 1) & 0xff);
                ss << static_cast<char>(kValidChars[((c << 4) | // remaining 2 bits from first bitset - shift them left to get 4-bit spaces 010000
                                                     (c2 >> 4) // first 4 bits of second bitset - shift them right to get 2 spaces and bitwise
                                                                      // to add them 000110
                                                     ) & 0x3f]);      // must be within 63 --
                                                                      // 010000
                                                                      // 000110
                                                                      // --|---
                                                                      // 010110
                                                                      // 111111
                                                                      // ---&--
                                                                      // 010110 ==> 22
                if (it + 2 < end) {
                    int c3 = static_cast<int>(*(it + 2) & 0xff);
                    ss << static_cast<char>(kValidChars[((c2 << 2) | // remaining 4 bits from second bitset - shift them to get 011000
                                                         (c3 >> 6)   // the first 2 bits from third bitset - shift them right to get 000001
                                                         ) & 0x3f]);
                                                                             // the rest of the explanation is same as above
                    ss << static_cast<char>(kValidChars[c3 & 0x3f]); // all the remaing bits
                } else {
                    ss << static_cast<char>(kValidChars[(c2 << 2) & 0x3f]); // we have 4 bits left from last byte need space for two 0-bits
                    ss << "=";
                }
            } else {
                ss << static_cast<char>(kValidChars[(c << 4) & 0x3f]); // remaining 2 bits from single byte
                ss << "==";
            }

            if (end - it < 4) break;
        }
        return ss.str() + padding;
    }

    ///
    /// \brief Decodes encoded base64
    /// \see decode(const Iter&, const Iter&)
    ///
    static std::string decode(const std::string& e)
    {
        // don't check for e's length to be multiple of 4
        // because of 76 character line-break format (MIME)
        // https://tools.ietf.org/html/rfc4648#section-3.1
        return decode(e.begin(), e.end());
    }

    ///
    /// \brief Decodes base64 iterator from begin to end
    /// \throws std::invalid_argument if invalid encoding. Another time it is thrown
    /// is if no padding is found
    /// std::invalid_argument::what() is set according to the error
    ///
    template <class Iter>
    static std::string decode(const Iter& begin, const Iter& end)
    {
        //
        // we use example following example for implementation basis
        // Bits              01100001   01100010  01100011
        // 24-bit stream:    011000   010110   001001   100011
        // result indices     24        22       9        35
        //

        auto findPosOf = [](char c) -> int {
            try {
                return kDecodeMap().at(static_cast<int>(c & 0xff));
            } catch (const std::exception& e) {
                throw e;
            }
        };

        std::stringstream ss;
        for (auto it = begin; it < end; it += 4) {
            try {
                while (iswspace(*it)) {
                    ++it;

                    if (it >= end) {
                        goto result;
                    }
                }
                int b0 = findPosOf(*it);
                if (b0 == kPadding) {
                    throw std::invalid_argument("No data available");
                }
                if (b0 == -1) {
                    throw std::invalid_argument("Invalid base64 encoding");
                }

                while (iswspace(*(it + 1))) {
                    ++it;

                    if (it >= end) {
                        goto result;
                    }
                }
                int b1 = findPosOf(*(it + 1));
                if (b1 == -1) {
                    throw std::invalid_argument("Invalid base64 encoding");
                }

                while (iswspace(*(it + 2))) {
                    ++it;

                    if (it >= end) {
                        goto result;
                    }
                }
                int b2 = findPosOf(*(it + 2));
                if (b2 == -1) {
                    throw std::invalid_argument("Invalid base64 encoding");
                }

                while (iswspace(*(it + 3))) {
                    ++it;

                    if (it >= end) {
                        goto result;
                    }
                }
                int b3 = findPosOf(*(it + 3));
                if (b3 == -1) {
                    throw std::invalid_argument("Invalid base64 encoding");
                }

                ss << static_cast<byte>(b0 << 2 |     // 011000 << 2 ==> 01100000
                                        b1 >> 4); // 000001 >> 4 ==> 01100001 ==> 11000001 = 97

                if (b1 != kPadding) {
                    if (b2 == kPadding) {
                        // second bitset is only 4 bits
                    } else {
                        ss << static_cast<byte>((b1 & ~(1 << 5) & ~(1 << 4)) << 4 |     // 010110 ==> 000110 << 4 ==> 1100000
                                                                                        // first we clear the bits at pos 4 and 5
                                                                                        // then we concat with next bit
                                                 b2 >> 2); // 001001 >> 2 ==> 00000010 ==> 01100010 = 98
                        if (b3 == kPadding) {
                            // third bitset is only 4 bits
                        } else {
                            ss << static_cast<byte>((b2 & ~(1 << 5) & ~(1 << 4) & ~(1 << 3) & ~(1 << 2)) << 6 |     // 001001 ==> 000001 << 6 ==> 01000000
                                                    // first we clear first 4 bits
                                                    // then concat with last byte as is
                                                     b3); // as is
                        }
                    }
                }
            } catch (const std::exception& e) {
                throw std::invalid_argument(std::string("Invalid base64 encoding: " + std::string(e.what())));
            }
        }
result:
        return ss.str();
    }


#ifdef MINE_WSTRING_CONVERSION
    ///
    /// \brief Converts wstring to corresponding string and returns
    /// encoding
    /// \see encode(const std::string&)
    ///
    /// \note You need to include <locale> and <codecvt> headers before mine.h
    ///
    static std::string encode(const std::wstring& raw) noexcept
    {
        std::string converted = std::wstring_convert
                <std::codecvt_utf8<wchar_t>, wchar_t>{}.to_bytes(raw);
        return encode(converted);
    }

    ///
    /// \brief Helper method to decode base64 encoding as wstring (basic_string<wchar_t>)
    /// \see decode(const std::string&)
    /// \note We do not recommend using it, instead have your own conversion function from
    /// std::string to wstring as it can give you invalid results with characters that are
    /// 5+ bytes long e.g, \x1F680. If you don't use such characters then it should be safe
    /// to use this
    ///
    /// \note You need to include <locale> and <codecvt> headers before mine.h
    ///
    static std::wstring decodeAsWString(const std::string& e)
    {
        std::string result = decode(e);
        std::wstring converted = std::wstring_convert
                <std::codecvt_utf8_utf16<wchar_t>>{}.from_bytes(result);
        return converted;
    }
#endif

    ///
    /// \brief expectedBase64Length Returns expected base64 length
    /// \param n Length of input (plain data)
    ///
    inline static std::size_t expectedLength(std::size_t n) noexcept
    {
        return ((4 * n / 3) + 3) & ~0x03;
    }

    ///
    /// \brief Calculates the length of string
    /// \see countChars()
    ///
    template <typename T = std::string>
    inline static std::size_t expectedLength(const T& str) noexcept
    {
        return expectedLength(MineCommon::countChars(str));
    }

private:
    Base64() = delete;
    Base64(const Base64&) = delete;
    Base64& operator=(const Base64&) = delete;
};
} // end namespace mine


#endif // Base64_H
