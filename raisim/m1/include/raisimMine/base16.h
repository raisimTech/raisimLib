//
//  base16.h
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

#ifndef Base16_H
#define Base16_H

#include <cstdlib>
#include <algorithm>
#include <string>
#include <sstream>
#include <unordered_map>
#include <vector>
#include "mine/mine-common.h"

namespace mine {

///
/// \brief Provides base16 encoding / decoding
///
/// This class also contains some helpers to convert various input types
/// to byte array and also provides public interface to encode
/// the iterators for other containers like vector etc.
///
class Base16 {
public:

    ///
    /// \brief List of valid hex encoding characters
    ///
    static const std::string kValidChars;

    ///
    /// \brief Map for fast lookup corresponding character
    /// \see Base64::kDecodeMap
    ///
    static const std::unordered_map<byte, byte> kDecodeMap;

    ///
    /// \brief Encodes input to hex encoding
    ///
    static inline std::string encode(const std::string& raw) noexcept
    {
        return encode(raw.begin(), raw.end());
    }

    ///
    /// \brief Wrapper function to encode single hex char to corresponding byte
    ///
    static byte encode(const char* e)
    {
        return static_cast<byte>(strtol(e, nullptr, 16));
    }

    ///
    /// \brief Encodes input iterator to hex encoding
    ///
    template <class Iter>
    static std::string encode(const Iter& begin, const Iter& end) noexcept
    {
        std::ostringstream ss;
        for (auto it = begin; it < end; ++it) {
            encode(*it, ss);
        }
        return ss.str();
    }

    ///
    /// \brief Converts hex stream (e.g, 48656C6C6F) to byte array
    /// \param hex String stream e.g, 48656C6C6F (Hello)
    /// \return Byte array (mine::ByteArray) containing bytes e.g, 0x48, 0x65, 0x6C, 0x6C, 0x6F
    /// \throws invalid_argument if hex is not valid
    ///
    static ByteArray fromString(const std::string& hex);

    ///
    /// \brief Encodes integer to hex
    ///
    template <typename T>
    static std::string encode(T n) noexcept
    {
        std::stringstream ss;
        const int t16(16);
        int remainder;
        while (n != 0) {
            remainder = static_cast<int>(n % t16);
            n /= t16;
            ss << kValidChars[remainder];
        }
        std::string res(ss.str());
        std::reverse(res.begin(), res.end());
        return res;
    }

    ///
    /// \brief Decodes encoded hex
    /// \throws std::invalid_argument if invalid encoding.
    /// std::invalid_argument::what() is set accordingly
    ///
    static std::string decode(const std::string& enc)
    {
        if (enc.size() % 2 != 0) {
            throw std::invalid_argument("Invalid base-16 encoding");
        }
        return decode(enc.begin(), enc.end());
    }

    ///
    /// \brief Decodes encoding to single integer of type T
    ///
    template <typename T>
    static T decodeInt(const std::string& e)
    {
        T result = 0;
        for (auto it = e.begin(); it != e.end() && result >= 0; ++it) {
            try {
                result = ((result << 4) | kDecodeMap.at(*it & 0xff));
            } catch (const std::exception&) {
                throw std::runtime_error("Invalid base-16 encoding");
            }
        }
        return result;
    }

    ///
    /// \brief Encodes single byte
    ///
    static inline void encode(char b, std::ostringstream& ss) noexcept
    {
        int h = (b & 0xff);
        ss << kValidChars[(h >> 4) & 0xf] << kValidChars[(h & 0xf)];
    }

private:
    Base16() = delete;
    Base16(const Base16&) = delete;
    Base16& operator=(const Base16&) = delete;

    ///
    /// \brief Decodes input iterator to hex encoding
    /// \note User should check for the valid size or use decode(std::string)
    /// \throws runtime_error if invalid base16-encoding
    ///
    template <class Iter>
    static std::string decode(const Iter& begin, const Iter& end)
    {
        std::ostringstream ss;
        for (auto it = begin; it != end; it += 2) {
            decode(*it, *(it + 1), ss);
        }
        return ss.str();
    }

    ///
    /// \brief Decodes single byte pair
    ///
    static void decode(char a, char b, std::ostringstream& ss);
};
} // end namespace mine

#endif // Base16_H
