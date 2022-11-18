//
//  mine-common.h
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

#include <vector>
#include <string>

#ifdef MINE_WSTRING_CONVERSION
#   include <locale>
#   include <codecvt>
#endif

#ifndef Common_H
#define Common_H

namespace mine {

using byte = unsigned char;

///
/// \brief Handy safe byte array
///
using ByteArray = std::vector<byte>;

///
/// \brief Contains common functions used across the lib
///
class MineCommon {
public:

    ///
    /// \brief Convert mode for various functions
    ///
    enum class Encoding {
        Raw,
        Base16,
        Base64
    };

    ///
    /// \brief Total items in random bytes list
    ///
    static const int kRandomBytesCount = 256;

    ///
    /// \brief List to choose random byte from
    ///
    static const byte kRandomBytesList[];

#ifdef MINE_WSTRING_CONVERSION
    ///
    /// \brief Converts it to std::string and calls countChars on it
    ///
    /// \note You need to include <locale> and <codecvt> headers before mine.h
    ///
    static std::size_t countChars(const std::wstring& raw) noexcept
    {
        std::string converted = std::wstring_convert
                <std::codecvt_utf8<wchar_t>, wchar_t>{}.to_bytes(raw);
        return countChars(converted);
    }
#endif

    ///
    /// \brief Replacement for better d.size() that consider unicode bytes too
    /// \see https://en.wikipedia.org/wiki/UTF-8#Description
    ///
    static std::size_t countChars(const std::string& d) noexcept;

    ///
    /// \brief Generates random bytes of length
    ///
    static ByteArray generateRandomBytes(const std::size_t len) noexcept;

    ///
    /// \brief Converts byte array to linear string
    ///
    static std::string byteArrayToRawString(const ByteArray& input) noexcept;

    ///
    /// \brief Converts string to byte array
    ///
    static ByteArray rawStringToByteArray(const std::string& str) noexcept;

    ///
    /// \brief Version of mine
    ///
    static std::string version() noexcept;
private:
    MineCommon() = delete;
    MineCommon(const MineCommon&) = delete;
    MineCommon& operator=(const MineCommon&) = delete;
};
} // end namespace mine

#endif // Common_H
