////////////////////////////////////////////////////////////////////////////////////
///
///  \file RangeScan.cpp
///  \brief Implementation of Range Scan methods.
///
///  <br>Author(s): Daniel Barber
///  <br>Created: 13 February 2012
///  <br>Copyright (c) 2012
///  <br>Applied Cognition and Training in Immersive Virtual Environments
///  <br>(ACTIVE) Laboratory
///  <br>Institute for Simulation and Training (IST)
///  <br>University of Central Florida (UCF)
///  <br>All rights reserved.
///  <br>Email: dbarber@ist.ucf.edu
///  <br>Web:  http://active.ist.ucf.edu
///
///  Redistribution and use in source and binary forms, with or without
///  modification, are permitted provided that the following conditions are met:
///      * Redistributions of source code must retain the above copyright
///        notice, this list of conditions and the following disclaimer.
///      * Redistributions in binary form must reproduce the above copyright
///        notice, this list of conditions and the following disclaimer in the
///        documentation and/or other materials provided with the distribution.
///      * Neither the name of the ACTIVE LAB, IST, UCF, nor the
///        names of its contributors may be used to endorse or promote products
///        derived from this software without specific prior written permission.
/// 
///  THIS SOFTWARE IS PROVIDED BY THE ACTIVE LAB''AS IS'' AND ANY
///  EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
///  WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
///  DISCLAIMED. IN NO EVENT SHALL UCF BE LIABLE FOR ANY
///  DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
///  (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
///  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND
///  ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
///  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
///  SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
///
////////////////////////////////////////////////////////////////////////////////////
#include "jaus/environment/range/RangeScan.h"
#include <zlib.h>
#include <bzlib.h>
#include <LzmaEnc.h>
#include <LzmaDec.h>

using namespace JAUS;

/** Writes JAUS formatted record data to packet. 
            \return Bytes written, -1 on error. */
int RangeScan::Write(Packet& packet, int compressionField) const
{
    unsigned int startLength = packet.Length();
    if(mErrorMessage.length() > 0)
    {
        // RangeSensorDataErrorRec
        packet.WriteByte(0);
        packet.Write(mSensorID);
        packet.Write(mDataErrorCode);
        packet.WriteByte((unsigned char)mErrorMessage.length());
        packet.Write(mErrorMessage);
    }
    // Only support some compression methods right now
    else if(compressionField > LZMA)
    {
        // RangeSensorDataErrorRec
        packet.WriteByte(0);
        packet.Write(mSensorID);
        packet.Write(255);
        std::string errorMessage = "Unsupported Compression Type";
        packet.WriteByte((unsigned char)errorMessage.length());
        if(errorMessage.length() > 0)
        {
            packet.Write(errorMessage);
        }
    }
    else
    {
        *( (Byte*)&mCompressionType ) = (Byte)None;

        // RangeSensorDataRec
        packet.WriteByte(1);
        packet.Write(mSensorID);
        packet.Write(mCoordinateSystem);
        packet.Write(mTimeStamp.ToUInt());

        // RangeSensorDataPointList

        if(compressionField >= 0)
        {
            Packet* buffer;
            buffer = (Packet*)&mRawBuffer;
            buffer->Clear();

            buffer->Write((UShort)mRangeData.size());
            Range::List::const_iterator r;
            for(r = mRangeData.begin();
                r != mRangeData.end();
                r++)
            {
                r->Write(*buffer);
            }
            
            packet.Write((Byte)compressionField);

            Packet* compressed = (Packet*)&mCompressedData;
            *( (Byte*)&mCompressionType ) = (Byte)compressionField;
            switch(compressionField)
            {
            case DEFLATE:
                CompressDEFLATE(*buffer, *compressed, 9);
                break;
            case bzip2:
                CompressBZip2(*buffer, *compressed);
                break;
            case LZMA:
                CompressLZMA(*buffer, *compressed);
                break;
            default:
                compressed = buffer;
                packet.Write((UInt)buffer->Length());
                packet.Write(*buffer);
                break;
            }
            if(compressed->Length() == 0)
            {
                return -1;
            }
            packet.Write((UInt)compressed->Length());
            packet.Write(*compressed);
        }
        else
        {
            packet.Write((UShort)mRangeData.size());
            Range::List::const_iterator r;
            for(r = mRangeData.begin();
                r != mRangeData.end();
                r++)
            {
                r->Write(packet);
            }
        }
    }
    return packet.Length() - startLength;
}

/** Reads JAUS formatted record data from packet. 
    \return Bytes read, -1 on error. */
int RangeScan::Read(const Packet& packet, bool compressedMessage)
{
    Clear();

    unsigned int startPos = packet.GetReadPos();
    Byte variant = 0;
    packet.Read(variant);

    if(variant == 0)
    {
        // RangeSensorDataErrorRec
        packet.Read(mSensorID);
        packet.Read(mDataErrorCode);
        Byte mesgLength;
        packet.Read(mesgLength);
        if(mesgLength > 0)
        {
            packet.Read(mErrorMessage, (unsigned int)mesgLength);
        }
    }
    else
    {
        // RangeSensorDataRec
        packet.Read(mSensorID);
        packet.Read(mCoordinateSystem);
        UInt t;
        packet.Read(t);
        mTimeStamp.SetTime(t);

        if(compressedMessage)
        {
            packet.Read(mCompressionType);
            Packet* decompressed = (Packet *)&packet;
            UInt compressedSize = 0;
            switch(mCompressionType)
            {
            case None:
                // Do nothing, read from regular buffer.
                break;
            case bzip2:
                packet.Read(compressedSize);
                packet.Read(mCompressedData, compressedSize);
                DecompressBZip2(mCompressedData, mRawBuffer);
                decompressed = &mRawBuffer;
                break;
            case LZMA:
                packet.Read(compressedSize);
                packet.Read(mCompressedData, compressedSize);
                DecompressLZMA(mCompressedData, mRawBuffer);
                decompressed = &mRawBuffer;
                break;
            case DEFLATE:
                packet.Read(compressedSize);
                packet.Read(mCompressedData, compressedSize);
                DecompressDEFLATE(mCompressedData, mRawBuffer);
                decompressed = &mRawBuffer;
                break;
            default:
                return -1;
                break;
            }
            // RangeSensorDataPointList
            UShort count = 0;
            decompressed->Read(count);
            Range::List::const_iterator r;
            for(UShort i = 0; i < count; i++)
            {
                Range r;
                r.Read(*decompressed);
                mRangeData.push_back(r);
            }
        }
        else
        {
            // RangeSensorDataPointList
            UShort count = 0;
            packet.Read(count);
            Range::List::const_iterator r;
            for(UShort i = 0; i < count; i++)
            {
                Range r;
                r.Read(packet);
                mRangeData.push_back(r);
            }
        }
    }
    return packet.GetReadPos() - startPos;
}

/** 
    \brief Compress data using the DEFLATE format from zLib.
    \param raw Raw data to compress.
    \param compressed The resuling compressed data.
    \param level Compression level, -1 is default, 0 none, 1-9, 9 being best
                 compression, but slowest.
    \return True on success, false on failure.
*/
bool RangeScan::CompressDEFLATE(const Packet& raw, Packet& compressed, int level)
{
    compressed.Clear();
    ::uLong maxCompressSize = ::compressBound((::uLong)raw.Length());
    // Make sure we have more than enough memory in the buffer.
    if(compressed.Reserved() < maxCompressSize)
    {
        compressed.Reserve((unsigned int)maxCompressSize);
    }
    ::uLong compressedSize = (::uLong)compressed.Reserved();
    int result = ::compress2(compressed.Ptr(), &compressedSize, 
        raw.Ptr(), raw.Length(), level);
    if(result == Z_OK)
    {
        compressed.SetLength((unsigned int)compressedSize);
        return true;
    }
    return false;
}


/** 
    \brief Decompresses data using the DEFLATE format from zLib.
    \param compressed Raw data to decompress.
    \param decompressed The resulting data.
    \param level Compression level, -1 is default, 0 none, 1-9, 9 being best
                 compression, but slowest.
    \return True on success, false on failure.
*/
bool RangeScan::DecompressDEFLATE(const Packet& compressed, Packet& decompressed)
{
    decompressed.Clear();
    if(decompressed.Reserved() < compressed.Length()*10)
    {
        decompressed.Reserve(compressed.Length()*10);
    }
    ::uLong destSize = (::uLong)decompressed.Reserved();
    int result = ::uncompress((Bytef*)decompressed.Ptr(), &destSize,
                              (Bytef*)compressed.Ptr(), (uLong)compressed.Length());
    // Need a larger buffer.
    while(result == Z_MEM_ERROR || result == Z_BUF_ERROR)
    {
        decompressed.Reserve(decompressed.Reserved()*2);
        destSize = (::uLong)decompressed.Reserved();
        result = ::uncompress((Bytef*)decompressed.Ptr(), &destSize,
                              (Bytef*)compressed.Ptr(), (uLong)compressed.Length());
    }
    if(result == Z_OK)
    {
        decompressed.SetLength(destSize);
        return true;
    }

    return false;
}


/** 
    \brief Compress data using the bzip2 format from zLib.
    \param raw Raw data to compress.
    \param compressed The resulting compressed data.
    \param blockSize100k Block size is level of compression, and is
                in the range of [1,9] with 9 the best
                compression but most memory, 9 is default.
    \param verbosity [0,4], 0 is silent, 4 is how much
                     info to display to STDIO, 0 is default.
    \param workFactor How compression behaves in worst case, highly
                      repetitive data. Lower values reduce
                      amount of effort standard algorithm will use.
                      Values range from [0, 250] with 30 as default.
    \return True on success, false on failure.
*/
bool RangeScan::CompressBZip2(const Packet& raw, 
                              Packet& compressed, 
                              int blockSize100k,
                              int verbosity,
                              int workFactor)
{
    compressed.Clear();
    // Go guarantee fit, use 1% larger buffer than raw
    // data as advised by bzip2 documentation
    unsigned int maxCompressSize = (unsigned int)(1.1*raw.Length());
    // Make sure we have more than enough memory in the buffer.
    if(compressed.Reserved() < maxCompressSize)
    {
        compressed.Reserve(maxCompressSize);
    }
    unsigned int compressedSize = (unsigned int)compressed.Reserved();
    int code = ::BZ2_bzBuffToBuffCompress((char *)compressed.Ptr(), 
            &compressedSize, 
            (char *)raw.Ptr(), 
            (unsigned int)raw.Length(), 
            blockSize100k,
            verbosity,
            workFactor);
    bool result = false;
    switch(code)
    {
    case BZ_PARAM_ERROR:
        std::cout << "BZIP2 - Invalid parameters\n";
        break;
    case BZ_MEM_ERROR:
        std::cout << "BZIP2 - Memory error\n";
        break;
    case BZ_OUTBUFF_FULL:
        std::cout << "BZIP2 - Output buffer full\n";
        break;
    case BZ_CONFIG_ERROR:
        std::cout << "BZIP2 - Library mis-compiled\n";
        break;
    default:
        compressed.SetLength((unsigned int)compressedSize);
        result = true;
        break;
    }
    return result;
}


/** 
    \brief Decompresses data using the bzip2 format.
    \param compressed Raw data to decompress.
    \param decompressed The resulting data.
    \param small If non-zero alternative decompression algorithms are used
                 that use less memory but are slower. Default is 0.
    \param verbosity [0,4], 0 is silent, 4 is how much
                     info to display to STDIO, 0 is default.
    \return True on success, false on failure.
*/
bool RangeScan::DecompressBZip2(const Packet& compressed, 
                                    Packet& decompressed, 
                                    int small,
                                    int verbosity)
{
    decompressed.Clear();
    if(decompressed.Reserved() < compressed.Length()*10)
    {
        decompressed.Reserve(compressed.Length()*10);
    }
    unsigned int destSize = (unsigned int)decompressed.Reserved();
    int code = ::BZ2_bzBuffToBuffDecompress((char*)decompressed.Ptr(), 
                            &destSize,
                            (char*)compressed.Ptr(), 
                            compressed.Length(),
                            small,
                            verbosity);
    // Need a larger buffer.
    while(code == BZ_OUTBUFF_FULL || code == BZ_MEM_ERROR)
    {
        decompressed.Reserve(decompressed.Reserved()*2);
        destSize = (unsigned int)decompressed.Reserved();
        code = ::BZ2_bzBuffToBuffDecompress((char*)decompressed.Ptr(), 
                            &destSize,
                            (char*)compressed.Ptr(), 
                            compressed.Length(),
                            small,
                            verbosity);
    }
    bool result = false;
    switch(code)
    {
    case BZ_PARAM_ERROR:
        std::cout << "BZIP2 - Invalid parameters\n";
        break;
    case BZ_MEM_ERROR:
        std::cout << "BZIP2 - Memory error\n";
        break;
    case BZ_OUTBUFF_FULL:
        std::cout << "BZIP2 - Output buffer full\n";
        break;
    case BZ_CONFIG_ERROR:
        std::cout << "BZIP2 - Library mis-compiled\n";
        break;
    default:
        decompressed.SetLength((unsigned int)destSize);
        result = true;
        break;
    }
    return result;
}

SRes OnLZMAProgress(void *p, UInt64 inSize, UInt64 outSize)
{
    // Update progress bar.
    return SZ_OK;
}

static void * AllocForLZMA(void *p, size_t size) { return malloc(size); }
static void FreeForLZMA(void *p, void *address) { free(address); }
static ISzAlloc gSzAllocForLZMA = { &AllocForLZMA, &FreeForLZMA };
static ICompressProgress gProgressLZMACallback = { &OnLZMAProgress };

/** 
    \brief Compress data using the bzip2 format.
    \param raw Raw data to compress.
    \param compressed The resulting compressed data.
    \return True on success, false on failure.
*/
bool RangeScan::CompressLZMA(const Packet& raw, 
                              Packet& compressed)
{
    compressed.Clear();
    // Go guarantee fit, use larger buffer than raw
    // data as advised by bzip2 documentation
    unsigned int maxCompressSize = 
            (unsigned int)(raw.Length() + raw.Length()/3 + 128);
    // Make sure we have more than enough memory in the buffer.
    if(compressed.Reserved() < maxCompressSize)
    {
        compressed.Reserve(maxCompressSize);
    }

    unsigned int compressedSize = (unsigned int)compressed.Reserved();

    unsigned propsSize = LZMA_PROPS_SIZE;
    CLzmaEncProps props;
    LzmaEncProps_Init(&props);
    props.dictSize = 1 << 16; // 64 KB
    props.writeEndMark = 1; // 0 or 1

    int code = ::LzmaEncode((::Byte*)(compressed.Ptr() + LZMA_PROPS_SIZE), 
            (::SizeT*)&compressedSize, 
            (::Byte*)raw.Ptr(), 
            (::SizeT)raw.Length(), 
            &props, (::Byte*)(compressed.Ptr()), (::SizeT*)&propsSize,
            (int)props.writeEndMark,
            &gProgressLZMACallback, 
            &gSzAllocForLZMA, &gSzAllocForLZMA);
    bool result = false;
    switch(code)
    {
    case SZ_ERROR_PARAM:
        std::cout << "LZMA - Invalid parameters\n";
        break;
    case SZ_ERROR_MEM:
        std::cout << "LZMA - Memory error\n";
        break;
    case SZ_ERROR_OUTPUT_EOF:
        std::cout << "LZMA - Output buffer full\n";
        break;
    case SZ_ERROR_THREAD:
        std::cout << "LZMA - Library thread error\n";
        break;
    default:
        // Must add the properties size, since compressedSize is only
        // the compressed data without header.
        compressed.SetLength((unsigned int)compressedSize + LZMA_PROPS_SIZE);
        result = true;
        break;
    }
    return result;
}


/** 
    \brief Decompresses data using the  LZMA format..
    \param compressed Raw data to decompress.
    \param decompressed The resulting data.
    \return True on success, false on failure.
*/
bool RangeScan::DecompressLZMA(const Packet& compressed, 
                                    Packet& decompressed)
{
    decompressed.Clear();
    if(decompressed.Reserved() < compressed.Length()*10)
    {
        decompressed.Reserve(compressed.Length()*10);
    }
    unsigned int destSize = (unsigned int)decompressed.Reserved();
    ::SizeT sourceLen = (::SizeT)(compressed.Length() - LZMA_PROPS_SIZE);
    ELzmaStatus status;
    int code = ::LzmaDecode((::Byte*)decompressed.Ptr(), 
                            (::SizeT*)&destSize,
                            (::Byte*)(compressed.Ptr() + LZMA_PROPS_SIZE), 
                            &sourceLen,
                            (::Byte*)(compressed.Ptr()), LZMA_PROPS_SIZE,
                            LZMA_FINISH_END,
                            &status,
                            &gSzAllocForLZMA);

    // Need a larger buffer.
    while(code == SZ_ERROR_MEM)
    {
        decompressed.Reserve(decompressed.Reserved()*2);
        destSize = (unsigned int)decompressed.Reserved();
        code = ::LzmaDecode((::Byte*)decompressed.Ptr(), 
                            (::SizeT*)&destSize,
                            (::Byte*)(compressed.Ptr() + LZMA_PROPS_SIZE), 
                            &sourceLen,
                            (::Byte*)(compressed.Ptr()), LZMA_PROPS_SIZE,
                            LZMA_FINISH_END,
                            &status,
                            &gSzAllocForLZMA);
    }
    bool result = false;
    switch(code)
    {
    case SZ_ERROR_MEM:
        std::cout << "LZMA - Memory error\n";
        break;
    case SZ_ERROR_INPUT_EOF:
        std::cout << "LZMA - Missing input data\n";
        break;
    case SZ_OK:
        decompressed.SetLength((unsigned int)destSize);
        result = true;
        break;
    default:
        result = false;
        break;
    }
    return result;
}

/*  End of File */
