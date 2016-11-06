////////////////////////////////////////////////////////////////////////////////////
///
///  \file RangeScan.h
///  \brief Data structure representing a RangeScan Sensor Data Record.
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
#ifndef __JAUS_ENVIRONMENT_SENSING_RANGE_SCAN_DATA_RECORD__H
#define __JAUS_ENVIRONMENT_SENSING_RANGE_SCAN_DATA_RECORD__H

#include "jaus/environment/range/Range.h"
#include <algorithm>

namespace JAUS
{
    ////////////////////////////////////////////////////////////////////////////////////
    ///
    ///   \class RangeScan
    ///   \brief Data representing a Range Sensor Data List from a Range Sensor.
    ///
    ////////////////////////////////////////////////////////////////////////////////////
    class JAUS_ENVIRONMENT_DLL RangeScan
    {
    public:
        enum Compression
        {
            None = 0,
            DEFLATE,
            bzip2,
            LZMA
        };
        static const Byte NativeCoordinate = Range::NativeCoordinate;    ///< Range point is in the native coordinate system of sensor (Default).
        static const Byte VehicleCoordinate = Range::VehicleCoordinate;  ///< Range point is in the coordinate system of the vehicle.
        typedef std::vector<RangeScan> List;    ///<  List of range data points.
        typedef std::map<int, RangeScan> Map;   ///<  Map of range scans by sensor.
        RangeScan() : mSensorID(1),
                  mDataErrorCode(1),
                  mCoordinateSystem(NativeCoordinate),
                  mCompressionType((Byte)None)
        {
        }
        RangeScan(const RangeScan& data) { *this = data; }
        ~RangeScan() {}
        /** Sets sensor ID value (0 is not valid.) */
        inline void SetSensorID(const UShort id) 
        {
            mSensorID = id;
        }
        inline bool SetCoordinateSystem(const Byte system) 
        {
            if(system >= 0 && system <= 1)
            {
                mCoordinateSystem = system; return true;
            }
            return false;
        }
        inline void SetTimeStamp(const Time& t) { mTimeStamp = t; }
        /** 
            Sets the data error code for message if present. 
            \param code 0 = sensor not active, 1 = invalid compression, 255 = 
                        uknown error/failure.
            \param message Human readable text message to go with the error code.
        */
        inline void SetError(const Byte code, const std::string& message)
        {
            mDataErrorCode = code; mErrorMessage = message;
        }
        inline void SetRangeDataList(const Range::List& rangeDataList) { mRangeData = rangeDataList; }
        inline Range::List& GetRangeDataList() { return mRangeData; }
        inline Range::List* GetRangeDataListPtr() { return &mRangeData; }
        inline const Range::List& GetRangeDataList() const { return mRangeData; }
        inline const Range::List* GetRangeDataListPtr() const { return &mRangeData; }
        inline Byte GetCompressionType() const { return mCompressionType; }
        inline void SetCompressionType(const Byte cType) { mCompressionType = cType; }
        inline UShort GetSensorID() const { return mSensorID; }
        inline Byte GetCoordinateSystem() const { return mCoordinateSystem; }
        inline std::string GetErrorMessage() const { return mErrorMessage; }
        inline Time GetTimeStamp() const { return mTimeStamp; }
        static bool CompressDEFLATE(const Packet& raw, Packet& compressed, int level = -1);
        static bool DecompressDEFLATE(const Packet& commpressed, Packet& decompressed);
        static bool CompressBZip2(const Packet& raw, Packet& 
                                  compressed, 
                                  int blockSize100k = 9,
                                  int verbosity = 0,
                                  int workFactor = 30);
        static bool DecompressBZip2(const Packet& compressed, Packet& decompressed, int doSmall = 0, int verbosity = 0);
        static bool CompressLZMA(const Packet& raw, Packet& compressed);
        static bool DecompressLZMA(const Packet& compressed,
            Packet& decompressed);
        /** JAUS formatted record data to packet. 
            If compressionFiled >= 0, then encoding is done for compressed data message.
            \return Bytes written, -1 on error. */
        int Write(Packet& packet, int compressionField = -1) const;
        /** Reads JAUS formatted record data from packet. 
            \return Bytes read, -1 on error. */
        int Read(const Packet& packet, bool compressedMessage = false);
        /** Clears all values. */
        virtual void Clear()
        {
            mSensorID = 1;
            mDataErrorCode = 255;
            mErrorMessage.clear();
            mCoordinateSystem = Range::NativeCoordinate;
            mTimeStamp.Clear();
            mRangeData.clear();
            mCompressedData.Clear();
            mCompressionType = (Byte)None;
            mRawBuffer.Clear();
        }
        RangeScan& operator=(const RangeScan& range)
        {
            if(this != &range)
            {
                mSensorID = range.mSensorID;
                mDataErrorCode = range.mDataErrorCode;
                mErrorMessage = range.mErrorMessage;
                mCoordinateSystem = range.mCoordinateSystem;
                mTimeStamp = range.mTimeStamp;
                mRangeData = range.mRangeData;
                mRawBuffer = range.mRawBuffer;
                mCompressedData = range.mCompressedData;
                mCompressionType = range.mCompressionType;
            }
            return *this;
        }
    protected:
        UShort mSensorID;           ///<  ID of the sensor, 0 is not valid.
        Byte mDataErrorCode;        ///<  Data error code, 0 = sensor not active, 1 = invalid, 255 = Unknown Error/Failure.
        std::string mErrorMessage;  ///<  Error message data.
        Byte mCoordinateSystem;     ///<  Coordinate system.
        Time mTimeStamp;            ///<  Time when data was valid.
        Range::List mRangeData;     ///<  Decompressed range data.
        Packet mRawBuffer;          ///<  Raw JAUS formatted data buffer (used for compression).
        Packet mCompressedData;     ///<  Compressed data.
        Byte mCompressionType;      ///<  Compression format.
    };

    static RangeScan::Compression FromStringToCompression(const std::string& str)
    {
        std::string lcase(str);
        std::transform(lcase.begin(), lcase.end(), lcase.begin(), ::tolower);
        if(lcase == "none")
        {
            return RangeScan::None;
        }
        if(lcase == "deflate")
        {
            return RangeScan::DEFLATE;
        }
        if(lcase == "bzip2")
        {
            return RangeScan::bzip2;
        }
        if(lcase == "lzma")
        {
            return RangeScan::LZMA;
        }
        return RangeScan::None;
    }

    static std::string FromCompressionToString(const RangeScan::Compression type)
    {
        std::string value;
        switch(type)
        {
        case RangeScan::None:
            value = "None";
            break;
        case RangeScan::DEFLATE:
            value = "DEFLATE";
            break;
        case RangeScan::bzip2:
            value = "bzip2";
            break;
        default:
            value = "LZMA";
            break;
        }
        return value;
    }
}

#endif
/*  End of File */
