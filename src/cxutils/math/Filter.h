/////////////////////////////////////////////////////////////////////////////////////
///
/// \file Filter.h
/// \brief Defines some basic filters and a filter interfaces for mpData
///        processing.
///
/// Author(s): Gary Stein<br>
/// Created: 2010<br>
/// Copyright (c) 2010<br>
/// Modified by Daniel Barber on 3/9/2013
/// Robotics Laboratory and Club<br>
/// University of Central Florida (UCF) <br>
/// Email: gstein@ucf.edu <br>
/// Web: http://robotics.ucf.edu <br>
///
///  Redistribution and use in source and binary forms, with or without
///  modification, are permitted provided that the following conditions are met:
///      * Redistributions of source code must retain the above copyright
///        notice, this list of conditions and the following disclaimer.
///      * Redistributions in binary form must reproduce the above copyright
///        notice, this list of conditions and the following disclaimer in the
///        documentation and/or other materials provided with the distribution.
///      * Neither the name of the ROBOTICS CLUB AT UCF, UCF, nor the
///        names of its contributors may be used to endorse or promote products
///        derived from this software without specific prior written permission.
/// 
///  THIS SOFTWARE IS PROVIDED BY THE ROBOTICS CLUB AT UCF ''AS IS'' AND ANY
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
#ifndef _ZEBULON_CXUTILS_UTILITY_FILTER__H
#define _ZEBULON_CXUTILS_UTILITY_FILTER__H


#include <cstdio>
#include <cmath>
#include <string.h>
#include "cxutils/CxBase.h"
#include "cxutils/CircularArray.h"

namespace CxUtils
{
    ////////////////////////////////////////////////////////////////////////////////////
    ///
    ///   \class Filter
    ///   \brief Defines a filter object.  All filters are derived from this
    ///          interface.
    ///
    ////////////////////////////////////////////////////////////////////////////////////
    class Filter
    {
    public:
        //  All filters require a size (data values count/buffer size).
        Filter(int size) : mData(size)
        {
            mFilterOutput = 0.0;
            mPreWrapOut = 0.0;
            //  Assume not wrapped originally
            mWrap = 0;
            mValidFlag = 0;
            mMinValue = mMaxValue = mHalfRange = mRange = 0;
        }
        // Destructor.
        virtual ~Filter() {}
        ////////////////////////////////////////////////////////////////////////////////////
        ///
        ///   \brief Method used to setup limits for data.  This method only really needs to
        ///   to be used if you have data that wraps around a point.  For example, a compass
        ///   has values pass 0,360 or -180, 180, data will then be wrapped then
        ///   unwrapped.
        ///
        ///   \param[in] minVal Minimum value.
        ///   \param[in] maxVal Maximum value.
        ///
        ///   \return 0 on error, 1 on success.
        ///
        ////////////////////////////////////////////////////////////////////////////////////
        virtual int SetLimits(double minVal,double maxVal)
        {
            mWrap = 1;
            mMinValue = minVal;
            mMaxValue = maxVal;
            mRange = (maxVal - minVal);
            mHalfRange = mRange/2.0;
            return 1;
        }
        ////////////////////////////////////////////////////////////////////////////////////
        ///
        ///   \brief Add a new data point to be filtered.
        ///
        ///   \param[in] value Data point to add to filter.
        ///
        ////////////////////////////////////////////////////////////////////////////////////
        virtual void PushBack(double value)
        {
            // Value wrapped to be like output
            if(mWrap == 1)
            {
                while(value >= mPreWrapOut+mHalfRange)
                {
                    value -= mRange;
                }
                while(value < mPreWrapOut - mHalfRange)
                {
                    value += mRange;
                }
            }
            mData.push_back(value);
            mValidFlag = RunFilter();
            // if mWrap, fix on mOutput level to inside mRange
            if(mWrap == 1)
            {
                mPreWrapOut = mFilterOutput;
                while(mFilterOutput >= mMaxValue)
                {
                    mFilterOutput -= mRange;
                }
                while(mFilterOutput < mMinValue)
                {
                    mFilterOutput += mRange;
                }
            }
        }
        ////////////////////////////////////////////////////////////////////////////////////
        ///
        ///   \brief Gets the output of the filter.
        ///
        ///   \param[in] value Output value of filter.
        ///
        ///   \return 1 if data is valid, 0 otherwise.
        ///
        ////////////////////////////////////////////////////////////////////////////////////
        virtual int GetFilteredOutput(double &value) const
        {
            value = mFilterOutput;
            return mValidFlag;
        }
        virtual void Copy(const Filter* f)
        {
            mData = f->mData;
            mFilterOutput = f->mFilterOutput;
            mPreWrapOut = f->mPreWrapOut;
            mValidFlag = f->mValidFlag;
            mWrap = f->mWrap;
            mMinValue = f->mMinValue;
            mMaxValue = f->mMaxValue;
            mHalfRange = f->mHalfRange;
            mRange = f->mRange;
        }
    protected:
        // Overload this method for different filters.
        virtual int RunFilter()
        {
            if(mData.size() > 0)
            {
                mFilterOutput = mData.back();
                return 1;
            }
            else
            {
                mFilterOutput = 0;
                return 0;
            }
        }
        CircularArray<double> mData;
        double mFilterOutput;   ///<  Previous output value generated.
        double mPreWrapOut;     ///<  Previous wrapping value (for data that wraps around 0 like a compass).
        int mValidFlag;         ///<  1 if data is valid (have enough points for valid output) 0 otherwise.
    private:
        int mWrap;              ///<  Flag set when wrapping data points.
        double mMinValue;       ///<  Minimum value (limit).
        double mMaxValue;       ///<  Maximum value (limit).
        double mHalfRange;      ///<  Half range value.
        double mRange;          ///<  Range of values.
    };

    ////////////////////////////////////////////////////////////////////////////////////
    ///
    ///   \class LowPassFilter
    ///   \brief Low pass filter (only low values/changes pass through).
    ///
    ////////////////////////////////////////////////////////////////////////////////////
    class LowPassFilter : public Filter
    {
    public:
        ////////////////////////////////////////////////////////////////////////////////////
        ///
        ///   \brief Constructor.
        ///
        ///   \param[in] dt Time between values in seconds (dt).
        ///   \param[in] rc Time constant seconds.  
        ///                 (like a capacitor, will smooth data the larger it is).
        ///
        ////////////////////////////////////////////////////////////////////////////////////
        LowPassFilter(double dt, double rc) : Filter(2)
        {
            mAlpha = dt/(dt + rc);
        }
        LowPassFilter(const LowPassFilter& f) : Filter(2)
        {
            *this = f;
        }
        virtual ~LowPassFilter()
        {
        }
        LowPassFilter& operator=(const LowPassFilter& f)
        {
            if(this != &f)
            {
                Copy(&f);
                mAlpha = f.mAlpha;
            }
            return *this;
        }
    protected:
        // Overload this method for different filters.
        virtual int RunFilter()
        {
            if(mData.size() < mData.capacity())
            {
                mFilterOutput = 0.0;
                return 0;
            }
            else
            {
                mFilterOutput = 
                    mFilterOutput + mAlpha*(mData.back() - mFilterOutput);
                return 0;
            }
        }
        double mAlpha; ///< Ratio of sampling time (dt) to time constant (rc).
    };
    /** Original Interfaces 
    ////////////////////////////////////////////////////////////////////////////////////
    ///
    ///   \class Filter
    ///   \brief Defines a filter object.  All filters are derived from this
    ///          interface.
    ///
    ////////////////////////////////////////////////////////////////////////////////////
    class ZEB_FILTER_UTIL_DLL Filter
    {
    public:
        //  All filters require a size (data values count/buffer size).
        Filter(int size);
        // Destructor.
        virtual ~Filter();
        ////////////////////////////////////////////////////////////////////////////////////
        ///
        ///   \brief Method used to setup limits for data.  This method only really needs to
        ///   to be used if you have data that wraps around a point.  For example, a compass
        ///   has values pass 0,360 or -180, 180, data will then be wrapped then
        ///   unwrapped.
        ///
        ///   \param[in] minVal Minimum value.
        ///   \param[in] maxVal Maximum value.
        ///
        ///   \return 0 on error, 1 on success.
        ///
        ////////////////////////////////////////////////////////////////////////////////////
        virtual int Setup(double minVal,double maxVal);
        ////////////////////////////////////////////////////////////////////////////////////
        ///
        ///   \brief Add a new data point to be filtered.
        ///
        ///   \param[in] value Data point to add to filter.
        ///
        ////////////////////////////////////////////////////////////////////////////////////
        virtual void Add(double value);
        ////////////////////////////////////////////////////////////////////////////////////
        ///
        ///   \brief Gets the output of the filter.
        ///
        ///   \param[in] value Output value of filter.
        ///
        ///   \return 1 if data is valid, 0 otherwise.
        ///
        ////////////////////////////////////////////////////////////////////////////////////
        virtual int Output(double &value) const;
    protected:
        // Overload this method.
        virtual int Process();
        CxUtils::Mutex mMutex;; ///<  For thread protection of data.
        int mSize;              ///<  Number of values used in filter.
        int mCount;             ///<  Keeps track of elements in circular array.
        double *mpData;         ///<  Data points collected so far for filter use.
        double mOutput;         ///<  Previous output value generated.
        double mPreWrapOut;     ///<  Previoius wrapping value (for data that wraps around 0 like a compass).
        int mValidFlag;             ///<  1 if data is valid (have enough points for valid output) 0 otherwise.
    private:
        int mWrap;              ///<  Flag set when wrapping data points.
        double mMinValue;       ///<  Minimum value (limit).
        double mMaxValue;       ///<  Maximum value (limit).
        double mHalfRange;      ///<  Half range value.
        double mRange;          ///<  Range of values.
    };
    */
}
#endif
