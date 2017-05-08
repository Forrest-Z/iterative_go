/*
    Author: David Z.H. Feb. 7th 2014
    this class used to process scan info
*/

#ifndef NODE_SCAN_H
#define NODE_SCAN_H

#include <vector>
#include <utils/point.h>
#include <sensor/sensor_range/rangereading.h>
#include <sensor/sensor_range/rangesensor.h>
#include "macro_params.h"
#include <cmath>

using namespace std;

#define D2R(d) (((d)*M_PI)/180.)

/*
namespace GMapping
{
    class RangeReading;
    class OrientedPoint; 
}
*/
template<int N>
class AngleProject
{
public:
    static AngleProject* getSingletonAngle();
    template<typename T1, typename T2>
    void projectXY(T1* beam, T2* px, T2 *py);
    template<typename T>
    void projectXY(T* beam, T* px, T* py);

private: 
    AngleProject(double angleMin, double angleMax);
    double angleMin_;
    double angleMax_; 
    double angleIncr_;
    const static int nBeams_ = N;
    double angles_[N];
};

template<int N>
AngleProject<N>* AngleProject<N>::getSingletonAngle()
{
    static AngleProject<N> * single = new AngleProject<N>(D2R(-90), D2R(90)); 
    return single;
}

template<int N> 
AngleProject<N>::AngleProject(double minAngle, double maxAngle)
{
    assert(maxAngle > minAngle); 
    angleIncr_ = (maxAngle - minAngle)/(double)(nBeams_-1); 
    for(int i=0; i< nBeams_; i++)
        angles_[i] = minAngle + i*angleIncr_; 
}

template<int N>
template<typename T>
void AngleProject<N>::projectXY(T* beams, T * px, T *py)
{
    for(int i=0; i<nBeams_; i++) 
    {
        px[i] = beams[i] * cosf(angles_[i]); 
        py[i] = beams[i] * sinf(angles_[i]);
    }
}

template<int N>
template<typename T1, typename T2>
void AngleProject<N>::projectXY(T1* beams, T2 * px, T2 *py)
{
    for(int i=0; i<nBeams_; i++) 
    {
        px[i] = beams[i] * cosf(angles_[i]); 
        py[i] = beams[i] * sinf(angles_[i]);
    }
}

class CNodeScan 
{
 public:
    CNodeScan(GMapping::RangeReading &);        
    ~CNodeScan();
    static const int N_BEAMS = 361;

    GMapping::OrientedPoint* getPose(){return m_pose;}
    void setPose(GMapping::OrientedPoint& p) {*m_pose = p;} 
    double * getPlainReading(){return &m_reading[0];}
    
// protected:
    GMapping::OrientedPoint* m_pose; 
    int m_id;
    // GMapping::RangeReading* m_reading;
    vector<double> m_reading; 
    vector<float> m_x; 
    vector<float> m_y; 
 
    PARAM_SET_GET(double, timestamp, protected, public, public);
    
    // double m_timestamp;
private:
    CNodeScan(const CNodeScan&);
    CNodeScan& operator=(const CNodeScan&);
};

ostream& operator<<(ostream& ouf, GMapping::OrientedPoint* );

#endif
