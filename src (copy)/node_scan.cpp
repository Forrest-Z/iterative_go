#include "node_scan.h"

using namespace GMapping;


CNodeScan::CNodeScan(GMapping::RangeReading& reading)
{
    m_pose = new OrientedPoint(reading.getPose());
    int m_beams = reading.size();
    assert(m_beams == N_BEAMS); 
    m_reading.resize(m_beams);
    m_x.resize(m_beams); 
    m_y.resize(m_beams); 
    for(int i=0; i<m_beams; i++)
        m_reading[i] = reading[i];
    AngleProject<N_BEAMS>::getSingletonAngle()->projectXY(&m_reading[0], &m_x[0], &m_y[0]);
}

CNodeScan::~CNodeScan()
{
    if(m_pose) delete m_pose;
    // if(m_reading) delete m_reading;
}

ostream& operator<<(ostream& ouf, GMapping::OrientedPoint* p)
{
    ouf<<p->x<<"\t"<<p->y<<"\t"<<p->theta;
    return ouf;
}

