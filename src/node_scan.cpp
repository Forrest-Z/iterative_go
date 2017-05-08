#include "node_scan.h"
#include <stdio.h>
#include <sstream>

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

// only be used in CKeyNode
CNodeScan::CNodeScan(){}

void CNodeScan::reverseR(vector<double>& r)
{
    r.resize(m_x.size()); 
    AngleProject<N_BEAMS>::getSingletonAngle()->inv_projectX(&r[0], &m_x[0]);
}

bool CNodeScan::saveScanXY(const char* fname, bool binary)
{
    bool ret ;
    if(binary)
    {
        ret = saveScanXY_bin_impl(m_x, m_y, fname);
    }else
    {
        ret = saveScanXY_impl(m_x, m_y, fname);
    }
    return ret;
}

bool CNodeScan::savePose_bin_impl(FILE* fp)
{
    // write 
    double pose[3] = {m_pose->x, m_pose->y, m_pose->theta};
    if(fwrite(pose, sizeof(pose), 1, fp) != 1)
    {
        cout<<"node_scan.cpp: failed to savePose!"<<endl;
        return false;
    }
    return true;
}

bool CNodeScan::savePose_impl(fstream& out)
{
    out<<m_pose->x<<" "<<m_pose->y<<" "<<m_pose->theta<<endl;
    return true;
}

bool CNodeScan::saveScanXY_bin_impl(vector<float>& x_, vector<float>& y_, const char* fname)
{
    assert(x_.size() == y_.size() && "x_.size()!=y_.size()");
    FILE* fp  = 0;
    if(fname == 0)
    {
        char fn[255];
        sprintf(fn, "n%d_scan.txt", m_id);
        fp = fopen(fn, "w+"); 
    }else{
        fp = fopen(fname, "w+"); 
    }
    if(fp == NULL) 
    {
        cout<<"node_scan.cpp: failed to open file: "<<fname<<endl;
        return false; 
    }
    int n = x_.size();
    if(fwrite(&n, sizeof(int), 1, fp) != 1)
    {
        cout<<"node_scan.cpp: failed to write N!"<<endl;
        return false; 
    }
    if(fwrite(&x_[0], sizeof(float)*n, 1, fp) != 1 )
    {
        cout<<"node_scan.cpp: failed to write x_!"<<endl;
        cout<<"node_scan.cpp: x_size: "<<x_.size()<<" y_size: "<<y_.size()<<endl;
        return false; 
    }
    if(fwrite(&y_[0], sizeof(float)*n, 1, fp) != 1)
    {
        cout<<"node_scan.cpp: failed to write x_!"<<endl;
        return false; 
    }
    if(!savePose_bin_impl(fp)) 
        return false;
    cout<<"node_scan.cpp: succeed to save scanXY in node "<<m_id<<endl;
    fclose(fp);
}

bool CNodeScan::saveScanXY_impl(vector<float>& x_, vector<float>& y_, const char* fname)
{
    fstream ouf; 
    if(fname == 0)
    {
        stringstream ss; 
        ss<<"n"<<m_id<<"_scan.txt";
        ouf.open(ss.str().c_str(), std::ios_base::out); 
    }else
    {
        ouf.open(fname, std::ios_base::out); 
    }
    if(!ouf.is_open())
    {
        cout<<"node_scan.cpp: failed to open stream!"<<endl;
        return false;
    }
    assert(x_.size() == y_.size() && "x_.size() != y_size()");
    int n = x_.size();
    ouf<<n<<endl;
    for(int i=0; i<n; i++)
    {
        ouf<<x_[i]<<" "; 
    }
    ouf<<endl; 
    for(int i=0; i<n; i++)
    {
        ouf<<y_[i]<<" ";
    }
    ouf<<endl;
    savePose_impl(ouf); 
    return true;
}

ostream& operator<<(ostream& ouf, GMapping::OrientedPoint* p)
{
    ouf<<p->x<<"\t"<<p->y<<"\t"<<p->theta;
    return ouf;
}



