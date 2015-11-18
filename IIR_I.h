#pragma once
#include <vector>
using namespace std;
class IIR_I  
{  
  public:  
    double *m_pNum;  
    double *m_pDen;  
    double *m_px;  
    double *m_py;  
    int m_num_order;  
    int m_den_order;  
   public:  
    IIR_I();  
    void reset();  
    void setPara(double num[], int num_order, double den[], int den_order);  
    void resp(double data_in[], int m, double data_out[], int n);  
    double filter(double data);  
    //void filter(double data[], int len);  
    double filter(double data_in, double data_out);  
};  