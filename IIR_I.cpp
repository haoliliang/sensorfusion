#include "IIR_I.h"
#include <stdio.h>
/** \brief 将滤波器的内部状态清零，滤波器的系数保留 
 * \return 
 */  
void IIR_I::reset()  
{  
    for(int i = 0; i <= m_num_order; i++)  
    {  
        m_pNum[i] = 0.0;  
    }  
    for(int i = 0; i <= m_den_order; i++)  
    {  
        m_pDen[i] = 0.0;  
    }  
}  
IIR_I::IIR_I()  
{  
    m_pNum = NULL;  
    m_pDen = NULL;  
    m_px = NULL;  
    m_py = NULL;  
    m_num_order = -1;  
    m_den_order = -1;  
};  
/** \brief 
 * 
 * \param num 分子多项式的系数，升序排列,num[0] 为常数项 
 * \param m 分子多项式的阶数 
 * \param den 分母多项式的系数，升序排列,den[0] 为常数项 
 * \param m 分母多项式的阶数 
 * \return 
 */  
void IIR_I::setPara(double num[], int num_order, double den[], int den_order)  
{  
    delete[] m_pNum;  
    delete[] m_pDen;  
    delete[] m_px;  
    delete[] m_py;  
    m_pNum = new double[num_order + 1];  
    m_pDen = new double[den_order + 1];  
    m_num_order = num_order;  
    m_den_order = den_order;  
    m_px = new double[num_order + 1];  
    m_py = new double[den_order + 1];  
    for(int i = 0; i <= m_num_order; i++)  
    {  
        m_pNum[i] = num[i];  
        m_px[i] = 0.0;
		
    }  
    for(int i = 0; i <= m_den_order; i++)  
    {  
        m_pDen[i] = den[i];  
        m_py[i] = 0.0000;
    }  

}  
  
/** \brief 滤波函数，采用直接I型结构 
 * 
 * \param data 传入输入数据 
 * \return 滤波后的结果 
 */  
double IIR_I::filter(double data)  
{  
    m_py[0] = 0.0; // 存放滤波后的结果  
    m_px[0] = data;  
    for(int i = 0; i <= m_num_order; i++)  
    {  
        m_py[0] = m_py[0] + m_pNum[i] * m_px[i];  
    }  
    for(int i = 1; i <= m_den_order; i++)  
    {  
        m_py[0] = m_py[0] - m_pDen[i] * m_py[i];  
    }  
    for(int i = m_num_order; i >= 1; i--)  
    {  
        m_px[i] = m_px[i-1];  
    }  
    for(int i = m_den_order; i >= 1; i--)  
    {  
        m_py[i] = m_py[i-1];  
    }  
    return m_py[0];  
}  
  
  
/** \brief 滤波函数，采用直接I型结构 
 * 
 * \param data[] 传入输入数据，返回时给出滤波后的结果 
 * \param len data[] 数组的长度 
 * \return 
 */  
/*void IIR_I::filter(double data[], int len)  
{  
    int i, k;  
    for(k = 0; k < len; k++)  
    {  
        m_px[0] = data[k];  
        data[k] = 0.0;  
        for(i = 0; i <= m_num_order; i++)  
        {  
            data[k] = data[k] + m_pNum[i] * m_px[i];  
        }  
        for(i = 1; i <= m_den_order; i++)  
        {  
            data[k] = data[k] - m_pDen[i] * m_py[i];  
        }  
        // we get the y value now  
        m_py[0] = data[k];  
        for(i = m_num_order; i >= 1; i--)  
        {  
            m_px[i] = m_px[i-1];  
        }  
        for(i = m_den_order; i >= 1; i--)  
        {  
            m_py[i] = m_py[i-1];  
        }  
    }  
}  */
/** \brief 滤波函数，采用直接I型结构 
 * 
 * \param data_in[] 输入数据 
 * \param data_out[] 保存滤波后的数据 
 * \param len 数组的长度 
 * \return 
 */  
double IIR_I::filter(double data_in, double data_out)  
{  
    int i;  
   // for(k = 0; k < len; k++)  
  //  {  
        m_px[0] = data_in;  
        m_py[0] = 0.0000;  
        for(i = 0; i <= m_num_order; i++)  
        {  
            m_py[0] = m_py[0] + m_pNum[i] * m_px[i];  
        }  
        for(i = 1; i <= m_den_order; i++)  
        {  
            m_py[0] = m_py[0] - m_pDen[i] * m_py[i];  
        }  
        for(i = m_num_order; i >= 1; i--)  
        {  
            m_px[i] = m_px[i-1];  
        }  
        for(i = m_den_order; i >= 1; i--)  
        {  
            m_py[i] = m_py[i-1];  
        }  
        data_out=m_py[0];
		return data_out;
   // }  
}  




/** \brief 计算 IIR 滤波器的时域响应，不影响滤波器的内部状态 
 * \param data_in 为滤波器的输入，0 时刻之前的输入默认为 0，data_in[M] 及之后的输入默认为data_in[M-1] 
 * \param data_out 滤波器的输出 
 * \param M 输入数据的长度 
 * \param N 输出数据的长度 
 * \return 
 */  
void IIR_I::resp(double data_in[], int M, double data_out[], int N)  
{  
    int i, k, il;  
    for(k = 0; k < N; k++)  
    {  
        data_out[k] = 0.0;  
        for(i = 0; i <= m_num_order; i++)  
        {  
            if( k - i >= 0)  
            {  
                il = ((k - i) < M) ? (k - i) : (M - 1);  
                data_out[k] = data_out[k] + m_pNum[i] * data_in[il];  
            }  
        }  
        for(i = 1; i <= m_den_order; i++)  
        {  
            if( k - i >= 0)  
            {  
                data_out[k] = data_out[k] - m_pDen[i] * data_out[k - i];  
            }  
        }  
    }  
}  