#include "Fitting.h"
#include "FastMath.h"

#include <stdio.h>

#define   ABS(x)   (x)>0?(x):-(x)
#define   SWAP(a,b)   {temp=(a);(a)=(b);(b)=temp;}

void   solve(float   **a,float   *b,float   *x)      
{      
    int   i,j,k,ik;      
    float   mik,temp;      
    for(k=0;k<n;k++)      
    {      
        mik=-1;      
        for(i=k;i<n;i++)      
            if(ABS(a[i][k])>mik)      
            {      
                mik=ABS(a[i][k]);      
                ik=i;      
            }      
        for(j=k;j<n;j++)      
            SWAP(a[ik][j],a[k][j]);      
        SWAP(b[k],b[ik]);      
        b[k]/=a[k][k];      
        for(i=n-1;i>=k;i--)      
            a[k][i]/=a[k][k];      
        for(i=k+1;i<n;i++)      
        {      
            b[i]-=a[i][k]*b[k];      
            for(j=n-1;j>=k;j--)      
                a[i][j]-=a[i][k]*a[k][j];      
        }      
    }      
    for(i=n-1;i>=0;i--)      
    {      
        x[i]=b[i];      
        for(j=i+1;j<n;j++)      
            x[i]-=a[i][j]*x[j];      
    }      
}      

void   linear(float   **x,float   *y,float   *beta)      
{           
    int   i,j,k;      
    float b[p] ;      
    float a[p][p];      
    for(i=0;i<p;i++)      
        for(j=0;j<p;j++)      
        {      
            a[i][j]=0;      
            for(k=0;k<n;k++)      
                a[i][j]+=x[k][i]*x[k][j];      
        }      
    for(i=0;i<p;i++)      
    {      
        b[i]=0;      
        for(j=0;j<n;j++)      
            b[i]+=x[j][i]*y[j];      
    }      
    solve((float**)a,b,beta); 
} 

float SphereFit(float * *data, float* offset)
{
    int   i;
    float   x0,y0,z0,r;      
    float beta[4];
    float x[n][4];      
    float y[n];      
    for(i = 0; i < n; i++)      
    {
        x[i][0] = data[i][0];
        x[i][1] = data[i][1];
        x[i][2] = data[i][2];
        x[i][3]=1;      
        y[i]=-x[i][0]*x[i][0]-x[i][1]*x[i][1]-x[i][2]*x[i][2];      
    }      
    linear((float**)x,y,beta);      
    x0=-beta[0]/2;
    y0=-beta[1]/2;
    z0=-beta[2]/2;      
    r=FastSqrt(x0*x0+y0*y0+z0*z0-beta[3]); 
    offset[0] = x0;
    offset[1] = y0;
    offset[2] = z0;
    return r;
}
