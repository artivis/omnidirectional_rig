#include "sssoft.h"

#include <math.h>
#include <stdlib.h>

#include <fftw3.h>

#include <soft20/makeweights.h>
#include <soft20/csecond.h>
#include <soft20/so3_correlate_fftw.h>
#include <soft20/soft_fftw.h>
#include <soft20/s2_cospmls.h>
#include <soft20/s2_legendreTransforms.h>
#include <soft20/s2_semi_memo.h>
#include <soft20/s2_primitive.h>
#include <soft20/wrap_fftw.h>

#include <soft20/wrap_fftw.h>

#include <iostream>


#define NORM( x ) ( (x[0])*(x[0]) + (x[1])*(x[1]) )


void SOFTWRAPP::WrapSphCorr2(int bw, const cv::Mat &sphPattern, const cv::Mat &sphSignal, cv::Vec3f &EulerAngle)
{
    int i, j, k, n;
    double *signal, *pattern ;
    double alpha, beta, gamma ;

    cv::Mat Ssignal, Spattern;

    sphPattern.convertTo(Spattern,CV_64F);
    sphSignal.convertTo(Ssignal,CV_64F);

    n = 2 * bw ;

    /* allocate space to hold signal, pattern */
    signal = (double *) malloc( sizeof(double) * (n * n) );
    pattern = (double *) malloc( sizeof(double) * (n * n) );


    /****
        At this point, check to see if all the memory has been
        allocated. If it has not, there's no point in going further.
    ****/
    if ( (signal == NULL) || (pattern == NULL) )
    {
        perror("Error in allocating memory");
        return;
    }

    k = 0;

    /* retrieve signal data*/
    for(i=0; i<sphSignal.rows; i++)
    {
        for(j=0;j<sphSignal.cols;j++)
        {
            signal[k] = sphSignal.at<double>(i,j);
            k++;
        }
    }

    k = 0;

    /* retrieve pattern data*/
    for(i=0; i<sphPattern.rows; i++)
    {
        for(j=0;j<sphPattern.cols;j++)
        {
            pattern[k] = sphPattern.at<double>(i,j);
            k++;
        }
    }


    /* now correlate */
    softFFTWCor2( bw,
                  signal,
                  pattern,
                  &alpha, &beta, &gamma,
                  1) ;

    EulerAngle[0] = alpha;
    EulerAngle[1] = beta;
    EulerAngle[2] = gamma;

    /* clean up */
    free( pattern );
    free( signal ) ;
}


void SOFTWRAPP::DispSphHarm(const std::vector< std::vector< std::complex<double> > > &sphHarm)
{
    std::vector< std::complex<double> >::const_iterator _it_begin, _it_end;

    int m = 0;

    for (int i=0; i < sphHarm.size();i++)
    {
        _it_begin = sphHarm.at(i).begin();

        _it_end = sphHarm.at(i).end();

        m = - i;

        while(_it_begin != _it_end)
        {
            std::cout<<"l : "<<i<<" m : "<<m<<" val : "<<*_it_begin<<std::endl;

            _it_begin++;
            m++;
        }
    }

    std::cout<<std::endl;
}


void SOFTWRAPP::DispRotEst(const cv::Vec3f &rotation)
{
    std::cout<<"\nRotation :\n alpha = "<<rotation[0]<<
             "\n beta  = "<<rotation[1]<<
             "\n gamma = "<<rotation[2]<<std::endl<<std::endl;
}






