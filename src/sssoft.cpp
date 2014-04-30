#include "sssoft.h"

#include <math.h>
#include <stdlib.h>

#include <fftw3.h>

#include <soft20/makeweights.h>
#include <soft20/s2_primitive.h>
#include <soft20/s2_cospmls.h>

#include <soft20/s2_semi_memo.h>

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
        free( pattern );
        free( signal ) ;
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

void SOFTWRAPP::HarmDesc(const harmCoeff &coeff, std::vector< std::complex<double> > &descriptor)
{
    std::vector< std::complex<double> >::const_iterator _it_begin, _it_end;

    std::complex<double> sum_order;

    int m = 0;

    for (int i=0; i < coeff.size();i++)
    {
        _it_begin = coeff.at(i).begin();

        _it_end = coeff.at(i).end();

        m = - i;

        sum_order.real(0.);
        sum_order.imag(0.);

        while(_it_begin != _it_end)
        {
            std::cout<<"l : "<<i<<" m : "<<m<<" val : "<<*_it_begin<<std::endl;

            sum_order += (*_it_begin * *_it_begin);

            std::cout<<"square"<<*_it_begin * *_it_begin<<std::endl;

            _it_begin++;
            m++;
        }

        descriptor.push_back(sum_order);
    }
}

void SOFTWRAPP::HarmDesc(const harmCoeff &coeff, std::vector< double > &descriptor)
{
    std::vector< std::complex<double> >::const_iterator _it_begin, _it_end;

    double sum_order;

    std::complex<double> tmp;

    int m = 0;

    for (int i=0; i < coeff.size();i++)
    {
        _it_begin = coeff.at(i).begin();

        _it_end = coeff.at(i).end();

        m = - i;

        sum_order = 0;

        while(_it_begin != _it_end)
        {
            std::cout<<"l : "<<i<<" m : "<<m<<" val : "<<*_it_begin<<std::endl;

            tmp = *_it_begin;

            sum_order += ( (double)(tmp.real()*tmp.real()) + (double)(tmp.imag()*tmp.imag()) );

            std::cout<<"square"<<( (double)(tmp.real()*tmp.real()) + (double)(tmp.imag()*tmp.imag()) )<<std::endl;

            _it_begin++;
            m++;
        }

        descriptor.push_back(sum_order);
    }
}


void SOFTWRAPP::WrapSphHarm(int bw, const cv::Mat &signal, harmCoeff &coeff)
{
    int i, j, l, m, n, dummy;

    int isReal = 1;

    double *tmpR, *tmpI ;
    double *workspace3 ;
    double *sigCoefR, *sigCoefI ;
    double *weights ;
    double *seminaive_naive_tablespace ;
    double **seminaive_naive_table ;
    fftw_complex *workspace2  ;
    fftw_plan dctPlan, fftPlan ;

    int rank, howmany_rank;
    fftw_iodim dims[1], howmany_dims[1];

    std::complex<double> tmp_deg;
    std::vector< std::complex<double> > tmp_ord;

    coeff.clear();

    n = 2 * bw ;

    workspace2 = (fftw_complex *) malloc( sizeof(fftw_complex) * ((14*bw*bw) + (48 * bw)));

    tmpR = new double[n*n];
    tmpI = new double[n*n];
    weights = new double[4*bw];
    sigCoefR = new double[bw*bw];
    sigCoefI = new double[bw*bw];

    seminaive_naive_tablespace = new double[Reduced_Naive_TableSize(bw,bw) +
                                            Reduced_SpharmonicTableSize(bw,bw)];


    /****
        At this point, check to see if all the memory has been
        allocated. If it has not, there's no point in going further.
    ****/

    if ( (seminaive_naive_tablespace == NULL) ||
         (tmpR == NULL) || ( tmpI == NULL) ||
         (weights == NULL) || (workspace2 == NULL) ||
         (sigCoefR == NULL) || (sigCoefI == NULL) )
    {
        perror("Error in allocating memory");
        exit( 1 ) ;
    }


    /* create fftw plans for the S^2 transforms */
    /* first for the dct */
    dctPlan = fftw_plan_r2r_1d( 2*bw, weights, workspace3,
                      FFTW_REDFT10, FFTW_ESTIMATE ) ;

    makeweights( bw, weights ) ;

    rank = 1 ;
    dims[0].n = 2*bw ;
    dims[0].is = 1 ;
    dims[0].os = 2*bw ;
    howmany_rank = 1 ;
    howmany_dims[0].n = 2*bw ;
    howmany_dims[0].is = 2*bw ;
    howmany_dims[0].os = 1 ;

    fftPlan = fftw_plan_guru_split_dft( rank, dims,
                          howmany_rank, howmany_dims,
                          tmpR, tmpI,
                          (double *) workspace2,
                          (double *) workspace2 + (n*n),
                          FFTW_ESTIMATE );

    seminaive_naive_table = SemiNaive_Naive_Pml_Table(bw, bw,
                                seminaive_naive_tablespace,
                                (double *) workspace2);

    /* load SIGNAL samples into temp array */

    for (i = 0 ; i < signal.rows ; i++ )
    {
        for (j = 0 ; j < signal.cols ; j++ )
        {
            tmpR[i] = signal.at<float>(i,j) ;
            tmpI[i] = 0. ;
        }
    }


    /* spherical transform of SIGNAL */
    FST_semi_memo( tmpR, tmpI,
                   sigCoefR, sigCoefI,
                   bw, seminaive_naive_table,
                   (double *) workspace2, isReal, bw,
                   &dctPlan, &fftPlan,
                   weights );  // blarg


    for(l = 0 ; l < bw ; l++ )
    {
        for(m = -l; m < l+1 ; m++ )
        {
            dummy = seanindex(m,l,bw);
            tmp_deg.real(sigCoefR[dummy]);
            tmp_deg.imag(   sigCoefI[dummy]);
            tmp_ord.push_back(tmp_deg);
        }
        coeff.push_back(tmp_ord);
        tmp_ord.clear();
    }


    free( seminaive_naive_table ) ;
    free( workspace2 );

    delete[] seminaive_naive_tablespace;

    delete[] weights;
    delete[] sigCoefI;
    delete[] sigCoefR;

    delete[] tmpI;
    delete[] tmpR;
}




