#include "sssoft.h"

#include <math.h>
#include <stdlib.h>
#include <fftw3.h>

#include "opencv2/core/core.hpp"
#include "opencv2/imgproc/imgproc.hpp"

#include <boost/foreach.hpp>

//#include <soft20/soft_fftw.h>
//#include <soft20/s2_legendreTransforms.h>

#include <soft20/makeweights.h>
#include <soft20/s2_primitive.h>
#include <soft20/s2_cospmls.h>
#include <soft20/rotate_so3_fftw.h>
#include <soft20/s2_semi_memo.h>

#include <soft20/wrap_fftw.h>

#include <iostream>
#include <fstream>


#define NORM( x ) ( (x[0])*(x[0]) + (x[1])*(x[1]) )


void SOFTWRAPP::WrapSphCorr2(int bw, const cv::Mat &sphPattern, const cv::Mat &sphSignal, cv::Vec3f &EulerAngle)
{
    int i, j, k, n;
    double *signal, *pattern ;
    double alpha, beta, gamma ;

//    cv::Mat Ssignal, Spattern;

//    sphPattern.convertTo(Spattern,CV_64F);
//    sphSignal.convertTo(Ssignal,CV_64F);

    n = 2 * bw ;

    /* allocate space to hold signal, pattern */
    signal = new double[n * n];
    pattern = new double[n * n];


    /****
        At this point, check to see if all the memory has been
        allocated. If it has not, there's no point in going further.
    ****/
    if ( (signal == NULL) || (pattern == NULL) )
    {
        perror("Error in allocating memory");
        delete[] pattern;
        delete[] signal;
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
    delete[] pattern;
    delete[] signal;
}


void SOFTWRAPP::WarpS2Rotate(int bw, cv::Vec3f eulerang, const harmCoeff& harmcoeff, harmCoeff& rotharmcoeff)
{
    double alpha = eulerang[0];
    double beta = eulerang[1];
    double gamma = eulerang[2];

    double degOut = bw - 1;

    int m, l, i = 0;

    double *sigR, *sigI ;
    double *scratch ;
    double *seminaive_naive_tablespace ;
    double *trans_seminaive_naive_tablespace;
    double **seminaive_naive_table ;
    double **trans_seminaive_naive_table;

    std::complex<double> tmp_deg;
    std::vector< std::complex<double> > tmp_ord;

    sigR = new double[4*bw*bw];
    sigI = new double[4*bw*bw];
    scratch = new double[(10*bw*bw) + (48 * bw)];

    seminaive_naive_tablespace = new double[Reduced_Naive_TableSize(bw,bw) + Reduced_SpharmonicTableSize(bw,bw)];

    trans_seminaive_naive_tablespace = new double[Reduced_Naive_TableSize(bw,bw) + Reduced_SpharmonicTableSize(bw,bw)];


    if ((scratch == NULL) ||
        (sigR == NULL ) || (sigI == NULL ) ||
        (seminaive_naive_tablespace == NULL) ||
        (trans_seminaive_naive_tablespace == NULL) )
    {
        perror("Error in allocating memory");
        exit( 1 ) ;
    }

    seminaive_naive_table = SemiNaive_Naive_Pml_Table(bw, bw,
                                seminaive_naive_tablespace,
                                scratch);


    trans_seminaive_naive_table = Transpose_SemiNaive_Naive_Pml_Table(seminaive_naive_table,
                                                bw, bw,
                                                trans_seminaive_naive_tablespace,
                                                scratch);

    std::vector< std::complex<double> >::const_iterator _it_begin, _it_end;
    std::complex<double> tmpc;

    for (int i=0; i < harmcoeff.size();i++)
    {
        _it_begin = harmcoeff.at(i).begin();

        _it_end = harmcoeff.at(i).end();

        while(_it_begin != _it_end)
        {

            tmpc = *_it_begin;

            sigR[i] = tmpc.real();
            sigI[i] = tmpc.imag();

            _it_begin++;
        }
    }


    rotateFctFFTWS_mem( bw, degOut,
                        sigR, sigI,
                        alpha, beta, gamma ) ;


    for(l = 0 ; l < bw ; l++ )
    {
        for(m = -l; m < l+1 ; m++ )
        {
            tmp_deg.real(sigR[i]);
            tmp_deg.imag(sigI[i]);
            tmp_ord.push_back(tmp_deg);

            i++;
        }

        rotharmcoeff.push_back(tmp_ord);
        tmp_ord.clear();
    }


    free(trans_seminaive_naive_table);
    free(seminaive_naive_table);

    delete[] trans_seminaive_naive_tablespace;
    delete[] seminaive_naive_tablespace;

    delete[] scratch;
    delete[] sigI;
    delete[] sigR;
}


void SOFTWRAPP::DispSphHarm(const harmCoeff &sphHarm)
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
    std::cout<<"\n %Rotation\n %alpha = "<<rotation[0]<<
             ";\n %beta  = "<<rotation[1]<<
             ";\n %gamma = "<<rotation[2]<<";"<<std::endl<<std::endl;

    std::cout<<"\n sumZ = [sumZ;"<<rotation[0]+rotation[2]<<"];"<<std::endl<<std::endl;

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
//    double *workspace2  ;

    fftw_complex *workspace2;
    fftw_plan dctPlan, fftPlan ;

    int rank, howmany_rank;
    fftw_iodim dims[1], howmany_dims[1];

    std::complex<double> tmp_deg;
    std::vector< std::complex<double> > tmp_ord;

    n = 2 * bw ;

    workspace2 =  (fftw_complex *)fftw_malloc( sizeof(fftw_complex) * ((14*bw*bw) + (48 * bw)));

//    workspace2 = new double[(14*bw*bw) + (48 * bw)];

    workspace3 = new double[(12*n) + (n*bw)];

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
         (weights == NULL) ||
         (workspace3 == NULL) || (workspace2 == NULL) ||
         (sigCoefR == NULL) || (sigCoefI == NULL) )
    {
        perror("Error in allocating memory");

        fftw_free( workspace2 );

//        delete[] workspace2;

        delete[] seminaive_naive_tablespace;

        delete[] workspace3;
        delete[] weights;
        delete[] sigCoefI;
        delete[] sigCoefR;

        delete[] tmpI;
        delete[] tmpR;

        return;
    }


    /* Make the weights */
    makeweights( bw, weights ) ;

    /* create fftw plans for the S^2 transforms */
    /* first for the dct */
    dctPlan = fftw_plan_r2r_1d( 2*bw, weights, workspace3,
                      FFTW_REDFT10, FFTW_ESTIMATE ) ;


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
                          (double *)workspace2,
                          (double *)workspace2 + (n*n),
                          FFTW_ESTIMATE );



    /* Precompute Legendres */
    seminaive_naive_table = SemiNaive_Naive_Pml_Table(bw, bw,
                                seminaive_naive_tablespace,
                                (double *)workspace2);


    assert((signal.rows*signal.cols) == (n*n));
    int tt = 0;

    /* load SIGNAL samples into temp array */
    for (i = 0 ; i < signal.rows ; i++ )
    {
        for (j = 0 ; j < signal.cols ; j++ )
        {
            tmpR[tt] = signal.at<double>(i,j) ;
            tmpI[tt] = 0. ;
            tt++;
        }
    }


    /* spherical transform of SIGNAL */
    FST_semi_memo( tmpR, tmpI,
                   sigCoefR, sigCoefI,
                   bw, seminaive_naive_table,
                   (double *)workspace2, isReal, bw,
                   &dctPlan, &fftPlan,
                   weights );


    for(l = 0 ; l < bw ; l++ )
    {
        for(m = -l; m < l+1 ; m++ )
        {
            dummy = seanindex(m,l,bw);
            tmp_deg.real(sigCoefR[dummy]);
            tmp_deg.imag(sigCoefI[dummy]);
            tmp_ord.push_back(tmp_deg);
        }
        coeff.push_back(tmp_ord);
        tmp_ord.clear();
    }


    free( seminaive_naive_table ) ;

    fftw_free( workspace2 );

    fftw_destroy_plan( fftPlan );
    fftw_destroy_plan( dctPlan );

//    delete[] workspace2;
    delete[] workspace3;

    delete[] seminaive_naive_tablespace;

    delete[] weights;
    delete[] sigCoefI;
    delete[] sigCoefR;

    delete[] tmpI;
    delete[] tmpR;
}


void SOFTWRAPP::SampSphFct(int bw,const cv::Mat &pano, cv::Mat &sampFct)
{
    cv::Mat tmp;
    double min, max;
    cv::Size gridSize;

    gridSize.height = bw*2;
    gridSize.width = bw*2;

    cv::cvtColor(pano,tmp,CV_BGR2GRAY);

    tmp.convertTo(tmp ,CV_64F);

    cv::minMaxLoc(tmp,&min,&max);

    tmp -= min;
    tmp /= max;

    tmp /= cv::norm(tmp);

    cv::resize(tmp,tmp,gridSize);

    tmp.convertTo(sampFct,CV_64F);
}


void SOFTWRAPP::SaveSphHarm(const std::string &filename, const harmCoeff &harmcoeff)
{
//    std::ofstream os;
//    os.open((char*)filename.c_str());

//    if ( !os.is_open() )
//    {
//        std::cout<<"Failed to open "<<filename<< std::endl;
//        return;
//    }

//    typedef std::vector< std::complex<double> > my_vec;
//    typedef std::complex<double> my_comp;

//    BOOST_FOREACH(const my_vec &l, harmcoeff)
//    {
//        BOOST_FOREACH(my_comp &m, l)
//        {
//            os << m.real();
//            os << "\n";
//            os << m.imag();
//            os << "\n";
//        }
//    }

//    os.close();
}

