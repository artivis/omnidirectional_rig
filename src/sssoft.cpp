#include "sssoft.h"

#include <math.h>
#include <stdlib.h>

#include <fftw3.h>

#include <soft20/makeweights.h>
#include <csecond.h>
#include <soft20/so3_correlate_fftw.h>
#include <soft20/soft_fftw.h>
#include <soft20/s2_cospmls.h>
#include <soft20/s2_legendreTransforms.h>
#include <soft20/s2_semi_memo.h>
#include <soft20/s2_primitive.h>
#include <soft20/wrap_fftw.h>

#include <iostream>


#define NORM( x ) ( (x[0])*(x[0]) + (x[1])*(x[1]) )


SOFTWRAPP::SOFTWRAPP()
{

}


void SOFTWRAPP::CorrSO3(int bwIn, int bwOut, const std::vector< std::vector< std::complex<double> > > &pattern,
                        const std::vector< std::vector< std::complex<double> > > &signal, cv::Vec3f &rotation, int degLim)
{

      int i, l, m;
      int n, dummy ;
      fftw_complex *workspace1, *workspace2  ;
      double *workspace3 ;
      double *sigCoefR, *sigCoefI ;
      double *patCoefR, *patCoefI ;
      fftw_complex *so3Sig, *so3Coef ;
      fftw_plan p1 ;
      int na[2], inembed[2], onembed[2] ;
      int rank, howmany, istride, idist, ostride, odist ;
      int tmp, maxloc, ii, jj, kk ;
      double maxval, tmpval ;

      if (degLim == 0) degLim = bwOut-1;

      n = 2 * bwIn ;

      so3Sig = (fftw_complex*) fftw_malloc( sizeof(fftw_complex) * (8*bwOut*bwOut*bwOut) );
      so3Coef = (fftw_complex*) fftw_malloc( sizeof(fftw_complex) * ((4*bwOut*bwOut*bwOut-bwOut)/3) ) ;
      workspace1 = (fftw_complex*) fftw_malloc( sizeof(fftw_complex) * (8*bwOut*bwOut*bwOut) );
      workspace2 = (fftw_complex*) fftw_malloc( sizeof(fftw_complex) * ((14*bwIn*bwIn) + (48 * bwIn)));
      workspace3 = (double *) malloc( sizeof(double) * (12*n + n*bwIn));
      sigCoefR = (double *) malloc( sizeof(double) * bwIn * bwIn ) ;
      sigCoefI = (double *) malloc( sizeof(double) * bwIn * bwIn ) ;
      patCoefR = (double *) malloc( sizeof(double) * bwIn * bwIn ) ;
      patCoefI = (double *) malloc( sizeof(double) * bwIn * bwIn ) ;


      /****
           At this point, check to see if all the memory has been
           allocated. If it has not, there's no point in going further.
      ****/

      if ( (so3Coef == NULL) ||
           (workspace1 == NULL) || (workspace2 == NULL) ||
           (workspace3 == NULL) ||
           (sigCoefR == NULL) || (sigCoefI == NULL) ||
           (patCoefR == NULL) || (patCoefI == NULL) ||
           (so3Sig == NULL) )
        {
          perror("Error in allocating memory");
          exit( 1 ) ;
        }



//      BOOST_FOREACH(myvect2 v, vect1)
//      {
//          BOOST_FOREACH(element e, v)
//          {
//              do_something_with_e(e);
//          }
//      }



    /* retrieve input data from human readable order */

    std::vector< std::vector< std::complex<double> > >::const_iterator _it_pattdeg, _it_pattdeg_end ;
    std::vector< std::complex<double> >::const_iterator _it_pattord, _it_pattord_end;

    std::vector< std::vector< std::complex<double> > >::const_iterator _it_sigdeg, _it_sigdeg_end ;
    std::vector< std::complex<double> >::const_iterator _it_sigord, _it_sigord_end;

    std::vector< std::complex<double> > tmp_vec;
    std::complex<double> tmp_cpl;

    _it_pattdeg = pattern.begin();
    _it_pattdeg_end = pattern.end();

    _it_sigdeg = signal.begin();
    _it_sigdeg_end = signal.end();

    for ( l = 0 ; l < bwIn ; l++ )
    {
        for ( m = -l ; m < l + 1 ; m++ )
        {
            //retrieve pattern
            tmp_vec = *_it_pattdeg;

            _it_pattord = tmp_vec.begin();
            _it_pattord_end = tmp_vec.end();

            dummy = seanindex(m, l, bwIn);

            tmp_cpl = *_it_pattord;

            patCoefR[dummy] = tmp_cpl.real();
            patCoefI[dummy] = tmp_cpl.imag();

            //retrieve signal
            tmp_vec = *_it_sigdeg;

            _it_sigord = tmp_vec.begin();
            _it_sigord_end = tmp_vec.end();

            tmp_cpl = *_it_sigord;

            sigCoefR[dummy] = tmp_cpl.real();
            sigCoefI[dummy] = tmp_cpl.imag();

            if (_it_pattord == _it_pattord_end)
            {
                std::cout<<"Should not come here, error with order !"<<std::endl;
                break;
            }

            _it_pattord++;
            _it_sigord++;
            }

        if (_it_pattdeg == _it_pattdeg_end)
        {
            std::cout<<"Should not come here, error with degree !"<<std::endl;
            break;
        }

        _it_pattdeg++;
        _it_sigdeg++;
    }

    /* create plan for inverse SO(3) transform */
    n = 2 * bwOut ;
    howmany = n*n ;
    idist = n ;
    odist = n ;
    rank = 2 ;
    inembed[0] = n ;
    inembed[1] = n*n ;
    onembed[0] = n ;
    onembed[1] = n*n ;
    istride = 1 ;
    ostride = 1 ;
    na[0] = 1 ;
    na[1] = n ;

    p1 = fftw_plan_many_dft( rank, na, howmany,
                             workspace1, inembed,
                             istride, idist,
                             so3Sig, onembed,
                             ostride, odist,
                             FFTW_FORWARD, FFTW_ESTIMATE );


    so3CombineCoef_fftw( bwIn, bwOut, degLim,
                         sigCoefR, sigCoefI,
                         patCoefR, patCoefI,
                         so3Coef ) ;


    /* now inverse so(3) */
    Inverse_SO3_Naive_fftw( bwOut, so3Coef, so3Sig,
                            workspace1, workspace2, workspace3,
                            &p1, 0 ) ;


    /* now find max value */
    maxval = 0.0 ;
    maxloc = 0 ;

    for ( i = 0 ; i < 8*bwOut*bwOut*bwOut ; i ++ )
    {
        tmpval = NORM( so3Sig[i] );

        if ( tmpval > maxval )
        {
            maxval = tmpval;
            maxloc = i ;
        }
    }

    ii = floor( maxloc / (4.*bwOut*bwOut) );
    tmp = maxloc - (ii*4.*bwOut*bwOut);
    jj = floor( tmp / (2.*bwOut) );
    tmp = maxloc - (ii *4*bwOut*bwOut) - jj*(2*bwOut);
    kk = tmp ;

    rotation[0] = M_PI*jj/((double) bwOut); //alpha
    rotation[1] = M_PI*(2*ii+1)/(4.*bwOut); //beta
    rotation[2] = M_PI*kk/((double) bwOut); //gamma



        /* free memory */

    fftw_destroy_plan( p1 );

    fftw_free( so3Coef ) ;
    free( patCoefI );
    free( patCoefR );
    free( sigCoefI );
    free( sigCoefR );
    free( workspace3 );
    fftw_free( workspace2 );
    fftw_free( workspace1 );
    fftw_free( so3Sig ) ;
}


void SOFTWRAPP::SphericalHarmonics(int bw, const cv::Mat &sampSphfunc,  std::vector< std::vector< std::complex<double> > > &SphHarm)
{
    int l, m, dummy;
    int rank, howmany_rank ;
    double *rdata, *idata ;
    double *rcoeffs, *icoeffs ;
    double *weights ;
    double *seminaive_naive_tablespace, *workspace1;
    double **seminaive_naive_table ;
    fftw_plan dctPlan, fftPlan ;
    fftw_complex  *workspace2;
    fftw_iodim dims[1], howmany_dims[1];

    int n = 2 * bw ;

    /* allocate memory */
    rdata = (double *) malloc(sizeof(double) * (n * n));
    idata = (double *) malloc(sizeof(double) * (n * n));
    rcoeffs = (double *) malloc(sizeof(double) * (bw * bw));
    icoeffs = (double *) malloc(sizeof(double) * (bw * bw));
    weights = (double *) malloc(sizeof(double) * 4 * bw);

    workspace1 = (double *) malloc( sizeof(double) * (12*n + n*bw));
    workspace2 = (fftw_complex*)fftw_malloc( sizeof(fftw_complex) * ((14*bw*bw) + (48 * bw)));

    seminaive_naive_tablespace = (double *) malloc(sizeof(double) *
                                    ( Reduced_Naive_TableSize(bw,bw) +
                                      Reduced_SpharmonicTableSize(bw,bw)));

    /****
        At this point, check to see if all the memory has been
        allocated. If it has not, there's no point in going further.
    ****/

    if ((rdata == NULL) || (idata == NULL) ||
        (rcoeffs == NULL) || (icoeffs == NULL) ||
        (seminaive_naive_tablespace == NULL) ||
        (workspace1 == NULL) || (workspace2 == NULL)
            || (weights == NULL))
    {
        perror("Error in allocating memory");
        exit( 1 ) ;
    }


    /* create fftw plans for the S^2 transforms */
    /* first for the dct */
    dctPlan = fftw_plan_r2r_1d( 2*bw, weights, workspace1,
                                FFTW_REDFT10, FFTW_ESTIMATE ) ;


    /*
        fftw "preamble" ;
        note that this plan places the output in a transposed array
    */
    rank = 1 ;
    dims[0].n = 2*bw ;
    dims[0].is = 1 ;
    dims[0].os = 2*bw ;
    howmany_rank = 1 ;
    howmany_dims[0].n = 2*bw ;
    howmany_dims[0].is = 2*bw ;
    howmany_dims[0].os = 1 ;

    /* forward fft */
    fftPlan = fftw_plan_guru_split_dft( rank, dims,
                                        howmany_rank, howmany_dims,
                                        rdata, idata,
                                        (double *) workspace2,
                                        (double *) workspace2 + (n*n),
                                        FFTW_ESTIMATE ); //


    seminaive_naive_table = SemiNaive_Naive_Pml_Table(bw, bw,
                                seminaive_naive_tablespace,
                                (double *) workspace2);


    /* now make the weights */
    makeweights( bw, weights );


    /* retrieve input data*/
    for(int i=0; i<n*n; i++)
    {
        rdata[i] = sampSphfunc.at<double>(0,i);
        idata[i] = 0.0;
    }



    /* now do the forward spherical transform */
    FST_semi_memo(rdata, idata,
                  rcoeffs, icoeffs,
                  bw,
                  seminaive_naive_table,
                  (double *) workspace2,
                  1, bw,
                  &dctPlan,
                  &fftPlan,
                  weights );


    std::complex<double> tmpOrd;
    std::vector< std::complex<double> > tmpDeg;

    /* retrieve data in human readable order */

    for ( l = 0 ; l < bw ; l++ )
    {
        for ( m = -l ; m < l + 1 ; m++ )
        {
            dummy = seanindex(m, l, bw);
            tmpOrd.real(rcoeffs[dummy]);
            tmpOrd.imag(icoeffs[dummy]);

            tmpDeg.push_back(tmpOrd);

//            std::cout<<"l : "<<l<<"m : "<<m<<" coeff : "<<rcoeffs[dummy]<<" "<<icoeffs[dummy]<<" I"<<std::endl;
        }

        SphHarm.push_back(tmpDeg);
        tmpDeg.clear();
    }

    /* clean up */

    tmpDeg.clear();

    fftw_destroy_plan( fftPlan );
    fftw_destroy_plan( dctPlan );

    free(workspace1);
    fftw_free(workspace2);

    free(seminaive_naive_table);
    free(seminaive_naive_tablespace);

    free(weights);
    free(icoeffs);
    free(rcoeffs);
    free(idata);
    free(rdata);
}

static void SphericalHarmonics(int bw, const cv::Mat &sampSphfunc, std::vector< std::complex<double> > &SphHarm)
{
    int size ;
    int l, m, dummy;
    int cutoff;
    int rank, howmany_rank ;
    double *rdata, *idata ;
    double *rcoeffs, *icoeffs ;
    double *weights ;
    double *seminaive_naive_tablespace, *workspace;
    double **seminaive_naive_table ;
    fftw_plan dctPlan, fftPlan ;
    fftw_iodim dims[1], howmany_dims[1];

    /*** ASSUMING WILL SEMINAIVE ALL ORDERS ***/
    cutoff = bw ;
    size = 2*bw;

    /* allocate memory */
    rdata = (double *) malloc(sizeof(double) * (size * size));
    idata = (double *) malloc(sizeof(double) * (size * size));
    rcoeffs = (double *) malloc(sizeof(double) * (bw * bw));
    icoeffs = (double *) malloc(sizeof(double) * (bw * bw));
    weights = (double *) malloc(sizeof(double) * 4 * bw);
    seminaive_naive_tablespace = (double *) malloc(sizeof(double) *
                                    ( Reduced_Naive_TableSize(bw,cutoff) +
                                      Reduced_SpharmonicTableSize(bw,cutoff)));
    workspace = (double *) malloc(sizeof(double) * ((8 * (bw*bw)) +
                     (7 * bw)));

    /****
        At this point, check to see if all the memory has been
        allocated. If it has not, there's no point in going further.
    ****/

    if ((rdata == NULL) || (idata == NULL) ||
        (rcoeffs == NULL) || (icoeffs == NULL) ||
        (seminaive_naive_tablespace == NULL) ||
        (workspace == NULL) )
    {
        perror("Error in allocating memory");
        exit( 1 ) ;
    }

      /* now precompute the Legendres */
    seminaive_naive_table = SemiNaive_Naive_Pml_Table(bw, cutoff,
                            seminaive_naive_tablespace,
                            workspace);

    /* construct fftw plans */

    /* make DCT plan -> note that I will be using the GURU
       interface to execute these plans within the routines*/

    /* forward DCT */
    dctPlan = fftw_plan_r2r_1d( 2*bw, weights, rdata,
                FFTW_REDFT10, FFTW_ESTIMATE ) ;

    /*
        fftw "preamble" ;
        note that this plan places the output in a transposed array
    */
    rank = 1 ;
    dims[0].n = 2*bw ;
    dims[0].is = 1 ;
    dims[0].os = 2*bw ;
    howmany_rank = 1 ;
    howmany_dims[0].n = 2*bw ;
    howmany_dims[0].is = 2*bw ;
    howmany_dims[0].os = 1 ;

    /* forward fft */
    fftPlan = fftw_plan_guru_split_dft( rank, dims,
                howmany_rank, howmany_dims,
                rdata, idata,
                workspace, workspace+(4*bw*bw),
                FFTW_ESTIMATE );


    /* now make the weights */
    makeweights( bw, weights );

    for(int i=0; i<size*size; i++)
    {
        rdata[i] = sampSphfunc.at<double>(0,i); //*_it;
        idata[i] = 0.0;
    }

    /* now do the forward spherical transform */

    FST_semi_memo(rdata, idata,
                rcoeffs, icoeffs,
                bw,
                seminaive_naive_table,
                workspace,
                0,
                cutoff,
                &dctPlan,
                &fftPlan,
                weights );


    std::complex<double> tmpOrd;

    /* retrieve data in human readable order */

    for ( l = 0 ; l < bw ; l++ )
    {
        for ( m = -l ; m < l + 1 ; m++ )
        {
            dummy = seanindex(m, l, bw);
            tmpOrd.real(rcoeffs[dummy]);
            tmpOrd.imag(icoeffs[dummy]);

            SphHarm.push_back(tmpOrd);

            //std::cout<<"l : "<<l<<"m : "<<m<<" coeff : "<<rcoeffs[dummy]<<" "<<icoeffs[dummy]<<" I"<<std::endl;
        }
    }

    /* clean up */
    fftw_destroy_plan( fftPlan );
    fftw_destroy_plan( dctPlan );

    free(workspace);
    free(seminaive_naive_table);
    free(seminaive_naive_tablespace);
    free(weights);
    free(icoeffs);
    free(rcoeffs);
    free(idata);
    free(rdata);
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






