#include "sssoft.h"

#include <math.h>
#include <stdlib.h>

#include "fftw3.h"

#include <soft20/makeweights.h>
#include <soft20/so3_correlate_fftw.h>
#include <soft20/soft_fftw.h>
#include <soft20/s2_cospmls.h>
//#include <soft20/s2_legendreTransforms.h>
#include <soft20/s2_semi_memo.h>
#include <soft20/s2_primitive.h>

#include <iostream>


SOFTWRAPP::SOFTWRAPP()
{
}


void SOFTWRAPP::CorrSO3(int bwIn, int bwOut, std::vector< std::complex<double> > &pattern, std::vector< std::complex<double> > &signal, cv::Vec3f &rotation)
{

      int i ;
      int n, degLim ;
      fftw_complex *workspace1, *workspace2  ;
      double *workspace3 ;
      double *sigR, *sigI ;
      double *sigCoefR, *sigCoefI ;
      double *patCoefR, *patCoefI ;
      fftw_complex *so3Sig, *so3Coef ;
      fftw_plan p1 ;
      int tmp, maxloc, ii, jj, kk ;
      double maxval, tmpval ;
      double *weights ;
      double *seminaive_naive_tablespace  ;
      fftw_plan dctPlan, fftPlan ;

      degLim = std::floor(bwIn/2);

      n = 2 * bwIn ;

      //sigR = (double *) calloc( n * n, sizeof(double) );
      //sigI = (double *) calloc( n * n, sizeof(double) );
      so3Sig = fftw_malloc( sizeof(fftw_complex) * (8*bwOut*bwOut*bwOut) );
      workspace1 = fftw_malloc( sizeof(fftw_complex) * (8*bwOut*bwOut*bwOut) );
      workspace2 = fftw_malloc( sizeof(fftw_complex) * ((14*bwIn*bwIn) + (48 * bwIn)));
      workspace3 = (double *) malloc( sizeof(double) * (12*n + n*bwIn));
      sigCoefR = (double *) malloc( sizeof(double) * bwIn * bwIn ) ;
      sigCoefI = (double *) malloc( sizeof(double) * bwIn * bwIn ) ;
      patCoefR = (double *) malloc( sizeof(double) * bwIn * bwIn ) ;
      patCoefI = (double *) malloc( sizeof(double) * bwIn * bwIn ) ;
      so3Coef = fftw_malloc( sizeof(fftw_complex) * ((4*bwOut*bwOut*bwOut-bwOut)/3) ) ;

      seminaive_naive_tablespace =
        (double *) malloc(sizeof(double) *
                  (Reduced_Naive_TableSize(bwIn,bwIn) +
                   Reduced_SpharmonicTableSize(bwIn,bwIn)));

      weights = (double *) malloc(sizeof(double) * (4*bwIn));

      /****
           At this point, check to see if all the memory has been
           allocated. If it has not, there's no point in going further.
      ****/

      if ( (seminaive_naive_tablespace == NULL) || (weights == NULL) ||
           //(sigR == NULL) || (sigI == NULL) ||
           (so3Coef == NULL) ||
           (workspace1 == NULL) || (workspace2 == NULL) ||
           (workspace3 == NULL) ||
           (sigCoefR == NULL) || (sigCoefI == NULL) ||
           (patCoefR == NULL) || (patCoefI == NULL) ||
           (so3Sig == NULL) )
        {
          perror("Error in allocating memory");
          exit( 1 ) ;
        }











    so3CombineCoef_fftw( bwIn, bwOut, degLim,
                 sigCoefR, sigCoefI,
                 patCoefR, patCoefI,
                 so3Coef ) ;




    /* now inverse so(3) */
      Inverse_SO3_Naive_fftw( bwOut,
                  so3Coef,
                  so3Sig,
                  workspace1,
                  workspace2,
                  workspace3,
                  &p1,
                  0 ) ;



      /* now find max value */
      maxval = 0.0 ;
      maxloc = 0 ;
      for ( i = 0 ; i < 8*bwOut*bwOut*bwOut ; i ++ )
        {
          /*
        if (so3Sig[i][0] >= maxval)
        {
        maxval = so3Sig[i][0];
        maxloc = i ;
        }
          */
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

      printf("ii = %d\tjj = %d\tkk = %d\n", ii, jj, kk);

      printf("alpha = %f\nbeta = %f\ngamma = %f\n",
         M_PI*jj/((double) bwOut),
         M_PI*(2*ii+1)/(4.*bwOut),
         M_PI*kk/((double) bwOut) );



      fftw_destroy_plan( p1 );
        fftw_destroy_plan( fftPlan );
        fftw_destroy_plan( dctPlan );



        free( weights );
        fftw_free( so3Coef ) ;
        free( patCoefI );
        free( patCoefR );
        free( sigCoefI );
        free( sigCoefR );
        free( workspace3 );
        fftw_free( workspace2 );
        fftw_free( workspace1 );
        fftw_free( so3Sig ) ;
        free( sigI );
        free( sigR );


}


void SOFTWRAPP::SphericalHarmonics(int bw, const cv::Mat &sampSphfunc,  std::vector< std::vector< std::complex<double> > > &SphHarm)
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

            //std::cout<<"l : "<<l<<"m : "<<m<<" coeff : "<<rcoeffs[dummy]<<" "<<icoeffs[dummy]<<" I"<<std::endl;
        }

        SphHarm.push_back(tmpDeg);
        tmpDeg.clear();
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
