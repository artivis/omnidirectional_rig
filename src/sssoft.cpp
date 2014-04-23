#include "sssoft.h"

#include <math.h>
#include <stdlib.h>

#include "fftw3.h"

#include <soft20/makeweights.h>
//#include <soft20/so3_correlate_fftw.h>
//#include <soft20/soft_fftw.h>
#include <soft20/s2_cospmls.h>
//#include <soft20/s2_legendreTransforms.h>
#include <soft20/s2_semi_memo.h>
#include <soft20/s2_primitive.h>

#include <iostream>


SOFTWRAPP::SOFTWRAPP()
{
}


void SOFTWRAPP::CorrSO3(const cv::Mat &sphSample, const cv::Mat&, cv::Vec3f&)
{

//fftw_complex *workspace1, *workspace2  ;






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
            //std::cout<<"l : "<<l<<"m : "<<m<<" coeff : "<<rcoeffs[dummy]<<" "<<icoeffs[dummy]<<" I"<<std::endl;
            tmpOrd.real(rcoeffs[dummy]);
            tmpOrd.imag(icoeffs[dummy]);

            tmpDeg.push_back(tmpOrd);
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
