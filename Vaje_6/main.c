#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>
#include <unistd.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <time.h>


#define N 2048
#define TILE 512

//#define NAIVE
//#define LOOP_EXCHANGE
//#define LOOP_TILING
#define LOOP_TILING_EXCHG

float* A;
float* B;
float* C;


clock_t start;
clock_t end;
double duration;



int main(int argc, char **argv)
{
    A = (float *)malloc(sizeof(float) * N * N);
    B = (float *)malloc(sizeof(float) * N * N);
    C = (float *)malloc(sizeof(float) * N * N);

    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            A[i*N + j] = 1.0;
            B[i*N + j] = 2.0;
            C[i*N + j] = 0.0;
        }
        
    }

#ifdef NAIVE
    //*********************************************
    // NAIVE 
    //*********************************************
    start = clock();
    for (int i = 0; i < N; i++)
    {
        for (int j = 0; j < N; j++)
        {
            for (int k = 0; k < N; k++)
            {
                C[i*N + j] += A[i*N + k] * B[k*N + j];
            }
            
        } 
    }
    end = clock();
    duration = ((double)(end-start)) / (CLOCKS_PER_SEC);
    printf("Naive time (s): %f \n", duration);
#endif



#ifdef LOOP_EXCHANGE
    //*********************************************
    // LOOP EXCHANGE 
    //*********************************************
    start = clock();
    for (int i = 0; i < N; i++)
    {
        for (int k = 0; k < N; k++)
        {
            for (int j = 0; j < N; j++)
            {
                C[i*N + j] += A[i*N + k] * B[k*N + j];
            }
            
        } 
    }
    end = clock();
    duration = ((double)(end-start)) / (CLOCKS_PER_SEC);
    printf("Loop exchange time (s): %f \n", duration);
#endif



#ifdef LOOP_TILING
    //*********************************************
    // LOOP TILING
    //*********************************************
    start = clock();
    for (int i = 0; i < N; i+=TILE)
    {
        for (int j = 0; j < N; j+=TILE)
        {
            for (int k = 0; k < N; k+=TILE)
            {
                for (int ii = i; ii < i+TILE; ii++)
                {
                    for (int jj = j; jj < j+TILE; jj++)
                    {
                        for (int kk = k; kk < k+TILE; kk++)
                        {
                            C[(ii)*N + (jj)] += A[(ii)*N + (kk)] * B[(kk)*N + (jj)];
                        }
                    }
                }
            }
            
        } 
    }
    end = clock();
    duration = ((double)(end-start)) / (CLOCKS_PER_SEC);
    printf("Loop tiling time (s): %f \n", duration);
#endif


#ifdef LOOP_TILING_EXCHG
    //*********************************************
    // LOOP TILING + LOOP EXCHANGE
    //*********************************************
    start = clock();
    for (int i = 0; i < N; i+=TILE)
    {
        for (int j = 0; j < N; j+=TILE)
        {
            for (int k = 0; k < N; k+=TILE)
            {
                for (int ii = i; ii < i+TILE; ii++)
                {
                    for (int kk = k; kk < k+TILE; kk++)
                    {
                        for (int jj = j; jj < j+TILE; jj++)
                        {
                            C[(ii)*N + (jj)] += A[(ii)*N + (kk)] * B[(kk)*N + (jj)];
                        }
                    }
                }
            }
            
        } 
    }
    end = clock();
    duration = ((double)(end-start)) / (CLOCKS_PER_SEC);
    printf("Loop tiling + loop exchange time (s): %f \n", duration);
#endif

}