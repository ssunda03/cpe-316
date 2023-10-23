#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <math.h>

// this file is to generate the lookup tables for P1 of CPE 316
// as of 20231022 5:28p

typedef enum WAVE_TYPE {
    SINE        = 1,
    RAMP        = 2,
    TRIANGLE    = 3
} WAVE_TYPE;

#define TABLE_SIZE 600
#define VPP 3.0
#define DC_BIAS 1.5
#define PI 3.141592653589793
#define Y 0.1 // y calibration
#define MAX 2.93 // max
#define VREF 3.3 // DAC Vref

void print_table(double* table); // prints all values of the table, output looks like an array, for copy+pasting
void print_table_mv(double* table); // prints all values of the table, output looks like an array, for copy+pasting
void print_mv_table(uint16_t* table); // prints a table_size table of uint16_t values
void print_DAC_table(uint16_t* table); // prints a table_size talbe of uint16_t values in hex

void make_sin_table(double* values); // prints table_size number of points to output in the format of an array
void make_ramp_table(double* values); // prints table_size number of points to output in the format of an array 
void make_tri_table(double* values); // prints table_size number of points to output in the format of an array

void make_mv_table(double* values, uint16_t* mv); // turns the elements of values into ints and puts them in mv
void make_DAC_table(double* values, uint16_t* table); // makes a table of 16 bit DAC values to input to the DAC



int main()
{
    WAVE_TYPE wave = RAMP; /* ---------------- CHANGE THIS TO THE WAVEFORM YOU WANT THE TABLE FROM ---------------- */
    double table[TABLE_SIZE];
    uint16_t mv_table[TABLE_SIZE];
    uint16_t DAC_table[TABLE_SIZE];

    switch(wave)
    {
        case SINE: make_sin_table(table); break;
        case RAMP: make_ramp_table(table); break;
        case TRIANGLE: make_tri_table(table); break;
        default: fprintf(stdout, "idk which wave you want.\n");
    }

    //print_table(table);
    //print_table_mv(table); // doesn't change any tables, just prints

    make_mv_table(table, mv_table);
    //print_mv_table(mv_table);

    make_DAC_table(table, DAC_table);
    print_DAC_table(DAC_table);
    


    return 0;
}


// makes a table of 16 bit DAC values to input to the DAC
void make_DAC_table(double* values, uint16_t* table)
{
    int i;

    for(i = 0; i < TABLE_SIZE; i++)
    {
        table[i] = ( 0x3000 | ( 0xFFF & (uint16_t)(values[i] / VREF * 0xFFF) ) );
    }
}

// turns the elements of values into ints and puts them in mv
void make_mv_table(double* values, uint16_t* mv)
{
    int i;

    for(i = 0; i < TABLE_SIZE; i++)
    {
        mv[i] = (uint16_t)(values[i] * 1000);
    }
}

// prints table_size number of points to output in the format of an array
void make_tri_table(double* values)
{
    int i;

    for(i = 0; i < TABLE_SIZE / 2; i ++)
    {
        values[i] = (double)VPP * i * 2 / TABLE_SIZE;
        values[TABLE_SIZE - i - 1] = values[i];
    }
}

// prints table_size number of points to output in the format of an array 
void make_ramp_table(double* values)
{
    int i;

    for(i = 0; i < TABLE_SIZE; i++)
    {
        values[i] = (double)VPP * i / TABLE_SIZE;
    }
}

// prints table_size number of points to output in the format of an array
void make_sin_table(double values[])
{
    double a = VPP / 2; // amplitude of the wave 
    int i;

    for(i = 0; i < TABLE_SIZE; i++)
    {
        values[i] = a * sin((double)(2 * PI * i / TABLE_SIZE)) + DC_BIAS;
    }
}

// prints all values of the table, output looks like an array, for copy+pasting
void print_table(double* table)
{
    int i;

    fprintf(stdout, "[ ");

    for(i = 0; i < TABLE_SIZE - 1; i++)
    {
        fprintf(stdout, "%.4f, ", table[i]); // prints the value with 4 decimal places
    }

    fprintf(stdout, "%.4f ]\n", table[TABLE_SIZE - 1]);


    fprintf(stdout, "i = 0: %.4f\n", table[0]);
    fprintf(stdout, "i = 1: %.4f\n", table[1]);
    fprintf(stdout, "i = 150: %.4f\n", table[150]);
    fprintf(stdout, "i = 299: %.4f\n", table[299]);
    fprintf(stdout, "i = 300: %.4f\n", table[300]);
    fprintf(stdout, "i = 301: %.4f\n", table[301]);
    fprintf(stdout, "i = 450: %.4f\n", table[450]);
    fprintf(stdout, "i = 599: %.4f\n", table[599]);
}

// prints all values of the table, output looks like an array, for copy+pasting
void print_table_mv(double* table)
{
    int i;

    fprintf(stdout, "[ ");

    for(i = 0; i < TABLE_SIZE - 1; i++)
    {
        fprintf(stdout, "%d, ", (int)(table[i] * 1000)); // prints the value in millivolts
    }

    fprintf(stdout, "%d ]\n", (int)(table[TABLE_SIZE - 1] * 1000));


    fprintf(stdout, "i = 0: %d\n", (int)(table[0] * 1000));
    fprintf(stdout, "i = 1: %d\n", (int)(table[1] * 1000));
    fprintf(stdout, "i = 150: %d\n", (int)(table[150] * 1000));
    fprintf(stdout, "i = 299: %d\n", (int)(table[299] * 1000));
    fprintf(stdout, "i = 300: %d\n", (int)(table[300] * 1000));
    fprintf(stdout, "i = 301: %d\n", (int)(table[301] * 1000));
    fprintf(stdout, "i = 450: %d\n", (int)(table[450] * 1000));
    fprintf(stdout, "i = 599: %d\n", (int)(table[599] * 1000));
}

// prints a table_size table of uint16_t values
void print_mv_table(uint16_t* table)
{
    int i;

    fprintf(stdout, "[ ");
    for(i = 0; i < TABLE_SIZE - 1; i++)
    {
        fprintf(stdout, "%d, ", table[i]);
    }
    fprintf(stdout, "%d ]\n", table[TABLE_SIZE - 1]);
}

// prints a table_size talbe of uint16_t values in hex
void print_DAC_table(uint16_t* table)
{
    int i;

    fprintf(stdout, "[ ");
    for(i = 0; i < TABLE_SIZE - 1; i++)
    {
        fprintf(stdout, "0x%x, ", table[i]);
    }
    fprintf(stdout, "0x%x ]\n", table[TABLE_SIZE - 1]);
}


