#pragma once

class Utils {
public:

    static void swap (int* a, int*b) {
        int c = *a;
        *a = *b;
        *b = c;
    }   

    static void swap (double* a, double*b) {
        double c = *a;
        *a = *b;
        *b = c;
    }

    static int twosToDecimal(int v) {
        int x = log2(v) + 1; 
      
        for (int i = 0; i < x; i++)  
           v = (v ^ (1 << i));  

        return -(v + 1);
    }
};