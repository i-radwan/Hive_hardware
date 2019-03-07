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
};