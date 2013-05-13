#include<stdio.h>
#include<math.h>
#include<malloc.h>
#include"Preprocessing.h"
#include"FirstMatching.h"
#include"SecondMatching.h"
#include"SimilarityDegree.h"

int main()
{
	/*Mld formwork={
		{123,345,222,267,331,214,167,189,221,244,288,300,340,270,255,211,134,156,210,250},
		{0},
		20,
		0};
	Mld test={
		{287,351,234,187,209,241,264,308,320,330,260},
		{0},
		11,
		0};*/

	double formwork[21]={261.62557,261.62557,391.99544,391.99544,440,440,391.99544,
		349.22823,349.22823,329.62756,329.62756,293.66477,293.66477,261.62557,391.99544,
		391.99544,349.22823,349.22823,329.62756,329.62756,293.66477};
	double test[7]={339.62756,349.62756,303.66477,309.66477,269.62557,404.99544,
		385.99544,



};
		int fn=21;
		int tn=7;

		double Similarity;


		Similarity=Matching(formwork, test,fn,tn);
		if(Similarity==NOTFOUND)
		{
			printf("Not Found!\n");
			return 1;
		}
		printf("%lf\n",Similarity);

		return 0;

}