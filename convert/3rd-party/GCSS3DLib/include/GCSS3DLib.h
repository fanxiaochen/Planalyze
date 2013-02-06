#include <afx.h>


int DecodeImgs(CString filepath, int Threshold_Contrast, int Threshold_Saturation);
 
unsigned char** Create2DArrayChar(int h, int w);

double** Create2DArrayDouble(int h, int w);

int ** Create2DArrayInt(int h, int w);

void CreatBMPfromArrayInt(unsigned char *lpimg, int **array, int imgW, int imgH, double scale);

void CreatBMPfromArrayDouble(unsigned char *lpimg, double **array, int imgW, int imgH, double scale);

void ReleaseMemory();

void SaveData(CString savefilename, bool b_outputXC, bool outputXP);

void SaveData_Img(CString savefilename);

double **GetXC();

double **GetXP();

double *GetImg_x();

double *GetImg_y();