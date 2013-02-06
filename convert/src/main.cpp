#include <iostream>
#include <fstream>
#include <string>
#include <limits>
#include <iomanip>

#include "GCSS3DLib.h"

int main(int argc, char *argv[])
{
  if (argc != 4) {
    std::string exe_filename(argv[0]);
    size_t pos = exe_filename.find_last_of('/');
    if (pos != std::string::npos) {
      exe_filename = exe_filename.substr(pos+1);
    }
    std::cout << "[EvoGeoConvert-YY]-Usage: " << exe_filename << " image_folder ctr_threshold sat_threshold" << std::endl;
    return 1;
  }

  std::string folder(argv[1]);
  folder += "/";
  int ctr_threshold = atoi(argv[2]);
  int sat_threshold = atoi(argv[3]);

  int num_points = 0;
  std::cout << "Decoding images in folder [" << folder << "]"
    << " with ctr_threshold=" << ctr_threshold
    << " and sat_threshold=" << sat_threshold << "...";
  num_points = DecodeImgs(CString(folder.c_str()), ctr_threshold, sat_threshold);
  std::cout << "Done with " << num_points << " points." << std::endl;

  if(num_points == 0)
	  return 1;

  double **points ;
  points = GetXC();

  double *image_x;
  double *image_y;
  image_x = GetImg_x();
  image_y = GetImg_y();

  std::string filename = folder+"points.bxyzuv";
  std::cout << "Saving points to " << filename << "...";
  std::ofstream fout(filename.c_str(), std::ios::binary);


  for (int i = 0; i < num_points; ++ i)
  {
	  for(int j = 0; j < 3; ++ j)
	  {
		  double value = points[j][i];
		  fout.write((const char*)(&value), sizeof(double));
	  }
	  double u = image_x[i];
	  double v = image_y[i];
	  fout.write((const char*)(&u), sizeof(double));
	  fout.write((const char*)(&v), sizeof(double));
  }
  std::cout << "Done." << std::endl;

  delete[] points[0];
  delete[] points[1];
  delete[] points[2];
  delete[] image_x;
  delete[] image_y;

  fout.close();

  return 0;
}
