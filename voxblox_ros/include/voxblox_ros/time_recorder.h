#ifndef TIME_RECORDER_H
#define TIME_RECORDER_H

#include <iostream>
#include <chrono>
#include <fstream>
#include <string>
using namespace std;
class tm_rcd
{
public:
  tm_rcd(const string &_out_path):out_path(_out_path)
  {
    out_file.open(out_path);
  }
  ~tm_rcd()
  {
    out_file.close();
  }

  void record(float duration)
  {
    out_file<<duration<<endl;
  }



public:
  string out_path;
  ofstream out_file;
//  std::chrono::time_point<std::chrono::steady_clock> start, end;
};


#endif // TIME_RECORDER_H
