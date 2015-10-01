/*
2015.9.29---

*/


#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include "picojson.h"

#include <time.h> 

using namespace std;




//データの取得、データ構造の変換
void init(string filename, vector< vector< vector<double> > > *data_frame, vector< double > *times)
{

  ifstream ifs(filename.c_str());
  if( ifs.fail() )
    {
      cerr << "Error: Input file not found!" << endl;
      return;
    }
  stringstream ss;
  string thisline;
  
  while(getline(ifs, thisline))
    {
      ss << thisline;
    }
  
  picojson::value val;
  picojson::parse(val, ss);
  picojson::array& all = val.get<picojson::array>();

  
  for(picojson::array::iterator it = all.begin(); 
      it != all.end(); ++it)
    {
      picojson::object& datas = it->get<picojson::object>();
      picojson::array& datas_array = datas["datas"].get<picojson::array>();

      vector< vector<double> > thetas;
      for(picojson::array::iterator it2 = datas_array.begin(); 
	  it2 != datas_array.end(); ++it2)
	{
	  picojson::object& data = it2->get<picojson::object>();
	  picojson::array& data_array = data["data"].get<picojson::array>();

	  vector<double> theta;
	  for(picojson::array::iterator it3 = data_array.begin(); 
	      it3 != data_array.end(); ++it3)
	    {
	      theta.push_back(it3->get<double>());
	    }
	  thetas.push_back(theta);

	  times->push_back(data["time"].get<double>());
	}

      vector< vector<double> > thetas_t;
      for(int i=0; i < thetas[0].size(); ++i)
	{
	  vector<double> theta_t;
	  for(int j=0; j<thetas.size(); ++j)
	    {
	      theta_t.push_back(thetas[j][i]);
	    }
	  thetas_t.push_back(theta_t);
	}
      data_frame->push_back(thetas_t);
    }
}

//相関係数の計算
void cor_calc(vector< vector<double> > datas, double *r)
{
  vector< double > data_aves;
  for(int i=0; i<datas.size(); ++i)
    {
      double sum = 0;
      for(int j=0; j<datas[i].size(); ++j)
	{
	  sum += datas[i][j];
	}
      data_aves.push_back(sum/datas[i].size());
    }

  double sigma1sigma2 = 0, sigma1sq = 0, sigma2sq = 0;
  for(int i=0; i<datas[0].size(); ++i)
    {
      double sigma1 = datas[0][i]-data_aves[0];
      double sigma2 = datas[1][i]-data_aves[1];

      sigma1sigma2 += sigma1*sigma2;
      sigma1sq += sigma1*sigma1;
      sigma2sq += sigma2*sigma2;
    }
  *r = sigma1sigma2 / (sqrt(sigma1sq)*sqrt(sigma2sq));  
}


void process(int winsize, double threshold, 
	     vector<double> d1, vector<double> d2, vector<double> times,
	     vector< double > *r_vec, 
	     int i_index, int j_index)
{

  /*
  for(int ii=0; ii<d1.size(); ++ii)
    cout << "d1[" << ii << "]:"<<d1[ii]<<", ";
  cout << endl;

  for(int ii=0; ii<d2.size(); ++ii)
    cout << "d2[" << ii << "]:"<<d2[ii]<<", ";
  cout << endl;
  */
  int max_range = d1.size() - winsize;
  if(max_range < 0)
    {
      cerr << "Error: window_size over all data size! max_size:" << max_range << endl;
      return;
    }

  //dtは遅れ
  for(int dt= -max_range; dt <= max_range; ++dt)
    {
      //相関係数
      double r_max=0;
      int st_max=0;
      //stは開始の時点
      for(int st=0; st <= d1.size()-abs(dt)-winsize; ++st)
	{
	  vector< vector<double> > sets;
	  vector<double> set1;
	  vector<double> set2;
	  for(int idx=0; idx<winsize; ++idx)
	    {

	      int i_idx, j_idx;
	      if(dt < 0)
		{
		  i_idx = st+idx;
		  j_idx = st+abs(dt)+idx;
		}
	      else if(dt >= 0)
		{
		  i_idx = st+abs(dt)+idx;
		  j_idx = st+idx;
		}

	      //cout << "idx:" << i_idx <<",st:"<<st<< ",dt:" << dt << endl; 
	      //cout << "d1[" << i_idx <<"]:"<< d1[i_idx] <<endl;
	      //cout << "d2[" << j_idx <<"]:"<< d2[j_idx] <<endl; 

	      set1.push_back(d1[i_idx]);
	      set2.push_back(d2[j_idx]);
	    }
	  sets.push_back(set1);
	  sets.push_back(set2);
	  double r_tmp;
	  cor_calc(sets, &r_tmp);
	  if(fabs(r_max) < fabs(r_tmp))
	    {
	      r_max = r_tmp;
	      st_max = st;
	    }
	}
      
      if(fabs(r_max) > threshold) 
	{
	  double delay_time, start_time;
	  if(dt < 0)
	    {
	      delay_time = times[0] - times[abs(dt)];
	      start_time = times[st_max]-times[0];
	    }
	  else
	    {
	      delay_time = times[abs(dt)]-times[0];
	      start_time = times[st_max]-times[abs(dt)];
	    }
	  //double start_time = times[st_max]-times[0];

	  double delay_start_time = times[st_max+abs(dt)]-times[0];

	  /*
	  cout <<"("<<i_index+1<<", "<<j_index+1 
	       << ") dt:(frame:"<< dt << ", time:" << delay_time
	       << " st:(frame:"<< st_max << ", time:" << start_time 
	       << "), r:" << r_max << endl;
	  */
	  printf("(%2d, %2d) dt:(frame:%5d, time:%4.4f), st:(frame:%5d, time:%4.4f), r: %2.7f\n", i_index+1, j_index+1, dt, delay_time, st_max, start_time, r_max);
	  //printf("(%d, %d)  dt:(frame:%d, time:%f), st:(frame:%d, time:%f),", i_index+1, j_index+1, dt, delay_time);
	}
      r_vec->push_back(r_max);
    } 

}


int main(int argc, char** argv)
{
  clock_t start_clock = clock();  

  string file;
  vector< vector< vector<double> > > data;
  vector< double > times; 
  file="outdata2.json";

  int winsize = 20;
  double threshold = 0.7;

  init(file, &data, &times);
 
  cout << "data[0] size (num: " << data[0].size() 
       << ", frame: " << data[0][0].size() << ")"<< endl; 
  cout << "data[1] size (num: " << data[1].size() 
       << ", frame: " << data[1][0].size() << ")"<< endl; 
  cout << "window_size: "<< winsize <<", threshold: " << threshold << endl;

  cout << "time:" << times.back() - times.front() <<endl;

  if(data[0].size() != data[1].size())
    {
      cout << "miss match! data[0] size:"<<data[0].size()<<", data[1] size:"<<data[1].size()<<endl;
      return 0; 
    }
  

  for(int i=0; i<data[0].size(); ++i)
    {  
      for(int j=0; j<data[1].size(); ++j)
	{
	  //jointは1からスタートという表記
	  cout << "("<< i+1 << ", "<< j+1 << "): "<< endl;
	  //int range = data[1][j].size()-winsize;
	  //cout << "range:"<< range << endl; 
	  //ウィンドウサイズがデータサイズを越えていた場合、計算不可

	  vector< double > r_vec;
	  process(winsize, threshold, data[0][i], data[1][j], times, &r_vec, i, j);
	}
    }

  clock_t end_clock = clock();

  cout << "duration:" << (double)(end_clock - start_clock) / CLOCKS_PER_SEC << "[sec]" << endl;
}
