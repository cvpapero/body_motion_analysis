/*
2015.10.1---
windowの位置は最新に固定する

自己相関の分母が小さい＝変化がない場合はハネる
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

double mean(vector<double> data)
{
  double sum = 0;
  
  for(int j=0; j<data.size(); ++j)
    {
      sum += data[j];
    }

  return sum/data.size();
}


//相関係数の計算
void corr_calc(vector<double> d1, vector<double> d2, double *r, bool denom)
{
  double d1_ave = mean(d1);
  double d2_ave = mean(d2);

  //cout<< "ave d1, d2:"<<d1_ave<<", "<<d2_ave<<", size:"<<d1.size()<<endl;

  double sigma1sigma2 = 0, sigma1sq = 0, sigma2sq = 0;
  for(int i=0; i<d1.size(); ++i)
    {
      double sigma1 = d1[i] - d1_ave;
      double sigma2 = d2[i] - d2_ave;

      sigma1sigma2 += sigma1*sigma2;
      sigma1sq += sigma1*sigma1;
      sigma2sq += sigma2*sigma2;
    }

  if(denom)
    *r = sigma1sigma2 / (sqrt(sigma1sq)*sqrt(sigma2sq));  
  else
    *r = sqrt(sigma1sq)*sqrt(sigma2sq);  
  //cout<< "sqrt:" <<sqrt(sigma1sq)*sqrt(sigma2sq) <<endl;
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
      
      vector< vector<double> > sets;
      vector<double> set1;
      vector<double> set2;
      
      //int s_j_idx;
      for(int idx=0; idx<winsize; ++idx)
	{
	  int d1_idx, d2_idx;
	  if(dt < 0)
	    {
	      d1_idx = d1.size()-winsize-abs(dt);
	      d2_idx = d2.size()-winsize;
	    }
	  else if(dt >= 0)
	    {
	      d1_idx = d1.size()-winsize;
	      d2_idx = d2.size()-winsize-abs(dt);
	    }
	 	  
	  set1.push_back(d1[d1_idx+idx]);
	  set2.push_back(d2[d2_idx+idx]);
	}
      //sets.push_back(set1);
      //sets.push_back(set2);
      //bool denom;

      double corr;

      //自己相関
      corr_calc(set1, set1, &corr, false);
      if(corr < 0.01)
	{
	  // cout << "denom:"<<corr<<endl;
	  continue;
	}

     //自己相関
      corr_calc(set2, set2, &corr, false);
      if(corr < 0.01)
	{
	  //cout << "denom:"<<corr<<endl;
	  continue;
	}

      corr_calc(set1, set2, &corr, true);
           
      if(fabs(corr) > threshold) 
	{
	  double delay_time;
	  if(dt < 0)
	    {
	      delay_time = times[0] - times[abs(dt)];
	    }
	  else
	    {
	      delay_time = times[abs(dt)]-times[0];
	    }	  	  
	  
	  printf("(%2d, %2d) dt:(frame:%5d, time:%4.4f), r: %2.7f\n", i_index+1, j_index+1, dt, delay_time, corr);	  
	}      
      r_vec->push_back(corr);
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
  cout << "time:" << times.back() - times.front() <<", threshold: " << threshold << endl;


  cout << "window_size: "<< winsize<<endl;

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
	  //cout << "("<< i+1 << ", "<< j+1 << "): "<< endl;
	  //cout << "range:"<< range << endl; 
	  vector< double > r_vec;
	  process(winsize, threshold, data[0][i], data[1][j], times, &r_vec, i, j);
	}
    }

  clock_t end_clock = clock();
  cout << "process:" << (double)(end_clock - start_clock) / CLOCKS_PER_SEC << "[sec]" << endl;
}
