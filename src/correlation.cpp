
#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include "picojson.h"

using namespace std;

void init(string filename, vector< vector<double> > *thetas_frame)
{

  vector< vector<double> > thetas_array;
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
  picojson::object& all = val.get<picojson::object>();
  picojson::array& array = all["data"].get<picojson::array>();

  for(picojson::array::iterator it = array.begin(); 
      it != array.end(); ++it)
    {
      vector<double> thetas;
      picojson::array array_in = it->get<picojson::array>();
      for(picojson::array::iterator it_in = array_in.begin(); 
	  it_in != array_in.end(); ++it_in)
	{
	  thetas.push_back(it_in->get<double>());
	}
      thetas_array.push_back(thetas);
    }

  for(int i=0; i < thetas_array[0].size(); ++i)
    {
      vector<double> thetas_t;
      for(int j=0; j<thetas_array.size(); ++j)
	{
	  thetas_t.push_back(thetas_array[j][i]);
	}
      thetas_frame->push_back(thetas_t);
    }
}


void cor_calc(vector< vector<double> > datas, double *r)
{
  vector< double > data_aves;
  for(int i=0; i<datas[0].size(); ++i)
    {
      double sum = 0;
      for(int j=0; j<datas.size(); ++j)
	{
	  sum += datas[j][i];
	  //cout << datas[j][i] << ", ";
	}
      data_aves.push_back(sum/datas.size());

      //cout << endl << "size: "<< datas.size() << ", aves "<<i<<": " << data_aves[i] << endl;
    }

  double sigma1sigma2 = 0, sigma1sq = 0, sigma2sq = 0;
  for(int i=0; i<datas.size(); ++i)
    {
      double sigma1 = datas[i][0]-data_aves[0];
      double sigma2 = datas[i][1]-data_aves[1];

      sigma1sigma2 += sigma1*sigma2;
      sigma1sq += sigma1*sigma1;
      sigma2sq += sigma2*sigma2;
    }
  *r = sigma1sigma2 / (sqrt(sigma1sq)*sqrt(sigma2sq));  
}


int main(int argc, char** argv)
{
  vector< vector<double> > thetas_frame;
  vector< vector<double> > thetas_all;
  vector< vector<double> > thetas_part;
  vector< vector<double> > cor_mat;
  string filename;
  filename="output.json";
  int start_frame = 0, winsize = 30;
  int joint_index = 0;

  //時間毎に保持された各関節角度を、
  //各関節毎に保持された時間的な変化のある関節角度に変換する
  init(filename, &thetas_all);

  /*
  for(int i=0; i<thetas_frame.size(); ++i)
    {
      cout << i <<": ";
      for(int j=0; j<thetas_frame[i].size(); ++j)
	{
	  cout << thetas_frame[i][j] << ", ";
	}
      cout << endl;
    }
  */

  cout << "data_size (theta: " << thetas_all.size() 
       << ", frame: " << thetas_all[0].size() << ")"<< endl; 
  cout << "range: "<< thetas_all[0].size()-winsize << endl;
  vector< vector< vector<double> > > r_mat3;

  for(int i=0; i<thetas_all.size(); ++i)
    {  
      vector< vector<double> > r_mat2;
      for(int j=0; j<thetas_all.size(); ++j)
	{
	  cout << "("<< i << ", "<< j << "): "<< endl;
	  int range = thetas_all[j].size()-winsize;

	  //ウィンドウサイズがデータサイズを越えていた場合、計算不可
	  if(range < 0)
	    {
	      cerr << "Error: window_size over all data size!" << endl;
	      return 0;
	    }

	  //dtは遅れ
	  vector< double > r_vec;
	  for(int dt= -range; dt <= range; ++dt)
	    {
	      //相関係数
	      double r_max=0, r_tmp, st_max=0;
	      //stは開始の時点
	      for(int st=0; st <= thetas_all[j].size()-abs(dt)-winsize; ++st)
		{
		  vector< vector<double> > sets;
		  for(int idx=0; idx<winsize; ++idx)
		    {
		      vector<double> set;
		      int i_idx, j_idx;
		      if(dt < 0)
			{
			  i_idx = st+idx;
			  j_idx = st+dt+idx;
			}
		      else if(dt >= 0)
			{
			  i_idx = st+dt+idx;
			  j_idx = st+idx;
			}
		      set.push_back(thetas_all[i][i_idx]);
		      set.push_back(thetas_all[j][j_idx]);
		      sets.push_back(set);
		    }
		  cor_calc(sets, &r_tmp);
		  if(r_max < r_tmp)
		    {
		      r_max = r_tmp;
		      st_max = st;
		    }
		}
	      //cout << "max_st: "<< thetas_all[j].size()-winsize-abs(dt) << endl;
	      if(i==j)
		cout <<"dt: "<< dt << ", st:"<< st_max << ", r:"<< r_max << endl;
	      r_vec.push_back(r_max);
	    }
	  // r_mat2.push_back(r_vec);
	}
      //r_mat3.push_back(r_mat2);
    }
}
