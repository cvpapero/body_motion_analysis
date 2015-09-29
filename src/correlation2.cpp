/*
同時期にとった関節角度のデータファイルを二つ読み込み
それらの相関をとる

やらないといけないのは
1.同時に二人の人物の関節角度をとり、jsonファイルに出力するモジュール
2.相関係数を可視化する。どの関節とどの関節が、どれぐらいの遅れの時相関が強いか
3.実際の時刻情報の追加(前の関節角度が取得された時から今の関節角度が取得された時の時間)
*/


#include <iostream>
#include <string>
#include <fstream>
#include <sstream>
#include "picojson.h"

using namespace std;

//データの取得、データ構造の変換
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

//相関係数の計算
void cor_calc(vector< vector<double> > datas, double *r)
{
  vector< double > data_aves;
  for(int i=0; i<datas[0].size(); ++i)
    {
      double sum = 0;
      for(int j=0; j<datas.size(); ++j)
	{
	  sum += datas[j][i];
	}
      data_aves.push_back(sum/datas.size());
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


void process(int range, int winsize, double threshold, 
	     vector<double> d1, vector<double> d2, vector< double > *r_vec, 
	     int i_index, int j_index)
{
  //dtは遅れ
  for(int dt= -range; dt <= range; ++dt)
    {
      //相関係数
      double r_max=0, st_max=0;
      //stは開始の時点
      for(int st=0; st <= d1.size()-abs(dt)-winsize; ++st)
	{
	  vector< vector<double> > sets;
	  for(int idx=0; idx<winsize; ++idx)
	    {
	      vector<double> set;
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

	      set.push_back(d1[i_idx]);
	      set.push_back(d2[j_idx]);
	      sets.push_back(set);
	    }
	  double r_tmp;
	  cor_calc(sets, &r_tmp);
	  if(i_index == j_index && dt == 0)
	    cout <<"("<<i_index<<", "<<j_index << ") dt:"<< dt << ", st:" << st << ", r_tmp:" << r_tmp <<endl;

	  if(fabs(r_max) <= fabs(r_tmp))
	    {
	      r_max = r_tmp;
	      st_max = st;
	    }
	}
      if(fabs(r_max) > threshold) 
	cout <<"("<<i_index<<", "<<j_index << ") dt:"<< dt << ", st:"<< st_max << ", r:"<< r_max << endl;
      
      r_vec->push_back(r_max);
    } 
}


int main(int argc, char** argv)
{
  vector< vector<double> > thetas_frame;
  vector< vector<double> > thetas_all;

  vector< vector<double> > data1;
  vector< vector<double> > data2;
  string file1, file2;
  file1="output.json";
  file2="output.json";
  int winsize = 30;
  double threshold = 0.7;
  /*
  if(argc > 1)
    {
      
    }
  */

  init(file1, &data1);
  init(file2, &data2);

  cout << "data_size (num: " << data1.size() 
       << ", frame: " << data1[0].size() << ")"<< endl; 
  cout << "window_size: "<< winsize <<", threshold: " << threshold << endl;

  if(data1.size() != data2.size())
    {
      cout << "miss match! file1 size:"<<data1.size()<<", file2 size:"<<data2.size()<<endl;
      return 0; 
    }

  for(int i=0; i<data1.size(); ++i)
    {  
      for(int j=0; j<data2.size(); ++j)
	{
	  //cout << "("<< i << ", "<< j << "): "<< endl;
	  int range = data2[j].size()-winsize;

	  //ウィンドウサイズがデータサイズを越えていた場合、計算不可
	  if(range < 0)
	    {
	      cerr << "Error: window_size over all data size!" << endl;
	      return 0;
	    }
	  vector< double > r_vec;
	  process(range, winsize, threshold, data1[i], data2[j], &r_vec, i, j);
	}
    }
}
