#include<iostream>
#include<vector>
#include <stdio.h>
#include<string>
#include "thread_pool.h";

using namespace std;

class DataSetFloder {
public :
	//IPS EXE abs path,need to support Batch process
	string m_TaskExe;
	//批处理一级目录
	string m_DataSetPath;
	//是否指定处理的子数据集
	//bool ProcessSubDataSet;
	//指定子数据集名称
	vector<string> m_SubDataSetFolderName = {};
	//指定设备、测站名称
	vector<string> m_Equip = {};
	//指定设备编号
	vector<string> m_EquipNum = {};
	//处理类型，0,1,2,3
	//int ProcessTyple;
	//单独optPath
	string m_SingleOptPath ;

	//目标opt文件所在文件夹
	string m_OptFolderName = "opt";
	string m_OptFileSuffix = "opt";
	int runOneBatch(int ProcessTyple);
	//最大线程数
	int m_MaxThreadOnce = 10;
private:
	//opt文件绝对路径
	vector<string> m_Optfiles;
	//获取Opt文件
	//void getFiles(string path, vector<string>& files, vector<string>& Equip, vector<string>& EqiuipNum);
	void getFiles(string path,int layer);
	void Stringsplit(const string& str, const string& splits, vector<string>& res);
	int runSingleOptPathLinear();
	int runSingleOptPathPoolThread();
	int runLinearProcess();
	int runPoolThreadProcess();
	
	//ThreadPool pool;
	int m_ProcNum = 0;

};
