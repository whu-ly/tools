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
	//������һ��Ŀ¼
	string m_DataSetPath;
	//�Ƿ�ָ������������ݼ�
	//bool ProcessSubDataSet;
	//ָ�������ݼ�����
	vector<string> m_SubDataSetFolderName = {};
	//ָ���豸����վ����
	vector<string> m_Equip = {};
	//ָ���豸���
	vector<string> m_EquipNum = {};
	//�������ͣ�0,1,2,3
	//int ProcessTyple;
	//����optPath
	string m_SingleOptPath ;

	//Ŀ��opt�ļ������ļ���
	string m_OptFolderName = "opt";
	string m_OptFileSuffix = "opt";
	int runOneBatch(int ProcessTyple);
	//����߳���
	int m_MaxThreadOnce = 10;
private:
	//opt�ļ�����·��
	vector<string> m_Optfiles;
	//��ȡOpt�ļ�
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
