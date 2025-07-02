#include<iostream>
#include<vector>
#include <stdio.h>
#include<string>
#include "DataSetFloder.h"
#include<fstream>  //ifstream
#include <io.h>
#include <stdlib.h>
#include<Windows.h>
//#include "thread_pool.h"
#include <thread>
#include<algorithm>

using namespace std;

// �̺߳���������ִ��CMD����
void executeCmdCommand(const std::string& command) {
	int result = system(command.c_str());
	printf("%s\n", command.c_str());
}
int DataSetFloder::runOneBatch(int ProcessTyple)
{
	switch (ProcessTyple)
	{
	case 1:
		//Opt·������Ϊ�ļ���ʽ���Դ���
		runSingleOptPathLinear();
			break;
	case 2:
		//Opt·������Ϊ�ļ���ʽ���̴߳���
		runSingleOptPathPoolThread();
		break;
	case 3:
		//�������ݼ�����opt�ļ������Դ���
		runLinearProcess();
		break;
	case 4:
		//�������ݼ�����opt�ļ��Ķ��̴߳���
		runPoolThreadProcess();
		break;
		default:
			return 0;
		break;
	}

}
int DataSetFloder::runSingleOptPathLinear()
{
	vector<string> files;
	char line[1024] = { '\0' };
	char cmds[500];
	FILE* fpr = fopen(m_SingleOptPath.c_str(), "r");
	if (!fpr)
	{
		printf("Fail to open MultiProcTest.opt.\n");
		return 0;
	}
	while (fgets(line, 1024, fpr))
	{
		files.push_back(line);

	}
	fclose(fpr);



	for (int i = 0; i < files.size(); i++)
	{
		if (files[i] != "end")
		{
			sprintf_s(cmds, "%s %s", m_TaskExe.c_str(), files[i].c_str()); // ����cmd����
			printf("Data processing......\n");
			printf("%s", files[i].c_str());
			int res = system(cmds);
		}
		else break;
	}
	
	
	return 0;
}

int DataSetFloder::runSingleOptPathPoolThread()
{
	vector<string> files;
	char line[1024] = { '\0' };
	char cmds[500];
	vector<string> DataCmds;
	FILE* fpr = fopen(m_SingleOptPath.c_str(), "r");
	if (!fpr)
	{
		printf("Fail to open MultiProcTest.opt.\n");
		return 0;
	}
	while (fgets(line, 1024, fpr))
	{
		files.push_back(line);

	}
	fclose(fpr);

	for (int i = 0; i < files.size(); i++)
	{
		if (files[i] != "end")
		{
			sprintf_s(cmds, "%s %s", m_TaskExe.c_str(), files[i].c_str()); // ����cmd����
			DataCmds.push_back(cmds);
			m_ProcNum++;
		}
		else break;
	}

	if (m_MaxThreadOnce < 1)
	{
		printf("�����߳���������%d\n", m_MaxThreadOnce);
		return 0;
	}
	if (m_ProcNum < 1)
	{
		printf("����������������opt����Ϊ��%d\n", m_ProcNum);
		return 0;
	}
	printf("׼����%d�����ݽ��д�����������߳���Ϊ:%d \n", m_ProcNum, m_MaxThreadOnce);

	ThreadPool pool(m_MaxThreadOnce);

	// ����n������
	for (int i = 0; i < DataCmds.size(); ++i) {

		pool.enqueue(std::bind(executeCmdCommand, DataCmds[i]));
	}
	m_Optfiles.clear();
	DataCmds.clear();
	// �ȴ������������
	this_thread::sleep_for(chrono::seconds(20));
}

int DataSetFloder::runLinearProcess()
{
	char cmds[500];
	m_ProcNum = 0;
	//getAimOptPath();
	getFiles(m_DataSetPath, 1);
	for (int i = 0; i < m_Optfiles.size(); i++)
	{
		//����cmd����
		sprintf_s(cmds, "%s %s", m_TaskExe.c_str(), m_Optfiles[i].c_str());
		printf("���ڽ��㡭������\n");
		printf("%s\n", m_Optfiles[i].c_str());
		//����cmd����IPSexe
		int res = system(cmds);
		m_ProcNum++;
	}
	m_Optfiles.clear();

	return 0;
}

int DataSetFloder::runPoolThreadProcess()
{
	char cmds[500];
	getFiles(m_DataSetPath, 1);
	m_ProcNum =0;
	vector<string> DataCmds;
	//2.����ÿ�������ݼ�opt�ļ��л�ȡopt�ļ���·��

	for (int j = 0; j < m_Optfiles.size(); j++)
	{
		//����cmd����
		sprintf_s(cmds, "%s %s", m_TaskExe.c_str(), m_Optfiles[j].c_str());
		DataCmds.push_back(cmds);
		m_ProcNum++;
	}
		
	if (m_MaxThreadOnce < 1)
	{
		printf("�����߳���������%d\n", m_MaxThreadOnce);
		return 0;
	}
	if (m_ProcNum < 1)
	{
		printf("����������������opt����Ϊ��%d\n", m_ProcNum);
		return 0;
	}
	printf("׼����%d�����ݽ��д�����������߳���Ϊ:%d \n", m_ProcNum, m_MaxThreadOnce);

	ThreadPool pool(m_MaxThreadOnce);

	// ����n������
	for (int i = 0; i < DataCmds.size(); ++i) {

		pool.enqueue(std::bind(executeCmdCommand, DataCmds[i]));
	}
	m_Optfiles.clear();
	DataCmds.clear();
	// �ȴ������������
	this_thread::sleep_for(chrono::seconds(20));
}


//Ѱ�����ݼ�Opt�ļ�
//void DataSetFloder::getFiles(string path, vector<string>& files, vector<string>& Equip, vector<string>& EqiuipNum)
void DataSetFloder::getFiles(string path,  int layer)
{
	//�ļ����
	intptr_t hFile = 0;
	//�ļ���Ϣ
	struct _finddata_t fileinfo;
	string p;
	bool EquipMatchFlag = false;
	bool EquipNumMatchFlag = false;
	bool SubDataSetMatchFlag = false;
	if ((hFile = _findfirst(p.assign(path).append("\\*").c_str(), &fileinfo)) != -1)
	{
		do
		{

			//�����Ŀ¼,����֮
			//�������,�����б�
			if ((fileinfo.attrib & _A_SUBDIR))
			{
				//���뵥�����ݼ��ڣ�ǰ��Ŀ��opt�ļ���Ѱ��opt�ļ�
				if (layer == 2)
				{
					if (fileinfo.name != m_OptFolderName)
						continue;
				}
				SubDataSetMatchFlag = true;
				//ƥ��Ŀ���Ӽ�
				if (m_SubDataSetFolderName.size() > 0 && layer == 1)
				{
					SubDataSetMatchFlag = false;
					for (int i = 0; i < m_SubDataSetFolderName.size(); i++)
					{
						string FloderName = string(fileinfo.name);
						if (m_SubDataSetFolderName[i] == FloderName)
						{
							SubDataSetMatchFlag = true;
							break;
						}
					}
					
				}
				if (!SubDataSetMatchFlag)
					continue;
				if (strcmp(fileinfo.name, ".") != 0 && strcmp(fileinfo.name, "..") != 0)
					getFiles(p.assign(path).append("\\").append(fileinfo.name),2);
			}
			else
			{
				vector<string> strListFileTyple;
				vector<string> OptInfor;
				EquipNumMatchFlag = false;
				EquipMatchFlag = false;
				//string str2("This-is-a-test");
				Stringsplit(fileinfo.name, ".", strListFileTyple);
				if (strListFileTyple.size() > 1 && strListFileTyple[1] == m_OptFileSuffix)
				{
					Stringsplit(strListFileTyple[0], "_", OptInfor);
					//opt��Ҫ�������ݼ��淶
					if (OptInfor.size() == 5)
					{
						//ƥ���豸����
						if (m_Equip.size() > 0)
						{
							for (int i = 0; i < m_Equip.size(); i++)
							{
								if (m_Equip[i] == OptInfor[3])
								{
									EquipMatchFlag = true;
									break;
								}
							}
						}
						else
							EquipMatchFlag = true;
						//ƥ���豸���
						if (m_EquipNum.size() > 0)
						{
							for (int i = 0; i < m_EquipNum.size(); i++)
							{
								if (m_EquipNum[i] == OptInfor[4])
								{
									EquipNumMatchFlag = true;
									break;
								}
							}
						}
						else
							EquipNumMatchFlag = true;


						if (EquipNumMatchFlag && EquipMatchFlag)
							m_Optfiles.push_back(p.assign(path).append("\\").append(fileinfo.name));
					}

				}



				strListFileTyple.clear();
				OptInfor.clear();
			}
		} while (_findnext(hFile, &fileinfo) == 0);
		_findclose(hFile);
	}
}

void DataSetFloder::Stringsplit(const string& str, const string& splits, vector<string>& res)
{
	if (str == "")		return;
	//���ַ���ĩβҲ����ָ����������ȡ���һ��
	string strs = str + splits;
	size_t pos = strs.find(splits);
	int step = splits.size();

	// ���Ҳ����������ַ��������������� npos
	while (pos != strs.npos)
	{
		string temp = strs.substr(0, pos);
		res.push_back(temp);
		//ȥ���ѷָ���ַ���,��ʣ�µ��ַ����н��зָ�
		strs = strs.substr(pos + step, strs.size());
		pos = strs.find(splits);
	}
}


	