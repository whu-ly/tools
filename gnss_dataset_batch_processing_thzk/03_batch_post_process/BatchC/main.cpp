#include<iostream>
#include<Windows.h>
#include<vector>
#include <stdio.h>
#include<algorithm>
#include <io.h>
#include <stdlib.h>
#include<fstream>
#include<string>
#include<cmath>
#include <thread>
#include "DataSetFloder.h";
using namespace std;

clock_t ls_start, ls_end;
int main()
{
	// ������ģʽ��֧����������ת��(TaskType = 1)�ͽ���(TaskType = 2)
	// ProcType = 1��"�������ļ�"·������Ϊ�ļ���ʽ���Դ���	|--------���߳�------|
	// ProcType = 2��"�������ļ�"·������Ϊ�ļ���ʽ���̴߳���	|--------���߳�------|
	// ProcType = 3���������ݼ�����"�������ļ�"�����Դ���		|------���߳�ģʽ-----|
	// ProcType = 4: �������ݼ�����"�������ļ�"�Ķ��̴߳���	|------���߳�ģʽ-----|
	// ���̣߳�Ŀ���ļ��������
	// ���̣߳�Ŀ���ļ����д���ͬʱ�����̡߳��������ޡ�������߳�������̫���з��գ���!
	int TaskType = 2, ProcType = 4;

	DataSetFloder DataSet; // ��������

	//------------ProcType< 1��3 >----------------//
	DataSet.m_MaxThreadOnce = 6;	// ������������߳������ӵ��Զ���

	//------------ProcType< 1��2 >----------------//
	DataSet.m_SingleOptPath = "";	// �����ļ�·��

	//------------ProcType< 3��4 >----------------//
	DataSet.m_DataSetPath = "I:\\202410@CDTH@CropperGNSS@TH1100"; // ��Ҫ����������ݼ�һ��Ŀ¼
	DataSet.m_Equip = { "TH1100" }; // ָ���豸���ͣ��ö��ŷָ��ָ����Ϊ��
	DataSet.m_SubDataSetFolderName = { "20250619@THAM03@LG290P@QXVRS@ThreeWalls" }; // ָ���Ӽ����ö��ŷָ��ָ����Ϊ��

	if (TaskType == 1)
	{
		DataSet.m_TaskExe = "IPSConverter_20250702.exe"; // ��ִ�г���(.exe)λ��
		DataSet.m_OptFolderName = "raw"; // "�������ļ�"���ڵ��ļ��У���ProcType = 3/4��Ҫ
		DataSet.m_OptFileSuffix = "rtcm3"; // "�������ļ�"��׺����ProcType = 3/4��Ҫ
	}
	else if (TaskType == 2)
	{
		DataSet.m_TaskExe = "20250702_POSRTKDLLV341.exe"; // ��ִ�г���(.exe)λ��
		DataSet.m_OptFolderName = "opt"; // "�������ļ�"���ڵ��ļ��У���ProcType = 3/4��Ҫ
		DataSet.m_OptFileSuffix = "opt"; // "�������ļ�"��׺����ProcType = 3/4��Ҫ
	}
	else
	{
		return 0;
	}

	//����
	DataSet.runOneBatch(ProcType);

	return 0;
}
