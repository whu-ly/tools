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
	// 批处理模式：支持数据批量转换(TaskType = 1)和解算(TaskType = 2)
	// ProcType = 1："待处理文件"路径保存为文件形式线性处理，	|--------单线程------|
	// ProcType = 2："待处理文件"路径保存为文件形式多线程处理，	|--------多线程------|
	// ProcType = 3：面向数据集搜索"待处理文件"的线性处理，		|------单线程模式-----|
	// ProcType = 4: 面向数据集搜索"待处理文件"的多线程处理，	|------多线程模式-----|
	// 单线程：目标文件逐个处理
	// 多线程：目标文件并行处理，同时处理线程《存在上限》，最大线程数设置太大有风险！！!
	int TaskType = 2, ProcType = 4;

	DataSetFloder DataSet; // 批处理类

	//------------ProcType< 1，3 >----------------//
	DataSet.m_MaxThreadOnce = 6;	// 单次运行最大线程任务，视电脑而定

	//------------ProcType< 1，2 >----------------//
	DataSet.m_SingleOptPath = "";	// 汇总文件路径

	//------------ProcType< 3，4 >----------------//
	DataSet.m_DataSetPath = "I:\\202410@CDTH@CropperGNSS@TH1100"; // 需要批处理的数据集一级目录
	DataSet.m_Equip = { "TH1100" }; // 指定设备类型，用逗号分割，不指定置为空
	DataSet.m_SubDataSetFolderName = { "20250619@THAM03@LG290P@QXVRS@ThreeWalls" }; // 指定子集，用逗号分割，不指定置为空

	if (TaskType == 1)
	{
		DataSet.m_TaskExe = "IPSConverter_20250702.exe"; // 可执行程序(.exe)位置
		DataSet.m_OptFolderName = "raw"; // "待处理文件"所在的文件夹，仅ProcType = 3/4需要
		DataSet.m_OptFileSuffix = "rtcm3"; // "待处理文件"后缀，仅ProcType = 3/4需要
	}
	else if (TaskType == 2)
	{
		DataSet.m_TaskExe = "20250702_POSRTKDLLV341.exe"; // 可执行程序(.exe)位置
		DataSet.m_OptFolderName = "opt"; // "待处理文件"所在的文件夹，仅ProcType = 3/4需要
		DataSet.m_OptFileSuffix = "opt"; // "待处理文件"后缀，仅ProcType = 3/4需要
	}
	else
	{
		return 0;
	}

	//运行
	DataSet.runOneBatch(ProcType);

	return 0;
}
