function [satData] = getDataFromFile(fileName,sTime,eTime,excSat,bReadCom)
% getDataFromFile 从输入文件中获取satData
% sTime,eTime,截止时间段,0为不截取
% excSat,剔除卫星,如{"G01"}
fp=fopen(fileName);
k=1;
StaIndex=ones(200,1);
while ~feof(fp)
    %前7行获取头文件信息
    if k<=7
        line=fgetl(fp);
        [FreqInforTemp,Index,freqNum]=CheckHeadInfor(line);
        if Index>0
            FreqInfor(Index,1)=FreqInforTemp(1,1);
            FreqInfor(Index,2)=freqNum;
            for j=1:freqNum
                FreqInfor(Index,2+j)=FreqInforTemp(1,1+j);
            end
        end
    else
        line=fgetl(fp);% 读这个 示例：>2272,458709.000,37,null,null,null,
        bodyFirstLine=strsplit(line(2:length(line)),',');% 逗号分割这个 示例：2272,458709.000,37,null,null,null,
        %获取各系统各频点的基准星信息
        if k==8
            StartWeek=str2double(bodyFirstLine{1});% 读取到的观测数据的第一个GPST周
        end
        NowWeek=str2double(bodyFirstLine{1});% 目前读取的观测数据的GPST周
        if NowWeek-StartWeek>0% 如果跨周的话，周内秒+604800秒
            ExpentSecOfWeek=604800;
        else
            ExpentSecOfWeek=0;% 不跨周的话，就保持原GPST周内秒
        end
        secofWeek=str2double(bodyFirstLine{2})+ExpentSecOfWeek;
        nSat=round(str2double(bodyFirstLine{3}));% 该时刻流动站可视卫星数 示例：37
        % 如果不是处理所有时间，就截取需要的时间段
        if sTime>0||eTime>0
            if secofWeek < sTime||secofWeek > eTime %不是所需的时间段就continue
                %如果当前历元不需要读取进去，那么就把数据体读完不保存
                for i=1:nSat
                    line=fgetl(fp);
                end
                continue;
            end
        end
        for i=1:nSat
            line=fgetl(fp);%读取这个，示例：G05 40.62 66.9697 29.9072 0.0000 0.0000 0.0000 0.0000 0.0000 0.0000 0 0.0000 0.0000
            str1=regexp(line,"\s+",'split');
            SatName=string(str1(1));
            %剔除不符合要求的卫星
            if ~isempty(excSat)
                ExcludeFlag=false;
                for kk=1:size(excSat,1)
                    if SatName==excSat(kk,1)
                        ExcludeFlag=true;
                        break;
                    end
                end
                if ExcludeFlag==true
                    continue
                end
            end
            ObsIndex=str2num(char(str1(3)));
            if bReadCom==true&&ObsIndex==0
                continue
            end
            Prn=str2num(char(str1(2)));
            satData(Prn).time(StaIndex(Prn,1),1)=secofWeek;
            satData(Prn).satName=SatName;
            satData(Prn).obsIndex(StaIndex(Prn,1),1)=ObsIndex;
            %卫星方位角、高度角
            satData(Prn).pos(StaIndex(Prn,1),1)=str2num(char(str1(3+1)));
            satData(Prn).pos(StaIndex(Prn,1),2)=str2num(char(str1(4+1)));

            if satData(Prn).pos(StaIndex(Prn,1),2)<10.0 % 截止高度角
               for f=1:3
                    for j=1:10
                        satData(Prn).freqData(f).data(StaIndex(Prn,1),j)=nan;
                    end

               end
            else
                for f=1:3
                    for j=1:10
                        satData(Prn).freqData(f).data(StaIndex(Prn,1),j)=str2num(char(str1((f-1)*10+4+j+1)));
                        if satData(Prn).freqData(f).data(StaIndex(Prn,1),j)==0.0
                            satData(Prn).freqData(f).data(StaIndex(Prn,1),j)=nan;
                        end
                    end
                    if satData(Prn).freqData(f).data(StaIndex(Prn,1),1)<20 % 截止信噪比
                            satData(Prn).freqData(f).data(StaIndex(Prn,1),1:7)=nan;
                    end

                    if ~isnan(satData(Prn).freqData(f).data(StaIndex(Prn,1),3))% 如果当前的相位双差有值，只保留它的半周内部分并取绝对值
                        tcres_obs_data=satData(Prn).freqData(f).data(StaIndex(Prn,1),3); % 获取当前卫星的相位双差
                        tcres_obs_data=abs(tcres_obs_data-round(tcres_obs_data)); % 保留半周内的部分并取绝对值
                        satData(Prn).freqData(f).data(StaIndex(Prn,1),3)=tcres_obs_data;
                    end

                    if ~isnan(satData(Prn).freqData(f).data(StaIndex(Prn,1),4))% 如果当前的相位三差有值，只保留它的半周内部分并取绝对值
                        tcres_obs_data=satData(Prn).freqData(f).data(StaIndex(Prn,1),4); % 获取当前卫星的相位三差
                        %tcres_obs_data=abs(tcres_obs_data-round(tcres_obs_data)); % 保留半周内的部分并取绝对值
                        satData(Prn).freqData(f).data(StaIndex(Prn,1),4)=tcres_obs_data;
                    end
                end
            end
            StaIndex(Prn,1)=StaIndex(Prn,1)+1;
        end
    end
    k=k+1;
end
fclose(fp);
end


%确定头文件中的频点信息
function [FreqInfor,Index,freqNum]=CheckHeadInfor(line)
    Index=0;
    freqNum=0;
    FreqInfor="";
    if line(1:3)=="GPS"
        str=regexp(line(5:end),",",'split');
        for i=1:size(str,2)
            FreqInfor(i+1)=string(str{1,i});
            freqNum=freqNum+1;
        end
        FreqInfor(1)="GPS";
        Index=1;
    elseif line(1:3)=="BD2"
        str=regexp(line(5:end),",",'split');
        for i=1:size(str,2)
            FreqInfor(i+1)=string(str{1,i});
            freqNum=freqNum+1;
        end
        FreqInfor(1)="BD2";
        Index=2;
    elseif line(1:3)=="BD3"
        str=regexp(line(5:end),",",'split');
        for i=1:size(str,2)
            FreqInfor(i+1)=string(str{1,i});
            freqNum=freqNum+1;
        end
        FreqInfor(1)="BD3";
        Index=3;
    elseif line(1:3)=="GAL"
        str=regexp(line(5:end),",",'split');
        for i=1:size(str,2)
            FreqInfor(i+1)=string(str{1,i});
            freqNum=freqNum+1;
        end
        FreqInfor(1)="GAL";
        Index=4;
    end
end

