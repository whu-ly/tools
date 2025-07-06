%% ���� "GNGGA" ���
% 2025-07-02, Implement single file processing, edit by Lying
% 2025-07-06, Implement batch processing of folders, edit by Lying

clear;clc;

% ��������
year = 2025; month = 05; day = 20;
InputFileName = 'E:\01Prog\04CDTH\02Data\202410@CDTH@CropperGNSS@TH1100\20241202@TH1100@BSDVRS@UGV@Trees\res\POSRTKDLLV342_TH1100';  % �����ļ�Ҳ�������ļ���

if isfolder(InputFileName)
    fileList = dir(fullfile(InputFileName, '*'));
    fileList = fileList(~[fileList.isdir]);  % ȥ���ļ���
    for k = 1:length(fileList)
        fullPath = fullfile(InputFileName, fileList(k).name);
        fprintf('�����ļ���%s\n', fullPath);
        try
            processSingleFile(fullPath, year, month, day);
        catch ME
            warning('�����ļ�ʧ��: %s\nԭ��: %s', fullPath, ME.message);
        end
    end
else
    processSingleFile(InputFileName, year, month, day);
end

%% �Ӻ���
% �������ļ�
function processSingleFile(InputFileName, year, month, day)
    % �Զ���������ļ�·��
    [filepath, name, ~] = fileparts(InputFileName);
    OutputFileName = fullfile(filepath, [name, '.txt']);
    
    % ���ļ�
    fid_input = fopen(InputFileName, 'rt');
    if fid_input == -1
        error('�޷��������ļ�: %s', InputFileName);
    end
    
    fid_output = fopen(OutputFileName, 'w');
    if fid_output == -1
        fclose(fid_input); % �ȹر��Ѵ򿪵������ļ�
        error('�޷�������ļ�: %s', OutputFileName);
    end
    
    % 1.��ȡNMEA��ʽ��GGA�ֶ���Ϣ������ת���ʹ���
    weeks=[]; secs=[];
    posXs=[]; posYs=[]; posZs=[];
    status=[]; satNums=[];
    while ~feof(fid_input)
        line = fgetl(fid_input);
        answer = findstr(line,'GNGGA');
        if (~isempty(answer)&&~isempty(line)&&(length(line)>36))
           [~,line]=strtok(line,',');
    
           [strtempt,line]=strtok(line,',');%ʱ��
           hour=str2double(strtempt(1:2)); min=str2double(strtempt(3:4)); sec=str2double(strtempt(5:9));
           [gt.week, gt.SecOfWeek]=YMDHMS2GPST_inMain(year,month,day,hour,min,sec);
           weeks=[weeks;gt.week]; secs=[secs;gt.SecOfWeek+18];
    
           [strtempt,line]=strtok(line,',');%γ��
           if size(strtempt)<=1
               weeks(size(weeks,1))=[];
               secs(size(secs,1))=[];
               continue
           end
           if 0
               Bdd=str2double(strtempt(1:3));
               Bmm=str2double(strtempt(4:end));
           else
               Bdd=str2double(strtempt(1:2));
               Bmm=str2double(strtempt(3:end));%��о��ublox
           end
           B=Bdd+Bmm/60.0;
           [~,line]=strtok(line,',');
    
           [strtempt,line]=strtok(line,',');%����
           Ldd=str2double(strtempt(1:3));Lmm=str2double(strtempt(4:end));
           L=Ldd+Lmm/60.0;
           [~,line]=strtok(line,',');
    
           [strtempt,line]=strtok(line,',');%��״̬
           status=[status;str2double(strtempt)];
    
           [strtempt,line]=strtok(line,',');%GPS������
           satNums=[satNums;str2double(strtempt)];
    
           [strtempt,line]=strtok(line,',');%HDOP
           HDOP=str2double(strtempt);
    
           [strtempt,line]=strtok(line,',');%���θ�
           altitude=str2double(strtempt);
           [~,line]=strtok(line,',');
           [strtempt,line]=strtok(line,',');%�����
           H=altitude+str2double(strtempt);
    
           [XYZ] = LLH2XYZ_inMain(B*pi/180.0, L*pi/180.0, H);
           posXs=[posXs; XYZ(1)];
           posYs=[posYs; XYZ(2)];
           posZs=[posZs; XYZ(3)];
        end
    end
    
    % 2.����������̶���ƽ��ֵ��Ϊ�ο�ֵ
    XYZ_ref=[0,0,0];
    enum=0;
    for i=1:length(weeks)
        if status(i)==4
            XYZ_ref(1)=XYZ_ref(1)*enum/(enum+1)+posXs(i)/(enum+1);
            XYZ_ref(2)=XYZ_ref(2)*enum/(enum+1)+posYs(i)/(enum+1);
            XYZ_ref(3)=XYZ_ref(3)*enum/(enum+1)+posZs(i)/(enum+1);
            enum=enum+1;
        end
    end
    fprintf("%13.4f %13.4f %13.4f\n",XYZ_ref(1),XYZ_ref(2),XYZ_ref(3));
    
    % �����������
    dE=[]; dN=[]; dU=[];
    BLH_ref=XYZ2LLH_inMain(XYZ_ref(1),XYZ_ref(2),XYZ_ref(3));
    for i=1:length(weeks)
        vxyz=[posXs(i)-XYZ_ref(1) posYs(i)-XYZ_ref(2) posZs(i)-XYZ_ref(3)];
        venu=BL2ENU_inMain(BLH_ref(1),BLH_ref(2),vxyz);
        dE=[dE;venu(1)];
        dN=[dN;venu(2)];
        dU=[dU;venu(3)];
    end
    
    % 3.���ѡ������������룬X���꣬Y���꣬Z����
    for i=1:length(weeks)
        fprintf(fid_output,'%4d,%8.1f,%d,%14.3f,%14.3f,%14.3f\n',weeks(i),secs(i),status(i),posXs(i),posYs(i),posZs(i));
        %fprintf(fid_output,'%4d %8.1f %18.10f %18.10f %14.3f
        %%2d\n',GPSweek(i),SecOfWeek(i),B(i),L(i),H(i),status(i)); % ��ͨ��rtkpost.exeת����kml�ļ�
    end
    
    fclose('all');
end

% ������ʱ����ת��ΪGPSʱ������������
function [gpsWeek, sow] = YMDHMS2GPST_inMain(year,month,day,hour,min,sec)
    if(month <= 2)
        year = year - 1;
        month = month + 12;
    end

    UT = hour + min / 60.0 + sec / 3600.0;
    UT = UT / 24.0;

    fracDay = UT - floor(UT);
    days = floor(365.25 * year) + floor(30.6001 * (month + 1)) + day + floor(UT) - 679019;

    gpsWeek = floor(((days + fracDay) - 44244) / 7);
    sow = (days - 44244 - gpsWeek * 7) * 86400 + fracDay * 86400;
end

% XYZתBLH
function [LLH] = XYZ2LLH_inMain(X,Y,Z)
    a = 6378137.0; %WGS84
    e = 0.0818191908425;
    e2 = e*e;
    PI = 3.1415926535897932384626433832795;

    r2 = X * X + Y * Y;
    z = Z;
    zk = 0.0;
    v = a;
    sinp = 0.0;

    while abs(z - zk) >= 1E-4
        zk = z;
	    sinp = z / sqrt(r2 + z * z);
	    v = a / sqrt(1.0 - e2 * sinp*sinp);
	    z = Z + v * e2*sinp;
    end

    if r2 > 1E-12
        LLH(1) = atan(z / sqrt(r2));
        LLH(2) = atan2(Y, X);
    else
        if Z > 0.0
        LLH(1) = PI / 2.0;
        else
        LLH(1) = -PI / 2.0;
        end
        LLH(2) = 0.0;
    end
    LLH(3) = sqrt(r2 + z * z) - v;
end

% BLHתXYZ
function [X] = LLH2XYZ_inMain(B,L,H)
    gs_WGS84_FE = 1.0 / 298.257223563;
    gs_WGS84_a = 6378137.0;
    gs_WGS84_e2 = 2.0 * gs_WGS84_FE - (gs_WGS84_FE)*(gs_WGS84_FE);
    
    sinp = sin(B);
    cosp = cos(B);
    sinl = sin(L);
    cosl = cos(L);
    v = gs_WGS84_a / sqrt(1.0 - gs_WGS84_e2 * sinp * sinp);
    
    X(1) = (v + H)*cosp*cosl;
    X(2) = (v + H)*cosp*sinl;
    X(3) = (v * (1.0 - gs_WGS84_e2) + H)*sinp;
end

% BLתENU
function [venu] = BL2ENU_inMain(B,L,vxyz)
    R=zeros(3,3);
    sinp=sin(B); cosp=cos(B); sinl=sin(L); cosl=cos(L);
    R(1,1)=-sinl;      R(1,2)=cosl;       R(1,3)=0.0;
    R(2,1)=-sinp*cosl; R(2,2)=-sinp*sinl; R(2,3)=cosp;
    R(3,1)=cosp*cosl;  R(3,2)=cosp*sinl;  R(3,3)=sinp;
    venu=R*vxyz';
    venu=venu';
end



