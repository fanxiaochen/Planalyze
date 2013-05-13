
#include<math.h>

#define  N  500      //定义旋律长度上限
#define  K  6   //参数，用于计算相似度，大小为自定义
#define  THRESHOLD  0.8  //阈值，第一阶段匹配的比较参数
#define  MAXLIMIT   99999 //最大上界
#define LEN_LIMIT  50     //测试片段的上限
#define  MIN(X,Y)  ((X)<(Y)?(X):(Y))    //返回最小值
#define  W1	 0.5  //第一阶段的权值
#define  W2   0.5	 //第二阶段的权值
#define  M  50    //模板片段个数的上限
#define  NOTFOUND  -1  //第一阶段未超过阈值，返回-1

/********旋律的数据结构***************/
typedef struct 
{
	double String[N];      // 频率序列
	double Interval[N-1];   //相邻频率的差值序列，去掉高低的影响，考虑频率的变化
	int s_n;   //频率序列的总长度
	int I_n;  //频率差值序列的总长度，等于s_n-1
}Mld,*Melody;

/*******旋律片段定义********/
typedef struct  
{
	int start;      //取出一个模板片段与测试片段匹配，模板片段第一个值的位置
	double sim;  //记录当前模板片段的相似度
	int change;  //记录需要修正的测试片段的位置
	double tempavg_d;   //记录修改时需要的平均值
	double oldtest[2];   //保存旧的测试片段的值
}ClipSim;



/******将输入的double型的频率序列转换成旋律结构*******/
Melody Convert(double *prior,int n)
{
	Melody m=new Mld;
	for(int i=0;i<n;i++)
		m->String[i]=prior[i];
	m->s_n=n;
	return m;
}

/*************删除旋律的结构**************/
void DeleteMelody(Melody m)
{
	delete m;
}

/*******初始化旋律序列，计算出频率差值序列*********/
int  Init(Melody m)
{
	int i;
	for (i=1;i<m->s_n;i++)
		m->Interval[i-1]=12*(log(m->String[i]/m->String[i-1])/log(2.0));  //计算公式
	return m->s_n -1;  //返回差值序列长度
}

/**********初始化模板和测试序列*************/
void TwoMelodyInit(Melody formwork ,Melody test)
{
	formwork->I_n =Init(formwork);
	test->I_n =Init(test);
}


/********************************取一个模板片段与测试片段匹配****************************/
double  FirstSimilarity(Melody formwork,Melody test,int start,int &change,double &tempavg_d)
{
	int i;
	double distance=0,avg_d=abs(formwork->Interval[start+0]-test->Interval[0]);  //初始化平均值
	double Similarity;
	change=0;    // 初始化记录值
	tempavg_d=0;
	for (i=0;i<test->I_n;i++)
	{
		double temp_d=abs(formwork->Interval[start+i]-test->Interval[i]);
		if(temp_d>=2*avg_d&&temp_d>=2)   //当满足此条件时认为这个点为误差点
		{
			change=i;
			tempavg_d=avg_d;

			distance+=2*avg_d;
			i++;

			avg_d=MAXLIMIT;   //平均值置最大上界，只允许一个误差点

		}
		else
			distance+=temp_d;  //相应模板片段与测试片段计算欧氏距离

		if(avg_d!=MAXLIMIT)
			avg_d=distance/(i+1);
	}
	Similarity=(K*test->I_n-distance)/(K*test->I_n);  //计算相似度

	return Similarity;	
}

/***************模板与测试片段的匹配，返回大于等于阈值的片段********************/
int  FirstMatching(Melody formwork,Melody test,ClipSim CS[])
{
	int start,change;  
	int k=0;   
	double tempavg_d;
	for (start=0;start<=formwork->I_n-test->I_n;start++)   //模板从第一个值开始，寻找满足条件的片段
	{
		double S=FirstSimilarity(formwork,test,start,change,tempavg_d); //计算每个模板片段和测试片段和相似度
		if(S>=THRESHOLD)   //满足阈值条件的模板片段记录下来
		{
			CS[k].start=start;
			CS[k].sim=S;
			CS[k].change=change;
			CS[k++].tempavg_d=tempavg_d;
		}
	}
	return k;   //返回满足的模板片段的个数
}



/***************初始化测试片段，修正误差*****************/
void InitModifiedTest(Melody formwork,Melody test,ClipSim &OneCS)
{

	OneCS.oldtest[0]=test->Interval[OneCS.change];
	test->Interval[OneCS.change]=formwork->Interval[OneCS.start+OneCS.change]
	-OneCS.tempavg_d;

	if(OneCS.change+1<test->I_n)
	{
		OneCS.oldtest[1]=test->Interval[OneCS.change+1];
		test->Interval[OneCS.change+1]=formwork->Interval[OneCS.start+OneCS.change+1]
		+OneCS.tempavg_d;	
	}

}


/********************恢复原来未经修正的测试片段***********************/
void StopModifiedTest(Melody formwork,Melody test,ClipSim &OneCS)
{
	test->Interval[OneCS.change]=OneCS.oldtest[0];
	OneCS.oldtest[0]=0;
	if(OneCS.change+1<test->I_n)
	{
		test->Interval[OneCS.change+1]=OneCS.oldtest[1];
		OneCS.oldtest[1]=0;
	}
}

/****************DTW算法，动态规划的思想********************/
void DTW(double Distance[LEN_LIMIT][LEN_LIMIT],char Record[LEN_LIMIT][LEN_LIMIT]
,double AddDistance[LEN_LIMIT][LEN_LIMIT],int len)
{
	int i,j;
	for (int t=0;t<len;t++)            //初始化累积距离
	{
		AddDistance[0][t]=Distance[0][t];
		AddDistance[t][0]=Distance[t][0];
	}

	for (i=1;i<len;i++)
	{
		for (j=1;j<len;j++)
		{
			double right=AddDistance[i][j-1];                  //状态转移方程
			double left=AddDistance[i-1][j];
			double middle=AddDistance[i-1][j-1];
			AddDistance[i][j]=MIN(MIN(left,right),			//状态转移方程
				middle);
			if(AddDistance[i][j]==middle)  //记录匹配路径
				Record[i][j]='m';
			else if(AddDistance[i][j]==left)
				Record[i][j]='l';
			else
				Record[i][j]='r';

			AddDistance[i][j]+=Distance[i][j];		//状态转移方程
		}
	}
}

/*****************获取所需的最短累积距离********************/
double GetDistance(double AddDistance[LEN_LIMIT][LEN_LIMIT],int k)
{
	return AddDistance[k-1][k-1];
}



/**************根据路径得到匹配的频率值的个数，为了计算相似度****************/
int FindRoutineNumber(char Record[LEN_LIMIT][LEN_LIMIT],int &k)
{
	if (k==1)             //只有一对数时返回1
	{
		k--;
		return 1;
	}
	int i,j;
	int count=1;
	i=k-1;
	j=k-1;
	while(i>0&&j>0)    //根据记录逐步回溯，寻找路径
	{
		if(Record[i][j]=='l')
			i--;
		else if(Record[i][j]=='r')
			j--;
		else
		{
			i--;
			j--;
		}
		count++;
	}

	if (i==0&&j==0)        //重新给k赋值，便于进一步的回溯
		k=0;
	else if(i!=0)
		k=i;
	else
		k=j;

	return count;	
}

/************************计算在改进的DTW算法下的相似度***************************/
double DTWSimilarity(double Distance[LEN_LIMIT][LEN_LIMIT],char Record[LEN_LIMIT][LEN_LIMIT]
,double AddDistance[LEN_LIMIT][LEN_LIMIT],int len)
{
	int k=len;
	int TotalNum=0;
	double TotalDistance=0;
	DTW(Distance,Record,AddDistance,len);
	while (k>0)
	{
		TotalDistance+=GetDistance(AddDistance,k);
		TotalNum+=FindRoutineNumber(Record,k);
	}
	return (K*TotalNum-TotalDistance)/(K*TotalNum);
}


/************第二阶段匹配，返回此时的模板片段和测试片段的相似度************/
double SecondSimilarity(Melody formwork,Melody test,int start)
{
	double Distance[LEN_LIMIT][LEN_LIMIT],AddDistance[LEN_LIMIT][LEN_LIMIT];
	char Record[LEN_LIMIT][LEN_LIMIT];
	double Similarity;

	int i,j;
	for(i=0;i<test->I_n;i++)//DTW算法的初始化，计算两段要进行匹配片段之间每两个频率值的距离
		for(j=0;j<test->I_n;j++)
		{
			Distance[i][j]=abs(formwork->Interval[i+start]-test->Interval[j]);
		}

		Similarity=DTWSimilarity(Distance, Record,AddDistance,test->I_n);   //计算DTW相似度

		return Similarity;
}


/************第二阶段匹配，返回最大相似度***************/
double SecondMatching(Melody formwork,Melody test,ClipSim CS[],int k,int& location)
{
	int i; 
	double Similarity=0;
	location=0;   
	for (i=0;i<k;i++)
	{
		InitModifiedTest(formwork,test,CS[i]);
		double temp;
		temp=SecondSimilarity(formwork,test, CS[i].start);
		if (temp>Similarity)
		{
			Similarity=temp;
			location=i;
		}
		StopModifiedTest(formwork,test,CS[i]);
	}
	return Similarity;
}



/**************匹配的总接口******************/
extern "C" _declspec(dllexport) double Matching(double*PriorFormwork,double*PriorTest,int fn,int tn)
{
	int k,location;
	ClipSim CS[M];
	double S,Similarity;
	Melody formwork,test;
	formwork=Convert(PriorFormwork,fn);  //将输入转换成内部的数据结构
	test=Convert(PriorTest,tn);
	TwoMelodyInit(formwork ,test);  //模板和测试片段的初始化
	k=FirstMatching(formwork,test,CS);  //第一阶段匹配
	if(k==0)
		return NOTFOUND;
	S=SecondMatching(formwork,test,CS,k,location);   //第二阶段匹配
	Similarity=W1*CS[location].sim+W2*S;   //计算两阶段总的相似度
	DeleteMelody(formwork);
	DeleteMelody(test);
	return Similarity;
}