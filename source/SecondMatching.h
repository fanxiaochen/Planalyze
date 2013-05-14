
/**************************************************/
/*                           第二阶段匹配                            */
/**************************************************/

#define LEN_LIMIT  100     //测试片段的上限
#define  MIN(X,Y)  ((X)<(Y)?(X):(Y))    //返回最小值

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
	printf("\n%lf\n\n",Similarity);
	return Similarity;
}

