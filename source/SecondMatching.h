
/**************************************************/
/*                           �ڶ��׶�ƥ��                            */
/**************************************************/

#define LEN_LIMIT  100     //����Ƭ�ε�����
#define  MIN(X,Y)  ((X)<(Y)?(X):(Y))    //������Сֵ

/***************��ʼ������Ƭ�Σ��������*****************/
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


/********************�ָ�ԭ��δ�������Ĳ���Ƭ��***********************/
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

/****************DTW�㷨����̬�滮��˼��********************/
void DTW(double Distance[LEN_LIMIT][LEN_LIMIT],char Record[LEN_LIMIT][LEN_LIMIT]
,double AddDistance[LEN_LIMIT][LEN_LIMIT],int len)
{
	int i,j;
	for (int t=0;t<len;t++)            //��ʼ���ۻ�����
	{
		AddDistance[0][t]=Distance[0][t];
		AddDistance[t][0]=Distance[t][0];
	}

	for (i=1;i<len;i++)
	{
		for (j=1;j<len;j++)
		{
			double right=AddDistance[i][j-1];                  //״̬ת�Ʒ���
			double left=AddDistance[i-1][j];
			double middle=AddDistance[i-1][j-1];
			AddDistance[i][j]=MIN(MIN(left,right),			//״̬ת�Ʒ���
				middle);
			if(AddDistance[i][j]==middle)  //��¼ƥ��·��
				Record[i][j]='m';
			else if(AddDistance[i][j]==left)
				Record[i][j]='l';
			else
				Record[i][j]='r';

			AddDistance[i][j]+=Distance[i][j];		//״̬ת�Ʒ���
		}
	}
}

/*****************��ȡ���������ۻ�����********************/
double GetDistance(double AddDistance[LEN_LIMIT][LEN_LIMIT],int k)
{
	return AddDistance[k-1][k-1];
}



/**************����·���õ�ƥ���Ƶ��ֵ�ĸ�����Ϊ�˼������ƶ�****************/
int FindRoutineNumber(char Record[LEN_LIMIT][LEN_LIMIT],int &k)
{
	if (k==1)             //ֻ��һ����ʱ����1
	{
		k--;
		return 1;
	}
	int i,j;
	int count=1;
	i=k-1;
	j=k-1;
	while(i>0&&j>0)    //���ݼ�¼�𲽻��ݣ�Ѱ��·��
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

	if (i==0&&j==0)        //���¸�k��ֵ�����ڽ�һ���Ļ���
		k=0;
	else if(i!=0)
		k=i;
	else
		k=j;

	return count;	
}

/************************�����ڸĽ���DTW�㷨�µ����ƶ�***************************/
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


/************�ڶ��׶�ƥ�䣬���ش�ʱ��ģ��Ƭ�κͲ���Ƭ�ε����ƶ�************/
double SecondSimilarity(Melody formwork,Melody test,int start)
{
	double Distance[LEN_LIMIT][LEN_LIMIT],AddDistance[LEN_LIMIT][LEN_LIMIT];
	char Record[LEN_LIMIT][LEN_LIMIT];
	double Similarity;

	int i,j;
	for(i=0;i<test->I_n;i++)//DTW�㷨�ĳ�ʼ������������Ҫ����ƥ��Ƭ��֮��ÿ����Ƶ��ֵ�ľ���
		for(j=0;j<test->I_n;j++)
		{
			Distance[i][j]=abs(formwork->Interval[i+start]-test->Interval[j]);
		}
	
	 Similarity=DTWSimilarity(Distance, Record,AddDistance,test->I_n);   //����DTW���ƶ�

	return Similarity;
}


/************�ڶ��׶�ƥ�䣬����������ƶ�***************/
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

