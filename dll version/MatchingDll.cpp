
#include<math.h>

#define  N  500      //�������ɳ�������
#define  K  6   //���������ڼ������ƶȣ���СΪ�Զ���
#define  THRESHOLD  0.8  //��ֵ����һ�׶�ƥ��ıȽϲ���
#define  MAXLIMIT   99999 //����Ͻ�
#define LEN_LIMIT  50     //����Ƭ�ε�����
#define  MIN(X,Y)  ((X)<(Y)?(X):(Y))    //������Сֵ
#define  W1	 0.5  //��һ�׶ε�Ȩֵ
#define  W2   0.5	 //�ڶ��׶ε�Ȩֵ
#define  M  50    //ģ��Ƭ�θ���������
#define  NOTFOUND  -1  //��һ�׶�δ������ֵ������-1

/********���ɵ����ݽṹ***************/
typedef struct 
{
	double String[N];      // Ƶ������
	double Interval[N-1];   //����Ƶ�ʵĲ�ֵ���У�ȥ���ߵ͵�Ӱ�죬����Ƶ�ʵı仯
	int s_n;   //Ƶ�����е��ܳ���
	int I_n;  //Ƶ�ʲ�ֵ���е��ܳ��ȣ�����s_n-1
}Mld,*Melody;

/*******����Ƭ�ζ���********/
typedef struct  
{
	int start;      //ȡ��һ��ģ��Ƭ�������Ƭ��ƥ�䣬ģ��Ƭ�ε�һ��ֵ��λ��
	double sim;  //��¼��ǰģ��Ƭ�ε����ƶ�
	int change;  //��¼��Ҫ�����Ĳ���Ƭ�ε�λ��
	double tempavg_d;   //��¼�޸�ʱ��Ҫ��ƽ��ֵ
	double oldtest[2];   //����ɵĲ���Ƭ�ε�ֵ
}ClipSim;



/******�������double�͵�Ƶ������ת�������ɽṹ*******/
Melody Convert(double *prior,int n)
{
	Melody m=new Mld;
	for(int i=0;i<n;i++)
		m->String[i]=prior[i];
	m->s_n=n;
	return m;
}

/*************ɾ�����ɵĽṹ**************/
void DeleteMelody(Melody m)
{
	delete m;
}

/*******��ʼ���������У������Ƶ�ʲ�ֵ����*********/
int  Init(Melody m)
{
	int i;
	for (i=1;i<m->s_n;i++)
		m->Interval[i-1]=12*(log(m->String[i]/m->String[i-1])/log(2.0));  //���㹫ʽ
	return m->s_n -1;  //���ز�ֵ���г���
}

/**********��ʼ��ģ��Ͳ�������*************/
void TwoMelodyInit(Melody formwork ,Melody test)
{
	formwork->I_n =Init(formwork);
	test->I_n =Init(test);
}


/********************************ȡһ��ģ��Ƭ�������Ƭ��ƥ��****************************/
double  FirstSimilarity(Melody formwork,Melody test,int start,int &change,double &tempavg_d)
{
	int i;
	double distance=0,avg_d=abs(formwork->Interval[start+0]-test->Interval[0]);  //��ʼ��ƽ��ֵ
	double Similarity;
	change=0;    // ��ʼ����¼ֵ
	tempavg_d=0;
	for (i=0;i<test->I_n;i++)
	{
		double temp_d=abs(formwork->Interval[start+i]-test->Interval[i]);
		if(temp_d>=2*avg_d&&temp_d>=2)   //�����������ʱ��Ϊ�����Ϊ����
		{
			change=i;
			tempavg_d=avg_d;

			distance+=2*avg_d;
			i++;

			avg_d=MAXLIMIT;   //ƽ��ֵ������Ͻ磬ֻ����һ������

		}
		else
			distance+=temp_d;  //��Ӧģ��Ƭ�������Ƭ�μ���ŷ�Ͼ���

		if(avg_d!=MAXLIMIT)
			avg_d=distance/(i+1);
	}
	Similarity=(K*test->I_n-distance)/(K*test->I_n);  //�������ƶ�

	return Similarity;	
}

/***************ģ�������Ƭ�ε�ƥ�䣬���ش��ڵ�����ֵ��Ƭ��********************/
int  FirstMatching(Melody formwork,Melody test,ClipSim CS[])
{
	int start,change;  
	int k=0;   
	double tempavg_d;
	for (start=0;start<=formwork->I_n-test->I_n;start++)   //ģ��ӵ�һ��ֵ��ʼ��Ѱ������������Ƭ��
	{
		double S=FirstSimilarity(formwork,test,start,change,tempavg_d); //����ÿ��ģ��Ƭ�κͲ���Ƭ�κ����ƶ�
		if(S>=THRESHOLD)   //������ֵ������ģ��Ƭ�μ�¼����
		{
			CS[k].start=start;
			CS[k].sim=S;
			CS[k].change=change;
			CS[k++].tempavg_d=tempavg_d;
		}
	}
	return k;   //���������ģ��Ƭ�εĸ���
}



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
	return Similarity;
}



/**************ƥ����ܽӿ�******************/
extern "C" _declspec(dllexport) double Matching(double*PriorFormwork,double*PriorTest,int fn,int tn)
{
	int k,location;
	ClipSim CS[M];
	double S,Similarity;
	Melody formwork,test;
	formwork=Convert(PriorFormwork,fn);  //������ת�����ڲ������ݽṹ
	test=Convert(PriorTest,tn);
	TwoMelodyInit(formwork ,test);  //ģ��Ͳ���Ƭ�εĳ�ʼ��
	k=FirstMatching(formwork,test,CS);  //��һ�׶�ƥ��
	if(k==0)
		return NOTFOUND;
	S=SecondMatching(formwork,test,CS,k,location);   //�ڶ��׶�ƥ��
	Similarity=W1*CS[location].sim+W2*S;   //�������׶��ܵ����ƶ�
	DeleteMelody(formwork);
	DeleteMelody(test);
	return Similarity;
}