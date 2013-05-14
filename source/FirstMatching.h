
/**************************************************/
/*                           ��һ�׶�ƥ��                            */
/**************************************************/

#define  K  6   //���������ڼ������ƶȣ���СΪ�Զ���
#define  THRESHOLD  0.8  //��ֵ����һ�׶�ƥ��ıȽϲ���
#define  MAXLIMIT   99999 //����Ͻ�


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
	printf("%lf\n",Similarity);
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

