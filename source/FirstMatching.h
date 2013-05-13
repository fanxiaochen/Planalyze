
/**************************************************/
/*                           第一阶段匹配                            */
/**************************************************/

#define  K  6   //参数，用于计算相似度，大小为自定义
#define  THRESHOLD  0.8  //阈值，第一阶段匹配的比较参数
#define  MAXLIMIT   99999 //最大上界


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
	printf("%lf\n",Similarity);
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

