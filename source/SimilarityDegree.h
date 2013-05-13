
/**************************************************/
/*         综合两阶段匹配，返回最终相似度           */
/**************************************************/

#define  W1	 0.5  //第一阶段的权值
#define  W2   0.5	 //第二阶段的权值
#define  M  100    //模板片段个数的上限
#define  NOTFOUND  -1  //第一阶段未超过阈值，返回-1

/**************匹配的总接口******************/
double Matching(double*PriorFormwork,double*PriorTest,int fn,int tn)
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