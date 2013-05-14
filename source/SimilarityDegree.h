
/**************************************************/
/*         �ۺ����׶�ƥ�䣬�����������ƶ�           */
/**************************************************/

#define  W1	 0.5  //��һ�׶ε�Ȩֵ
#define  W2   0.5	 //�ڶ��׶ε�Ȩֵ
#define  M  100    //ģ��Ƭ�θ���������
#define  NOTFOUND  -1  //��һ�׶�δ������ֵ������-1

/**************ƥ����ܽӿ�******************/
double Matching(double*PriorFormwork,double*PriorTest,int fn,int tn)
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