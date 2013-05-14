/********************************************/
/*              Ԥ����������������                   */ 
/********************************************/

#define  N  1000      //�������ɳ�������

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

