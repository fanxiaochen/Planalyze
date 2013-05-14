/********************************************/
/*              预处理输入旋律序列                   */ 
/********************************************/

#define  N  1000      //定义旋律长度上限

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

